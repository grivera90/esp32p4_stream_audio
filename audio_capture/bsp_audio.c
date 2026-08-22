/**
*******************************************************************************
* @file           : bsp_audio.c
* @brief          : Description of C implementation module
* @author         : 
* @date           : dd/mm/aaaa
*******************************************************************************
* @attention
*
* Copyright (c) <date> grivera. All rights reserved.
*
*/
/******************************************************************************
    Includes
******************************************************************************/
#include <stdlib.h>
#include <string.h>
#include <stddef.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"

#include "esp_log.h"
#include "esp_err.h"
#include "esp_check.h"
#include "driver/gpio.h"
#include "driver/i2s_std.h"
#include "soc/i2s_struct.h"

#include "bsp_audio.h"
/******************************************************************************
    Defines and constants
******************************************************************************/
static const char *TAG = "[BSP_AUDIO]";
/********************************************** 
	CHOICE MICS     						  *
**********************************************/
#define BSP_MIC_TYPE						(BSP_MIC_SPH0645)
/*
 * Prueba A/B SPH0645 (OSR 64 → BCLK = Fs * 64):
 *   16000  |  44100  |  48000 (datasheet typ)
 * Con XTAL 40 MHz, 44100 puede quedar aproximado; 16k/48k exactos.
 */
#define BSP_SPH0645_SAMPLE_RATE_HZ			(48000u)
/********************************************** 
	MIC 0 and MIC 1							  *
**********************************************/
#define BSP_BUS0_BCLK_GPIO      			(24)
#define BSP_BUS0_WS_GPIO        			(26)
#define BSP_BUS0_DIN_GPIO       			(46)
/********************************************** 
	MIC 2 and MIC 3							  *
**********************************************/
#define BSP_BUS1_BCLK_GPIO      			(48)   /* Puente PCB con GPIO 4 */
#define BSP_BUS1_WS_GPIO        			(53)   /* Puente PCB con GPIO 5 */
#define BSP_BUS1_DIN_GPIO       			(47)
/********************************************** 
	MICS CONTROL							  *
**********************************************/
#define BSP_HW_SYNC_ENABLED     			true
#define BSP_MIC_PWR_EN_GPIO     			(-1)
#define BSP_MIC_PWR_ACTIVE_HIGH 			true

#define GPIO_BIT_MASK(pin)  (((pin) >= 0) ? (1ULL << (pin)) : 0ULL)
/******************************************************************************
    Data types
******************************************************************************/
struct bsp_audio_handle_s
{
	bsp_audio_config_t config;
	i2s_chan_handle_t rx_bus0;
	i2s_chan_handle_t rx_bus1;
	TaskHandle_t capture_task;
	volatile bool running;
	QueueHandle_t ready_queue;
	QueueHandle_t free_queue;
	bsp_audio_block_t *blocks_pool;
	int32_t *raw_buffer_pool;
};
/******************************************************************************
    Local variables
******************************************************************************/
/* INMP441: 24-bit useful in 32-bit slot; Fs típico 48 kHz. */
static const bsp_audio_mic_format_t s_mic_params_inmp441 =
{
	.data_bits = 24, 
	.lsb_padding_bits = 0, 
	.bclk_hz = 3072000, 
	.sample_rate_hz = 48000, 
	.slot_bit_width = 32
};
/* SPH0645LM4H-B: 18-bit left-justified in slot 32; OSR 64 → BCLK = Fs * 64. */
static const bsp_audio_mic_format_t s_mic_params_sph0645 =
{
	.data_bits = 18,
	.lsb_padding_bits = 14,
	.bclk_hz = BSP_SPH0645_SAMPLE_RATE_HZ * 64u,
	.sample_rate_hz = BSP_SPH0645_SAMPLE_RATE_HZ,
	.slot_bit_width = 32
};
/******************************************************************************
    Local function prototypes
******************************************************************************/
static const bsp_audio_mic_format_t *bsp_get_mic_params (bsp_mic_type_t type);
static i2s_std_slot_config_t bsp_slot_cfg_for_mic (void);
static i2s_std_clk_config_t bsp_clk_cfg_for_mic (uint32_t sample_rate_hz);
static void bsp_i2s_apply_sph0645_rx_delay (i2s_dev_t *hw);
static void bsp_i2s_capture_task (void *arg);
/******************************************************************************
    Local function definitions
******************************************************************************/
static const bsp_audio_mic_format_t *bsp_get_mic_params (bsp_mic_type_t type)
{
	return (type == BSP_MIC_SPH0645) ? &s_mic_params_sph0645 : &s_mic_params_inmp441;
}

static i2s_std_slot_config_t bsp_slot_cfg_for_mic (void)
{
	const bsp_audio_mic_format_t *p = bsp_get_mic_params(BSP_MIC_TYPE);
	i2s_data_bit_width_t data_width = (p->slot_bit_width >= 32) ? I2S_DATA_BIT_WIDTH_32BIT : I2S_DATA_BIT_WIDTH_24BIT;

	/*
	 * t02: Philips (bit_shift=1) + rx_sd_in_dm=2. Equiv. atomic14/RoSchmi en P4.
	 */
	if (BSP_MIC_TYPE == BSP_MIC_SPH0645)
	{
		i2s_std_slot_config_t slot = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(data_width, I2S_SLOT_MODE_STEREO);
		slot.slot_bit_width = (i2s_slot_bit_width_t)p->slot_bit_width;
		slot.ws_width = p->slot_bit_width;
		return slot;
	}

	i2s_std_slot_config_t slot = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(data_width, I2S_SLOT_MODE_STEREO);
	slot.slot_bit_width = (i2s_slot_bit_width_t)p->slot_bit_width;
	slot.ws_width = p->slot_bit_width;
	return slot;
}

static void bsp_i2s_apply_sph0645_rx_delay (i2s_dev_t *hw)
{
	/* 2 = delay SD por flanco neg. (ESP32 clasico: I2S_TIMING_REG BIT(9)). */
	uint32_t timing = hw->rx_timing.val;
	timing &= ~0x3u;
	timing |= 2u;
	hw->rx_timing.val = timing;
}

static i2s_std_clk_config_t bsp_clk_cfg_for_mic (uint32_t sample_rate_hz)
{
	/* P4 rev < 3: PLL_F160M no es fuente I2S valida; XTAL 40 MHz si. */
	i2s_std_clk_config_t clk = I2S_STD_CLK_DEFAULT_CONFIG(sample_rate_hz);
	clk.clk_src = I2S_CLK_SRC_XTAL;
	clk.mclk_multiple = I2S_MCLK_MULTIPLE_256;
	return clk;
}

static void bsp_i2s_capture_task (void *arg)
{
    bsp_audio_handle_t dev = (bsp_audio_handle_t)arg;
    uint8_t mics = dev->config.active_mic_count;
    size_t samples_per_bus = dev->config.block_size_samples * 2; // 2 canales L/R por bus
    size_t bytes_per_bus = samples_per_bus * sizeof(int32_t);

    int32_t *bus0_raw = malloc(bytes_per_bus);
    int32_t *bus1_raw = (mics == 4) ? malloc(bytes_per_bus) : NULL;
    if (bus0_raw == NULL || (mics == 4 && bus1_raw == NULL)) 
	{
        ESP_LOGE(TAG, "Sin memoria para buffers I2S raw");
        vTaskDelete(NULL);
        return;
    }

    while (dev->running) 
	{
        size_t r0 = 0, r1 = 0;
        esp_err_t err0 = i2s_channel_read(dev->rx_bus0, bus0_raw, bytes_per_bus, &r0, pdMS_TO_TICKS(1000));
        esp_err_t err1 = ESP_OK;
        if (mics == 4) 
		{
            err1 = i2s_channel_read(dev->rx_bus1, bus1_raw, bytes_per_bus, &r1, pdMS_TO_TICKS(1000));
        }

        if (err0 == ESP_OK && err1 == ESP_OK && r0 == bytes_per_bus) 
		{
            bsp_audio_block_t *blk = NULL;
            if (xQueueReceive(dev->free_queue, &blk, pdMS_TO_TICKS(10)) == pdTRUE) 
			{ 
                int32_t *dst = blk->data;
                if (mics == 2) 
				{
                    memcpy(dst, bus0_raw, bytes_per_bus);
                } else 
				{
                    for (size_t i = 0; i < dev->config.block_size_samples; i++) 
					{
                        dst[i * 4 + 0] = bus0_raw[i * 2 + 0];
                        dst[i * 4 + 1] = bus0_raw[i * 2 + 1];
                        dst[i * 4 + 2] = bus1_raw[i * 2 + 0];
                        dst[i * 4 + 3] = bus1_raw[i * 2 + 1];
                    }
                }

                blk->len_bytes = dev->config.block_size_samples * mics * sizeof(int32_t);
                blk->samples_per_ch = dev->config.block_size_samples;
                blk->timestamp_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;

                xQueueSend(dev->ready_queue, &blk, portMAX_DELAY);
            }
        }
    }

    free(bus0_raw);
    
    if (bus1_raw) 
    {
        free(bus1_raw);
    }

    vTaskDelete(NULL);
}
/******************************************************************************
    Public function definitions
******************************************************************************/
void bsp_audio_get_caps(bsp_audio_caps_t *out_caps)
{
    if (out_caps != NULL) 
	{
        *out_caps = (bsp_audio_caps_t)
		{
            .mic_count = 4,
            .i2s_bus_count = 2,
            .mic_type = BSP_MIC_TYPE,
            .hw_sync_capable = BSP_HW_SYNC_ENABLED,
            .mic_power_switchable = (BSP_MIC_PWR_EN_GPIO >= 0),
        };
    }
}

void bsp_audio_get_mic_format (bsp_audio_mic_format_t *out_fmt)
{
	if (out_fmt != NULL)
	{
		*out_fmt = *bsp_get_mic_params(BSP_MIC_TYPE);
	}
}

bsp_audio_ret_t bsp_audio_init(const bsp_audio_config_t *cfg, bsp_audio_handle_t *out_handle)
{
    ESP_RETURN_ON_FALSE(cfg && out_handle, ESP_ERR_INVALID_ARG, TAG, "Args inválidos");
    
    // Validar cantidad de micrófonos solicitados
    if (cfg->active_mic_count != 2 && cfg->active_mic_count != 4) 
	{
        ESP_LOGE(TAG, "active_mic_count no válido: debe ser 2 o 4");
        return BSP_AUDIO_BAD_ARGS;
    }

    bsp_audio_handle_t dev = calloc(1, sizeof(struct bsp_audio_handle_s));
    ESP_RETURN_ON_FALSE(dev, ESP_ERR_NO_MEM, TAG, "Sin memoria");

    dev->config = *cfg;

	/* 1. Power On Mics si corresponde */
	if (BSP_MIC_PWR_EN_GPIO >= 0) 
	{
	    gpio_config_t io_cfg = 
		{
	        .pin_bit_mask = GPIO_BIT_MASK(BSP_MIC_PWR_EN_GPIO),
	        .mode = GPIO_MODE_OUTPUT,
	        .pull_up_en = GPIO_PULLUP_DISABLE,
	        .pull_down_en = GPIO_PULLDOWN_DISABLE,
	        .intr_type = GPIO_INTR_DISABLE,
	    };
	    gpio_config(&io_cfg);
	    gpio_set_level(BSP_MIC_PWR_EN_GPIO, BSP_MIC_PWR_ACTIVE_HIGH ? 1 : 0);
	}

    /* 2. Configurar I2S Bus 0 (Master) -> Siempre requerido */
    i2s_chan_config_t chan_cfg0 = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
    ESP_RETURN_ON_ERROR(i2s_new_channel(&chan_cfg0, NULL, &dev->rx_bus0), TAG, "Error I2S 0");

    i2s_std_config_t std_cfg0 = 
	{
        .clk_cfg = bsp_clk_cfg_for_mic(cfg->sample_rate_hz),
        .slot_cfg = bsp_slot_cfg_for_mic(),
        .gpio_cfg = 
		{
            .mclk = I2S_GPIO_UNUSED,
            .bclk = BSP_BUS0_BCLK_GPIO,
            .ws   = BSP_BUS0_WS_GPIO,
            .dout = I2S_GPIO_UNUSED,
            .din  = BSP_BUS0_DIN_GPIO,
			.invert_flags = 
			{
				.mclk_inv = false,
				.bclk_inv = false,
				.ws_inv = false,
			},
        },
    };
    ESP_RETURN_ON_ERROR(i2s_channel_init_std_mode(dev->rx_bus0, &std_cfg0), TAG, "Error std mode I2S 0");

    /* 3. Configurar I2S Bus 1 (Slave Sync) -> Solo si active_mic_count == 4 */
    if (cfg->active_mic_count == 4) 
	{
        i2s_chan_config_t chan_cfg1 = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_1, I2S_ROLE_SLAVE);
        ESP_RETURN_ON_ERROR(i2s_new_channel(&chan_cfg1, NULL, &dev->rx_bus1), TAG, "Error I2S 1");

        i2s_std_config_t std_cfg1 = 
		{
            .clk_cfg = bsp_clk_cfg_for_mic(cfg->sample_rate_hz),
            .slot_cfg = bsp_slot_cfg_for_mic(),
            .gpio_cfg = 
			{
                .mclk = I2S_GPIO_UNUSED,
                .bclk = BSP_BUS1_BCLK_GPIO,
                .ws   = BSP_BUS1_WS_GPIO,
                .dout = I2S_GPIO_UNUSED,
                .din  = BSP_BUS1_DIN_GPIO,
				.invert_flags = 
				{
					.mclk_inv = false,
					.bclk_inv = false,
					.ws_inv = false,
				},
            },
        };
        ESP_RETURN_ON_ERROR(i2s_channel_init_std_mode(dev->rx_bus1, &std_cfg1), TAG, "Error std mode I2S 1");
    }

    /* 4. Asignación del Pool de Memoria según la cantidad de mics */
    uint8_t count = cfg->pool_block_count;
    dev->ready_queue = xQueueCreate(count, sizeof(bsp_audio_block_t *));
    dev->free_queue  = xQueueCreate(count, sizeof(bsp_audio_block_t *));
    dev->blocks_pool = calloc(count, sizeof(bsp_audio_block_t));
    
    size_t samples_per_block = cfg->block_size_samples * cfg->active_mic_count;
    dev->raw_buffer_pool = calloc(count * samples_per_block, sizeof(int32_t));

    for (int i = 0; i < count; i++) 
	{
        dev->blocks_pool[i].data = &dev->raw_buffer_pool[i * samples_per_block];
        bsp_audio_block_t *ptr = &dev->blocks_pool[i];
        xQueueSend(dev->free_queue, &ptr, 0);
    }

    *out_handle = dev;
    ESP_LOGI(TAG, "BSP Audio init OK (%u mics, Fs=%lu Hz, BCLK=%lu Hz, block=%u samp/ch)", (unsigned)cfg->active_mic_count, (unsigned long)cfg->sample_rate_hz, (unsigned long)bsp_get_mic_params(BSP_MIC_TYPE)->bclk_hz, (unsigned)cfg->block_size_samples);
    return BSP_AUDIO_OK;
}

bsp_audio_ret_t bsp_audio_start(bsp_audio_handle_t handle)
{
	ESP_RETURN_ON_FALSE(handle, ESP_ERR_INVALID_ARG, TAG, "Handle nulo");

	ESP_RETURN_ON_ERROR(i2s_channel_enable(handle->rx_bus0), TAG, "Error start bus 0");
	bsp_i2s_apply_sph0645_rx_delay(&I2S0);
	if (handle->config.active_mic_count == 4)
	{
		ESP_RETURN_ON_ERROR(i2s_channel_enable(handle->rx_bus1), TAG, "Error start bus 1");
		bsp_i2s_apply_sph0645_rx_delay(&I2S1);
	}

	/* Datasheet tPOWERUP: max 50 ms after Fclock >= 1 MHz. */
	vTaskDelay(pdMS_TO_TICKS(50));

	handle->running = true;
	BaseType_t created = xTaskCreate(bsp_i2s_capture_task, "bsp_i2s_task", 4096, handle, tskIDLE_PRIORITY + 5, &handle->capture_task);
	if (created != pdPASS)
	{
		ESP_LOGE(TAG, "No se pudo crear bsp_i2s_task");
		handle->running = false;
		return BSP_AUDIO_ERROR;
	}
	return BSP_AUDIO_OK;
}

bsp_audio_ret_t bsp_audio_acquire_block(bsp_audio_handle_t handle, bsp_audio_block_t *out_block, int timeout_ticks)
{
    ESP_RETURN_ON_FALSE(handle && out_block, ESP_ERR_INVALID_ARG, TAG, "Args nulos");
    bsp_audio_block_t *blk_ptr = NULL;

    if (xQueueReceive(handle->ready_queue, &blk_ptr, timeout_ticks) == pdTRUE) 
	{
        *out_block = *blk_ptr;
        return BSP_AUDIO_OK;
    }
	
    return BSP_AUDIO_TIMEOUT;
}

bsp_audio_ret_t bsp_audio_release_block(bsp_audio_handle_t handle, bsp_audio_block_t *block)
{
    ESP_RETURN_ON_FALSE(handle && block, ESP_ERR_INVALID_ARG, TAG, "Args nulos");
    
    // Buscar la referencia original en el pool
    bsp_audio_block_t *orig_ptr = NULL;
    for (int i = 0; i < handle->config.pool_block_count; i++) 
	{
        if (handle->blocks_pool[i].data == block->data) 
		{
            orig_ptr = &handle->blocks_pool[i];
            break;
        }
    }

    if (orig_ptr) 
	{
        xQueueSend(handle->free_queue, &orig_ptr, 0);
        return BSP_AUDIO_OK;
    }
    
	return BSP_AUDIO_ERROR;
}

bsp_audio_ret_t bsp_audio_stop(bsp_audio_handle_t handle)
{
    ESP_RETURN_ON_FALSE(handle, ESP_ERR_INVALID_ARG, TAG, "Handle nulo");
    handle->running = false;
    i2s_channel_disable(handle->rx_bus0);
    if (handle->config.active_mic_count == 4) 
    {
        i2s_channel_disable(handle->rx_bus1);
    }
    return BSP_AUDIO_OK;
}




