#include <stdbool.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#include "bsp_audio.h"
#include "wifi_connect.h"
#include "audio_udp_streamer.h"

static const char *TAG = "[AUDIO_CAPTURE_APP]";

void app_main(void)
{
	ESP_LOGI(TAG, "WiFi connection start...");
	vTaskDelay(pdMS_TO_TICKS(1000));

	if (wifi_connect_start() == ESP_OK) 
    {
		char ip[16];
		wifi_connect_get_ip_str(ip, sizeof(ip));
		ESP_LOGI(TAG, "WiFi Connected. Local IP: %s", ip);
	} else 
	{
		ESP_LOGE(TAG, "WiFi connection error, abort app");
		return;		
	}

	bsp_audio_mic_format_t mic_fmt;
	bsp_audio_get_mic_format(&mic_fmt);

	/* ~20 ms por bloque: 320@16k, 882@44.1k, 960@48k. */
	uint16_t block_samples = (uint16_t)(mic_fmt.sample_rate_hz / 50u);
	if (block_samples < 160u)
	{
		block_samples = 160u;
	}

	bsp_audio_config_t audio_cfg =
	{
		.sample_rate_hz = mic_fmt.sample_rate_hz,
		.slot_bit_width = mic_fmt.slot_bit_width,
		.block_size_samples = block_samples,
		.pool_block_count = 6,
		.active_mic_count = 2,
	};

	bsp_audio_handle_t audio_bsp = NULL;
	ESP_ERROR_CHECK(bsp_audio_init(&audio_cfg, &audio_bsp));
	ESP_ERROR_CHECK(bsp_audio_start(audio_bsp));

	audio_udp_streamer_config_t streamer_cfg = 
    {
		.dest_ip = "192.168.100.68",
		.dest_port = 5000,
		.audio_bsp = audio_bsp,
		.channels = audio_cfg.active_mic_count,
	};

	audio_udp_streamer_handle_t streamer = NULL;
	if (audio_udp_streamer_start(&streamer_cfg, &streamer) != UDP_STREAM_OK) 
    {
		ESP_LOGE(TAG, "UDP streamer start failed");
		return;
	}

	ESP_LOGI(TAG, "Pipeline ready: I2S -> int16 UDP @ %s:%u  Fs=%lu Hz block=%u", 
																					streamer_cfg.dest_ip, 
																					(unsigned)streamer_cfg.dest_port, 
																					(unsigned long)audio_cfg.sample_rate_hz, 
																					(unsigned)audio_cfg.block_size_samples);

}
