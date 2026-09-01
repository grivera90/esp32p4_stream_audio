/**
*******************************************************************************
* @file           : template.h
* @brief          : Description of header file
* @author         : 
* @date           : dd/mm/aaaa
*******************************************************************************
* @attention
*
* Copyright (c) <date> grivera. All rights reserved.
*
*/
#ifndef __BSP_AUDIO_H__
#define __BSP_AUDIO_H__
/******************************************************************************
        Includes
******************************************************************************/
#include <stdint.h>
#include <stdbool.h>
/******************************************************************************
        Constants
******************************************************************************/

/******************************************************************************
        Data types
******************************************************************************/
typedef enum
{
	BSP_AUDIO_OK = 0,
	BSP_AUDIO_ERROR,
	BSP_AUDIO_BAD_ARGS,
	BSP_AUDIO_TIMEOUT
	
} bsp_audio_ret_t;

typedef enum 
{
    BSP_MIC_INMP441 = 0,
    BSP_MIC_SPH0645,
	
} bsp_mic_type_t;

typedef struct 
{
    uint8_t mic_count;            		/**< Mics físicos instalados (ej. 4) */
    uint8_t i2s_bus_count;        		/**< Buses I2S (ej. 2) */
    bsp_mic_type_t mic_type;            /**< Modelo de mic */
    bool hw_sync_capable;      			/**< Sync de reloj por hardware */
    bool mic_power_switchable; 			/**< Control GPIO de VDD */
	
} bsp_audio_caps_t;

typedef struct
{
	uint8_t data_bits;				/**< Precisión útil (SPH0645: 18) */
	uint8_t lsb_padding_bits;		/**< Right-shift raw i32→PCM: SPH0645=14, INMP441=12 (8 pad+4 headroom) */
	uint32_t bclk_hz;				/**< Datasheet Typ fCLOCK = 3.072 MHz */
	uint32_t sample_rate_hz;		/**< Datasheet Typ Fs = bclk / 64 = 48 kHz */
	uint8_t slot_bit_width;			/**< OSR=64, stereo → 32 bits/slot */

} bsp_audio_mic_format_t;

typedef struct 
{
    uint32_t sample_rate_hz;     		/**< Ej. 16000 Hz */
    uint8_t slot_bit_width;     		/**< 24 o 32 bits */
    uint16_t block_size_samples; 		/**< Muestras por canal por bloque */
    uint8_t pool_block_count;   		/**< Cantidad de bloques en pool zero-copy */
	uint8_t active_mic_count;   		/**< Canales a capturar: 2 (solo Bus0) o 4 (Bus0 + Bus1) */
	
} bsp_audio_config_t;

/**
 * @brief Estructura de bloque de audio para Zero-Copy.
 */
typedef struct 
{
    int32_t *data;               		/**< Buffer de muestras de 32-bit interleaved */
    size_t len_bytes;          		/**< Tamaño total en bytes del bloque */
    size_t samples_per_ch;     		/**< Muestras por canal */
    uint32_t timestamp_ms;       		/**< Timestamp de adquisición */
	
} bsp_audio_block_t;

typedef struct bsp_audio_handle_s *bsp_audio_handle_t;
/******************************************************************************
        Public function prototypes
******************************************************************************/
#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Retorna las capacidades del subsistema de audio.
 *        Sin efectos secundarios. Safe antes de cualquier init.
 */
void bsp_audio_get_caps(bsp_audio_caps_t *out_caps);

/**
 * @brief Retorna el formato de dato del micrófono instalado.
 *        Sin efectos secundarios. Safe antes de cualquier init.
 */
void bsp_audio_get_mic_format(bsp_audio_mic_format_t *out_fmt);

/**
 * @brief Inicializa el subsistema de audio completo (Power + I2S Drivers).
 */
bsp_audio_ret_t bsp_audio_init(const bsp_audio_config_t *cfg, bsp_audio_handle_t *out_handle);

/**
 * @brief Inicia los canales DMA I2S y habilita la captura.
 */
bsp_audio_ret_t bsp_audio_start(bsp_audio_handle_t handle);

/**
 * @brief Detiene la captura y deshabilita los canales I2S.
 */
bsp_audio_ret_t bsp_audio_stop(bsp_audio_handle_t handle);

/**
 * @brief Adquiere un bloque de audio lleno desde la cola/pool zero-copy.
 */
bsp_audio_ret_t bsp_audio_acquire_block(bsp_audio_handle_t handle, bsp_audio_block_t *out_block, int timeout_ticks);

/**
 * @brief Devuelve el bloque al pool para ser rellenado por el I2S DMA.
 */
bsp_audio_ret_t bsp_audio_release_block(bsp_audio_handle_t handle, bsp_audio_block_t *block);

/**
 * @brief Desinicializa los drivers I2S y libera recursos del BSP.
 */
bsp_audio_ret_t bsp_audio_deinit(bsp_audio_handle_t handle);

#ifdef __cplusplus
} // extern "C"
#endif

#endif // EOF __BSP_AUDIO_H__

