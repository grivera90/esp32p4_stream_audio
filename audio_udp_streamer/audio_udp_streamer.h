/**
*******************************************************************************
* @file           : audio_udp_streamer.h
* @brief          : UDP audio streamer (int16 LE + framed packets)
* @author         :
* @date           : dd/mm/aaaa
*******************************************************************************
* @attention
*
* Copyright (c) <date> grivera. All rights reserved.
*
*/
#ifndef __AUDIO_UDP_STREAMER_H__
#define __AUDIO_UDP_STREAMER_H__
/******************************************************************************
        Includes
******************************************************************************/
#include <stdint.h>
#include <stdbool.h>

#include "bsp_audio.h"
/******************************************************************************
        Constants
******************************************************************************/
/** Magic little-endian on the wire: bytes 'A''U' */
#define AUDIO_UDP_MAGIC                 (0x5541u)

/** PCM format codes in audio_udp_packet_hdr_t::format */
#define AUDIO_UDP_FMT_PCM_S16LE         (1u)

/**
 * Max UDP payload (header + PCM). Kept under typical Wi-Fi/hosted MTU
 * to avoid IP fragmentation.
 */
#define AUDIO_UDP_MAX_PAYLOAD           (1400u)
/******************************************************************************
        Data types
******************************************************************************/
typedef enum
{
	UDP_STREAM_OK = 0,
	UDP_STREAM_ERROR,
	UDP_INVALID_ARGS
} udp_stream_ret_t;

/**
 * @brief Wire header prepended to every UDP datagram (little-endian fields).
 *
 * Layout: magic | seq | timestamp_ms | channels | format | samples_per_ch
 * followed by interleaved PCM (format == AUDIO_UDP_FMT_PCM_S16LE).
 */
typedef struct __attribute__((packed))
{
	uint16_t magic;
	uint16_t seq;
	uint32_t timestamp_ms;
	uint8_t  channels;
	uint8_t  format;
	uint16_t samples_per_ch;
} audio_udp_packet_hdr_t;

typedef struct
{
	const char *dest_ip;
	uint16_t dest_port;
	bsp_audio_handle_t audio_bsp;
	uint8_t channels; /**< Must match bsp active_mic_count */

} audio_udp_streamer_config_t;

typedef struct audio_udp_streamer_s *audio_udp_streamer_handle_t;
/******************************************************************************
        Public function prototypes
******************************************************************************/
#ifdef __cplusplus
extern "C" {
#endif

udp_stream_ret_t audio_udp_streamer_start(const audio_udp_streamer_config_t *cfg, audio_udp_streamer_handle_t *out_handle);
udp_stream_ret_t audio_udp_streamer_stop(audio_udp_streamer_handle_t handle);

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* __AUDIO_UDP_STREAMER_H__ */
