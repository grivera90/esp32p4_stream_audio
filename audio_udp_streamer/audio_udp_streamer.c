/**
*******************************************************************************
* @file           : audio_udp_streamer.c
* @brief          : UDP audio streamer — int16 LE, seq header, MTU-safe chunks
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
#include <stdint.h>
#include <stdbool.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "lwip/sockets.h"

#include "audio_udp_streamer.h"
/******************************************************************************
    Defines and constants
******************************************************************************/
static const char *TAG = "[UDP_STREAMER]";

#define AUDIO_UDP_HDR_SIZE              ((size_t)sizeof(audio_udp_packet_hdr_t))
#define AUDIO_UDP_MAX_PCM_BYTES         (AUDIO_UDP_MAX_PAYLOAD - AUDIO_UDP_HDR_SIZE)
#define AUDIO_UDP_STREAM_TASK_PRIO      (tskIDLE_PRIORITY + 6)
#define AUDIO_UDP_STREAM_TASK_STACK     (4096)
#define AUDIO_UDP_SOCK_SNDBUF           (64 * 1024)
#define AUDIO_UDP_SEND_RETRY_MAX        (3)
/*
 * HPF 1-polo (~100 Hz) 1 = HPF (DC + hum 50/200 Hz).
 */
#define AUDIO_UDP_HPF_ENABLE            (1)
#define AUDIO_UDP_HPF_FC_HZ             (100u)
#define AUDIO_UDP_HPF_MAX_CH            (4u)
/******************************************************************************
    Data types
******************************************************************************/
struct audio_udp_streamer_s
{
	int sock;
	struct sockaddr_in dest_addr;
	bsp_audio_handle_t audio_bsp;
	TaskHandle_t task_handle;
	volatile bool running;
	uint8_t channels;
	uint16_t seq;
	uint8_t *tx_buf;
	size_t tx_buf_size;
#if AUDIO_UDP_HPF_ENABLE
	int32_t hpf_x_z1[AUDIO_UDP_HPF_MAX_CH];
	int32_t hpf_y_z1[AUDIO_UDP_HPF_MAX_CH];
	int32_t hpf_r_q15;
#endif
};
/******************************************************************************
    Local function prototypes
******************************************************************************/
static void udp_streamer_task (void *arg);
static int32_t sample_i32_shift (int32_t sample, const bsp_audio_mic_format_t *fmt);
static int16_t saturate_i16 (int32_t sample);

#if AUDIO_UDP_HPF_ENABLE
static int32_t hpf_process_sample (audio_udp_streamer_handle_t streamer, uint8_t ch, int32_t x);
#endif

static int send_udp_payload (audio_udp_streamer_handle_t streamer, size_t packet_len);
static void send_block_mtu_safe (audio_udp_streamer_handle_t streamer, const bsp_audio_block_t *blk, const bsp_audio_mic_format_t *fmt, uint8_t channels, uint32_t timestamp_ms);
/******************************************************************************
    Local function definitions
******************************************************************************/
static int32_t sample_i32_shift (int32_t sample, const bsp_audio_mic_format_t *fmt)
{
	if (fmt->lsb_padding_bits > 0)
	{
		return sample >> fmt->lsb_padding_bits;
	}

	return sample >> 8;
}

static int16_t saturate_i16 (int32_t sample)
{
	if (sample > 32767)
	{
		return (int16_t)32767;
	}
	if (sample < -32768)
	{
		return (int16_t)-32768;
	}

	return (int16_t)sample;
}

#if AUDIO_UDP_HPF_ENABLE
/* DC blocker: y[n] = x[n] - x[n-1] + R * y[n-1], R≈1-2π·fc/fs (Q15). */
static int32_t hpf_process_sample (audio_udp_streamer_handle_t streamer, uint8_t ch, int32_t x)
{
	if (ch >= AUDIO_UDP_HPF_MAX_CH)
	{
		return x;
	}

	int32_t y = x - streamer->hpf_x_z1[ch] + (int32_t)(((int64_t)streamer->hpf_y_z1[ch] * (int64_t)streamer->hpf_r_q15) >> 15);
	streamer->hpf_x_z1[ch] = x;
	streamer->hpf_y_z1[ch] = y;
	return y;
}
#endif

static int send_udp_payload (audio_udp_streamer_handle_t streamer, size_t packet_len)
{
	static TickType_t last_send_error_log;
	for (int attempt = 0; attempt < AUDIO_UDP_SEND_RETRY_MAX; attempt++)
	{
		int sent = (int)sendto(streamer->sock, streamer->tx_buf, packet_len, 0, (struct sockaddr *)&streamer->dest_addr, sizeof(streamer->dest_addr));
		if (sent >= 0 && (size_t)sent == packet_len)
		{
			return sent;
		}
		if (errno == ENOMEM || errno == ENOBUFS)
		{
			vTaskDelay(pdMS_TO_TICKS(1));
			continue;
		}
		break;
	}
	if ((xTaskGetTickCount() - last_send_error_log) >= pdMS_TO_TICKS(5000))
	{
		ESP_LOGW(TAG, "Error enviando UDP: errno=%d (%s), bytes=%u", errno, strerror(errno), (unsigned)packet_len);
		last_send_error_log = xTaskGetTickCount();
	}

	return -1;
}

/* Convierte i32→i16 + HPF y envía en trozos MTU sin buffer PCM intermedio. */
static void send_block_mtu_safe (audio_udp_streamer_handle_t streamer, const bsp_audio_block_t *blk, const bsp_audio_mic_format_t *fmt, uint8_t channels, uint32_t timestamp_ms)
{
	if (channels == 0 || blk == NULL || blk->data == NULL || streamer->tx_buf == NULL)
	{
		return;
	}

	const size_t total_frames = blk->samples_per_ch;
	const size_t frame_bytes = (size_t)channels * sizeof(int16_t);
	size_t max_frames = AUDIO_UDP_MAX_PCM_BYTES / frame_bytes;
	if (max_frames == 0 || total_frames == 0)
	{
		return;
	}

	size_t frame_done = 0;
	while (frame_done < total_frames)
	{
		size_t frames = total_frames - frame_done;
		if (frames > max_frames)
		{
			frames = max_frames;
		}

		audio_udp_packet_hdr_t *hdr = (audio_udp_packet_hdr_t *)streamer->tx_buf;
		hdr->magic = AUDIO_UDP_MAGIC;
		hdr->seq = streamer->seq++;
		hdr->timestamp_ms = timestamp_ms;
		hdr->channels = channels;
		hdr->format = AUDIO_UDP_FMT_PCM_S16LE;
		hdr->samples_per_ch = (uint16_t)frames;

		int16_t *out_pcm = (int16_t *)(streamer->tx_buf + AUDIO_UDP_HDR_SIZE);
		for (size_t f = 0; f < frames; f++)
		{
			const size_t src_base = (frame_done + f) * channels;
			for (uint8_t ch = 0; ch < channels; ch++)
			{
				int32_t s = sample_i32_shift(blk->data[src_base + ch], fmt);
#if AUDIO_UDP_HPF_ENABLE
				s = hpf_process_sample(streamer, ch, s);
#endif
				out_pcm[f * channels + ch] = saturate_i16(s);
			}
		}

		(void)send_udp_payload(streamer, AUDIO_UDP_HDR_SIZE + frames * frame_bytes);
		frame_done += frames;
	}
}

static void udp_streamer_task (void *arg)
{
	audio_udp_streamer_handle_t streamer = (audio_udp_streamer_handle_t)arg;
	bsp_audio_mic_format_t fmt;
	bsp_audio_get_mic_format(&fmt);

	bsp_audio_block_t blk;

	while (streamer->running)
	{
		if (bsp_audio_acquire_block(streamer->audio_bsp, &blk, pdMS_TO_TICKS(200)) != BSP_AUDIO_OK)
		{
			continue;
		}

		send_block_mtu_safe(streamer, &blk, &fmt, streamer->channels, blk.timestamp_ms);
		bsp_audio_release_block(streamer->audio_bsp, &blk);
	}

	vTaskDelete(NULL);
}
/******************************************************************************
    Public function definitions
******************************************************************************/
udp_stream_ret_t audio_udp_streamer_start(const audio_udp_streamer_config_t *cfg, audio_udp_streamer_handle_t *out_handle)
{
	if (cfg == NULL || out_handle == NULL || cfg->audio_bsp == NULL || cfg->dest_ip == NULL || (cfg->channels != 2 && cfg->channels != 4)) 
    {
		return UDP_INVALID_ARGS;
	}

	struct audio_udp_streamer_s *streamer = calloc(1, sizeof(*streamer));
	if (streamer == NULL) 
    {
		return UDP_STREAM_ERROR;
	}

	streamer->audio_bsp = cfg->audio_bsp;
	streamer->channels = cfg->channels;
	streamer->tx_buf_size = AUDIO_UDP_MAX_PAYLOAD;
	streamer->tx_buf = malloc(streamer->tx_buf_size);
	if (streamer->tx_buf == NULL) 
    {
		free(streamer);
		return UDP_STREAM_ERROR;
	}

	streamer->sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
	if (streamer->sock < 0) 
    {
		free(streamer->tx_buf);
		free(streamer);
		return UDP_STREAM_ERROR;
	}

	streamer->dest_addr.sin_family = AF_INET;
	streamer->dest_addr.sin_port = htons(cfg->dest_port);
	if (inet_pton(AF_INET, cfg->dest_ip, &streamer->dest_addr.sin_addr) != 1) 
    {
		close(streamer->sock);
		free(streamer->tx_buf);
		free(streamer);
		return UDP_INVALID_ARGS;
	}

	int tos = 0xB8; /* DSCP EF-ish / high priority on many APs */
	setsockopt(streamer->sock, IPPROTO_IP, IP_TOS, &tos, sizeof(tos));

	int sndbuf = AUDIO_UDP_SOCK_SNDBUF;
	setsockopt(streamer->sock, SOL_SOCKET, SO_SNDBUF, &sndbuf, sizeof(sndbuf));

	streamer->running = true;
#if AUDIO_UDP_HPF_ENABLE
	{
		bsp_audio_mic_format_t fmt;
		bsp_audio_get_mic_format(&fmt);
		uint32_t fs = fmt.sample_rate_hz;
		if (fs < 1000u)
		{
			fs = 16000u;
		}
		/* R_q15 ≈ (1 - 2π·fc/fs)·32768 ; 2π·32768 ≈ 205887 */
		uint32_t r = 32768u - ((205887u * (uint32_t)AUDIO_UDP_HPF_FC_HZ) / fs);
		if (r < 30000u)
		{
			r = 30000u;
		}
		if (r > 32767u)
		{
			r = 32767u;
		}
		streamer->hpf_r_q15 = (int32_t)r;
	}
#endif
	BaseType_t ok = xTaskCreate(udp_streamer_task, "udp_stream_task", AUDIO_UDP_STREAM_TASK_STACK, streamer, AUDIO_UDP_STREAM_TASK_PRIO, &streamer->task_handle);
	if (ok != pdPASS) 
    {
		close(streamer->sock);
		free(streamer->tx_buf);
		free(streamer);
		return UDP_STREAM_ERROR;
	}

	*out_handle = streamer;
	ESP_LOGI(TAG, "UDP stream -> %s:%u | ch=%u | fmt=s16le | max_payload=%u", cfg->dest_ip, (unsigned)cfg->dest_port, (unsigned)cfg->channels, (unsigned)AUDIO_UDP_MAX_PAYLOAD);

	return UDP_STREAM_OK;
}

udp_stream_ret_t audio_udp_streamer_stop(audio_udp_streamer_handle_t handle)
{
	if (handle == NULL) 
    {
		return UDP_INVALID_ARGS;
	}

	handle->running = false;
	/* Give the task a moment to exit acquire timeout and free pcm */
	vTaskDelay(pdMS_TO_TICKS(250));

	if (handle->sock >= 0) 
    {
		close(handle->sock);
		handle->sock = -1;
	}

	free(handle->tx_buf);
	free(handle);
	return UDP_STREAM_OK;
}
