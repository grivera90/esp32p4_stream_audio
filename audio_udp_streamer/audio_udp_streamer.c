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
#define AUDIO_UDP_STREAM_TASK_PRIO      (tskIDLE_PRIORITY + 4)
#define AUDIO_UDP_STREAM_TASK_STACK     (4096)
#define AUDIO_UDP_RAW_LOG_PERIOD_MS     (2000)
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
static void log_raw_i32_diag (const bsp_audio_block_t *blk, uint8_t channels);
static void log_i16_diag (const int16_t *pcm, size_t n, uint8_t channels);
static size_t convert_block_to_i16 (audio_udp_streamer_handle_t streamer, const bsp_audio_block_t *blk, const bsp_audio_mic_format_t *fmt, uint8_t channels, int16_t *out_pcm);
static int send_udp_payload (audio_udp_streamer_handle_t streamer, size_t packet_len);
static void send_pcm_mtu_safe (audio_udp_streamer_handle_t streamer, const int16_t *pcm, size_t total_samples, uint8_t channels, uint32_t timestamp_ms);
/******************************************************************************
    Local function definitions
******************************************************************************/
static int32_t sample_i32_shift (int32_t sample, const bsp_audio_mic_format_t *fmt)
{
	if (fmt->lsb_padding_bits > 0)
	{
		/* t03: 18-bit left-justified en slot 32. */
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

/* #11: int32 crudo ANTES del shift. or/and muestran en qué bits vive el payload. */
static void log_raw_i32_diag (const bsp_audio_block_t *blk, uint8_t channels)
{
	if (blk == NULL || blk->data == NULL || channels == 0)
	{
		return;
	}

	for (uint8_t ch = 0; ch < channels; ch++)
	{
		int32_t vmin = INT32_MAX;
		int32_t vmax = INT32_MIN;
		uint32_t bits_or = 0;
		uint32_t bits_and = 0xFFFFFFFFu;
		uint32_t pos = 0;
		uint32_t neg = 0;
		uint32_t zero = 0;
		int32_t s0 = 0;
		int32_t s1 = 0;
		int32_t s2 = 0;
		int32_t s3 = 0;

		for (size_t i = 0; i < blk->samples_per_ch; i++)
		{
			int32_t s = blk->data[i * channels + ch];
			uint32_t u = (uint32_t)s;

			if (s < vmin)
			{
				vmin = s;
			}
			if (s > vmax)
			{
				vmax = s;
			}
			bits_or |= u;
			bits_and &= u;
			if (s > 0)
			{
				pos++;
			}
			else if (s < 0)
			{
				neg++;
			}
			else
			{
				zero++;
			}
			if (i == 0)
			{
				s0 = s;
			}
			else if (i == 1)
			{
				s1 = s;
			}
			else if (i == 2)
			{
				s2 = s;
			}
			else if (i == 3)
			{
				s3 = s;
			}
		}

		ESP_LOGI(TAG, "raw i32 ch%u n=%u min=%ld max=%ld hex=0x%08lX..0x%08lX or=0x%08lX and=0x%08lX pos=%lu neg=%lu z=%lu", (unsigned)ch, (unsigned)blk->samples_per_ch, (long)vmin, (long)vmax, (unsigned long)(uint32_t)vmin, (unsigned long)(uint32_t)vmax, (unsigned long)bits_or, (unsigned long)bits_and, (unsigned long)pos, (unsigned long)neg, (unsigned long)zero);
		ESP_LOGI(TAG, "raw i32 ch%u first4 0x%08lX 0x%08lX 0x%08lX 0x%08lX", (unsigned)ch, (unsigned long)(uint32_t)s0, (unsigned long)(uint32_t)s1, (unsigned long)(uint32_t)s2, (unsigned long)(uint32_t)s3);
	}
}

static void log_i16_diag (const int16_t *pcm, size_t n, uint8_t channels)
{
	if (pcm == NULL || channels == 0 || n < channels)
	{
		return;
	}

	const size_t frames = n / channels;
	for (uint8_t ch = 0; ch < channels; ch++)
	{
		int32_t vmin = 32767;
		int32_t vmax = -32768;
		uint32_t pos = 0;
		uint32_t neg = 0;
		uint32_t zero = 0;

		for (size_t i = 0; i < frames; i++)
		{
			int32_t s = pcm[i * channels + ch];
			if (s < vmin)
			{
				vmin = s;
			}
			if (s > vmax)
			{
				vmax = s;
			}
			if (s > 0)
			{
				pos++;
			}
			else if (s < 0)
			{
				neg++;
			}
			else
			{
				zero++;
			}
		}

		ESP_LOGI(TAG, "pcm i16 ch%u n=%u min=%ld max=%ld pos=%lu neg=%lu z=%lu", (unsigned)ch, (unsigned)frames, (long)vmin, (long)vmax, (unsigned long)pos, (unsigned long)neg, (unsigned long)zero);
	}
}

static size_t convert_block_to_i16 (audio_udp_streamer_handle_t streamer, const bsp_audio_block_t *blk, const bsp_audio_mic_format_t *fmt, uint8_t channels, int16_t *out_pcm)
{
	const size_t total_i32 = blk->len_bytes / sizeof(int32_t);
	const size_t expected = blk->samples_per_ch * (size_t)channels;
	const size_t n = (total_i32 < expected) ? total_i32 : expected;

	if (channels == 0 || blk->samples_per_ch == 0 || streamer == NULL)
	{
		return 0;
	}

#if AUDIO_UDP_HPF_ENABLE
	for (size_t i = 0; i < n; i++)
	{
		uint8_t ch = (uint8_t)(i % channels);
		int32_t s = sample_i32_shift(blk->data[i], fmt);
		s = hpf_process_sample(streamer, ch, s);
		out_pcm[i] = saturate_i16(s);
	}
#else
	/* Fallback: DC = media del bloque por canal, despues del >>14. */
	int64_t acc[4] = {0, 0, 0, 0};
	int32_t mean[4] = {0, 0, 0, 0};
	const uint8_t ch_n = (channels > 4u) ? 4u : channels;

	for (size_t i = 0; i < n; i++)
	{
		uint8_t ch = (uint8_t)(i % channels);
		if (ch < 4u)
		{
			acc[ch] += sample_i32_shift(blk->data[i], fmt);
		}
	}

	for (uint8_t ch = 0; ch < ch_n; ch++)
	{
		mean[ch] = (int32_t)(acc[ch] / (int64_t)blk->samples_per_ch);
	}

	for (size_t i = 0; i < n; i++)
	{
		uint8_t ch = (uint8_t)(i % channels);
		int32_t s = sample_i32_shift(blk->data[i], fmt);
		if (ch < 4u)
		{
			s -= mean[ch];
		}
		out_pcm[i] = saturate_i16(s);
	}
#endif

	return n;
}

static int send_udp_payload (audio_udp_streamer_handle_t streamer, size_t packet_len)
{
	int sent = (int)sendto(streamer->sock, streamer->tx_buf, packet_len, 0, (struct sockaddr *)&streamer->dest_addr, sizeof(streamer->dest_addr));
	if (sent < 0 && (errno == ENOMEM || errno == ENOBUFS))
	{
		vTaskDelay(pdMS_TO_TICKS(1));
		sent = (int)sendto(streamer->sock, streamer->tx_buf, packet_len, 0, (struct sockaddr *)&streamer->dest_addr, sizeof(streamer->dest_addr));
	}

	if (sent < 0 || (size_t)sent != packet_len)
	{
		return -1;
	}

	return sent;
}

static void send_pcm_mtu_safe (audio_udp_streamer_handle_t streamer, const int16_t *pcm, size_t total_samples, uint8_t channels, uint32_t timestamp_ms)
{
	if (channels == 0 || pcm == NULL || streamer->tx_buf == NULL) 
    {
		return;
	}

	const size_t frame_bytes = (size_t)channels * sizeof(int16_t);
	size_t max_frames = AUDIO_UDP_MAX_PCM_BYTES / frame_bytes;
	if (max_frames == 0) 
    {
		return;
	}

	size_t samples_done = 0;
	const size_t total_frames = total_samples / channels;

	while (samples_done < total_samples) 
    {
		size_t frames_left = total_frames - (samples_done / channels);
		size_t frames = (frames_left < max_frames) ? frames_left : max_frames;
		size_t pcm_bytes = frames * frame_bytes;

		audio_udp_packet_hdr_t *hdr = (audio_udp_packet_hdr_t *)streamer->tx_buf;
		hdr->magic = AUDIO_UDP_MAGIC;
		hdr->seq = streamer->seq++;
		hdr->timestamp_ms = timestamp_ms;
		hdr->channels = channels;
		hdr->format = AUDIO_UDP_FMT_PCM_S16LE;
		hdr->samples_per_ch = (uint16_t)frames;

		memcpy(streamer->tx_buf + AUDIO_UDP_HDR_SIZE, (const uint8_t *)pcm + samples_done * sizeof(int16_t), pcm_bytes);

		(void)send_udp_payload(streamer, AUDIO_UDP_HDR_SIZE + pcm_bytes);

		samples_done += frames * channels;
	}
}

static void udp_streamer_task (void *arg)
{
	audio_udp_streamer_handle_t streamer = (audio_udp_streamer_handle_t)arg;
	bsp_audio_mic_format_t fmt;
	bsp_audio_get_mic_format(&fmt);

	const size_t max_i16 = 1024u * 4u;
	int16_t *pcm = malloc(max_i16 * sizeof(int16_t));
	if (pcm == NULL)
	{
		ESP_LOGE(TAG, "Sin memoria para buffer PCM int16");
		vTaskDelete(NULL);
		return;
	}

	bsp_audio_block_t blk;
//	TickType_t last_raw_log = 0;
#if AUDIO_UDP_HPF_ENABLE
	ESP_LOGI(TAG, "raw i32 diag cada %d ms; pcm i16 = (>>pad) + HPF fc=%u Hz", AUDIO_UDP_RAW_LOG_PERIOD_MS, (unsigned)AUDIO_UDP_HPF_FC_HZ);
#else
	ESP_LOGI(TAG, "raw i32 diag cada %d ms; pcm i16 = (>>pad) - mean(bloque)", AUDIO_UDP_RAW_LOG_PERIOD_MS);
#endif

	while (streamer->running)
	{
		if (bsp_audio_acquire_block(streamer->audio_bsp, &blk, pdMS_TO_TICKS(200)) != BSP_AUDIO_OK)
		{
			continue;
		}

		const size_t needed = blk.samples_per_ch * (size_t)streamer->channels;
		if (needed > max_i16)
		{
			bsp_audio_release_block(streamer->audio_bsp, &blk);
			continue;
		}

//		TickType_t now = xTaskGetTickCount();
//		const bool do_diag = ((now - last_raw_log) >= pdMS_TO_TICKS(AUDIO_UDP_RAW_LOG_PERIOD_MS));
//		if (do_diag)
//		{
//			log_raw_i32_diag(&blk, streamer->channels);
//		}

		size_t n = convert_block_to_i16(streamer, &blk, &fmt, streamer->channels, pcm);
//		if (do_diag)
//		{
//			if (n > 0)
//			{
//				log_i16_diag(pcm, n, streamer->channels);
//			}
//			last_raw_log = now;
//		}

		if (n > 0)
		{
			send_pcm_mtu_safe(streamer, pcm, n, streamer->channels, blk.timestamp_ms);
		}

		bsp_audio_release_block(streamer->audio_bsp, &blk);
	}

	free(pcm);
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
