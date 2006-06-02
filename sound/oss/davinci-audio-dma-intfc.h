/*
 * linux/sound/oss/davinci-audio-dma-intfc.h
 *
 * Common audio DMA handling for the Davinci processors
 *
 * Copyright (C) 2006 Texas Instruments, Inc.
 *
 * Copyright (C) 2000, 2001 Nicolas Pitre <nico@cam.org>
 *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * THIS PACKAGE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
 * WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 * History:
 *
 * 2005-10-01   Rishi Bhattacharya / Sharath Kumar - Added support for TI Davinci DM644x processor
 */

#ifndef __DAVINCI_AUDIO_DMA_INTFC_H
#define __DAVINCI_AUDIO_DMA_INTFC_H

/******************************* INCLUDES *************************************/

/* Requires davinci-audio.h */
#include "davinci-audio.h"

/************************** GLOBAL MACROS *************************************/

/* Provide the Macro interfaces common across platforms */
#define DMA_REQUEST(e,s, cb)   {e=davinci_request_sound_dma(s->dma_dev, s->id, s, &s->master_ch, &s->lch);}
#define DMA_FREE(s)             davinci_free_sound_dma(s->master_ch,&s->lch)
#define DMA_CLEAR(s)            davinci_clear_sound_dma(s)

/************************** GLOBAL DATA STRUCTURES ****************************/

typedef void (*dma_callback_t) (int lch, u16 ch_status, void *data);

/************************** GLOBAL FUNCTIONS **********************************/

dma_callback_t audio_get_dma_callback(void);
int audio_setup_buf(audio_stream_t * s);
int audio_process_dma(audio_stream_t * s);
void audio_prime_rx(audio_state_t * state);
int audio_set_fragments(audio_stream_t * s, int val);
int audio_sync(struct file *file);
void audio_stop_dma(audio_stream_t * s);
u_int audio_get_dma_pos(audio_stream_t * s);
void audio_reset(audio_stream_t * s);
void audio_discard_buf(audio_stream_t * s);

/**************** ARCH SPECIFIC FUNCIONS **************************************/

void davinci_clear_sound_dma(audio_stream_t * s);

int davinci_request_sound_dma(int device_id, const char *device_name,
			      void *data, int *master_ch, int **channels);
int davinci_free_sound_dma(int master_ch, int **channels);

#endif				/* #ifndef __DAVINCI_AUDIO_DMA_INTFC_H */
