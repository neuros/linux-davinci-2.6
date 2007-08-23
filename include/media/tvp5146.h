/*
 *
 *
 * Copyright (C) 2006 Texas Instruments Inc
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */
/* tvp5146.h file */

#ifndef TVP5146_H
#define TVP5146_H

#ifdef TRUE
#undef TRUE
#endif
#ifdef FALSE
#undef FALSE
#endif

#define TRUE 1
#define FALSE 0

/* analog muxing mode */
#define    TVP5146_AMUX_COMPOSITE  0
#define    TVP5146_AMUX_SVIDEO     1

typedef enum {
	TVP5146_MODE_INV = -1,
	TVP5146_MODE_AUTO = 0,	        /* autoswitch mode (default)   */
	TVP5146_MODE_NTSC = 1,	        /* (M, J) NTSC      525-line   */
	TVP5146_MODE_PAL = 2,	        /* (B, D, G, H, I, N) PAL      */
	TVP5146_MODE_PAL_M = 3,	        /* (M) PAL          525-line   */
	TVP5146_MODE_PAL_CN = 4,	/* (Combination-N) PAL         */
	TVP5146_MODE_NTSC_443 = 5,	/* NTSC 4.43        525-line   */
	TVP5146_MODE_SECAM = 6,	        /* SECAM                       */
	TVP5146_MODE_PAL_60 = 7,	/* PAL 60          525-line    */
	TVP5146_MODE_AUTO_SQP = 8,	/* autoswitch mode (default)   */
	TVP5146_MODE_NTSC_SQP = 9,	/* (M, J) NTSC      525-line   */
	TVP5146_MODE_PAL_SQP = 0xA,	/* (B, D, G, H, I, N) PAL      */
	TVP5146_MODE_PAL_M_SQP = 0xB,	/* (M) PAL          525-line   */
	TVP5146_MODE_PAL_CN_SQP = 0xC,	/* (Combination-N) PAL         */
	TVP5146_MODE_NTSC_443_SQP = 0xD,/* NTSC 4.43        525-line   */
	TVP5146_MODE_SECAM_SQP = 0xE,	/* SECAM                       */
	TVP5146_MODE_PAL_60_SQP = 0xF,	/* PAL 60          525-line    */
} tvp5146_mode;

typedef struct {
	tvp5146_mode mode;
	int amuxmode;
	int enablebt656sync;
} tvp5146_params;

#ifdef __KERNEL__

typedef struct {
	int agc_enable;
	tvp5146_mode video_std;
	int brightness;
	int contrast;
	int saturation;
	int hue;
	int field_rate;		/* 50 or 60 in Hz */
	int lost_lock;
	int csubc_lock;
	int v_lock;
	int h_lock;
} tvp5146_status;

typedef unsigned int tvp5146_cmd;

/* commands for setup the decoder */
#define TVP5146_SET_AMUXMODE        4
#define TVP5146_SET_BRIGHTNESS      5
#define TVP5146_SET_CONTRAST        6
#define TVP5146_SET_HUE             7
#define TVP5146_SET_SATURATION      8
#define TVP5146_SET_AGC             9
#define TVP5146_SET_VIDEOSTD        10
#define TVP5146_CLR_LOSTLOCK        11
#define TVP5146_CONFIG              12
#define TVP5146_RESET               13
#define TVP5146_POWERDOWN           14

#define TVP5146_GET_STATUS          15
#define TVP5146_GET_STD             16

#define TVP5146_I2C_ADDR (0xBA >> 1)

extern int tvp5146_ctrl(tvp5146_cmd cmd, void *arg);

#endif
#endif
