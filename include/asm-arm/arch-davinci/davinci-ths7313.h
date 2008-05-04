/* *
 * Copyright (C) 2008 Neuros Technology International LLC
 *
 * This program is free software you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation version 2 of the License.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not,write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307 USA
 */
/* davinci-ths7313.h file */

#ifndef	DAVINCI_THS7313
#define DAVINCI_THS7313

/* channel registers */
#define CHANNEL1_REG  1          /* channel 1 register */
#define CHANNEL2_REG  2         /* channel 2 register */
#define CHANNEL3_REG  3         /* channel 3 register */

/* STC Low Pass Filter Selection */
/* 500-kHz Filter---Useful for poor video sync signals */
#define LOW_PASS_FILTER1   0x00
/* 2.5-MHz Filter---Useful for reasonable sync signals */
#define LOW_PASS_FILTER2   0x40
/* 5-MHz Filter---Useful for good sync signals */
#define LOW_PASS_FILTER3   0x80
/* 5-MHz Filter---Useful for good sync signals */
#define LOW_PASS_FILTER4   0xC0

/* Input MUX Selection */
#define INPUT_A_SELECT   0x00  /* input A select */
#define INPUT_B_SELECT   0x20  /* input B select */

/* Input Bias Mode Selection and Disable Control */
#define DISABLE_CHANNEL  0x00 /* Disbale Channle-Conserves Power */
#define MUTE_CHANNEL       0x01  /* Mute Function-No Output */
#define DC_BIAS                      0x02  /* DC Bias Select */
#define DC_BIAS_135MV       0x03  /* DC Bias + 135 mV Offset Select */
#define AC_BIAS                      0x04  /* AC Bias Select */
#define SYNC_CLAMP_LOW_BIAS   0x05  /* Sync Tip Clamp with Low Bias */
#define SYNC_CLAMP_MID_BIAS    0x06   /* Sync Tip Clamp with Mid Bias */
#define SYNC_CLAMP_HIGH_BIAS  0x07   /* Sync Tip Clamp with High Bias */

#define THS7313_I2C_ADDR  0x2C

/**
 * Selects the input biasing of the
 * THS7313 and the power-savings function.
 * When Sync-Tip Clamp is selected, the DC input sink bias
 * current is also selectable.
 * @param channel
 *     There are three channels to configure:
 *     Channel1, Channel2, Channel3.
 * @param mode
 * 		DISABLE_CHANNEL:  Disbale Channle-Conserves Power
 *		MUTE_CHANNEL:  Mute Function-No Output
 *		DC_BIAS:  DC Bias Select
 *		DC_BIAS_135MV:  DC Bias + 135 mV Offset Select
 *		AC_BIAS:  AC Bias Select
 *		SYNC_CLAMP_LOW_BIAS:  Sync Tip Clamp with Low Bias
 *		SYNC_CLAMP_MID_BIAS:  Sync Tip Clamp with Mid Bias
 *		SYNC_CLAMP_HIGH_BIAS:  Sync Tip Clamp with High Bias
 * @return int
 *     0: set successfully, -1: an error occurs.
 */
int ths7313_set_input_mode(int channel, int mode);


/**
 * Controls the input MUX of the THS7313
 * @param channel
 *      There are three channels to configure:
 *      Channel1, Channel2, Channel3.
 * @param select
 *      INPUT_A_SELECT: Input A Select
 *      INPUT_B_SELECT: Input B Select
 * @return int
 * 		0: set successfully, -1: an error occurs.
 */
int ths7313_set_input_mux(int channel, int select);


/**
 * Controls the AC-Sync Tip Clamp Low Pass Filter function. If
 * AC-STC mode is not used, this function is ignored.
 * @param channel
 *  	There are three channels to configure:
 *      Channel1, Channel2, Channel3.
 * @param filter
 *  	LOW_PASS_FILTER1: 500-kHz Filter---Useful for poor video
 *      sync signals.
 *      LOW_PASS_FILTER2: 2.5-MHz Filter---Useful for reasonable
 *      sync signals.
 *      LOW_PASS_FILTER3: 5-MHz Filter---Useful for good sync
 *      signals.
 *      LOW_PASS_FILTER4: 5-MHz Filter---Useful for good sync
 *      signals.
 * @return int
 * 		0: set successfully, -1: an error occurs.
 */
int ths7313_set_pass_filter(int channel, int filter);

#endif /* End of DAVINCI_THS7313 */
