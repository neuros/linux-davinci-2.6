/*
 * linux/sound/oss/davinci-aic32.h
 *
 * Glue driver for AIC32 for Davinci processors
 *
 * Copyright (C) 2008 Neuros Technology.
 *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * THIS PACKAGE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
 * WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 */
#ifndef __ASM_ARCH_AIC32_H
#define __ASM_ARCH_AIC32_H

/* aic32 control registers */
#define PAGE_SELECT_REG    0
#define SOFT_RESET_REG     1
#define CODEC_SAMPLERATE_SELECT_REG 2

#define PLL_A_REG                3
#define PLL_B_REG                4
#define PLL_C_REG                5
#define PLL_D_REG                6

#define CODEC_DATAPATH_SETUP_REG 7
#define SERIAL_DATA_CTRL_REG  8
#define LEFT_ADC_GAIN_CTRL_REG 15
#define RIGHT_ADC_GAIN_CTRL_REG 16
#define MIC3_TO_LEFT_ADC_CTRL_REG 17
#define MIC3_TO_RIGHT_ADC_CTRL_REG 18
#define LINE1L_TO_LEFT_ADC_CTRL_REG  19
#define LINE1R_TO_RIGHT_ADC_CTRL_REG 22
#define LINE2L_TO_LEFT_ADC_CTRL_REG  20
#define LINE2R_TO_RIGHT_ADC_CTRL_REG 23

#define MCBIAS_CTRL_REG 25

#define LEFT_AGC_CTRL_A_REG 26
#define LEFT_AGC_CTRL_B_REG 27
#define LEFT_AGC_CTRL_C_REG 28
#define RIGHT_AGC_CTRL_A_REG 29
#define RIGHT_AGC_CTRL_B_REG 30
#define RIGHT_AGC_CTRL_C_REG 31

#define DAC_POWER_CTRL_REG 37

#define OUTPUT_REDUCTION_REG 42

#define LEFT_DAC_VOLUME_CTRL_REG 43
#define RIGHT_DAC_VOLUME_CTRL_REG 44

#define DACL1_HPLOUT_VOLUME_REG  47
#define DACR1_HPLOUT_VOLUME_REG  64
#define DACL1_LEFT_LOP_VOLUME_REG 82
#define DACR1_RIGHT_LOP_VOLUME_REG 92

#define LEFTLOP_OUTPUT_LEVEL_CTRL_REG 86
#define RIGHTLOP_OUTPUT_LEVEL_CTRL_REG 93

#define HPLOUT_LEVEL_CTRL_REG 51
#define HPROUT_LEVEL_CTRL_REG 65
#define LEFTLOP_LEVEL_CTRL_REG 86
#define RIGHTLOP_LEVEL_CTRL_REG 93

/* aic32 control registers value */
#define PAGE0   0x0

/* Codec Datapath setup register 7 */
#define FS_REF_44_1             0x80
#define FS_REF_DEFAULT_48       0x00
#define ADC_DUAL_RATE_MODE      0x40
#define DAC_DUAL_RATE_MODE      0x20
#define LDAC_LCHAN              0x08
#define LDAC_RCHAN              0x10
#define LDAC_MONO_MIX           0x18
#define RDAC_RCHAN              0x02
#define RDAC_LCHAN              0x04
#define RDAC_MONO_MIX           0x06

/* soft reset control register 1 */
#define SELF_CLEAR_SOFT_RESET    0x80

/* used by register 17 */
#define MIC3L_LEFT_ADC_NOT_CONNECT 0xF0
#define MIC3R_LEFT_ADC_NOT_CONNECT 0x0F

/* used by register 18 */
#define MIC3L_RIGHT_ADC_NOT_CONNECT 0xF0
#define MIC3R_RIGHT_ADC_NOT_CONNECT 0x0F

/* used by register 19 */
#define LINE1L_NOT_CONNECT    0x78
#define LEFT_ADC_POWER_UP   0x04

/* used by register 22 */
#define LINE1R_NOT_CONNECT    0x78
#define RIGHT_ADC_POWER_UP   0x04

/* used by register 20 */
#define LINE2_LEFT_ADC_NOT_CONNECT 0x78

/* used by register 23 */
#define LINE2_RIGHT_ADC_NOT_CONNECT 0x78

/* MICBIAS control register 25 */
#define MICBIAS_OUTPUT_2_0V     0x40
#define MICBIAS_OUTPUT_2_5V     0x80
#define MICBIAS_OUTPUT_AVDD     0xC0

/* output reduction register 42 */
#define OUTPUT_POWERON_DELAY  0x60
#define RAMPUP_STEP_TIME               0x0C

/* HPLCOM output level register 51 */
#define HPLCOM_OUTPUT_LEVEL  0x90
#define HPLOUT_IS_MUTE   0x08
#define HPLOUT_FULL_POWER_UP 0x01

/* HPRCOM output level register 65 */
#define HPRCOM_OUTPUT_LEVEL  0x90
#define HPROUT_IS_MUTE   0x08
#define HPROUT_FULL_POWER_UP 0x01

/* LEFT_LOP output level register 86 */
#define LEFTLOP_OUTPUT_LEVEL  0x90
#define LEFTLOP_IS_MUTE   0x08
#define LEFTLOP_FULL_POWER_UP 0x01

/* RIGHT_LOP output level register 86 */
#define RIGHTLOP_OUTPUT_LEVEL  0x90
#define RIGHTLOP_IS_MUTE   0x08
#define RIGHTLOP_FULL_POWER_UP 0x01

/* output level control register 86&93 */
#define OUTPUT_LEVEL   0x90
#define LEFTLOP_NOT_MUTE 0x80
#define LEFTLOP_FULL_POWER_UP 0x01
#define RIGHTLOP_NOT_MUTE 0x80
#define RIGHTLOP_FULL_POWER_UP 0x01

/* PLL Programming registerA 3 */
#define PLL_ENABLE              0x80

/* Audio serial data interface control registerA 8 */
#define BIT_CLK_MASTER          0x80
#define WORD_CLK_MASTER         0x40
#define DOUT_TRI_STATE          0x20
#define CLK_TRANS_MASTER        0x10
#define ENABLE_3D               0x04
#define DM_ENABLE_128           0x01
#define DM_ENABLE_64            0x02
#define DM_ENABLE_32            0x03

/*Audio serial data interface control registerB 9 */
#define DSP_MODE                0x40
#define RJ_MODE                 0x80
#define LJ_MODE                 0xC0
#define WORD_LENGTH20           0x10
#define WORD_LENGTH24           0x20
#define WORD_LENGTH32           0x30
#define BITCLOCK_256CLK_FRAME   0x08

/*Left/Right ADC PGA gain control register 15 & 16 */
#define ADC_PGA_MUTE            0x80
#define ADC_PGA_GAIN_MAX        0x78
#define ADC_PGA_GAIN_MIN        0x00

/* MIC3L/R to left/right ADC control register 17 & 18 */
#define ADCPGA_GAIN_MAX         0x00
#define MIC3L_ADCPGA_GAIN_MIN   0x80
#define MIC3L_ADCPGA_DISCONNECT 0xF0

#define MIC3R_ADCPGA_GAIN_MIN   0x08
#define MIC3R_ADCPGA_DISCONNECT 0x0F

/*LINE1L to left ADC Control Register 19 */
#define DIFF_MODE               0x80
#define LINE_ADCPGA_GAIN_MIN    0x40
#define LINE_ADCPGA_DISCONNECT  0x78
#define ADC_CHAN_ON             0x04
#define ADCPGA_SOFT_STEP2FS     0x01
#define ADCPGA_SOFT_STEP_OFF    0x03

/*LINE2L to left ADC Control Register 20 */
#define ADC_WEAK_INPUT_BIAS     0x04

/*LEFT/RIGHT AGC Control registerA 26 & 29 */
#define AGC_ENABLE              0x80
#define AGC_TARGET_GAIN_MAX     0x00
#define AGC_TARGET_GAIN_MIN     0x70
#define AGC_ATTACK_TIME_11      0x04
#define AGC_ATTACK_TIME_16      0x08
#define AGC_ATTACK_TIME_20      0x0C
#define AGC_DECAY_TIME_200      0x01
#define AGC_DECAY_TIME_400      0x02
#define AGC_DECAY_TIME_500      0x03

/*LEFT AGC Control registerB 27 & 30 */
#define AGC_GAIN_ALLOWED_MAX    0xEE
#define AGC_GAIN_ALLOWED_MIN    0x00

/*DAC Power and output driver control register 37 */
#define LEFT_DAC_POWER_ON       0x80
#define RIGHT_DAC_POWER_ON      0x40
#define HPLCOM_INDEPENDENT  0x20


/*High Power Output Stage Control Register 40 */
#define LINE2L_BYPASS_DISABLE_DEFAULT    0x00
#define LINE2LP_BYPASS_SINGLE            0x10
#define LINE2LM_BYPASS_SINGLE            0x20
#define LINE2LPM_BYPASS_DIFFERENTIAL     0x30

#define LINE2R_BYPASS_DISABLE_DEFAULT    0x00
#define LINE2RP_BYPASS_SINGLE            0x04
#define LINE2RM_BYPASS_SINGLE            0x08
#define LINE2RPM_BYPASS_DIFFERENTIAL     0x0C

/*DAC Output Switching Control Register 41 */
#define LEFT_DAC_DEFAULT_L1     0x00
#define LEFT_DAC_L2             0x80
#define LEFT_DAC_L3             0x40
#define RIGHT_DAC_DEFAULT_R1    0x00
#define RIGHT_DAC_R2            0x08
#define RIGHT_DAC_R3            0x04

/*LEFT/RIGHT DAC Digital volume control register 43 & 44 */
#define DAC_CHAN_MUTE            0x80
#define DAC_DIG_VOL_GAIN_MAX     0x00	/* 0.0db */
#define DAC_DIG_VOL_GAIN_MIN     0x7F	/* -63.5db */

/*LINE2L to HPLOUT Volume Control Register 45 */
#define LINE2L_HPLOUT_ROUTED              0x80

/*PGA_L to HPLOUT Volume Control Register 46 */
#define PGAL_HPLOUT_ROUTED                0x80

/*any to LOP/M Volume control */
#define LOPM_ON                 0x80
#define LOPM_VOL_GAIN_MAX       0x00	/*0 db */
#define LOPM_VOL_GAIN_MIN       0x76	/*-78.3 db is MUTE */

/*MONO_LOP/M output level volume control register 79 */
#define LOPM_POWER_ON            0x01
#define LOPM_MUTE_OFF            0x08
#define LOPM_OUTPUT_LEVEL_MIN    0x00
#define LOPM_OUTPUT_LEVEL_MAX    0x90

/*Module Power Status Register 94 */
#define HPROUT_DRIVER_POWER_ON           0x02

#define LIV_MAX                         0x0077
#define LIV_MIN                         0x0000

#define LHV_MAX                         0x0077
#define LHV_MIN                         0x0000

#define LIG_MAX							0x0077
#define LIG_MIN							0x0000

#define LOG_MAX							0x007f
#define LOG_MIN							0x0000

#endif				/* __ASM_ARCH_AIC32_H */

