/*
 * linux/sound/oss/davinci-aic33.h
 *
 * Glue driver for AIC33 for Davinci processors
 *
 * Copyright (C) 2006 Texas Instruments, Inc.
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
 *  -------
 *  2005-10-18 Rishi Bhattacharya - Support for AIC33 codec and Davinci DM644x Processor
 */

#ifndef __ASM_ARCH_AIC33_H
#define __ASM_ARCH_AIC33_H

/* Codec TLV320AIC33 */
#define REGISTER_ADDR0  0x00
#define REGISTER_ADDR1          0x01
#define REGISTER_ADDR2          0x02
#define REGISTER_ADDR3          0x03
#define REGISTER_ADDR4          0x04
#define REGISTER_ADDR5          0x05
#define REGISTER_ADDR6          0x06
#define REGISTER_ADDR7          0x07
#define REGISTER_ADDR8          0x08
#define REGISTER_ADDR9          0x09
#define REGISTER_ADDR10         0x0A
#define REGISTER_ADDR11         0x0B
#define REGISTER_ADDR12         0x0C
#define REGISTER_ADDR15         0x0F
#define REGISTER_ADDR16         0x10
#define REGISTER_ADDR17         0x11
#define REGISTER_ADDR18         0x12
#define REGISTER_ADDR19         0x13
#define REGISTER_ADDR20         0x14
#define REGISTER_ADDR21         0x15
#define REGISTER_ADDR22         0x16
#define REGISTER_ADDR23         0x17
#define REGISTER_ADDR24         0x18
#define REGISTER_ADDR25         0x19
#define REGISTER_ADDR26         0x1A
#define REGISTER_ADDR27         0x1B
#define REGISTER_ADDR28         0x1C
#define REGISTER_ADDR29         0x1D
#define REGISTER_ADDR30         0x1E
#define REGISTER_ADDR31         0x1F
#define REGISTER_ADDR32         0x20
#define REGISTER_ADDR33         0x21
#define REGISTER_ADDR37         0x25
#define REGISTER_ADDR38         0x26
#define REGISTER_ADDR40         0x28
#define REGISTER_ADDR41         0x29
#define REGISTER_ADDR43         0x2B
#define REGISTER_ADDR44         0x2C
#define REGISTER_ADDR45         0x2D
#define REGISTER_ADDR46         0x2E
#define REGISTER_ADDR47         0x2F
#define REGISTER_ADDR51         0x33
#define REGISTER_ADDR58         0x3A
#define REGISTER_ADDR64         0x40
#define REGISTER_ADDR65         0x41
#define REGISTER_ADDR73         0x49
#define REGISTER_ADDR74         0x4A
#define REGISTER_ADDR75         0x4B
#define REGISTER_ADDR76         0x4C
#define REGISTER_ADDR77         0x4D
#define REGISTER_ADDR78         0x4E
#define REGISTER_ADDR79         0x4F
#define REGISTER_ADDR80         0x50
#define REGISTER_ADDR81         0x51
#define REGISTER_ADDR82         0x52
#define REGISTER_ADDR83         0x53
#define REGISTER_ADDR84         0x54
#define REGISTER_ADDR85         0x55
#define REGISTER_ADDR86         0x56
#define REGISTER_ADDR87         0x57
#define REGISTER_ADDR88         0x58
#define REGISTER_ADDR89         0x59
#define REGISTER_ADDR90         0x5A
#define REGISTER_ADDR91         0x5B
#define REGISTER_ADDR92         0x5C
#define REGISTER_ADDR93         0x5D
#define REGISTER_ADDR94         0x5E

// Page Select register 0
#define PAGE_SELECT0            0
#define PAGE_SELECT1            1

// Software reset register 1
#define SOFT_RESET              0x80

// Codec sample rate select register 2
#define ADC_FS_MAX              0xA0
#define ADC_FS_MIN              0x00

#define DAC_FS_MAX              0x0A
#define DAC_FS_MIN              0x00

// PLL Programming registerA 3
#define PLL_ENABLE              0x80

// Codec Datapath setup register 7
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

//Audio serial data interface control registerA 8
#define BIT_CLK_MASTER          0x80
#define WORD_CLK_MASTER         0x40
#define DOUT_TRI_STATE          0x20
#define CLK_TRANS_MASTER        0x10
#define ENABLE_3D               0x04
#define DM_ENABLE_128           0x01
#define DM_ENABLE_64            0x02
#define DM_ENABLE_32            0x03

//Audio serial data interface control registerB 9
#define DSP_MODE                0x40
#define RJ_MODE                 0x80
#define LJ_MODE                 0xC0
#define WORD_LENGTH20           0x10
#define WORD_LENGTH24           0x20
#define WORD_LENGTH32           0x30
#define BITCLOCK_256CLK_FRAME   0x08

//Left/Right ADC PGA gain control register 15 & 16
#define ADC_PGA_MUTE            0x80
#define ADC_PGA_GAIN_MAX        0x78
#define ADC_PGA_GAIN_MIN        0x00

// MIC3L/R to left/right ADC control register 17 & 18
#define ADCPGA_GAIN_MAX         0x00
#define MIC3L_ADCPGA_GAIN_MIN   0x80
#define MIC3L_ADCPGA_DISCONNECT 0xF0

#define MIC3R_ADCPGA_GAIN_MIN   0x08
#define MIC3R_ADCPGA_DISCONNECT 0x0F

//LINE1L to left ADC Control Register 19
#define DIFF_MODE               0x80
#define LINE_ADCPGA_GAIN_MIN    0x40
#define LINE_ADCPGA_DISCONNECT  0x78
#define ADC_CHAN_ON             0x04
#define ADCPGA_SOFT_STEP2FS     0x01
#define ADCPGA_SOFT_STEP_OFF    0x03

//LINE2L to left ADC Control Register 20
#define ADC_WEAK_INPUT_BIAS     0x04

//MICBIAS control register 25
#define MICBIAS_OUTPUT_2_0V     0x40
#define MICBIAS_OUTPUT_2_5V     0x80
#define MICBIAS_OUTPUT_AVDD     0xC0

//LEFT/RIGHT AGC Control registerA 26 & 29
#define AGC_ENABLE              0x80
#define AGC_TARGET_GAIN_MAX     0x00
#define AGC_TARGET_GAIN_MIN     0x70
#define AGC_ATTACK_TIME_11      0x04
#define AGC_ATTACK_TIME_16      0x08
#define AGC_ATTACK_TIME_20      0x0C
#define AGC_DECAY_TIME_200      0x01
#define AGC_DECAY_TIME_400      0x02
#define AGC_DECAY_TIME_500      0x03

//LEFT AGC Control registerB 27 & 30
#define AGC_GAIN_ALLOWED_MAX    0xEE
#define AGC_GAIN_ALLOWED_MIN    0x00

//DAC Power and output driver control register 37
#define LEFT_DAC_POWER_ON       0x80
#define RIGHT_DAC_POWER_ON      0x40

//High Power Output Stage Control Register 40
#define LINE2L_BYPASS_DISABLE_DEFAULT    0x00
#define LINE2LP_BYPASS_SINGLE            0x10
#define LINE2LM_BYPASS_SINGLE            0x20
#define LINE2LPM_BYPASS_DIFFERENTIAL     0x30

#define LINE2R_BYPASS_DISABLE_DEFAULT    0x00
#define LINE2RP_BYPASS_SINGLE            0x04
#define LINE2RM_BYPASS_SINGLE            0x08
#define LINE2RPM_BYPASS_DIFFERENTIAL     0x0C

//DAC Output Switching Control Register 41
#define LEFT_DAC_DEFAULT_L1     0x00
#define LEFT_DAC_L2             0x80
#define LEFT_DAC_L3             0x40
#define RIGHT_DAC_DEFAULT_R1    0x00
#define RIGHT_DAC_R2            0x08
#define RIGHT_DAC_R3            0x04

//LEFT/RIGHT DAC Digital volume control register 43 & 44
#define DAC_CHAN_MUTE            0x80
#define DAC_DIG_VOL_GAIN_MAX     0x00	// 0.0db
#define DAC_DIG_VOL_GAIN_MIN     0x7F	// -63.5db

//LINE2L to HPLOUT Volume Control Register 45
#define LINE2L_HPLOUT_ROUTED              0x80

//PGA_L to HPLOUT Volume Control Register 46
#define PGAL_HPLOUT_ROUTED                0x80

//any to LOP/M Volume control
#define LOPM_ON                 0x80
#define LOPM_VOL_GAIN_MAX       0x00	//0 db
#define LOPM_VOL_GAIN_MIN       0x76	//-78.3 db is MUTE

//MONO_LOP/M output level volume control register 79
#define LOPM_POWER_ON            0x01
#define LOPM_MUTE_OFF            0x08
#define LOPM_OUTPUT_LEVEL_MIN    0x00
#define LOPM_OUTPUT_LEVEL_MAX    0x90

//Module Power Status Register 94
#define HPROUT_DRIVER_POWER_ON           0x02

#define LIV_MAX                         0x0077
#define LIV_MIN                         0x0000

#define LHV_MAX                         0x0077
#define LHV_MIN                         0x0000

#define LIG_MAX							0x0077
#define LIG_MIN							0x0000

#define LOG_MAX							0x007f
#define LOG_MIN							0x0000

#endif				/* __ASM_ARCH_AIC33_H */
