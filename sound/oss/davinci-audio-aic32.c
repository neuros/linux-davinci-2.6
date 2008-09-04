/*
 * linux/sound/oss/davinci-audio-aic32.c
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

#include <linux/module.h>
#include <linux/init.h>
#include <linux/types.h>
#include <linux/delay.h>
#include <linux/pm.h>
#include <linux/errno.h>
#include <linux/sound.h>
#include <linux/soundcard.h>
#include <linux/clk.h>
#include <sound/davincisound.h>
#include <asm/gpio.h>

#include <linux/uaccess.h>
#include <linux/io.h>
#include <asm/hardware.h>
#include <asm/mach-types.h>

#include <asm/arch/mcbsp.h>
#include "davinci-aic32.h"

#include "davinci-audio.h"
#include "davinci-audio-dma-intfc.h"

#ifdef CONFIG_PROC_FS
#include <linux/proc_fs.h>
#define PROC_START_FILE "driver/aic32-audio-start"
#define PROC_STOP_FILE  "driver/aic32-audio-stop"
#endif

/*# define DEBUG */

#ifdef DEBUG
#define DPRINTK(ARGS...) \
	do { \
			printk(KERN_INFO, "<%s>: ", __FUNCTION__);\
			printk(KERN_INFO, ARGS); \
	} while (0)

#define FN_IN  printk("<%s>: start\n", __FUNCTION__)
#else
#define DPRINTK(x...)
#define FN_IN
#endif

#define CODEC_NAME               "AIC32"
#define PLATFORM_NAME            "DAVINCI"

/* Define to set the AIC32 as the master w.r.t McBSP */
#define AIC32_MASTER

/* codec clock frequency */
#define MCLK  27

/*
 * AUDIO related MACROS
 */
#define DEFAULT_BITPERSAMPLE          16
#define AUDIO_RATE_DEFAULT            48000
#define DEFAULT_MCBSP_CLOCK           81000000

/* Select the McBSP For Audio */
#define AUDIO_MCBSP                   DAVINCI_MCBSP1

#define REC_MASK                      (SOUND_MASK_LINE | SOUND_MASK_MIC)
#define DEV_MASK                      (REC_MASK | SOUND_MASK_VOLUME)

#define MONO			      1
#define STEREO			      2

#define SET_VOLUME                    1
#define SET_LINE                      2
#define SET_MIC                       3
#define SET_RECSRC		      4
#define SET_IGAIN		      5
#define SET_OGAIN		      6
#define SET_BASS                      7
#define SET_TREBLE                    8
#define SET_MICBIAS		      9

#define DEFAULT_OUTPUT_VOLUME         100
#define DEFAULT_INPUT_VOLUME          100	/* 0 ==> mute line in */
#define DEFAULT_INPUT_IGAIN	      100
#define DEFAULT_INPUT_OGAIN	      100

#define OUTPUT_VOLUME_MIN             LHV_MIN
#define OUTPUT_VOLUME_MAX             LHV_MAX
#define OUTPUT_VOLUME_RANGE           (OUTPUT_VOLUME_MAX - OUTPUT_VOLUME_MIN)

#define INPUT_VOLUME_MIN              LIV_MIN
#define INPUT_VOLUME_MAX              LIV_MAX
#define INPUT_VOLUME_RANGE            (INPUT_VOLUME_MAX - INPUT_VOLUME_MIN)

#define INPUT_GAIN_MIN		      LIG_MIN
#define INPUT_GAIN_MAX		      LIG_MAX
#define INPUT_GAIN_RANGE	      (INPUT_GAIN_MAX - INPUT_GAIN_MIN)

#define OUTPUT_GAIN_MIN		      LOG_MIN
#define OUTPUT_GAIN_MAX		      LOG_MAX
#define OUTPUT_GAIN_RANGE	      (INPUT_GAIN_MAX - INPUT_GAIN_MIN)

#define NUMBER_SAMPLE_RATES_SUPPORTED 11

#define MSB8BIT(x) ((unsigned char)((x) >> 6))
#define LSB6BIT(x)  ((unsigned char)((x) << 2))

#define CLK_OUT0_FUNCTION_BIT   16
#define CLK_OUT1_FUNCTION_BIT   17
#define TIMIN_FUNCTION_BIT 18
#define ASP_FUNCTION_BIT                10

#define SET_BIT(addr, bit)  outl(inl(addr) | (1U << bit), addr)
#define CLEAR_BIT(addr, bit) outl(inl(addr) & ~(1U << bit), addr)

#define PINMUX1     0x01C40004

static audio_stream_t output_stream = {
	.id = "AIC32 out",
	.dma_dev = DAVINCI_DMA_MCBSP1_TX,
	.input_or_output = FMODE_WRITE
};

static audio_stream_t input_stream = {
	.id = "AIC32 in",
	.dma_dev = DAVINCI_DMA_MCBSP1_RX,
	.input_or_output = FMODE_READ
};

static int audio_dev_id, mixer_dev_id;

static struct aic32_local_info {
	u8 volume;
	u16 volume_reg;
	u8 line;
	u8 mic;
	int recsrc;
	int nochan;
	u8 igain;
	u8 ogain;
	u8 micbias;
	u8 bass;
	u8 treble;
	u16 input_volume_reg;
	int mod_cnt;
} aic32_local;

struct sample_rate_reg_info {
	u32 sample_rate;
	u32 Fsref;
	float divider;
	u8 data;
};

/* To Store the default sample rate */
static long audio_samplerate = AUDIO_RATE_DEFAULT;

struct clk *davinci_mcbsp_get_clock(void);

/* DAC USB-mode sampling rates*/
static const struct sample_rate_reg_info
 reg_info[NUMBER_SAMPLE_RATES_SUPPORTED] = {
/*  {sample_rate, Fsref, divider, data}*/
	{96000, 96000, 1, 0x00},
	{88200, 88200, 1, 0x00},
	{48000, 48000, 1, 0x00},
	{44100, 44100, 1, 0x00},
	{32000, 48000, 1.5, 0x11},
	{24000, 96000, 4, 0x66},
	{22050, 44100, 2, 0x22},
	{16000, 48000, 3, 0x44},
	{12000, 48000, 4, 0x66},
	{11025, 44100, 4, 0x66},
	{8000, 48000, 6, 0xAA},
};

static struct davinci_mcbsp_reg_cfg initial_config = {
	.spcr2 = FREE | XINTM(3),
	.spcr1 = RINTM(3),
	.rcr2 = RWDLEN2(DAVINCI_MCBSP_WORD_16) | RDATDLY(1),
	.rcr1 = RFRLEN1(1) | RWDLEN1(DAVINCI_MCBSP_WORD_16),
	.xcr2 = XWDLEN2(DAVINCI_MCBSP_WORD_16) | XDATDLY(1) | XFIG,
	.xcr1 = XFRLEN1(1) | XWDLEN1(DAVINCI_MCBSP_WORD_16),
	.srgr1 = FWID(DEFAULT_BITPERSAMPLE - 1),
	.srgr2 = FSGM | FPER(DEFAULT_BITPERSAMPLE * 2 - 1),
#ifndef AIC32_MASTER
	/* configure McBSP to be the I2S master */
	.pcr0 = FSXM | FSRM | CLKXM | CLKRM | CLKXP | CLKRP,
#else
	/* configure McBSP to be the I2S slave */
	.pcr0 = CLKXP | CLKRP,
#endif				/* AIC32_MASTER */
};

static void davinci_aic32_initialize(void *dummy);
static void davinci_aic32_shutdown(void *dummy);
static int davinci_aic32_ioctl(struct inode *inode, struct file *file,
			       uint cmd, ulong arg);
static int davinci_aic32_probe(void);

#ifdef MODULE
static void davinci_aic32_remove(void);
#endif

static int davinci_aic32_suspend(void);
static int davinci_aic32_resume(void);
static inline void davinci_enable_mclk(void);
static inline void davinci_disable_mclk(void);
static inline void aic32_configure(void);
static int mixer_open(struct inode *inode, struct file *file);
static int mixer_release(struct inode *inode, struct file *file);
static int mixer_ioctl(struct inode *inode, struct file *file, uint cmd,
		       ulong arg);

#ifdef CONFIG_PROC_FS
static int codec_start(char *buf, char **start, off_t offset, int count,
		       int *eof, void *data);
static int codec_stop(char *buf, char **start, off_t offset, int count,
		      int *eof, void *data);
#endif

/* File Op structure for mixer */
static struct file_operations davinci_mixer_fops = {
	.open = mixer_open,
	.release = mixer_release,
	.ioctl = mixer_ioctl,
	.owner = THIS_MODULE
};

/* To store characteristic info regarding the codec for the audio driver */
static audio_state_t aic32_state = {
	.owner = THIS_MODULE,
	.output_stream = &output_stream,
	.input_stream = &input_stream,
/*    .need_tx_for_rx = 1, //Once the Full Duplex works  */
	.need_tx_for_rx = 0,
	.hw_init = davinci_aic32_initialize,
	.hw_shutdown = davinci_aic32_shutdown,
	.client_ioctl = davinci_aic32_ioctl,
	.hw_probe = davinci_aic32_probe,
	.hw_remove = __exit_p(davinci_aic32_remove),
	.hw_suspend = davinci_aic32_suspend,
	.hw_resume = davinci_aic32_resume,
	.sem = __SEMAPHORE_INIT(aic32_state.sem, 1),
};

/* This will be defined in the audio.h */
static struct file_operations *davinci_audio_fops;

int tlv320aic32_write_value(u8 reg, u16 value);

/* TLV320AIC32 write */
static __inline__ void audio_aic32_write(u8 address, u16 data)
{
	if (tlv320aic32_write_value(address, data) < 0)
		printk(KERN_INFO "aic32 write failed for reg = %d\n", address);
}

static void enable_adc(void)
{
	audio_aic32_write(LINE1L_TO_LEFT_ADC_CTRL_REG,
				LINE1L_NOT_CONNECT | LEFT_ADC_POWER_UP);
	audio_aic32_write(LINE1R_TO_RIGHT_ADC_CTRL_REG,
				LINE1R_NOT_CONNECT | RIGHT_ADC_POWER_UP);
}
/* enable ADC and enable line1 input*/
static void enable_line1_input(void)
{
	audio_aic32_write(LINE1L_TO_LEFT_ADC_CTRL_REG,
				LEFT_ADC_POWER_UP);
	audio_aic32_write(LINE1R_TO_RIGHT_ADC_CTRL_REG,
				RIGHT_ADC_POWER_UP);
}
/* enable ADC and disable line1 input */
static void disable_line1_input(void)
{
	audio_aic32_write(LINE1L_TO_LEFT_ADC_CTRL_REG,
				LINE1L_NOT_CONNECT | LEFT_ADC_POWER_UP);
	audio_aic32_write(LINE1R_TO_RIGHT_ADC_CTRL_REG,
				LINE1R_NOT_CONNECT | RIGHT_ADC_POWER_UP);
}

/* enable line2 input*/
static void enable_line2_input(void)
{
	audio_aic32_write(LINE2L_TO_LEFT_ADC_CTRL_REG, 0x0);
	audio_aic32_write(LINE2R_TO_RIGHT_ADC_CTRL_REG, 0x0);
}
static void disable_line2_input(void)
{
	audio_aic32_write(LINE2L_TO_LEFT_ADC_CTRL_REG,
				LINE2_LEFT_ADC_NOT_CONNECT);
	audio_aic32_write(LINE2R_TO_RIGHT_ADC_CTRL_REG,
				LINE2_RIGHT_ADC_NOT_CONNECT);
}

/* enable mic/line3 input*/
static void enable_mic_input(void)
{
	audio_aic32_write(MIC3_TO_LEFT_ADC_CTRL_REG, 0x0);
	audio_aic32_write(MIC3_TO_RIGHT_ADC_CTRL_REG, 0x0);
}
static void disable_mic_input(void)
{
	audio_aic32_write(MIC3_TO_LEFT_ADC_CTRL_REG,
				MIC3L_LEFT_ADC_NOT_CONNECT | MIC3R_LEFT_ADC_NOT_CONNECT);
	audio_aic32_write(MIC3_TO_RIGHT_ADC_CTRL_REG,
				MIC3L_RIGHT_ADC_NOT_CONNECT | MIC3R_RIGHT_ADC_NOT_CONNECT);
}
static int aic32_update(int flag, int val)
{
	u16 volume;
	u16 gain;

	/* Ignore separate left/right channel for now,
	   even the codec does support it. */
	val &= 0xff;

	switch (flag) {
	case SET_VOLUME:

		if (val < 0 || val > 100) {
			DPRINTK("Trying a bad volume value(%d)!\n", val);
			return -EPERM;
		}

		volume =
		    ((val * OUTPUT_VOLUME_RANGE) / 100) + OUTPUT_VOLUME_MIN;

		aic32_local.volume_reg = OUTPUT_VOLUME_MAX - volume;

		audio_aic32_write(DACL1_HPLOUT_VOLUME_REG,
					LOPM_ON | aic32_local.volume_reg);
		audio_aic32_write(DACR1_HPLOUT_VOLUME_REG,
					LOPM_ON | aic32_local.volume_reg);
		audio_aic32_write(DACL1_LEFT_LOP_VOLUME_REG,
					LOPM_ON | aic32_local.volume_reg);
		audio_aic32_write(DACR1_RIGHT_LOP_VOLUME_REG,
					LOPM_ON | aic32_local.volume_reg);

		break;

	case SET_LINE:
	case SET_MIC:

		if (val < 0 || val > 100) {
			DPRINTK("Trying a bad volume value(%d)!\n", val);
			return -EPERM;
		}

		volume = ((val * INPUT_VOLUME_RANGE) / 100) + INPUT_VOLUME_MIN;

		aic32_local.input_volume_reg = volume;

		audio_aic32_write(LEFT_ADC_GAIN_CTRL_REG,
						  aic32_local.input_volume_reg);
		audio_aic32_write(RIGHT_ADC_GAIN_CTRL_REG,
						  aic32_local.input_volume_reg);
		break;

	case SET_RECSRC:
		if (hweight32(val) > 1)
			val &= ~aic32_local.recsrc;

		if (val == SOUND_MASK_MIC) {

			/* enable the mic input*/
			DPRINTK("Enabling mic\n");
			audio_aic32_write(MIC3_TO_LEFT_ADC_CTRL_REG,
						0x0);
			audio_aic32_write(MIC3_TO_RIGHT_ADC_CTRL_REG,
						0x0);

			/* enable ADC's and disable the line input*/
			audio_aic32_write(LINE1L_TO_LEFT_ADC_CTRL_REG,
						LINE1L_NOT_CONNECT |
						LEFT_ADC_POWER_UP);
			audio_aic32_write(LINE1R_TO_RIGHT_ADC_CTRL_REG,
						LINE1R_NOT_CONNECT |
						RIGHT_ADC_POWER_UP);

		} else if (val == SOUND_MASK_LINE) {

			/* enable ADC's, enable line iput */
			DPRINTK(" Enabling line in\n");
			audio_aic32_write(LINE1L_TO_LEFT_ADC_CTRL_REG,
						LEFT_ADC_POWER_UP);
			audio_aic32_write(LINE1R_TO_RIGHT_ADC_CTRL_REG,
						RIGHT_ADC_POWER_UP);

			/* disable the mic input */
			audio_aic32_write(MIC3_TO_LEFT_ADC_CTRL_REG,
						MIC3L_LEFT_ADC_NOT_CONNECT |
						MIC3R_LEFT_ADC_NOT_CONNECT);
			audio_aic32_write(MIC3_TO_RIGHT_ADC_CTRL_REG,
						MIC3L_RIGHT_ADC_NOT_CONNECT |
						MIC3R_RIGHT_ADC_NOT_CONNECT);
		}

		aic32_local.recsrc = val;

		break;

	case SET_IGAIN:

		if (val < 0 || val > 100) {
			DPRINTK("Trying a bad igain value(%d)!\n", val);
			return -EPERM;
		}

		gain = ((val * INPUT_GAIN_RANGE) / 100) + INPUT_GAIN_MIN;

		DPRINTK("gain reg val = 0x%x", gain << 1);

		/* Left AGC control */
		audio_aic32_write(LEFT_AGC_CTRL_A_REG,
						  AGC_ENABLE);
		audio_aic32_write(LEFT_AGC_CTRL_B_REG,
						  gain << 1);
		audio_aic32_write(LEFT_AGC_CTRL_C_REG,
						  0x0);

		/* Right AGC control */
		audio_aic32_write(RIGHT_AGC_CTRL_A_REG,
						  AGC_ENABLE);
		audio_aic32_write(RIGHT_AGC_CTRL_B_REG,
						  gain << 1);
		audio_aic32_write(RIGHT_AGC_CTRL_C_REG,
						  0x0);

		break;

	case SET_OGAIN:

		if (val < 0 || val > 100) {
			DPRINTK("Trying a bad igain value(%d)!\n", val);
			return -EPERM;
		}

		gain = ((val * OUTPUT_GAIN_RANGE) / 100) + OUTPUT_GAIN_MIN;
		gain = OUTPUT_GAIN_MAX - gain;

		/* Left/Right DAC digital volume gain */
		audio_aic32_write(LEFT_DAC_VOLUME_CTRL_REG,
						  gain);
		audio_aic32_write(RIGHT_DAC_VOLUME_CTRL_REG,
						  gain);
		break;

	case SET_MICBIAS:

		if (val < 0 || val > 3) {
			DPRINTK
			    ("Request for non supported mic bias level(%d)!\n",
			     val);
			return -EPERM;
		}

		if (val == 0)
			audio_aic32_write(MCBIAS_CTRL_REG,
							  0x00);

		else if (val == 1)
			audio_aic32_write(MCBIAS_CTRL_REG,
							  MICBIAS_OUTPUT_2_0V);

		else if (val == 2)
			audio_aic32_write(MCBIAS_CTRL_REG,
							  MICBIAS_OUTPUT_2_5V);

		else if (val == 3)
			audio_aic32_write(MCBIAS_CTRL_REG,
							  MICBIAS_OUTPUT_AVDD);

		break;

	case SET_BASS:
		break;

	case SET_TREBLE:
		break;
	}
	return 0;
}

static int mixer_open(struct inode *inode, struct file *file)
{
	/* Any mixer specific initialization */
	return 0;
}

static int mixer_release(struct inode *inode, struct file *file)
{
	/* Any mixer specific Un-initialization */
	return 0;
}

int oss_recsrc_enum (oss_mixer_enuminfo * ei, const char *s)
{
	int n = 1, l;
	int i;

	memset (ei, 0, sizeof (*ei));

	strncpy (ei->strings, s, sizeof (ei->strings) - 1);
	ei->strings[sizeof (ei->strings) - 1] = 0;

	ei->strindex[0] = 0;

	l = strlen (ei->strings);
	for (i = 0; i < l; i++)
	{
		if (ei->strings[i] == ' ')
		{
			ei->strindex[n++] = i + 1;
			ei->strings[i] = 0;
		}
	}

	ei->nvalues = n;

	return 0;
}
static int mixer_get_recnames(ulong arg)
{
	oss_mixer_enuminfo ei;
	char *s = "line1 line2 line3";

	oss_recsrc_enum(&ei, s);
	return copy_to_user((int *)arg, &ei, sizeof(ei));
}
static int mixer_get_recroute(ulong arg)
{
	return copy_to_user((int *)arg, &(aic32_local.recsrc),sizeof(int));
}

static int mixer_set_recroute(ulong arg)
{
	long val;
	int ret;

	ret = get_user(val, (long *)arg);
	if(ret)
		return -ENOTTY;

	switch(val)
	{
		case 0:
			printk("LINE1 audio input selected\n");
			enable_line1_input();
			disable_line2_input();
			disable_mic_input();
			aic32_local.recsrc = val;
			ret = 0;
			break;
		case 1:
			printk("LINE2 audio input selected\n");
			disable_line1_input();
			enable_line2_input();
			disable_mic_input();
			aic32_local.recsrc = val;
			ret = 0;
			break;
		case 2:
			printk("LINE3 audio input selected\n");
			disable_line1_input();
			disable_line2_input();
			enable_mic_input();
			aic32_local.recsrc = val;
			ret = 0;
			break;
		default:
			printk("Invalid Record Source\n");
			ret = -ENOTTY;
			break;
	}

	return ret;
}


static int
mixer_ioctl(struct inode *inode, struct file *file, uint cmd, ulong arg)
{
	int val;
	int ret = 0;
	int nr = _IOC_NR(cmd);

	/*
     * choose which input source to use
     */
	switch(cmd)
	{
		case SNDCTL_DSP_GET_RECSRC_NAMES:
			ret = mixer_get_recnames(arg);
			return ret;
		case SNDCTL_DSP_SET_RECSRC:
			ret = mixer_set_recroute(arg);
			return ret;
		case SNDCTL_DSP_GET_RECSRC:
			ret = mixer_get_recroute(arg);
			return ret;
	}

	/*
	 * We only accept mixer (type 'M') ioctls.
	 */
	if (_IOC_TYPE(cmd) != 'M')
		return -EINVAL;

	DPRINTK(" 0x%08x\n", cmd);

	if (cmd == SOUND_MIXER_INFO) {
		struct mixer_info mi;

		strncpy(mi.id, "AIC32", sizeof(mi.id));
		strncpy(mi.name, "TI AIC32", sizeof(mi.name));
		mi.modify_counter = aic32_local.mod_cnt;

		return copy_to_user((void *)arg, &mi, sizeof(mi));
	}

	if (_IOC_DIR(cmd) & _IOC_WRITE) {
		ret = get_user(val, (int *)arg);
		if (ret)
			goto out;

		switch (nr) {
		case SOUND_MIXER_VOLUME:
			aic32_local.mod_cnt++;
			ret = aic32_update(SET_VOLUME, val);
			if (!ret)
				aic32_local.volume = val;
			break;

		case SOUND_MIXER_LINE:
			aic32_local.mod_cnt++;
			ret = aic32_update(SET_LINE, val);
			if (!ret)
				aic32_local.line = val;
			break;

		case SOUND_MIXER_MIC:
			aic32_local.mod_cnt++;
			ret = aic32_update(SET_MIC, val);
			if (!ret)
				aic32_local.mic = val;
			break;

		case SOUND_MIXER_RECSRC:
			if ((val & SOUND_MASK_LINE) ||
			    (val & SOUND_MASK_MIC)) {
				if (aic32_local.recsrc != val) {
					aic32_local.mod_cnt++;
					aic32_update(SET_RECSRC, val);
				}
			} else
				ret = -EINVAL;

			break;

		case SOUND_MIXER_BASS:
			aic32_local.mod_cnt++;
			ret = aic32_update(SET_BASS, val);
			if (!ret)
				aic32_local.bass = val;
			break;

		case SOUND_MIXER_TREBLE:
			aic32_local.mod_cnt++;
			ret = aic32_update(SET_TREBLE, val);
			if (!ret)
				aic32_local.treble = val;
			break;

		case SOUND_MIXER_IGAIN:
			aic32_local.mod_cnt++;
			ret = aic32_update(SET_IGAIN, val);
			if (!ret)
				aic32_local.igain = val;
			break;

		case SOUND_MIXER_OGAIN:
			aic32_local.mod_cnt++;
			ret = aic32_update(SET_OGAIN, val);
			if (!ret)
				aic32_local.ogain = val;
			break;

		case SOUND_MIXER_MICBIAS:
			aic32_local.mod_cnt++;
			ret = aic32_update(SET_MICBIAS, val);
			if (!ret)
				aic32_local.micbias = val;
			break;

		default:
			ret = -EINVAL;
		}
	}

	if (ret == 0 && _IOC_DIR(cmd) & _IOC_READ) {
		ret = 0;

		switch (nr) {
		case SOUND_MIXER_VOLUME:
			val = aic32_local.volume;
			break;
		case SOUND_MIXER_LINE:
			val = aic32_local.line;
			break;
		case SOUND_MIXER_MIC:
			val = aic32_local.mic;
			break;
		case SOUND_MIXER_RECSRC:
			val = aic32_local.recsrc;
			break;
		case SOUND_MIXER_RECMASK:
			val = REC_MASK;
			break;
		case SOUND_MIXER_IGAIN:
			val = aic32_local.igain;
			break;
		case SOUND_MIXER_OGAIN:
			val = aic32_local.ogain;
			break;
		case SOUND_MIXER_DEVMASK:
			val = DEV_MASK;
			break;
		case SOUND_MIXER_BASS:
			val = aic32_local.bass;
			break;
		case SOUND_MIXER_TREBLE:
			val = aic32_local.treble;
			break;
		case SOUND_MIXER_CAPS:
			val = 0;
			break;
		case SOUND_MIXER_STEREODEVS:
			val = SOUND_MASK_VOLUME;
			break;
		case SOUND_MIXER_MICBIAS:
			val = aic32_local.micbias;
			break;
		default:
			val = 0;
			ret = -EINVAL;
			break;
		}

		if (ret == 0)
			ret = put_user(val, (int *)arg);
	}
out:
	return ret;
}

int davinci_set_samplerate(long sample_rate)
{
	u8 count = 0;

	/* wait for any frame to complete */
	udelay(125);

	/* Search for the right sample rate */
	while ((reg_info[count].sample_rate != sample_rate) &&
	       (count < NUMBER_SAMPLE_RATES_SUPPORTED))
		count++;

	if (count == NUMBER_SAMPLE_RATES_SUPPORTED) {
		DPRINTK("Invalid Sample Rate %d requested\n", (int)sample_rate);
		return -EPERM;
	}

	/*   CODEC DATAPATH SETUP  */

	/* Fsref to 48kHz, dual rate mode upto 96kHz */
	if (reg_info[count].Fsref == 96000)
		audio_aic32_write(CODEC_DATAPATH_SETUP_REG,
				  FS_REF_DEFAULT_48 | ADC_DUAL_RATE_MODE |
				  DAC_DUAL_RATE_MODE | LDAC_LCHAN | RDAC_RCHAN);

	/* Fsref to 44.1kHz, dual rate mode upto 88.2kHz */
	else if (reg_info[count].Fsref == 88200)
		audio_aic32_write(CODEC_DATAPATH_SETUP_REG,
				  FS_REF_44_1 | ADC_DUAL_RATE_MODE |
				  DAC_DUAL_RATE_MODE | LDAC_LCHAN |
				  RDAC_RCHAN);

	/* Fsref to 48kHz */
	else if (reg_info[count].Fsref == 48000)
		audio_aic32_write(CODEC_DATAPATH_SETUP_REG,
				  FS_REF_DEFAULT_48 | LDAC_LCHAN | RDAC_RCHAN);

	/* Fsref to 44.1kHz */
	else if (reg_info[count].Fsref == 44100)
		audio_aic32_write(CODEC_DATAPATH_SETUP_REG,
				  FS_REF_44_1 | LDAC_LCHAN | RDAC_RCHAN);


	/* Codec sample rate select */
	audio_aic32_write(CODEC_SAMPLERATE_SELECT_REG,
					  reg_info[count].data);

	/* If PLL is to be used for generation of Fsref
	   Generate the Fsref using the PLL */
#if (MCLK == 33)

	if ((reg_info[count].Fsref == 96000) |
		(reg_info[count].Fsref == 48000)) {
		/* For MCLK = 33.8688 MHz and to get Fsref = 48kHz
		   Fsref = (MCLK * k * R)/(2048 * p);
		   Select P = 2, R= 1, K = 5.8049,
		   which results in J = 5, D = 8049 */

		/*Enable the PLL | Q-value | P-value */
		audio_aic32_write(PLL_A_REG,
					PLL_ENABLE | 0x10 | 0x02);
		audio_aic32_write(PLL_B_REG,
					0x14);	/* J-value */
		audio_aic32_write(PLL_C_REG,
					MSB8BIT(8049));	/* D-value 8-MSB's */
		audio_aic32_write(PLL_D_REG,
					LSB6BIT(8049));	/* D-value 6-LSB's */

	}

	else if ((reg_info[count].Fsref == 88200) |
			 (reg_info[count].Fsref == 44100)) {

		/* MCLK = 33.8688 MHz and to get Fsref = 44.1kHz
		   Fsref = (MCLK * k * R)/(2048 * p);
		   Select P = 2, R =1, K = 5.3333,
		   which results in J = 5, D = 3333 */

		/*Enable the PLL | Q-value | P-value */
		audio_aic32_write(PLL_A_REG,
					PLL_ENABLE | 0x10 | 0x02);
		audio_aic32_write(PLL_B_REG,
					(5 << 2));	/* J-value */
		audio_aic32_write(PLL_C_REG,
					MSB8BIT(3333));	/* D-value 8-MSB's */
		audio_aic32_write(PLL_D_REG,
					LSB6BIT(3333));	/* D-value 6-LSB's */
	}
#elif(MCLK == 22)

	if ((reg_info[count].Fsref == 96000) |
		(reg_info[count].Fsref == 48000)) {
		/* For MCLK = 22.5792 MHz and to get Fsref = 48kHz
		   Fsref = (MCLK * k * R)/(2048 * p);
		   Select P = 2, R= 1, K = 8.7075,
		   which results in J = 8, D = 7075 */

		/*Enable the PLL | Q-value | P-value */
		audio_aic32_write(PLL_A_REG,
					PLL_ENABLE | 0x10 | 0x02);
		audio_aic32_write(PLL_B_REG,
					(8 << 2));	/* J-value */
		audio_aic32_write(PLL_C_REG,
					MSB8BIT(7075));	/* D-value 8-MSB's */
		audio_aic32_write(PLL_D_REG,
					LSB6BIT(7075));	/* D-value 6-LSB's */

		}

	else if ((reg_info[count].Fsref == 88200) |
			 (reg_info[count].Fsref == 44100)) {

		/* MCLK = 22.5792 MHz and to get Fsref = 44.1kHz
		   Fsref = (MCLK * k * R)/(2048 * p);
		   Select P = 2, R =1, K = 8.0000,
		   which results in J = 8, D = 0000 */

		/*Enable the PLL | Q-value | P-value */
		audio_aic32_write(PLL_A_REG,
					PLL_ENABLE | 0x10 | 0x02);
		audio_aic32_write(PLL_B_REG,
					(8 << 2));	/* J-value */
		audio_aic32_write(PLL_C_REG,
					MSB8BIT(0x00));	/* D-value 8-MSB's */
		audio_aic32_write(PLL_D_REG,
					LSB6BIT(0x00));	/* D-value 6-LSB's */
	}
#elif(MCLK == 27)

	if ((reg_info[count].Fsref == 96000) |
		(reg_info[count].Fsref == 48000)) {
		/* For MCLK = 27 MHz and to get Fsref = 48kHz
		   Fsref = (MCLK * k * R)/(2048 * p);
		   Select P = 4, R= 1, K = 14.5635
		   which results in J = 14, D = 5635 */

		/*Enable the PLL | Q-value | P-value */
		audio_aic32_write(PLL_A_REG,
					PLL_ENABLE | 0x04);
		audio_aic32_write(PLL_B_REG,
					(14 << 2));	/* J-value */
		audio_aic32_write(PLL_C_REG,
					MSB8BIT(5635));	/* D-value 8-MSB's */
		audio_aic32_write(PLL_D_REG,
					LSB6BIT(5635));	/* D-value 6-LSB's */

	}

	else if ((reg_info[count].Fsref == 88200) |
			 (reg_info[count].Fsref == 44100)) {

		/* MCLK = 27 MHz and to get Fsref = 44.1kHz
		   Fsref = (MCLK * k * R)/(2048 * p);
		   Select P = 4, R =1, K = 13.3802
		   which results in J = 13, D = 3802 */

		/*Enable the PLL | Q-value | P-value */
		audio_aic32_write(PLL_A_REG,
					PLL_ENABLE | 0x04);
		audio_aic32_write(PLL_B_REG,
					(13 << 2));	/* J-value */
		audio_aic32_write(PLL_C_REG,
					MSB8BIT(3802));	/* D-value 8-MSB's */
		audio_aic32_write(PLL_D_REG,
					LSB6BIT(3802));	/* D-value 6-LSB's */
	}

#else
#error "unknown audio codec frequency"
#endif

	audio_samplerate = sample_rate;

#ifndef AIC32_MASTER
	{
		int clkgdv = 0;
		unsigned long clkval = 0;
		struct clk *mbspclk;

		/*
		   Set Sample Rate at McBSP

		   Formula :
		   Codec System Clock = Input clock to McBSP;
		   clkgdv = ((Codec System Clock /
				(SampleRate * BitsPerSample * 2)) - 1);

		   FWID = BitsPerSample - 1;
		   FPER = (BitsPerSample * 2) - 1;
		 */

		mbspclk = davinci_mcbsp_get_clock();
		if (mbspclk == NULL) {
			DPRINTK(" Failed to get internal clock to MCBSP");
			return -EPERM;
		}

		clkval = clk_get_rate(mbspclk);
		DPRINTK("mcbsp_clk = %ld\n", clkval);

		if (clkval)
			clkgdv =
			    (clkval /
			     (sample_rate * DEFAULT_BITPERSAMPLE * 2)) - 1;
		else {
			DPRINTK(" Failed to get the MCBSP clock\n");
			return -EPERM;
		}

		DPRINTK("clkgdv = %d\n", clkgdv);

		if (clkgdv > 255 || clkgdv < 0) {

			/* For requested sampling rate, the input clock to MCBSP
				cant be devided down to get the in range clock
				devider value for 16 bits sample */
			DPRINTK("Invalid Sample Rate %d requested\n",
				(int)sample_rate);
			return -EPERM;
		}

		initial_config.srgr1 =
		    (FWID(DEFAULT_BITPERSAMPLE - 1) | CLKGDV(clkgdv));

		initial_config.srgr2 =
		    (CLKSM | FSGM | FPER(DEFAULT_BITPERSAMPLE * 2 - 1));

		davinci_mcbsp_stop(AUDIO_MCBSP);
		davinci_mcbsp_config(AUDIO_MCBSP, &initial_config);
	}
#endif				/* AIC32_MASTER */

	return 0;
}

static inline void davinci_enable_mclk(void)
{
	/* Enable CLK_OUT0 function on default GPIO[48] pin */
	SET_BIT(IO_ADDRESS(PINMUX1), CLK_OUT0_FUNCTION_BIT);

	/* Enable ASP function on default GPIO[29:34] pins */
	SET_BIT(IO_ADDRESS(PINMUX1), ASP_FUNCTION_BIT);
}

static inline void davinci_disable_mclk(void)
{
	/* Disbale ASP function on default GPIO[29:34] pins */
	CLEAR_BIT(IO_ADDRESS(PINMUX1), ASP_FUNCTION_BIT);

	/* Disbale CLK_OUT0 function on default GPIO[48] pin */
	CLEAR_BIT(IO_ADDRESS(PINMUX1), CLK_OUT0_FUNCTION_BIT);
}

static void davinci_aic32_shutdown(void *dummy)
{
	/*
	   Turn off codec after it is done.
	   Can't do it immediately, since it may still have
	   buffered data.

	   Wait 20ms (arbitrary value) and then turn it off.
	 */

	set_current_state(TASK_INTERRUPTIBLE);
	schedule_timeout(2);

	davinci_mcbsp_stop(AUDIO_MCBSP);
	davinci_mcbsp_free(AUDIO_MCBSP);

	/* Self clearing aic32 software reset */
	audio_aic32_write(SOFT_RESET_REG,
					  SELF_CLEAR_SOFT_RESET);

	davinci_disable_mclk();
}

static void davinci_set_mono_stereo(int mode)
{
	if (mode == MONO) {

		/* Driver power ON pop control */
		audio_aic32_write(OUTPUT_REDUCTION_REG,
				OUTPUT_POWERON_DELAY | RAMPUP_STEP_TIME);

		/* LEFT_LOP/M, RIGHT_LOP/M output level control */
		audio_aic32_write(LEFTLOP_OUTPUT_LEVEL_CTRL_REG,
				OUTPUT_LEVEL | LEFTLOP_NOT_MUTE |
				LEFTLOP_FULL_POWER_UP);
		audio_aic32_write(RIGHTLOP_OUTPUT_LEVEL_CTRL_REG,
				OUTPUT_LEVEL | RIGHTLOP_NOT_MUTE |
				RIGHTLOP_FULL_POWER_UP);

		/* Left DAC power up, Right DAC power down */
		audio_aic32_write(DAC_POWER_CTRL_REG,
				LEFT_DAC_POWER_ON | HPLCOM_INDEPENDENT);
	} else if (mode == STEREO) {
		/* Driver power ON pop control */
		audio_aic32_write(OUTPUT_REDUCTION_REG,
				OUTPUT_POWERON_DELAY | RAMPUP_STEP_TIME);

		/* HPLOUT/HPROUT output level control */
		audio_aic32_write(HPLOUT_LEVEL_CTRL_REG,
				HPLCOM_OUTPUT_LEVEL | HPLOUT_IS_MUTE |
				HPLOUT_FULL_POWER_UP);
		audio_aic32_write(HPROUT_LEVEL_CTRL_REG,
				HPRCOM_OUTPUT_LEVEL | HPROUT_IS_MUTE |
				HPROUT_FULL_POWER_UP);

		/* LEFT_LOP/M, RIGHT_LOP/M output level control */
		audio_aic32_write(LEFTLOP_LEVEL_CTRL_REG,
				LEFTLOP_OUTPUT_LEVEL | LEFTLOP_IS_MUTE |
				LEFTLOP_FULL_POWER_UP);
		audio_aic32_write(RIGHTLOP_LEVEL_CTRL_REG,
				RIGHTLOP_OUTPUT_LEVEL | RIGHTLOP_IS_MUTE |
				RIGHTLOP_FULL_POWER_UP);

		/* Left/Right DAC power up */
		audio_aic32_write(DAC_POWER_CTRL_REG,
				LEFT_DAC_POWER_ON | RIGHT_DAC_POWER_ON |
				HPLCOM_INDEPENDENT);
	} else
		DPRINTK(" REQUEST FOR INVALID MODE\n");
}

static inline void aic32_configure()
{
	FN_IN;

	/* Page select register */
	audio_aic32_write(PAGE_SELECT_REG, PAGE0);

	davinci_set_mono_stereo(aic32_local.nochan);

#ifdef AIC32_MASTER
	/* Enable bit and word clock as Master mode, 3-d disabled */
	audio_aic32_write(SERIAL_DATA_CTRL_REG,
					  BIT_CLK_MASTER | WORD_CLK_MASTER);
#endif
	//PLLDIV_IN uses MCLK
	audio_aic32_write(CLOCK_GENERATION_CTRL_REG, 0x02);

	//CODEC_CLKIN = PLLDIV_OUT
	audio_aic32_write(CODEC_CLKIN_CTRL_REG, 0x0);


	aic32_update(SET_LINE, aic32_local.line);
	aic32_update(SET_VOLUME, aic32_local.volume);
	aic32_update(SET_RECSRC, aic32_local.recsrc);
	aic32_update(SET_IGAIN, aic32_local.igain);
	aic32_update(SET_OGAIN, aic32_local.ogain);
	aic32_update(SET_MICBIAS, aic32_local.micbias);
}

static void davinci_aic32_initialize(void *dummy)
{
	FN_IN;

	/* initialize with default sample rate */
	audio_samplerate = AUDIO_RATE_DEFAULT;

	if (davinci_mcbsp_request(AUDIO_MCBSP) < 0) {
		DPRINTK("MCBSP request failed\n");
		return;
	}

	/* if configured, then stop mcbsp */
	davinci_mcbsp_stop(AUDIO_MCBSP);

	/* set initial (default) sample rate */
	davinci_set_samplerate(audio_samplerate);

	davinci_mcbsp_config(AUDIO_MCBSP, &initial_config);

	/* enable davinci MCLK */
	davinci_enable_mclk();
}

static int
davinci_aic32_ioctl(struct inode *inode, struct file *file, uint cmd, ulong arg)
{
	long val;
	int ret = 0;

	DPRINTK(" 0x%08x\n", cmd);

	/*
	 * These are platform dependent ioctls which are not handled by the
	 * generic davinci-audio module.
	 */
	switch (cmd) {
	case SNDCTL_DSP_STEREO:
		ret = get_user(val, (int *)arg);
		if (ret)
			return ret;
		/* the Davinci supports AIC32 as stereo, mono on stereo jack */
		ret = (val == 0) ? -EINVAL : 1;
		return put_user(ret, (int *)arg);

	case SNDCTL_DSP_CHANNELS:

		ret = get_user(val, (long *)arg);
		if (ret) {
			DPRINTK("get_user failed\n");
			break;
		}
		if (val == STEREO) {
			DPRINTK("Driver support for AIC32 as stereo\n");
			aic32_local.nochan = STEREO;
			davinci_set_mono_stereo(aic32_local.nochan);
		} else if (val == MONO) {
			DPRINTK("Driver support for AIC32 as mono\n");
			aic32_local.nochan = MONO;
			davinci_set_mono_stereo(aic32_local.nochan);
		} else {
			DPRINTK
			    ("Driver support for AIC32 as stereo/mono mode\n");
			return -EPERM;
		}

	case SOUND_PCM_READ_CHANNELS:
		/* the Davinci supports AIC32 as stereo, mono on stereo jack */
		if (aic32_local.nochan == MONO)
			return put_user(MONO, (long *)arg);
		else
			return put_user(STEREO, (long *)arg);

	case SNDCTL_DSP_SPEED:
		ret = get_user(val, (long *)arg);
		if (ret) {
			DPRINTK("get_user failed\n");
			break;
		}
		ret = davinci_set_samplerate(val);
		if (ret) {
			DPRINTK("davinci_set_samplerate failed\n");
			break;
		}
		/* fall through */

	case SOUND_PCM_READ_RATE:
		return put_user(audio_samplerate, (long *)arg);

	case SNDCTL_DSP_SETFMT:	/* set Format */
		ret = get_user(val, (long *)arg);
		if (ret) {
			DPRINTK("get_user failed\n");
			break;
		}
		if (val != AFMT_S16_LE) {
			DPRINTK
			    ("Driver supports only AFMT_S16_LE audio format\n");
			return -EPERM;
		}

	case SOUND_PCM_READ_BITS:
	case SNDCTL_DSP_GETFMTS:
		/* we can do 16-bit only */
		return put_user(AFMT_S16_LE, (long *)arg);

	default:
		/* Maybe this is meant for the mixer (As per OSS Docs) */
		return mixer_ioctl(inode, file, cmd, arg);
	}

	return ret;
}

static int davinci_aic32_probe(void)
{
	/* Get the fops from audio oss driver */
	davinci_audio_fops = audio_get_fops();
	if (!davinci_audio_fops) {
		DPRINTK
		("Unable to get the file operations forAIC32 OSS driver\n");
		audio_unregister_codec(&aic32_state);
		return -EPERM;
	}

	aic32_local.volume = DEFAULT_OUTPUT_VOLUME;
	aic32_local.line = DEFAULT_INPUT_VOLUME;
	/* either of SOUND_MASK_LINE/SOUND_MASK_MIC */
    aic32_local.recsrc = SOUND_MASK_LINE;
	aic32_local.igain = DEFAULT_INPUT_IGAIN;
	aic32_local.ogain = DEFAULT_INPUT_OGAIN;
	aic32_local.nochan = STEREO;
	aic32_local.micbias = 1;
	aic32_local.mod_cnt = 0;

	/* register devices */
	audio_dev_id = register_sound_dsp(davinci_audio_fops, -1);
	mixer_dev_id = register_sound_mixer(&davinci_mixer_fops, -1);

#ifdef CONFIG_PROC_FS
	create_proc_read_entry(PROC_START_FILE, 0 /* default mode */ ,
			       NULL /* parent dir */ ,
			       codec_start, NULL/* client data */);

	create_proc_read_entry(PROC_STOP_FILE, 0 /* default mode */ ,
			       NULL /* parent dir */ ,
			       codec_stop, NULL/* client data */);
#endif

	aic32_configure();

	/* Announcement Time */
	DPRINTK(PLATFORM_NAME " " CODEC_NAME " audio support initialized\n");
	return 0;
}

#ifdef MODULE
static void __exit davinci_aic32_remove(void)
{
	/* Un-Register the codec with the audio driver */
	unregister_sound_dsp(audio_dev_id);
	unregister_sound_mixer(mixer_dev_id);

#ifdef CONFIG_PROC_FS
	remove_proc_entry(PROC_START_FILE, NULL);
	remove_proc_entry(PROC_STOP_FILE, NULL);
#endif
}
#endif				/* MODULE */

static int davinci_aic32_suspend(void)
{
	/* Empty for the moment */
	return 0;
}

static int davinci_aic32_resume(void)
{
	/* Empty for the moment */
	return 0;
}

static int __init audio_aic32_init(void)
{
	int err = 0;

	/* register the codec with the audio driver */
	err = audio_register_codec(&aic32_state);
	if (err) {
		DPRINTK
		    ("Failed to register AIC32 driver with Audio OSS Driver\n");
	} else {
		DPRINTK("codec driver register success\n");
	}

	/* configure aic32 with default params */
	aic32_configure();

	return err;
}

static void __exit audio_aic32_exit(void)
{
	davinci_aic32_shutdown(NULL);
	(void)audio_unregister_codec(&aic32_state);
	return;
}

#ifdef CONFIG_PROC_FS
static int codec_start(char *buf, char **start, off_t offset, int count,
		       int *eof, void *data)
{
	davinci_aic32_initialize(NULL);

	DPRINTK("AIC32 codec initialization done.\n");
	return 0;
}

static int codec_stop(char *buf, char **start, off_t offset, int count,
		      int *eof, void *data)
{
	davinci_aic32_shutdown(NULL);

	DPRINTK("AIC32 codec shutdown.\n");
	return 0;
}
#endif				/* CONFIG_PROC_FS */

module_init(audio_aic32_init);
module_exit(audio_aic32_exit);

MODULE_AUTHOR("Neuros Technology");
MODULE_DESCRIPTION("Glue audio driver for the TI AIC32 codec.");
MODULE_LICENSE("GPL");
