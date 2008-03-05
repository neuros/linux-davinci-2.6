/* *
 * Copyright (C) 2006 Texas Instruments Inc
 *
 * This program is free software you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation either version 2 of the License, or
 * (at your option) any later version.
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not,write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307 USA
 */
/* davinci_resizer.h file */

#ifndef	DAVINVI_RESIZER_H
#define	DAVINVI_RESIZER_H

#ifdef __KERNEL__
/* include Linux files */
#include <asm/arch/hardware.h>
#include <asm/io.h>
#include <linux/config.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/kernel.h>	/*     printk() */
#include <linux/slab.h>		/*     kmalloc() */
#include <linux/fs.h>		/*     everything... */
#include <linux/errno.h>	/*     error codes     */
#include <linux/types.h>	/*     size_t */
#include <linux/mm.h>
#include <linux/kdev_t.h>
#include <asm/page.h>
#include <linux/cdev.h>
#include <linux/string.h>
#include <linux/tty.h>
#include <linux/delay.h>
#include <linux/fb.h>
#include <linux/dma-mapping.h>
#include <linux/interrupt.h>
#include <asm/irq.h>
#include <asm/uaccess.h>
#include <linux/ioport.h>
#include <linux/version.h>
#include <asm/irq.h>
#include <asm/io.h>
#include <asm/hardware/clock.h>
#include <linux/errno.h>
#include <linux/sched.h>
#include <asm/arch/hardware.h>
#include <linux/err.h>
#include <linux/proc_fs.h>
#include <linux/sysctl.h>
#ifdef CONFIG_PREEMPT_RT
#include <linux/completion.h>
#endif
#endif				/* end of #ifdef __KERNEL__ */
/* Definitions */

/* ioctls definition */
#pragma	pack(1)
#define		RSZ_IOC_BASE			       'R'
#define		RSZ_IOC_MAXNR				11

/*Ioctl options which are to be passed while calling the ioctl*/
#define	RSZ_REQBUF		_IOWR(RSZ_IOC_BASE,1, rsz_reqbufs_t*)
#define	RSZ_QUERYBUF		_IOWR(RSZ_IOC_BASE,2,rsz_buffer_t  *)
#define	RSZ_S_PARAM		_IOWR(RSZ_IOC_BASE,3,rsz_params_t *)
#define	RSZ_G_PARAM		_IOWR(RSZ_IOC_BASE,4,rsz_params_t *)
#define	RSZ_RESIZE		_IOWR(RSZ_IOC_BASE,5,rsz_resize_t *)
#define	RSZ_G_STATUS		_IOWR(RSZ_IOC_BASE,6,rsz_status_t *)
#define	RSZ_S_PRIORITY		_IOWR(RSZ_IOC_BASE,7,rsz_priority_t*)
#define	RSZ_G_PRIORITY		_IOWR(RSZ_IOC_BASE,9,rsz_priority_t*)
#define	RSZ_GET_CROPSIZE	_IOWR(RSZ_IOC_BASE,10,rsz_cropsize_t *)
#define	RSZ_S_EXP		_IOWR(RSZ_IOC_BASE,11,int*)
#pragma	pack()
/* End of ioctls */

#define	RSZ_BUF_IN						0
#define	RSZ_BUF_OUT						1
#define	RSZ_YENH_DISABLE					0
#define	RSZ_YENH_3TAP_HPF					1
#define	RSZ_YENH_5TAP_HPF					2

#ifdef __KERNEL__

/* Defines and Constants*/
#define	MAX_BUFFER						3
#define	MAX_CHANNELS						16
#define	MAX_PRIORITY						5
#define	MIN_PRIORITY						0
#define	DEFAULT_PRIORITY					3
#define	MAX_IMAGE_WIDTH						1280
#define	MAX_IMAGE_WIDTH_HIGH					640
#define	MAX_IMAGE_HEIGHT					960
#define	MAX_INPUT_BUFFERS					8
#define	MAX_OUTPUT_BUFFERS					8
#define	DRIVER_NAME						"Resizer"
#define	FREE_BUFFER						0
#define	ALIGNMENT						16
#define	CHANNEL_BUSY						1
#define	CHANNEL_FREE						0
#define	PIXEL_EVEN						2
#define	RATIO_MULTIPLIER					256

/*Bit position	Macro*/
/* macro for bit set and clear */
#define	BITSET(variable,bit)		((variable)| (1<<bit))
#define	BITRESET(variable,bit)		((variable)& (~(0x00000001<<(bit))))

/* RSZ_CNT */
#define	CSL_RESZ_RSZ_CNT_CBILIN_MASK			(0x20000000u)
#define	CSL_RESZ_RSZ_CNT_CBILIN_SHIFT			(0x0000001Du)

/*RSZ_OUT_VSIZE_SHIFT*/

#define	RSZ_VSTPH_MASK					(0xfC7fffffu)
#define	RSZ_HSTPH_MASK					(0xff8fffffu)

#define	RSZ_CNT_VRSZ_MASK				(0xfff002ffu)
#define	RSZ_CNT_HRSZ_MASK				(0xfffffc00u)
/* OUT_SIZE	*/
#define	RSZ_OUT_SIZE_VERT_MASK				(0xf800ffffu)
#define	RSZ_OUT_SIZE_HORZ_MASK				(0xfffff800u)
/* IN_START	*/

#define	RSZ_IN_START_VERT_ST_MASK			(0xE000FFFFu)
#define	RSZ_IN_START_HORZ_ST_MASK			(0xFFFFE000u)
/* IN_SIZE */
#define	RSZ_IN_SIZE_VERT_MASK				(0xe000ffffu)
#define	RSZ_IN_SIZE_HORZ_MASK				(0xffffe000u)
/* SDR_INOFF */
#define	RSZ_SDR_INOFF_OFFSET_MASK			(0xffff0000u)
#define	RSZ_SDR_OUTOFF_OFFSET_MASK			(0xffff0000u)
 /**/
#define	RSZ_UWORD_MASK					(0x03FF0000u)
#define	RSZ_LWORD_MASK					(0x000003FFu)
/* YENH	*/
#define	RSZ_YEHN_CORE_MASK				(0xffffff00u)
#define	RSZ_YEHN_SLOP_MASK				(0xfffff0ffu)
#define	RSZ_YEHN_GAIN_MASK				(0xffff0fffu)
#define	RSZ_YEHN_ALGO_MASK				(0xfffcffffu)
/* Filter coeefceints */
#define	RSZ_FILTER_COEFF0_MASK				(0xfffffc00u)
#define	RSZ_FILTER_COEFF1_MASK				(0xfc00ffffu)
#define	RSZ_CNT_CBILIN_MASK				(0x20000000u)
#define	RSZ_CNT_INPTYP_MASK				(0x08000000u)
#define	RSZ_CNT_PIXFMT_MASK				(0x04000000u)
#define	RSZ_HSTP_SHIFT					20
#define	RSZ_HRSZ_MASK					(0xfffffc00)
#define	RSZ_VRSZ_MASK					(0xfff003ff)
#define	RSZ_VRSZ_SHIFT					10
/*///////Shifts*/
#define	RSZ_OUT_VSIZE_SHIFT				16
#define	SET_BIT_CBLIN					29
#define	SET_BIT_INPUTRAM				28
#define	INPUT_RAM					1
#define	SET_BIT_INPTYP					27
#define	SET_BIT_YCPOS					26
#define	RSZ_VSTPH_SHIFT					23
#define	RSZ_FILTER_COEFF_SHIFT				16
#define	RSZ_YENH_TYPE_SHIFT				16
#define	RSZ_YENH_GAIN_SHIFT				12
#define	RSZ_YENH_SLOP_SHIFT				8
#define	UP_RSZ_RATIO					64
#define	DOWN_RSZ_RATIO					512
#define	UP_RSZ_RATIO1					513
#define	DOWN_RSZ_RATIO1					1024
#define	SET_ENABLE_BIT					0
#define	RSZ_IN_SIZE_VERT_SHIFT				16
#define	MAX_HORZ_PIXEL_8BIT				31
#define	MAX_HORZ_PIXEL_16BIT			15
#define	BYTES_PER_PIXEL					2
#define	 NUM_PHASES					8
#define	 NUM_TAPS					4
#define	 NUM_D2PH					4	/* for downsampling
								   2+x ~ 4x, numberof phases */
#define	 NUM_D2TAPS					7	/* for downsampling
								   2+x ~ 4x,number of taps */
#define	 NUM_COEFS (NUM_PHASES * NUM_TAPS)
#define	ALIGN32						32
#define	ADDRESS_FOUND					1
#define	NEXT						1
/*#define DEBUG							0*/
#define	RESIZER_IOBASE_VADDR			IO_ADDRESS(0x01C70C00)
#define	MAX_COEF_COUNTER				16
#define	ZERO						0
#define	FIRSTENTRY					0
#define	SECONDENTRY					1
#define	EMPTY						0
#define	SUCESS						0
#endif				/* end of #ifdef __KERNEL__ */
#define	RSZ_INTYPE_YCBCR422_16BIT		0
#define	RSZ_INTYPE_PLANAR_8BIT			1
#define	RSZ_PIX_FMT_PLANAR			2	/* 8-bit planar input */
#define	RSZ_PIX_FMT_UYVY			0	/*    cb:y:cr:y */
#define	RSZ_PIX_FMT_YUYV			1	/*    y:cb:y:cr */
#ifdef __KERNEL__
#define	isbusy()				((regr(PCR)	& 0x02)>>1)
/*///////Enumerations*/
    enum config_done {
	STATE_CONFIGURED,	/* Resizer driver configured by application */
	STATE_NOT_CONFIGURED	/* Resizer driver not configured by
				   application */
};

#endif				/* end of #ifdef __KERNEL__ */

/*Structure Definitions*/
/* To allocate the memory*/
typedef struct rsz_reqbufs {
	int buf_type;		/* type of frame buffer */
	int size;		/* size of the frame bufferto be allocated */
	int count;		/* number of frame buffer to be allocated */
} rsz_reqbufs_t;

/* assed for quering the buffer to get physical address*/
typedef struct rsz_buffer {
	int index;		/* buffer index number, 0 -> N-1 */
	int buf_type;		/* buffer type, input or output */
	int offset;		/* physical     address of the buffer,
				   used in the mmap() system call */
	int size;
} rsz_buffer_t;

/* used	to luma	enhancement options*/

typedef struct rsz_yenh {
	int type;		/* represents luma enable or disable */
	unsigned char gain;	/*represents gain */
	unsigned char slop;	/*represents slop */
	unsigned char core;	/* Represents core value */
} rsz_yenh_t;

/* Conatins	all	the	parameters for resizing	. This structure
	is used	to configure resiser parameters*/
typedef struct rsz_params {
	int in_hsize;		/* input frame horizontal size */
	int in_vsize;		/* input frame vertical size */
	int in_pitch;		/* offset between two rows of input frame */
	int inptyp;		/* for determining 16 bit or 8 bit data */
	int vert_starting_pixel;	/* for specifying vertical
					   starting pixel in input */
	int horz_starting_pixel;	/* for specyfing horizontal
					   starting pixel in input */
	int cbilin;		/* # defined, filter with luma or bi-linear
				   interpolation */
	int pix_fmt;		/* # defined, UYVY or YUYV */
	int out_hsize;		/* output frame horizontal size */
	int out_vsize;		/* output frame vertical size */
	int out_pitch;		/* offset between two rows of output frame */
	int hstph;		/* for specifying horizontal starting phase */
	int vstph;		/* for specifying vertical starting phase */
	short hfilt_coeffs[32];	/* horizontal filter coefficients */
	short vfilt_coeffs[32];	/* vertical filter coefficients */
	rsz_yenh_t yenh_params;
} rsz_params_t;

/* resize structure passed during the resize IOCTL*/
typedef struct rsz_resize {
	rsz_buffer_t in_buf;
	rsz_buffer_t out_buf;
} rsz_resize_t;

/* Contains the status of hardware and channel*/
typedef struct rsz_status {
	int chan_busy;		/* 1: channel is busy, 0: channel is not busy */
	int hw_busy;		/*1: hardware  is busy, 0:
				   hardware is not     busy */
	int src;		/* # defined, can be either
				   SD-RAM or CCDC/PREVIEWER */
} rsz_status_t;

/* structure to	set the priroity of the the channel*/
typedef struct rsz_priority {
	int priority;		/* 0=>5, with 5 the highest priority */
} rsz_priority_t;

/* Passed by application for getting crop size*/
typedef struct rsz_cropsize {
	unsigned int hcrop;	/*number of pixels per line c
				   ropped in output image */

	unsigned int vcrop;	/*number of lines cropped
				   in output image */
} rsz_cropsize_t;

#ifdef __KERNEL__

/*Register mapped structure which contains the every register
information*/
typedef struct resizer_config {
	int rsz_pcr;		/*pcr register mapping variable */
	int rsz_in_start;	/* in_start register mapping variable */
	int rsz_in_size;	/* in_size register mapping variable */
	int rsz_out_size;	/* out_size register mapping variable */
	int rsz_cnt;		/* rsz_cnt register mapping     variable */
	int rsz_sdr_inadd;	/* sdr_inadd register mapping variable */
	int rsz_sdr_inoff;	/* sdr_inoff register mapping variable */
	int rsz_sdr_outadd;	/* sdr_outadd register mapping variable */
	int rsz_sdr_outoff;	/* sdr_outbuff register mapping  variable */
	int rsz_coeff_horz[16];	/* horizontal coefficients mapping array */
	int rsz_coeff_vert[16];	/* vertical  coefficients mapping  array */
	int rsz_yehn;		/* yehn(luma)register  mapping  variable */
} resizer_config_t;

/* Channel specific	structure contains information regarding
the	every channel */
typedef struct channel_config {
	resizer_config_t register_config;	/* instance of register set
						   mapping  structure */

	void *input_buffer[MAX_INPUT_BUFFERS];	/* for storing input buffers
						   pointers */

	void *output_buffer[MAX_OUTPUT_BUFFERS];	/* for storing output
							   buffers pointers */

	int in_bufsize, out_bufsize;	/* Contains input and output buffer size */

	int status;		/* specifies whether the channel */
	/* is busy or not */

	int priority;		/* stores priority of the application */
#ifdef CONFIG_PREEMPT_RT
	struct completion channel_sem;
#else
	struct semaphore channel_sem;
#endif
	struct semaphore chanprotection_sem;
	enum config_done config_state;
} channel_config_t;

/*Global structure which contains information about	number of chanels
and	protection variables */
typedef struct device_params {
	int module_usage_count;	/* For counting no of channels
				   created */
	struct completion sem_isr;	/*Semaphore for interrupt */
	struct semaphore array_sem;	/* Semaphore for array */
	struct semaphore device_mutex;	/* mutex protecting device_params */
	/* structure object */

	channel_config_t *channel_configuration[MAX_CHANNELS];
	/* Pointer to channel configuration */

	int array_count;	/* for counting number of elements
				   in arrray */
} device_params_t;

/*Functions Definition*/

int malloc_buff(rsz_reqbufs_t *, channel_config_t *);
int get_buf_address(rsz_buffer_t *, channel_config_t *);
int rsz_start(rsz_resize_t *, channel_config_t *);
int add_to_array(channel_config_t * rsz_configuration_channel);
int delete_from_array(channel_config_t * rsz_configuration_channel);
int rsz_set_params(rsz_params_t *, channel_config_t *);
int rsz_get_params(rsz_params_t *, channel_config_t *);
int free_buff(channel_config_t * rsz_configuration_channel);
irqreturn_t rsz_isr(int, void *, struct pt_regs *);
void rsz_calculate_crop(channel_config_t * rsz_conf_chan,
			rsz_cropsize_t * cropsize);
#endif				/* end of #ifdef __KERNEL__ */

#endif				/* End of #ifndef DAVINCI_RESIZER_H */
