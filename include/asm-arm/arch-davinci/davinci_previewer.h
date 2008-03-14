/*
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
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307 USA
 */

/* davinci_previewer.h file */

#ifndef DAVINCI_PREVIEWER_H
#define DAVINCI_PREVIEWER_H

#ifdef DEBUG
#undef DEBUG
#endif

/*#define DEBUG*/

#include <linux/ioctl.h>

#ifdef __KERNEL__

/* include linux specific header files */
#include <linux/completion.h>
#include <linux/interrupt.h>
#include <asm/semaphore.h>

#endif				/* End of #ifdef __KERNEL__ */

/* Feature lists */
#define PREV_INPUT_FORMATTER       0x1
#define PREV_INVERSE_ALAW          0x2
#define PREV_HORZ_MEDIAN_FILTER    0x4
#define PREV_NOISE_FILTER          0x8
#define PREV_CFA                   0x10
#define PREV_GAMMA                 0x20
#define PREV_LUMA_ENHANCE          0x40
#define PREV_CHROMA_SUPPRESS       0x80
#define PREV_DARK_FRAME_SUBTRACT   0x100
#define PREV_LENS_SHADING          0x200
#define PREV_DARK_FRAME_CAPTURE    0x400
/* -- */

#define DOWN_SAMPLE_RATE1       1	/* Down sampling rate 1 */
#define DOWN_SAMPLE_RATE2       2	/* Down sampling rate 2 */
#define DOWN_SAMPLE_RATE3       4	/* Down sampling rate 4 */
#define DOWN_SAMPLE_RATE4       8	/* Down sampling rate 8 */

#define LUMA_TABLE_SIZE            128
#define GAMMA_TABLE_SIZE           1024
#define CFA_COEFF_TABLE_SIZE       576
#define NOISE_FILTER_TABLE_SIZE    256

#define WB_GAIN_MAX     4
#define RGB_MAX         3

#define MAX_BUFFER      8

#define MAX_IMAGE_WIDTH   1280
#define MAX_IMAGE_HEIGHT  1920

#define PREV_BUF_IN     0	/* input buffer */
#define PREV_BUF_OUT    1	/* output buffer */

#define PREV_INWIDTH_8BIT   0	/* pixel width of 8 bitS */
#define PREV_INWIDTH_10BIT  1	/* pixel width of 10 bits */

/* list of structures */
/* structure for request buffer */
struct prev_reqbufs {
	int buf_type;		/* type of frame buffer */
	int size;		/* size of the frame buffer to be allocated */
	int count;		/* number of frame buffer to be allocated */
};
/* structure buffer */
struct prev_buffer {
	int index;		/* index number, 0 -> N-1 */
	int buf_type;		/* buffer type, input or output */
	int offset;		/* address of the buffer used in the mmap() 
				   system call */
	int size;		/* size of the buffer */
};
/* structure for size parameters */
struct prev_size_params {
	unsigned int hstart;	/* Starting pixel */
	unsigned int vstart;	/* Starting line */
	unsigned int hsize;	/* width of input image */
	unsigned int vsize;	/* height of input image */
	unsigned char pixsize;	/* pixel size of the image in 
				   terms of bits */
	unsigned short in_pitch;	/* line offset of input image */
	unsigned short out_pitch;	/* line offset of output image */
};
/* structure for white balancing parameters */
struct prev_white_balance {
	unsigned short wb_dgain;	/* white 
					   balance common
					   gain */
	unsigned char wb_gain[WB_GAIN_MAX];	/* individual 
						   color gains */
	unsigned char wb_coefmatrix[WB_GAIN_MAX][WB_GAIN_MAX];	/* 16 position
								   out of 4 
								   values */
};
/*structure for black adjustment for parameters */
struct prev_black_adjst {	/* black adjustments for three colors */
	char redblkadj;		/* black adjustment offset for red color */
	char greenblkadj;	/* black adjustment offset for green color */
	char blueblkadj;	/* black adjustment offset for blue color */
};
/*structure for RGB2RGB blending parameters */
struct prev_rgbblending {
	short blending[RGB_MAX][RGB_MAX];	/* color correlation 3x3 matrix */
	short offset[RGB_MAX];	/* color correlation offsets */
};
/* structure RGB2YCbCr parameters */
struct prev_rgb2ycbcr_coeffs {
	short coeff[RGB_MAX][RGB_MAX];	/* color conversion gains in 
					   3x3 matrix */
	short offset[RGB_MAX];	/* color conversion offsets */
};
/*structure for CFA coefficients */
struct prev_cfa_coeffs {
	char hthreshold, vthreshold;	/* horizontal an vertical 
					   threshold */
	int coeffs[CFA_COEFF_TABLE_SIZE];	/* cfa coefficients */
};
/* structure for Gamma Coefficients */
struct prev_gamma_coeffs {
	unsigned char red[GAMMA_TABLE_SIZE];	/* table of gamma correction 
						   values for red color */
	unsigned char green[GAMMA_TABLE_SIZE];	/* table of gamma correction 
						   values for green color */
	unsigned char blue[GAMMA_TABLE_SIZE];	/* table of gamma correction 
						   values for blue color */
};
/* structure for Nois Filter Coefficients */
struct prev_noiseflt_coeffs {
	unsigned char noise[NOISE_FILTER_TABLE_SIZE];	/* noise filter 
							   table */
	unsigned char strength;	/* to find out 
				   weighted average */
};
/*structure for Chroma Suppression */
struct prev_chroma_spr {
	unsigned char hpfy;	/* whether to use high passed 
				   version of Y or normal Y */
	char threshold;		/* threshold for chroma suppress */
	unsigned char gain;	/* chroma suppression gain */
};
/* enum data type for output pixel order */
enum prev_pixorder {
	PREV_PIXORDER_CBYCRY = 0,	/* LSB Cb0 Y0 Cr0 Y1 MSB */
	PREV_PIXORDER_CRYCBY,	/* LSB Cr0 Y0 Cb0 Y1 MSB */
	PREV_PIXORDER_YCRYCB,	/* LSB Y0 Cb0 Y1 Cr0 MSB */
	PREV_PIXORDER_YCBYCR,	/* LSB Y0 Cr0 Y1 Cb0 MSB */
};
/* -- */
/* structure for all configuration */
struct prev_params {
	unsigned short features;	/* Set of features 
					   enabled */
	struct prev_size_params size_params;	/* size parameters */
	struct prev_white_balance white_balance_params;	/* white balancing 
							   parameters */
	struct prev_black_adjst black_adjst_params;	/* black adjustment 
							   parameters */
	struct prev_rgbblending rgbblending_params;	/* rgb blending 
							   parameters */
	struct prev_rgb2ycbcr_coeffs rgb2ycbcr_params;	/* rgb to ycbcr 
							   parameters */
	unsigned char sample_rate;	/* down sampling 
					   rate for averager */
	short hmf_threshold;	/* horizontal median 
				   filter threshold */
	struct prev_cfa_coeffs cfa_coeffs;	/* CFA coefficients */
	struct prev_gamma_coeffs gamma_coeffs;	/* gamma 
						   coefficients */
	struct prev_noiseflt_coeffs nf_coeffs;	/* noise filter 
						   coefficients */
	unsigned int luma_enhance[LUMA_TABLE_SIZE];	/* luma enhancement 
							   coeffs */
	struct prev_chroma_spr chroma_suppress_params;	/* chroma suppression 
							   coefficients */
	void *dark_frame_addr;	/* dark frame 
				   address */
	unsigned short dark_frame_pitch;	/* dark frame 
						   lineoffset */
	unsigned char lens_shading_sift;	/* number of bits 
						   to be        shifted 
						   for lens shading */
	enum prev_pixorder pix_fmt;	/* output pixel 
					   format */
	int contrast;		/* contrast */
	int brightness;		/* brightness */
};
/* structure for input/output buffer, used while previewing */
struct prev_convert {
	struct prev_buffer in_buff;
	struct prev_buffer out_buff;
};
/* structure to know status of the hardware */
struct prev_status {
	char hw_busy;
};
/* structure to knwo crop size */
struct prev_cropsize {
	int hcrop;
	int vcrop;
};

#ifdef __KERNEL__
/* device structure keeps track of global information */
struct prev_device {
	struct prev_params *params;
	unsigned char opened;	/* state of the device */
	unsigned char in_numbuffers;	/* number of input 
					   buffers */
	unsigned char out_numbuffers;	/* number of output 
					   buffers */
	struct prev_buffer *in_buff[MAX_BUFFER];	/* pointer to input 
							   buffers */
	struct prev_buffer *out_buff[MAX_BUFFER];	/*pointer to output 
							   buffers */
	struct completion wfc;	/* used to wait for frame 
				   precessing to be 
				   completed */
	struct semaphore sem;
};

void calculate_slices(struct prev_params *, int *hslice, int *vslice);
void prev_calculate_crop(struct prev_params *, struct prev_cropsize *crop);
int preview(struct prev_device *, struct prev_convert *arg);
int get_status(struct prev_status *);
int request_buffer(struct prev_device *, struct prev_reqbufs *);
int query_buffer(struct prev_device *, struct prev_buffer *);
irqreturn_t previewer_isr(int, void *, struct pt_regs *);
int free_buffers(struct prev_device *);
int validate_params(struct prev_params *);

#endif				/* End of #ifdef __KERNEL__ */

/* ioctls definition */
#define PREV_IOC_BASE   	'P'
#define PREV_REQBUF     	_IOW(PREV_IOC_BASE, 1, struct prev_reqbufs)
#define PREV_QUERYBUF   	_IOR(PREV_IOC_BASE, 2, struct prev_buffer)
#define PREV_SET_PARAM  	_IOW(PREV_IOC_BASE, 3, struct prev_params)
#define PREV_GET_PARAM  	_IOR(PREV_IOC_BASE, 4, struct prev_params)
#define PREV_PREVIEW    	_IOWR(PREV_IOC_BASE,5, struct prev_convert)
#define PREV_GET_STATUS 	_IOR(PREV_IOC_BASE, 6, char)
#define PREV_GET_CROPSIZE 	_IOR(PREV_IOC_BASE, 7, struct prev_cropsize)
#define PREV_SET_EXP		_IOWR(PREV_IOC_BASE,8,int*)
#define PREV_IOC_MAXNR  8
/* End of ioctls */

#ifdef __KERNEL__
struct vm_struct_area;
struct inode;
struct file;
/* function definition for character driver interface functions */
int previewer_init(void);
void previewer_cleanup(void);
int previewer_open(struct inode *inode, struct file *);
int previewer_release(struct inode *inode, struct file *);
int previewer_ioctl(struct inode *inode, struct file *, unsigned int,
		    unsigned long);
int previewer_mmap(struct file *, struct vm_area_struct *);

#endif				/* End of #ifdef __KERNEL__ */

#endif				/* End of DAVINCI_PREVIEWER_H */
