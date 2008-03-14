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

/* davinci_previewer_hw.h file */

#ifndef DAVINCI_PREVIEWER_HW_H
#define DAVINCI_PREVIEWER_HW_H

#ifdef __KERNEL__

#include <linux/kernel.h>	/* printk() */
#include <asm/io.h>		/* For IO_ADDRESS */

#define PREVIEWER_IOBASE_VADDR      IO_ADDRESS(0x01C70800)

/* Register Offsets from the base address */
#define PID               0x0000
#define PCR               0x0004
#define HORZ_INFO         0x0008
#define VERT_INFO         0x000C
#define RSDR_ADDR         0x0010
#define RADR_OFFSET       0x0014
#define DSDR_ADDR         0x0018
#define DRKF_OFFSET       0x001C
#define WSDR_ADDR         0x0020
#define WADD_OFFSET       0x0024
#define AVE               0x0028
#define HMED              0x002C
#define NF                0x0030
#define WB_DGAIN          0x0034
#define WBGAIN            0x0038
#define WBSEL             0x003C
#define CFA               0x0040
#define BLKADJOFF         0x0044
#define RGB_MAT1          0x0048
#define RGB_MAT2          0x004C
#define RGB_MAT3          0x0050
#define RGB_MAT4          0x0054
#define RGB_MAT5          0x0058
#define RGB_OFF1          0x005C
#define RGB_OFF2          0x0060
#define CSC0              0x0064
#define CSC1              0x0068
#define CSC2              0x006C
#define CSC_OFFSET        0x0070
#define CNT_BRT           0x0074
#define CSUP              0x0078
#define SETUP_YC          0x007C
#define SET_TBL_ADDR      0x0080
#define SET_TBL_DATA      0x0084
/* End of register offsets */
#define VPSS_PCR          (0x3404-0x0800)
#define SDR_REQ_EXP       (0x3508-0x0800)

/* Register read/write */
#define regw(val, reg)    outl(val, (reg)+PREVIEWER_IOBASE_VADDR)
#define regr(reg)         inl((reg)+PREVIEWER_IOBASE_VADDR)
/* -- */

/* macro for bit set and clear */
#define SETBIT(reg, bit)   (reg = ((reg) | ((0x00000001)<<(bit))))
#define RESETBIT(reg, bit) (reg = ((reg) & (~(0x00000001<<(bit)))))
/* -- */

/* bit positions of the configurations in PCR register */
#define PREV_ENABLE_BIT         0
#define PREV_SOURCE_BIT         2
#define PREV_ONESHOT_BIT        3
#define PREV_WIDTH_BIT          4
#define INVALAW_BIT             5
#define DARK_FRAME_WRITE_BIT    6
#define DARK_FRAME_CAPTURE_BIT  7
#define HMF_BIT                 8
#define NOISE_FILTER_BIT        9
#define CFA_BIT                 10
#define LUMA_ENHANCE_BIT        15
#define CHROMA_SUPPRESS_BIT     16
#define RSZPORT_BIT             19
#define DDRAMPORT_BIT           20
#define SHADECOMP_BIT           21
#define GAMMA_BYPASS_BIT        26
#define PIXEL_ORDER_BIT         17

/* -- */

/* Internal RAM table addresses for NF */
#define NOISE_FILTER_START_ADDR  0x0C00
#define NOISE_FILTER_END_ADDR    0x0CFF

/* Internal RAM table addresses for gamma correction */
#define RED_GAMMA_START_ADDR      0x0000
#define RED_GAMMA_END_ADDR        0x03FF
#define GREEN_GAMMA_START_ADDR    0x0400
#define GREEN_GAMMA_END_ADDR      0x07FF
#define BLUE_GAMMA_START_ADDR     0x0800
#define BLUE_GAMMA_END_ADDR       0x0BFF
/* -- */

/* Internal RAM table addresses for Luma enhancement */
#define LUMA_ENHANCE_START_ADDR   0x1000
#define LUMA_ENHANCE_END_ADDR     0x107F
/* -- */

/* Internal RAM table addresses for CFA Coefficients */
#define CFA_COEFF_START_ADDR    0x1400
#define CFA_COEFF_END_ADDR      0x163F
/* -- */

/* bit position of whether to use high passed version of Y or not */
#define CHROMA_HPFY           16

#define AVE_ODD_PIXEL_DIST    16	/* (1 << 4)distance between two consecutive 
					   pixels of same color is 2 in bayer pattern
					   in odd lines. to be set in 2 and 3 bits 
					   of AVE */
#define AVE_EVEN_PIXEL_DIST   4	/*(1 << 2)distance between two consecutive 
				   pixels of same color is 2 in bayer pattern 
				   in even lines to be set in 4 and 5 
				   bits of AVE */

/* inline function to enable previewer */
static inline void previewer_enable(void)
{
	int pcr = regr(PCR);
	regw((pcr | 0x01), PCR);
}

/* inline function to set previewer in one shot mode */
static inline void set_oneshot_mode(void)
{
	int pcr = regr(PCR);
	regw((pcr | (0x01 << 3)), PCR);
}

/* inline function to set previewer input source to DDRAM */
static inline void set_input_source(int i)
{
	int pcr = regr(PCR);
	regw((pcr | (i << 2)), PCR);
}

/* inline function to set read line offset */
static inline void set_rsdr_offset(int offset)
{
	regw(offset, RADR_OFFSET);
}

/* inline function to set write line offset */
static inline void set_wsdr_offset(int offset)
{
	regw(offset, WADD_OFFSET);
}

/* inline function to set the size of the input image in register */
static inline void set_size(int hstart, int vstart, int width, int height)
{
	int horz_info = (width - 1 + hstart) & 0x3fff;
	int vert_info = (height - 1 + vstart) & 0x3fff;
	horz_info |= ((hstart & 0x3fff) << 16);
	vert_info |= ((vstart & 0x3fff) << 16);
	regw(horz_info, HORZ_INFO);
	regw(vert_info, VERT_INFO);
}

/* inline function to set input/output addresses in registers */
static inline void set_address(unsigned long input, unsigned long output)
{
	regw(input, RSDR_ADDR);
	regw(output, WSDR_ADDR);

}

#define isbusy()    ((regr(PCR) & 0x02)>>1)
static inline void prev_set_exp(int exp)
{
	regw(((exp & 0x3ff) << 20), SDR_REQ_EXP);
}

static inline int prev_writebuffer_status(void)
{
	return regr(VPSS_PCR);
}

/* Forward declaration */
struct prev_params;

extern int previewer_hw_setup(struct prev_params *);
#endif				/* End of #ifdef __KERNEL__ */
#endif				/* End of #ifdef DAVINCI_PREVIEWER_HW_H */
