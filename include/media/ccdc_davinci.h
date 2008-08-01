/*
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
/* ccdc_davinci.h */

#ifndef CCDC_DAVINCI_H
#define CCDC_DAVINCI_H
#include <linux/types.h>

#ifdef __KERNEL__
#include <asm/arch/hardware.h>
#include <asm/io.h>
#endif

#include <linux/videodev.h>

typedef enum ccdc_pixfmt {
	CCDC_PIXFMT_RAW = 0,
	CCDC_PIXFMT_YCBCR_16BIT = 1,
	CCDC_PIXFMT_YCBCR_8BIT = 2
} ccdc_pixfmt;

typedef enum ccdc_frmfmt {
	CCDC_FRMFMT_PROGRESSIVE = 0,
	CCDC_FRMFMT_INTERLACED = 1
} ccdc_frmfmt;

typedef enum ccdc_pinpol {
	CCDC_PINPOL_POSITIVE = 0,
	CCDC_PINPOL_NEGATIVE = 1
} ccdc_pinpol;

/* PIXEL ORDER IN MEMORY from LSB to MSB */
/* only applicable for 8-bit input mode  */
typedef enum ccdc_pixorder {
	CCDC_PIXORDER_CBYCRY = 1,
	CCDC_PIXORDER_YCBYCR = 0
} ccdc_pixorder;

typedef enum ccdc_buftype {
	CCDC_BUFTYPE_FLD_INTERLEAVED,
	CCDC_BUFTYPE_FLD_SEPARATED
} ccdc_buftype;

typedef struct v4l2_rect ccdc_imgwin;

typedef struct ccdc_params_ycbcr {
	ccdc_pixfmt pix_fmt;	/* pixel format                     */
	ccdc_frmfmt frm_fmt;	/* progressive or interlaced frame  */
	ccdc_imgwin win;	/* video window                     */
	ccdc_pinpol fid_pol;	/* field id polarity                */
	ccdc_pinpol vd_pol;	/* vertical sync polarity           */
	ccdc_pinpol hd_pol;	/* horizontal sync polarity         */
	int bt656_enable;	/* enable BT.656 embedded sync mode */
	ccdc_pixorder pix_order;/* cb:y:cr:y or y:cb:y:cr in memory */
	ccdc_buftype buf_type;	/* interleaved or separated fields  */
} ccdc_params_ycbcr;

#ifdef __KERNEL__
/**************************************************************************\
* Register OFFSET Definitions
\**************************************************************************/
#define PID                             0x0
#define PCR                             0x4
#define SYN_MODE                        0x8
#define HD_VD_WID                       0xc
#define PIX_LINES                       0x10
#define HORZ_INFO                       0x14
#define VERT_START                      0x18
#define VERT_LINES                      0x1c
#define CULLING                         0x20
#define HSIZE_OFF                       0x24
#define SDOFST                          0x28
#define SDR_ADDR                        0x2c
#define CLAMP                           0x30
#define DCSUB                           0x34
#define COLPTN                          0x38
#define BLKCMP                          0x3c
#define FPC                             0x40
#define FPC_ADDR                        0x44
#define VDINT                           0x48
#define ALAW                            0x4c
#define REC656IF                        0x50
#define CCDCFG                          0x54
#define FMTCFG                          0x58
#define FMT_HORZ                        0x5c
#define FMT_VERT                        0x50
#define FMT_ADDR0                       0x64
#define FMT_ADDR1                       0x68
#define FMT_ADDR2                       0x6c
#define FMT_ADDR3                       0x70
#define FMT_ADDR4                       0x74
#define FMT_ADDR5                       0x78
#define FMT_ADDR6                       0x7c
#define FMT_ADDR7                       0x80
#define PRGEVEN_0                       0x84
#define PRGEVEN_1                       0x88
#define PRGODD_0                        0x8c
#define PRGODD_1                        0x90
#define VP_OUT                          0x94

#define CCDC_IOBASE                     (0x01c70400)

#define regw(val, reg)    davinci_writel(val, (reg)+CCDC_IOBASE)
#define regr(reg)         davinci_readl((reg)+CCDC_IOBASE)

extern void ccdc_reset(void);
extern void ccdc_config_ycbcr(ccdc_params_ycbcr * params);
extern void ccdc_setwin(ccdc_params_ycbcr * params);

/* inline functions that must be fast because they are called frequently */
static inline void ccdc_enable(int flag)
{
	regw(flag, PCR);
}

static inline void ccdc_setfbaddr(unsigned long paddr)
{
	regw(paddr & 0xffffffe0, SDR_ADDR);
}

static inline int ccdc_getfid(void)
{
	int fid = (regr(SYN_MODE) >> 15) & 0x1;
	return fid;
}

static inline int ccdc_getfidmode(void)
{
	int fid = (regr(SYN_MODE) >> 7) & 0x1;
	return fid;
}
#endif

#endif /* CCDC_DAVINCI_H */
