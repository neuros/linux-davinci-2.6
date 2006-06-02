/*
 * linux/drivers/i2c/busses/davinci/i2c_davinci.h
 *
 * Copyright (C) 2006 Texas Instruments.
 *
 * ----------------------------------------------------------------------------
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
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 * ----------------------------------------------------------------------------
 Modifications:
 ver. 1.0: Feb 2005, Vinod/Sudhakar
 -
 *
 */

#define DAVINCI_I2C_ICOAR_OADDR_MASK        (0x03FFu)
#define DAVINCI_I2C_ICIMR_AAS_MASK          (0x0040u)
#define DAVINCI_I2C_ICIMR_SCD_MASK          (0x0020u)
#define DAVINCI_I2C_ICIMR_ICXRDY_MASK       (0x0010u)
#define DAVINCI_I2C_ICIMR_ICRRDY_MASK       (0x0008u)
#define DAVINCI_I2C_ICIMR_ARDY_MASK         (0x0004u)
#define DAVINCI_I2C_ICIMR_NACK_MASK         (0x0002u)
#define DAVINCI_I2C_ICIMR_AL_MASK           (0x0001u)
#define DAVINCI_I2C_ICSTR_SDIR_MASK         (0x4000u)
#define DAVINCI_I2C_ICSTR_NACKSNT_MASK      (0x2000u)
#define DAVINCI_I2C_ICSTR_BB_MASK           (0x1000u)
#define DAVINCI_I2C_ICSTR_RSFULL_MASK       (0x0800u)
#define DAVINCI_I2C_ICSTR_XSMT_MASK         (0x0400u)
#define DAVINCI_I2C_ICSTR_AAS_MASK          (0x0200u)
#define DAVINCI_I2C_ICSTR_AD0_MASK          (0x0100u)
#define DAVINCI_I2C_ICSTR_SCD_MASK          (0x0020u)
#define DAVINCI_I2C_ICSTR_ICXRDY_MASK       (0x0010u)
#define DAVINCI_I2C_ICSTR_ICRRDY_MASK       (0x0008u)
#define DAVINCI_I2C_ICSTR_ARDY_MASK         (0x0004u)
#define DAVINCI_I2C_ICSTR_NACK_MASK         (0x0002u)
#define DAVINCI_I2C_ICSTR_AL_MASK           (0x0001u)
#define DAVINCI_I2C_ICCLKL_ICCL_MASK        (0xFFFFu)
#define DAVINCI_I2C_ICCLKH_ICCH_MASK        (0xFFFFu)
#define DAVINCI_I2C_ICCNT_ICDC_MASK         (0xFFFFu)
#define DAVINCI_I2C_ICDRR_D_MASK            (0x00FFu)
#define DAVINCI_I2C_ICSAR_SADDR_MASK        (0x03FFu)
#define DAVINCI_I2C_ICDXR_D_MASK            (0x00FFu)
#define DAVINCI_I2C_ICMDR_NACKMOD_MASK      (0x8000u)
#define DAVINCI_I2C_ICMDR_FREE_MASK         (0x4000u)
#define DAVINCI_I2C_ICMDR_STT_MASK          (0x2000u)
#define DAVINCI_I2C_ICMDR_STP_MASK          (0x0800u)
#define DAVINCI_I2C_ICMDR_MST_MASK          (0x0400u)
#define DAVINCI_I2C_ICMDR_TRX_MASK          (0x0200u)
#define DAVINCI_I2C_ICMDR_XA_MASK           (0x0100u)
#define DAVINCI_I2C_ICMDR_RM_MASK           (0x0080u)
#define DAVINCI_I2C_ICMDR_DLB_MASK          (0x0040u)
#define DAVINCI_I2C_ICMDR_IRS_MASK          (0x0020u)
#define DAVINCI_I2C_ICMDR_STB_MASK          (0x0010u)
#define DAVINCI_I2C_ICMDR_FDF_MASK          (0x0008u)
#define DAVINCI_I2C_ICMDR_BC_MASK           (0x0007u)
#define DAVINCI_I2C_ICIVR_TESTMD_MASK       (0x0F00u)
#define DAVINCI_I2C_ICIVR_INTCODE_MASK      (0x0007u)

#define DAVINCI_I2C_ICIVR_INTCODE_NONE      (0x0000u)
#define DAVINCI_I2C_ICIVR_INTCODE_AL        (0x0001u)
#define DAVINCI_I2C_ICIVR_INTCODE_NACK      (0x0002u)
#define DAVINCI_I2C_ICIVR_INTCODE_RAR       (0x0003u)
#define DAVINCI_I2C_ICIVR_INTCODE_RDR       (0x0004u)
#define DAVINCI_I2C_ICIVR_INTCODE_TDR       (0x0005u)
#define DAVINCI_I2C_ICIVR_INTCODE_SCD       (0x0006u)
#define DAVINCI_I2C_ICIVR_INTCODE_AAS       (0x0007u)

#define DAVINCI_I2C_ICEMDR_BCM_MASK         (0x0001u)
#define DAVINCI_I2C_ICPSC_IPSC_MASK         (0x00FFu)
#define DAVINCI_I2C_ICPID1_CLASS_MASK       (0xFF00u)
#define DAVINCI_I2C_ICPID1_REVISION_MASK    (0x00FFu)
#define DAVINCI_I2C_ICPID2_TYPE_MASK        (0x00FFu)
#define DAVINCI_I2C_ICPFUNC_PFUNC_MASK      (0x00000001u)
#define DAVINCI_I2C_ICPDIR_PDIR1_MASK       (0x00000002u)
#define DAVINCI_I2C_ICPDIR_PDIR0_MASK       (0x00000001u)
#define DAVINCI_I2C_ICPDIN_PDIN1_MASK       (0x00000002u)
#define DAVINCI_I2C_ICPDIN_PDIN0_MASK       (0x00000001u)
#define DAVINCI_I2C_ICPDOUT_PDOUT1_MASK     (0x00000002u)
#define DAVINCI_I2C_ICPDOUT_PDOUT0_MASK     (0x00000001u)
#define DAVINCI_I2C_ICPDSET_PDSET1_MASK     (0x00000002u)
#define DAVINCI_I2C_ICPDSET_PDSET0_MASK     (0x00000001u)
#define DAVINCI_I2C_ICPDCLR_PDCLR1_MASK     (0x00000002u)
#define DAVINCI_I2C_ICPDCLR_PDCLR0_MASK     (0x00000001u)

/**************************************************************************\
* Register Overlay Structure
\**************************************************************************/
typedef struct {
	u16 icoar;
	u8 rsvd0[2];
	u16 icimr;
	u8 rsvd1[2];
	u16 icstr;
	u8 rsvd2[2];
	u16 icclkl;
	u8 rsvd3[2];
	u16 icclkh;
	u8 rsvd4[2];
	u16 iccnt;
	u8 rsvd5[2];
	u16 icdrr;
	u8 rsvd6[2];
	u16 icsar;
	u8 rsvd7[2];
	u16 icdxr;
	u8 rsvd8[2];
	u16 icmdr;
	u8 rsvd9[2];
	u16 icivr;
	u8 rsvd10[2];
	u16 icemdr;
	u8 rsvd11[2];
	u16 icpsc;
	u8 rsvd12[2];
	u16 icpid1;
	u8 rsvd13[2];
	u16 icpid2;
	u8 rsvd14[14];
	u32 ipcfunc;
	u32 icpdir;
	u32 icpdin;
	u32 icpdout;
	u32 icpdset;
	u32 icpdclr;
} davinci_i2cregs;

/**************************************************************************\
* Overlay structure typedef definition
\**************************************************************************/
typedef volatile davinci_i2cregs *davinci_i2cregsovly;

struct i2c_davinci_device {
	int cmd_err;
	struct completion cmd_complete;
	wait_queue_head_t cmd_wait;
	u8 *buf;
	size_t buf_len;
	davinci_i2cregsovly regs;

	int irq;
	struct i2c_adapter	adapter;
	struct clk              *clk;
	struct device           *dev;
};
