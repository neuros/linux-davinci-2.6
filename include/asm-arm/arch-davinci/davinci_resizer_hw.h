/* *
 * Copyright (C) 2006 Texas Instruments Inc
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 */
/* davinci_resizer_hw.c file */
#ifndef	DAVINCI_RESIZER_HW
#define	DAVINCI_RESIZER_HW

#include <asm/arch/davinci_resizer.h>
#include <asm/arch/hardware.h>
#include <asm/io.h>

/* Register	offset mapping*/
#define	RESIZER_IOBASE_VADDR	IO_ADDRESS(0x01C70C00)
#define	PID					0x0000
#define	PCR					0x0004
#define	RSZ_CNT					0x0008
#define	OUT_SIZE				0x000C
#define	IN_START				0x0010
#define	IN_SIZE					0x0014
#define	SDR_INADD				0x0018
#define	SDR_INOFF				0x001C
#define	SDR_OUTADD				0x0020
#define	SDR_OUTOFF				0x0024
#define	HFILT10					0x0028
#define	VFILT10					0x0068
#define	COEFF_ADDRESS_OFFSET			0x04
#define	YENH					0x00A8

#define	VPSS_PCR				(0x3404-0x0C00)
#define	SDR_REQ_EXP				(0x3508-0x0C00)

/* Register	read/write */
#define	regw(val,reg)		outl(val,((reg)+ RESIZER_IOBASE_VADDR))
#define	regr(reg)		inl((reg)+RESIZER_IOBASE_VADDR)

/* functions definition*/
void rsz_hardware_setup(channel_config_t * rsz_conf_chan);
int rsz_enable(channel_config_t * rsz_conf_chan);
static void inline rsz_set_exp(int exp)
{
	regw(((exp & 0x3ff) << 10), SDR_REQ_EXP);
}
int rsz_writebuffer_status(void);
#endif
