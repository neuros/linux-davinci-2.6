/*
 * include/asm-arm/arch-davinci/nand.h
 *
 * Copyright (C) 2006 Texas Instruments.
 *
 * ported to 2.6.23 (C) 2008 by
 * Sander Huijsen <Shuijsen@optelecom-nkf.com>
 * Troy Kisky <troy.kisky@boundarydevices.com>
 * Dirk Behme <Dirk.Behme@gmail.com>
 *
 * --------------------------------------------------------------------------
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
 * --------------------------------------------------------------------------
 *
 */

#ifndef __ARCH_ARM_DAVINCI_NAND_H
#define __ARCH_ARM_DAVINCI_NAND_H

#define NRCSR_OFFSET   		0x00
#define AWCCR_OFFSET    	0x04
#define A1CR_OFFSET		0x10
#define NANDFCR_OFFSET 		0x60
#define NANDFSR_OFFSET 		0x64
#define NANDF1ECC_OFFSET	0x70

#define	MASK_ALE		0x0A
#define	MASK_CLE		0x10

#define NAND_BUSY_FLAG		0x01

#endif	/* __ARCH_ARM_DAVINCI_NAND_H */
