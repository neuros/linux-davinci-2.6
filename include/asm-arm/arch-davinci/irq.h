/*
 *  linux/include/asm-arm/arch-davinci/irq.h
 *
 *  BRIEF MODULE DESCRIPTION
 *      DAVINCI Virtual irq definitions
 *
 *  Copyright (C) 2006 Texas Instruments.
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 *
 *  THIS  SOFTWARE  IS PROVIDED   ``AS  IS'' AND   ANY  EXPRESS OR IMPLIED
 *  WARRANTIES,   INCLUDING, BUT NOT  LIMITED  TO, THE IMPLIED WARRANTIES OF
 *  MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN
 *  NO  EVENT  SHALL   THE AUTHOR  BE    LIABLE FOR ANY   DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 *  NOT LIMITED   TO, PROCUREMENT OF  SUBSTITUTE GOODS  OR SERVICES; LOSS OF
 *  USE, DATA,  OR PROFITS; OR  BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 *  ANY THEORY OF LIABILITY, WHETHER IN  CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 *  THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *  You should have received a copy of the  GNU General Public License along
 *  with this program; if not, write  to the Free Software Foundation, Inc.,
 *  675 Mass Ave, Cambridge, MA 02139, USA.
 *
 */

#ifndef __ASM_ARCH_IRQ_H
#define __ASM_ARCH_IRQ_H

/**************************************************************************
 * Included Files
 **************************************************************************/

/**************************************************************************
 * Global Function Prototypes
 **************************************************************************/
typedef struct intc_registers_t {
	unsigned int fiq0;	/* 0x0 */
	unsigned int fiq1;	/* 0x4 */
	unsigned int irq0;	/* 0x8 */
	unsigned int irq1;	/* 0xC */
	unsigned int fiqentry;	/* 0x10 */
	unsigned int irqentry;	/* 0x14 */
	unsigned int eint0;	/* 0x18 */
	unsigned int eint1;	/* 0x1C */
	unsigned int inctl;	/* 0x20 */
	unsigned int eabase;	/* 0x24 */
	unsigned int resv1;	/* 0x28 */
	unsigned int resv2;	/* 0x2C */
	unsigned int intpri0;	/* 0x30 */
	unsigned int intpri1;	/* 0x34 */
	unsigned int intpri2;	/* 0x38 */
	unsigned int intpri3;	/* 0x3C */
	unsigned int intpri4;	/* 0x30 */
	unsigned int intpri5;	/* 0x34 */
	unsigned int intpri6;	/* 0x38 */
	unsigned int intpri7;	/* 0x3C */
} intc_registers;

/****************************************************
 * DaVinci Interrupt numbers
 ****************************************************/

#endif				/* __ASM_ARCH_IRQ_H */
