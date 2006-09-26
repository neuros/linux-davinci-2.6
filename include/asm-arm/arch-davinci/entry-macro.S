/*
 * include/asm-arm/arch-davinci/entry-macro.S
 *
 * Low-level IRQ helper macros for TI DaVinci-based platforms
 *
 * This file is licensed under  the terms of the GNU General Public
 * License version 2. This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */
#include <asm/hardware.h>

		.macro	disable_fiq
		.endm

		.macro	get_irqnr_and_base, irqnr, irqstat, base, tmp
		/* GIVEN:
		 *   EABASE = 0 ... so IRQNR = (IRQENTRY/4) - 1
		 * RETURN:
		 *   irqnr:  Interrupt number.  Zero corresponds
		 *     to bit 0 of the status register
		 *   irqstat, base, and tmp may be considered
		 *     as scratch registers
		 *   Z conditions means no outstanding interrupt
		 */
		ldr \base, =IO_ADDRESS(DAVINCI_ARM_INTC_BASE)
		ldr \tmp, [\base, #0x14]
		mov \tmp, \tmp, lsr #2
		sub \irqnr, \tmp, #1
		cmp \tmp, #0
		.endm

		.macro	irq_prio_table
		.endm

