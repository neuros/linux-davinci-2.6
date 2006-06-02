/* linux/include/asm-arm/arch-davinci/debug-macro.S
 *
 * Debugging macro include header
 *
 *  Copyright (C) 1994-1999 Russell King
 *  Moved from linux/arch/arm/kernel/debug.S by Ben Dooks
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
*/

		.macro addruart, rx
		mrc	p15, 0, \rx, c1, c0
		tst	\rx, #1			@ MMU enabled?
		moveq	\rx, #0x01c00000	@ physical base address
		movne	\rx, #0xe1000000	@ virtual base
		orr	\rx, \rx, #0x00020000   @ UART 0
		.endm

		.macro	senduart,rd,rx
		strb	\rd, [\rx]
		.endm

		.macro	busyuart,rd,rx
1001:		ldrb	\rd, [\rx, #(0x5 << 2)]
		and	\rd, \rd, #0x60
		teq	\rd, #0x60
		bne	1001b
		.endm

		.macro	waituart,rd,rx
		.endm

