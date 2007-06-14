/*
 * linux/include/asm-arm/arch-davinci/cpu.h
 *
 * Davinci cpu type detection
 *
 * Author: Steve Chen <schen@mvista.com>
 *
 * 2007 (c) MontaVista Software, Inc. This file is licensed under
 * the terms of the GNU General Public License version 2. This program
 * is licensed "as is" without any warranty of any kind, whether express
 * or implied.
 */

#ifndef _ASM_ARCH_CPU_H
#define _ASM_ARCH_CPU_H

extern unsigned int system_rev;

#define GET_DAVINCI_CPU_TYPE	((system_rev >> 16) & 0xffff)

#define IS_DAVINCI_CPU(type, id)			\
static inline int cpu_is_davinci_dm ##type (void)	\
{							\
        return (GET_DAVINCI_CPU_TYPE == (id)) ? 1 : 0;	\
}

/* following generates the cpu_is_davinci_dmxxx */
IS_DAVINCI_CPU(6443, 0x6443)	/* cpu_is_davinci_dm6443() */
IS_DAVINCI_CPU(6467, 0x6467)	/* cpu_is_davinci_dm6467() */
IS_DAVINCI_CPU(350, 0x350)	/* cpu_is_davinci_dm350() */

#endif
