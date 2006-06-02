/*
 * linux/arch/arm/mach-davinci/irq.c
 *
 * TI DaVinci INTC config file
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
 *
 */

#include <linux/config.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/sched.h>
#include <linux/interrupt.h>
#include <linux/ptrace.h>

#include <asm/hardware.h>
#include <asm/irq.h>
#include <asm/io.h>

#include <asm/mach/irq.h>
#include <asm/arch/irq.h>
#include <asm/arch/irqs.h>
#include <asm/arch/memory.h>
#include <asm/arch/hardware.h>

void intcInit(void);

#define INTNUM                      DAVINCI_MAXIRQNUM
#define EXCNUM                      ( 8 - 1 )
#define INTC_NUM_INTS_ONE_REGISTER  32

#define INTC_CLEAR_INTERRUPTS       0xFFFFFFFF
#define INTC_IDT_BASE			DAVINCI_IRAM_VIRT
#define INTC_DISABLE_WHEN_CLEARED_MODE   0x4
#define INTC_IRQ_ENTRY_RAW				0x2
#define INTC_FIQ_ENTRY_RAW				0x1
#define INTC_VECT_OFFSET				0x100

volatile intc_registers *pintc = (intc_registers *) IO_ADDRESS(DAVINCI_ARM_INTC_BASE);


void davinci_intcinit(void);

/* Disable interrupt */
static void davinci_xdisable_int(unsigned int intno)
{
	unsigned int mask;

	if (intno < INTC_NUM_INTS_ONE_REGISTER) {
		mask = 1 << intno;
		pintc->eint0 &= ~mask;
	} else if (intno <= INTNUM) {
		mask = 1 << (intno - INTC_NUM_INTS_ONE_REGISTER);
		pintc->eint1 &= ~mask;
	}
}

/* Enable interrupt */
static void davinci_xenable_int(unsigned int intno)
{
	unsigned int mask;
	if (intno < INTC_NUM_INTS_ONE_REGISTER) {
		mask = 1 << intno;
		pintc->eint0 |= mask;
	} else if (intno <= INTNUM) {
		mask = 1 << (intno - INTC_NUM_INTS_ONE_REGISTER);
		pintc->eint1 |= mask;
	}
}

/* EOI interrupt */
void davinci_xeoi_pic(unsigned int intno)
{
	unsigned int mask;
	if (intno < INTC_NUM_INTS_ONE_REGISTER) {
		mask = 1 << intno;
		pintc->irq0 = mask;
	} else if (intno < INTNUM) {
		mask = 1 << (intno - INTC_NUM_INTS_ONE_REGISTER);
		pintc->irq1 = mask;
	}
}

static struct irqchip irqchip_0 = {
	.ack = davinci_xeoi_pic,
	.mask = davinci_xdisable_int,
	.unmask = davinci_xenable_int,
};

void davinci_irq_init(void)
{
	int i;
	unsigned int *ptr = (unsigned int *)INTC_IDT_BASE;

	davinci_intcinit();

	ptr += INTC_VECT_OFFSET / (sizeof(*ptr));

	for (i = 0; i < INTNUM; i++) {
		if (i == 0) {
			*ptr = 0xFFFFFFFF;
		} else {
			*ptr = i - 1;
		}
		ptr++;
	}
	/* Proggam the irqchip structures for ARM INTC */

	for (i = 0; i < NR_IRQS; i++) {
                if (i != IRQ_TINT1_TINT34)
                        set_irq_handler(i, do_edge_IRQ);
                else
                        set_irq_handler(i, do_level_IRQ);
		set_irq_chip(i, &irqchip_0);
		set_irq_flags(i, IRQF_VALID | IRQF_PROBE);
	}
}

/* Interrupt Controller Initialize */
void davinci_intcinit(void)
{
	/* Clear all interrupt requests - write 1 to clear the interrupt */
	pintc->fiq0 = pintc->irq0 = INTC_CLEAR_INTERRUPTS;
	pintc->fiq1 = pintc->irq1 = INTC_CLEAR_INTERRUPTS;

	/* Disable all interrupts */
	pintc->eint0 = pintc->eint1 = 0;

	/* Interrupts disabled immediately. IRQ entry reflects all interrupts */
	pintc->inctl = 0;

	/* Set vector table base address
	 * last two bits are zero which means 4 byte entry */
	pintc->eabase = (unsigned int)INTC_VECT_OFFSET;

	/* Clear all interrupt requests - write 1 to clear the interrupt */
	pintc->fiq0 = pintc->irq0 = INTC_CLEAR_INTERRUPTS;
	pintc->fiq1 = pintc->irq1 = INTC_CLEAR_INTERRUPTS;

	return;
}
