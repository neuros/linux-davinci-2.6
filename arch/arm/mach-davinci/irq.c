/*
 * linux/arch/arm/mach-davinci/irq.c
 *
 * Interrupt handler for DaVinci boards.
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
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 * ----------------------------------------------------------------------------
 */
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/irq.h>

#include <asm/hardware.h>
#include <asm/io.h>
#include <asm/mach/irq.h>

#define IRQ_BIT(irq)  ((irq) & 0x1f)

#define INTC_IDT_BASE		DAVINCI_IRAM_VIRT
#define INTC_VECT_OFFSET	0x100

#define FIQ_REG0_OFFSET		0x0000
#define FIQ_REG1_OFFSET		0x0004
#define IRQ_REG0_OFFSET		0x0008
#define IRQ_REG1_OFFSET		0x000C
#define IRQ_ENT_REG0_OFFSET	0x0018
#define IRQ_ENT_REG1_OFFSET	0x001C
#define IRQ_INCTL_REG_OFFSET	0x0020
#define IRQ_EABASE_REG_OFFSET	0x0024

static void __init davinci_intcinit(void);

static inline unsigned int davinci_irq_readl(int offset)
{
	return davinci_readl(DAVINCI_ARM_INTC_BASE + offset);
}

static inline void davinci_irq_writel(unsigned long value, int offset)
{
	davinci_writel(value, DAVINCI_ARM_INTC_BASE + offset);
}

/* Disable interrupt */
static void davinci_mask_irq(unsigned int irq)
{
	unsigned int mask;
	u32 l;

	mask = 1 << IRQ_BIT(irq);

	if (irq > 31) {
		l = davinci_irq_readl(IRQ_ENT_REG1_OFFSET);
		l &= ~mask;
		davinci_irq_writel(l, IRQ_ENT_REG1_OFFSET);
	} else {
		l = davinci_irq_readl(IRQ_ENT_REG0_OFFSET);
		l &= ~mask;
		davinci_irq_writel(l, IRQ_ENT_REG0_OFFSET);
	}
}

/* Enable interrupt */
static void davinci_unmask_irq(unsigned int irq)
{
	unsigned int mask;
	u32 l;

	mask = 1 << IRQ_BIT(irq);

	if (irq > 31) {
		l = davinci_irq_readl(IRQ_ENT_REG1_OFFSET);
		l |= mask;
		davinci_irq_writel(l, IRQ_ENT_REG1_OFFSET);
	} else {
		l = davinci_irq_readl(IRQ_ENT_REG0_OFFSET);
		l |= mask;
		davinci_irq_writel(l, IRQ_ENT_REG0_OFFSET);
	}
}

/* EOI interrupt */
static void davinci_ack_irq(unsigned int irq)
{
	unsigned int mask;

	mask = 1 << IRQ_BIT(irq);

	if (irq > 31)
		davinci_irq_writel(mask, IRQ_REG1_OFFSET);
	else
		davinci_irq_writel(mask, IRQ_REG0_OFFSET);
}

static struct irq_chip davinci_irq_chip_0 = {
	.name	= "AINTC",
	.ack	= davinci_ack_irq,
	.mask	= davinci_mask_irq,
	.unmask = davinci_unmask_irq,
};

void __init davinci_irq_init(void)
{
	int i;
	unsigned int *idtbase = (unsigned int *)INTC_IDT_BASE;
	unsigned int *eabase;

	davinci_intcinit();

	eabase = idtbase +  INTC_VECT_OFFSET / sizeof(*idtbase);

	*eabase = ~0x0;
	eabase++;

	for (i = 1; i < NR_IRQS; i++) {
		*eabase = i - 1;
		eabase++;
	}

	/* Proggam the irqchip structures for ARM INTC */
	for (i = 0; i < NR_IRQS; i++) {
		set_irq_chip(i, &davinci_irq_chip_0);
		set_irq_flags(i, IRQF_VALID | IRQF_PROBE);
                if (i != IRQ_TINT1_TINT34)
                        set_irq_handler(i, do_edge_IRQ);
                else
                        set_irq_handler(i, do_level_IRQ);
	}
}

/* ARM Interrupt Controller Initialize */
static void __init davinci_intcinit(void)
{
	/* Clear all interrupt requests */
	davinci_irq_writel(~0x0, FIQ_REG0_OFFSET);
	davinci_irq_writel(~0x0, FIQ_REG1_OFFSET);
	davinci_irq_writel(~0x0, IRQ_REG0_OFFSET);
	davinci_irq_writel(~0x0, IRQ_REG1_OFFSET);

	/* Disable all interrupts */
	davinci_irq_writel(0x0, IRQ_ENT_REG0_OFFSET);
	davinci_irq_writel(0x0, IRQ_ENT_REG1_OFFSET);

	/* Interrupts disabled immediately. IRQ entry reflects all interrupts */
	davinci_irq_writel(0x0, IRQ_INCTL_REG_OFFSET);

	/* Set vector table base address - 4 byte entry */
	davinci_irq_writel(INTC_VECT_OFFSET, IRQ_EABASE_REG_OFFSET);

	/* Clear all interrupt requests */
	davinci_irq_writel(~0x0, FIQ_REG0_OFFSET);
	davinci_irq_writel(~0x0, FIQ_REG1_OFFSET);
	davinci_irq_writel(~0x0, IRQ_REG0_OFFSET);
	davinci_irq_writel(~0x0, IRQ_REG1_OFFSET);
}
