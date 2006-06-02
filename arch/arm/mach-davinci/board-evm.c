/*
 * linux/arch/arm/mach-davinci/board-evm.c
 *
 * TI DaVinci EVM board
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

/**************************************************************************
 * Included Files
 **************************************************************************/

#include <linux/config.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/major.h>
#include <linux/root_dev.h>
#include <linux/dma-mapping.h>
#include <linux/platform_device.h>
#if defined(CONFIG_USB_MUSB_HDRC) || defined(CONFIG_USB_MUSB_HDRC_MODULE)
#include <linux/usb_musb.h>
#endif

#include <asm/setup.h>
#include <asm/io.h>
#include <asm/mach-types.h>

#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <asm/mach/flash.h>

#include <asm/arch/hardware.h>
#include "clock.h"

static void davinci_nop_release(struct device *dev)
{
        /* Nothing */
}

/*
 * USB
 */
#if defined(CONFIG_USB_MUSB_HDRC) || defined(CONFIG_USB_MUSB_HDRC_MODULE)

static struct musb_hdrc_platform_data usb_data = {
#if     defined(CONFIG_USB_MUSB_OTG)
        /* OTG requires a Mini-AB connector */
        .mode           = MUSB_OTG,
#elif   defined(CONFIG_USB_MUSB_PERIPHERAL)
        .mode           = MUSB_PERIPHERAL,
#elif   defined(CONFIG_USB_MUSB_HOST)
        .mode           = MUSB_HOST,
#endif
        /* irlml6401 switches 5V */
        .power          = 255,          /* sustains 3.0+ Amps (!) */
        .potpgt         = 4,            /* ~8 msec */

        /* REVISIT multipoint is a _chip_ capability; not board specific */
        .multipoint     = 1,
};

static struct resource usb_resources [] = {
	{
		/* physical address */
		.start          = DAVINCI_USB_OTG_BASE,
		.end            = DAVINCI_USB_OTG_BASE + 0x5ff,
		.flags          = IORESOURCE_MEM,
	},
	{
		.start          = IRQ_USBINT,
		.flags          = IORESOURCE_IRQ,
	},
};

static u64 usb_dmamask = DMA_32BIT_MASK;

static struct platform_device usb_dev = {
        .name           = "musb_hdrc",
        .id             = -1,
        .dev = {
                .platform_data  = &usb_data,
                .dma_mask               = &usb_dmamask,
                .coherent_dma_mask      = DMA_32BIT_MASK,
        },
        .resource       = usb_resources,
        .num_resources  = ARRAY_SIZE(usb_resources),
};

static inline void setup_usb(void)
{
        /* REVISIT:  everything except platform_data setup should be
         * shared between all DaVinci boards using the same core.
         */
        int status;

        status = platform_device_register(&usb_dev);
        if (status != 0)
                pr_debug("setup_usb --> %d\n", status);
        else
                board_setup_psc(DAVINCI_GPSC_ARMDOMAIN, DAVINCI_LPSC_USB, 1);
}

#else
static inline void setup_usb(void)
{
        /* NOP */
}
#endif  /* CONFIG_USB_MUSB_HDRC */

#ifdef CONFIG_I2C_DAVINCI
static struct resource i2c_resources[] = {
	{
		.start		= IO_ADDRESS(DAVINCI_I2C_BASE),
		.end		= IO_ADDRESS(DAVINCI_I2C_BASE) + 0x40,
		.flags		= IORESOURCE_MEM,
	},
	{
		.start		= IRQ_I2C,
		.flags		= IORESOURCE_IRQ,
	},
};

static struct platform_device davinci_i2c_device = {
        .name           = "i2c_davinci",
        .id             = 1,
        .dev = {
                .release        = davinci_nop_release,
        },
	.num_resources	= ARRAY_SIZE(i2c_resources),
	.resource	= i2c_resources,
};
static inline void setup_i2c(void)
{
	(void) platform_device_register(&davinci_i2c_device);
}
#else
static inline void setup_i2c(void)
{
	/* NOP */
}
#endif

static void board_init(void)
{
	board_setup_psc(DAVINCI_GPSC_ARMDOMAIN, DAVINCI_LPSC_VPSSMSTR, 1);
	board_setup_psc(DAVINCI_GPSC_ARMDOMAIN, DAVINCI_LPSC_VPSSSLV, 1);
	board_setup_psc(DAVINCI_GPSC_ARMDOMAIN, DAVINCI_LPSC_TPCC, 1);
	board_setup_psc(DAVINCI_GPSC_ARMDOMAIN, DAVINCI_LPSC_TPTC0, 1);
	board_setup_psc(DAVINCI_GPSC_ARMDOMAIN, DAVINCI_LPSC_TPTC1, 1);

	/* Turn on WatchDog timer LPSC.  Needed for RESET to work */
	board_setup_psc(DAVINCI_GPSC_ARMDOMAIN, DAVINCI_LPSC_TIMER2, 1);
}

/*
 * DaVinci IO Mapping
 */
static struct map_desc davinci_io_desc[] __initdata = {
	{
		.virtual	= IO_VIRT,
		.pfn		= __phys_to_pfn(IO_PHYS),
		.length		= IO_SIZE,
		.type		= MT_DEVICE
	},
	{
		.virtual	= DAVINCI_IRAM_VIRT,
		.pfn		= __phys_to_pfn(DAVINCI_IRAM_BASE),
		.length		= SZ_16K,
		.type		= MT_DEVICE
	}
};

static void __init
davinci_map_io(void)
{
	iotable_init(davinci_io_desc, ARRAY_SIZE(davinci_io_desc));

	/* Initialize the DaVinci EVM board settigs */
	board_init ();
}

static __init void evm_init(void)
{
        setup_usb();
	setup_i2c();
}


extern void davinci_irq_init(void);
extern struct sys_timer davinci_timer;

MACHINE_START(DAVINCI_EVM, "DaVinci EVM")
	/* Maintainer: MontaVista Software <source@mvista.com> */
	.phys_io      = IO_PHYS,
	.io_pg_offst  = (io_p2v(IO_PHYS) >> 18) & 0xfffc,
	.boot_params  = (DAVINCI_DDR_BASE + 0x100),
	.map_io	      = davinci_map_io,
	.init_irq     = davinci_irq_init,
	.timer	      = &davinci_timer,
	.init_machine = evm_init,
MACHINE_END
