/*
 * TI DaVinci EVM board support
 *
 * Author: Kevin Hilman, MontaVista Software, Inc. <source@mvista.com>
 *
 * 2007 (c) MontaVista Software, Inc. This file is licensed under
 * the terms of the GNU General Public License version 2. This program
 * is licensed "as is" without any warranty of any kind, whether express
 * or implied.
 */
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/dma-mapping.h>
#include <linux/platform_device.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/nand.h>
#include <linux/mtd/partitions.h>
#include <linux/mtd/physmap.h>

#include <asm/setup.h>
#include <asm/io.h>
#include <asm/mach-types.h>
#include <asm/hardware.h>

#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <asm/mach/flash.h>

#include <asm/arch/common.h>
#include <asm/arch/hardware.h>
#include <asm/arch/psc.h>

/* other misc. init functions */
void __init davinci_psc_init(void);
void __init davinci_irq_init(void);
void __init davinci_map_common_io(void);
void __init davinci_init_common_hw(void);

/* NOR Flash base address set to CS0 by default */
#define NOR_FLASH_PHYS 0x02000000

static struct mtd_partition davinci_evm_norflash_partitions[] = {
	/* bootloader (U-Boot, etc) in first 4 sectors */
	{
		.name		= "bootloader",
		.offset		= 0,
		.size		= 4 * SZ_64K,
		.mask_flags	= MTD_WRITEABLE, /* force read-only */
	},
	/* bootloader params in the next 1 sectors */
	{
		.name		= "params",
		.offset		= MTDPART_OFS_APPEND,
		.size		= SZ_64K,
		.mask_flags	= 0,
	},
	/* kernel */
	{
		.name		= "kernel",
		.offset		= MTDPART_OFS_APPEND,
		.size		= SZ_2M,
		.mask_flags	= 0
	},
	/* file system */
	{
		.name		= "filesystem",
		.offset		= MTDPART_OFS_APPEND,
		.size		= MTDPART_SIZ_FULL,
		.mask_flags	= 0
	}
};

static struct physmap_flash_data davinci_evm_norflash_data = {
	.width		= 2,
	.parts		= davinci_evm_norflash_partitions,
	.nr_parts	= ARRAY_SIZE(davinci_evm_norflash_partitions),
};

/* NOTE: CFI probe will correctly detect flash part as 32M, but EMIF
 * limits addresses to 16M, so using addresses past 16M will wrap */
static struct resource davinci_evm_norflash_resource = {
	.start		= NOR_FLASH_PHYS,
	.end		= NOR_FLASH_PHYS + SZ_16M - 1,
	.flags		= IORESOURCE_MEM,
};

static struct platform_device davinci_evm_norflash_device = {
	.name		= "physmap-flash",
	.id		= 0,
	.dev		= {
		.platform_data	= &davinci_evm_norflash_data,
	},
	.num_resources	= 1,
	.resource	= &davinci_evm_norflash_resource,
};

#if defined(CONFIG_MTD_NAND_DAVINCI) || defined(CONFIG_MTD_NAND_DAVINCI_MODULE)
struct mtd_partition davinci_evm_nandflash_partition[] = {
	/* 5 MB space at the beginning for bootloader and kernel */
	{
		.name		= "NAND filesystem",
		.offset		= 5 * SZ_1M,
		.size		= MTDPART_SIZ_FULL,
		.mask_flags	= 0,
	}
};

static struct nand_platform_data davinci_evm_nandflash_data = {
	.parts		= davinci_evm_nandflash_partition,
	.nr_parts	= ARRAY_SIZE(davinci_evm_nandflash_partition),
};

static struct resource davinci_evm_nandflash_resource = {
	.start		= DAVINCI_ASYNC_EMIF_DATA_CE0_BASE,
	.end		= DAVINCI_ASYNC_EMIF_DATA_CE0_BASE + SZ_16K - 1,
	.flags		= IORESOURCE_MEM,
};

static struct platform_device davinci_evm_nandflash_device = {
	.name		= "davinci_nand",
	.id		= 0,
	.dev		= {
		.platform_data	= &davinci_evm_nandflash_data,
	},
	.num_resources	= 1,
	.resource	= &davinci_evm_nandflash_resource,
};
#endif

#if defined(CONFIG_FB_DAVINCI) || defined(CONFIG_FB_DAVINCI_MODULE)

static u64 davinci_fb_dma_mask = DMA_32BIT_MASK;

static struct platform_device davinci_fb_device = {
	.name		= "davincifb",
	.id		= -1,
	.dev = {
		.dma_mask		= &davinci_fb_dma_mask,
		.coherent_dma_mask      = DMA_32BIT_MASK,
	},
	.num_resources = 0,
};
#endif

/*
 * USB
 */
#if defined(CONFIG_USB_MUSB_HDRC) || defined(CONFIG_USB_MUSB_HDRC_MODULE)

#include <linux/usb/musb.h>

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
	.power          = 250,          /* sustains 3.0+ Amps (!) */
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
		.platform_data		= &usb_data,
		.dma_mask		= &usb_dmamask,
		.coherent_dma_mask      = DMA_32BIT_MASK,
        },
	.resource       = usb_resources,
	.num_resources  = ARRAY_SIZE(usb_resources),
};

#define setup_usb(void)	do {} while(0)
#endif  /* CONFIG_USB_MUSB_HDRC */

static struct platform_device rtc_dev = {
	.name           = "rtc_davinci_evm",
	.id             = -1,
};

static struct platform_device *davinci_evm_devices[] __initdata = {
	&davinci_evm_norflash_device,
#if defined(CONFIG_MTD_NAND_DAVINCI) || defined(CONFIG_MTD_NAND_DAVINCI_MODULE)
	&davinci_evm_nandflash_device,
#endif
#if defined(CONFIG_FB_DAVINCI) || defined(CONFIG_FB_DAVINCI_MODULE)
	&davinci_fb_device,
#endif
#if defined(CONFIG_USB_MUSB_HDRC) || defined(CONFIG_USB_MUSB_HDRC_MODULE)
	&usb_dev,
#endif
	&rtc_dev,
};

static void __init
davinci_evm_map_io(void)
{
	davinci_map_common_io();
}

static __init void davinci_evm_init(void)
{
	davinci_psc_init();

#if defined(CONFIG_BLK_DEV_DAVINCI) || defined(CONFIG_BLK_DEV_DAVINCI_MODULE)
	printk(KERN_WARNING "WARNING: both IDE and NOR flash are enabled, "
	       "but share pins.\n\t Disable IDE for NOR support.\n");
#endif

	platform_add_devices(davinci_evm_devices,
			     ARRAY_SIZE(davinci_evm_devices));

	setup_usb();
}

static __init void davinci_evm_irq_init(void)
{
	davinci_init_common_hw();
	davinci_irq_init();
}

MACHINE_START(DAVINCI_EVM, "DaVinci EVM")
	/* Maintainer: MontaVista Software <source@mvista.com> */
	.phys_io      = IO_PHYS,
	.io_pg_offst  = (io_p2v(IO_PHYS) >> 18) & 0xfffc,
	.boot_params  = (DAVINCI_DDR_BASE + 0x100),
	.map_io	      = davinci_evm_map_io,
	.init_irq     = davinci_evm_irq_init,
	.timer	      = &davinci_timer,
	.init_machine = davinci_evm_init,
MACHINE_END
