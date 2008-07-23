/*
 * Neuros OSD 644xA board support
 *
 * Modified from original EVM board support (see below).
 * ----------------------------------------- 03/06, 2008 <mgao@neuros>
 * 2008 (c) Neuros Technology, LLC.
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

#include <video/davincifb.h>

/* other misc. init functions */
void __init davinci_psc_init(void);
void __init davinci_irq_init(void);
void __init davinci_map_common_io(void);
void __init davinci_init_common_hw(void);

/* NOR Flash base address set to CS0 by default */
#define NOR_FLASH_PHYS 0x02000000

static struct mtd_partition ntosd_644xa_norflash_partitions[] = {
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

static struct physmap_flash_data ntosd_644xa_norflash_data = {
	.width		= 2,
	.parts		= ntosd_644xa_norflash_partitions,
	.nr_parts	= ARRAY_SIZE(ntosd_644xa_norflash_partitions),
};

/* NOTE: CFI probe will correctly detect flash part as 32M, but EMIF
 * limits addresses to 16M, so using addresses past 16M will wrap */
static struct resource ntosd_644xa_norflash_resource = {
	.start		= NOR_FLASH_PHYS,
	.end		= NOR_FLASH_PHYS + SZ_16M - 1,
	.flags		= IORESOURCE_MEM,
};

static struct platform_device ntosd_644xa_norflash_device = {
	.name		= "physmap-flash",
	.id		= 0,
	.dev		= {
		.platform_data	= &ntosd_644xa_norflash_data,
	},
	.num_resources	= 1,
	.resource	= &ntosd_644xa_norflash_resource,
};

#if defined(CONFIG_MTD_NAND_DAVINCI) || defined(CONFIG_MTD_NAND_DAVINCI_MODULE)
#define UBOOT_START 0xc0000
struct mtd_partition ntosd_644xa_nandflash_partition[] = {
	/* uboot parameter */
	{
		.name		= "u-boot-parameter",
		.offset		= 0,
		.size		= 1 * SZ_128K,
		.mask_flags	= 0,
	},
	/* ubl */
	{
		.name		= "ubl",
		.offset		= 1 * SZ_128K,
		.size		= 5 * SZ_128K,
		.mask_flags	= 0,
	},
	/* 1 MB space from bootloader start for bootloader */
	{
		.name		= "u-boot",
		.offset		= UBOOT_START,
		.size		= 1 * SZ_1M,
		.mask_flags	= 0,
	},
	/* 5 MB space for kernel */
	{
		.name		= "kernel",
		.offset		= 1 * SZ_1M + UBOOT_START,
		.size		= 5 * SZ_1M,
		.mask_flags	= 0,
	},
	/* the rest for rootfs */
	{
		.name		= "NAND filesystem",
		.offset		= 6 * SZ_1M + UBOOT_START,
		.size		= MTDPART_SIZ_FULL,
		.mask_flags	= 0,
	}
};

static struct nand_platform_data ntosd_644xa_nandflash_data = {
	.parts		= ntosd_644xa_nandflash_partition,
	.nr_parts	= ARRAY_SIZE(ntosd_644xa_nandflash_partition),
};

static struct resource ntosd_644xa_nandflash_resource = {
	.start		= DAVINCI_ASYNC_EMIF_DATA_CE0_BASE,
	.end		= DAVINCI_ASYNC_EMIF_DATA_CE0_BASE + SZ_16K - 1,
	.flags		= IORESOURCE_MEM,
};

static struct platform_device ntosd_644xa_nandflash_device = {
	.name		= "davinci_nand",
	.id		= 0,
	.dev		= {
		.platform_data	= &ntosd_644xa_nandflash_data,
	},
	.num_resources	= 1,
	.resource	= &ntosd_644xa_nandflash_resource,
};
#endif

#if defined(CONFIG_FB_DAVINCI) || defined(CONFIG_FB_DAVINCI_MODULE) || \
defined(CONFIG_FB_DM) || defined(CONFIG_FB_DM_MODULE)

static struct davincifb_mach_info davincifb_mach_info = {
	.size[DAVINCIFB_WIN_VID0] = 12441600, /* 1920x1080*16bpp*3buffers */
	.size[DAVINCIFB_WIN_VID1] = 691200, /* 720x480*16bpp*1buffer */
	.size[DAVINCIFB_WIN_OSD0] = 3686400, /* 1280x720*16bpp*2buffers */
	.size[DAVINCIFB_WIN_OSD1] = 184320, /* 768x480*4bpp*1buffer */
};

static u64 davinci_fb_dma_mask = DMA_32BIT_MASK;

static struct platform_device davinci_fb_device = {
	.name		= "davincifb",
	.id		= -1,
	.dev = {
		.dma_mask		= &davinci_fb_dma_mask,
		.coherent_dma_mask      = DMA_32BIT_MASK,
		.platform_data 		= &davincifb_mach_info,
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
	.name           = "rtc_ntosd_644xa",
	.id             = -1,
};

static struct platform_device *ntosd_644xa_devices[] __initdata = {
	&ntosd_644xa_norflash_device,
#if defined(CONFIG_MTD_NAND_DAVINCI) || defined(CONFIG_MTD_NAND_DAVINCI_MODULE)
	&ntosd_644xa_nandflash_device,
#endif
#if defined(CONFIG_FB_DAVINCI) || defined(CONFIG_FB_DAVINCI_MODULE) || \
defined(CONFIG_FB_DM) || defined(CONFIG_FB_DM_MODULE)
	&davinci_fb_device,
#endif
#if defined(CONFIG_USB_MUSB_HDRC) || defined(CONFIG_USB_MUSB_HDRC_MODULE)
	&usb_dev,
#endif
	&rtc_dev,
};

static void __init
ntosd_644xa_map_io(void)
{
	davinci_map_common_io();
}

static __init void ntosd_644xa_init(void)
{
	davinci_psc_init();

#if defined(CONFIG_BLK_DEV_DAVINCI) || defined(CONFIG_BLK_DEV_DAVINCI_MODULE)
	printk(KERN_WARNING "WARNING: both IDE and NOR flash are enabled, "
	       "but share pins.\n\t Disable IDE for NOR support.\n");
#endif

	platform_add_devices(ntosd_644xa_devices,
			     ARRAY_SIZE(ntosd_644xa_devices));

	setup_usb();
}

static __init void ntosd_644xa_irq_init(void)
{
	davinci_init_common_hw();
	davinci_irq_init();
}

MACHINE_START(NTOSD_644XA, "Neuros OSD 644x Revision A")
	/* Maintainer: Neuros Technology, LLC */
	.phys_io      = IO_PHYS,
	.io_pg_offst  = (io_p2v(IO_PHYS) >> 18) & 0xfffc,
	.boot_params  = (DAVINCI_DDR_BASE + 0x100),
	.map_io	      = ntosd_644xa_map_io,
	.init_irq     = ntosd_644xa_irq_init,
	.timer	      = &davinci_timer,
	.init_machine = ntosd_644xa_init,
MACHINE_END
