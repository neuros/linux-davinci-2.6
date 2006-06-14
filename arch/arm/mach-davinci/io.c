/*
 * linux/arch/arm/mach-davinci/io.c
 *
 * DaVinci I/O mapping code
 *
 * Copyright (C) 2005-2006 Texas Instruments
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/config.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>

#include <asm/io.h>

#include <asm/mach/map.h>
#include <asm/arch/memory.h>

/*
 * The machine specific code may provide the extra mapping besides the
 * default mapping provided here.
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
	},
};

void __init davinci_map_common_io(void)
{
	iotable_init(davinci_io_desc, ARRAY_SIZE(davinci_io_desc));
}

void __init davinci_init_common_hw(void)
{
	davinci_clk_init();
}
