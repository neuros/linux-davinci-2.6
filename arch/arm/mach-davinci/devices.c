/*
 * linux/arch/arm/mach-davinci/devices.c
 *
 * DaVinci platform device setup/initialization
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include <linux/config.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>

#include <asm/hardware.h>
#include <asm/io.h>
#include <asm/mach-types.h>
#include <asm/mach/map.h>

#if 	defined(CONFIG_I2C_DAVINCI) || defined(CONFIG_I2C_DAVINCI_MODULE)

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
	.num_resources	= ARRAY_SIZE(i2c_resources),
	.resource	= i2c_resources,
};

static void davinci_init_i2c(void)
{
	(void) platform_device_register(&davinci_i2c_device);
}

#else

static void davinci_init_i2c(void) {}

#endif

/*-------------------------------------------------------------------------*/

static int __init davinci_init_devices(void)
{
	/* please keep these calls, and their implementations above,
	 * in alphabetical order so they're easier to sort through.
	 */
	davinci_init_i2c();

	return 0;
}
arch_initcall(davinci_init_devices);

