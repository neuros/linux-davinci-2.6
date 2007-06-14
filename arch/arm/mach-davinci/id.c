/*
 * linux/arch/arm/mach-davinci/id.c
 *
 * Davinci CPU identification code
 *
 * Copyright (C) 2006 Komal Shah <komal_shah802003@yahoo.com>
 *
 * Derived from OMAP1 CPU identification code.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>

#include <asm/io.h>

#define DAVINCI_DEV_ID		0x01c40028

#define DAVINCI_DM6443_CPU_ID	0x64430000
#define DAVINCI_DM6467_CPU_ID	0x64670000
#define DAVINCI_DM350_CPU_ID	0x03500000

struct davinci_id {
	u16	jtag_id;	/* Device Part No. (Unique JTAG id)*/
	u8	dev_rev;	/* Processor revision */
	u32	mfg_jtag_id;	/* Manufacturer JTAP id */
	u32	type;		/* Cpu id bits [31:08], cpu class bits [07:00] */
};

/* Register values to detect the DaVinci version */
static struct davinci_id davinci_ids[] __initdata = {
	{ .jtag_id = 0xb700, .dev_rev = 0x2, .mfg_jtag_id = 0x017, 
	  .type = DAVINCI_DM6443_CPU_ID }, /* DaVinci */

	{ .jtag_id = 0xb770, .dev_rev = 0x0, .mfg_jtag_id = 0x017, 
	  .type = DAVINCI_DM6467_CPU_ID }, /* DaVinci HD */

	{ .jtag_id = 0xb73b, .dev_rev = 0x0, .mfg_jtag_id = 0x00f, 
	  .type = DAVINCI_DM350_CPU_ID },
};

/*
 * Get Device Part No. from DEV_ID.
 */
static u16 __init davinci_get_jtag_id(void)
{
	u32 dev_id, jtag_id;

	dev_id = davinci_readl(DAVINCI_DEV_ID);

	jtag_id = ((dev_id >> 12) & 0xffff);

	return jtag_id;
}

/*
 * Get Device Revision from DEV_ID.
 */
static u8 __init davinci_get_dev_rev(void)
{
	u32 dev_rev;

	dev_rev = davinci_readl(DAVINCI_DEV_ID);

	dev_rev = (dev_rev >> 28) & 0xf;

	return dev_rev;
}

void __init davinci_check_revision(void)
{
	int i;
	u16 jtag_id;
	u8 dev_rev;

	jtag_id = davinci_get_jtag_id();
	dev_rev = davinci_get_dev_rev();

#ifdef DEBUG
	printk("JTAG_ID: 0x%04x DEV_REV: %i\n", jtag_id, dev_rev);
#endif

	/* First check only the major version in a safe way */
	for (i = 0; i < ARRAY_SIZE(davinci_ids); i++) {
		if (jtag_id == (davinci_ids[i].jtag_id)) {
			system_rev = davinci_ids[i].type;
			break;
		}
	}

	/* Check if we can find the dev revision */
	for (i = 0; i < ARRAY_SIZE(davinci_ids); i++) {
		if (jtag_id == davinci_ids[i].jtag_id &&
		    dev_rev == davinci_ids[i].dev_rev) {
			system_rev = davinci_ids[i].type;
			break;
		}
	}

	printk("DM%04x\n", system_rev >> 16);
}

