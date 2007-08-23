/*
 * <arch/arm/mach-davinci/i2c-emac.c
 *
 * Read MAC address from i2c-attached EEPROM
 * FIXME: Move into network driver once stabilized
 *
 * Author: Texas Instruments
 *
 * 2006 (c) Texas Instruments, Inc. This file is licensed under
 * the terms of the GNU General Public License version 2. This program
 * is licensed "as is" without any warranty of any kind, whether express
 * or implied.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/string.h>
#include <linux/errno.h>

#include <asm/arch/i2c-client.h>

/* Get Ethernet address from kernel boot params */
static unsigned char cpmac_eth_string[20] = "deadbeaf";

/* This function gets the Ethernet MAC address from EEPROM
 * Input buffer to be of atlease 20 bytes in length
 */
int davinci_get_macaddr (char *ptr)
{
#ifndef CONFIG_I2C_DAVINCI
	printk(KERN_INFO "DaVinci EMAC: Unable to read MAC from EEPROM, "
	       "no i2c support in kernel.\n");
#else
	char data[2] = { 0x7f, 0 };
        char temp[20];
        int  i = 0;

	if (ptr == NULL) {
		return -EFAULT;
	}

	davinci_i2c_write (2, data, 0x50);
	davinci_i2c_read (8, temp, 0x50);

	/* check whether MAC address is available in ERPROM else try to
	 * to get it from bootparams for now.  From Delta EVM MAC address
	 * should be available from I2C EEPROM.
	 */
	if ((temp [0] != 0xFF) |
	    (temp [1] != 0xFF) |
	    (temp [2] != 0xFF) |
	    (temp [3] != 0xFF) |
	    (temp [4] != 0xFF) |
	    (temp [5] != 0xFF) |
	    (temp [6] != 0xFF) )
	{
		ptr[0] = (*(temp+0) & 0xF0) >> 4;
		ptr[1] = (*(temp+0) & 0x0F);
		ptr[2] = ':';
		ptr[3] = (*(temp+1) & 0xF0) >> 4;
		ptr[4] = (*(temp+1) & 0x0F);
		ptr[5] = ':';
		ptr[6] = (*(temp+2) & 0xF0) >> 4;
		ptr[7] = (*(temp+2) & 0x0F);
		ptr[8] = ':';
		ptr[9] = (*(temp+3) & 0xF0) >> 4;
		ptr[10]= (*(temp+3) & 0x0F);
		ptr[11]= ':';
		ptr[12]= (*(temp+4) & 0xF0) >> 4;
		ptr[13]= (*(temp+4) & 0x0F);
		ptr[14]= ':';
		ptr[15]= (*(temp+5) & 0xF0) >> 4;
		ptr[16]= (*(temp+5) & 0x0F);

		for (i = 0; i < 17; i++)
		{
			if (ptr[i] == ':')
				continue;
			else if (ptr[i] <= 9)
				ptr[i] = ptr[i] + 48;
			else
				ptr[i] = ptr[i] + 87;
		}
	} else
#endif
	{
		strcpy (ptr, cpmac_eth_string);
	}
	return 0;
}
EXPORT_SYMBOL(davinci_get_macaddr);

static int davinci_cpmac_eth_setup(char *str)
{
	/* The first char passed from the bootloader is '=', so ignore it */
        strcpy(&cpmac_eth_string[0], &str[1]);

        printk("TI DaVinci EMAC: Kernel Boot params Eth address: %s\n",
               cpmac_eth_string);

        return (1);
}
__setup("eth", davinci_cpmac_eth_setup);

