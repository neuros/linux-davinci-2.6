/*
 * drivers/i2c/chips/gpio_expander_davinci.c
 *
 * Author: Vladimir Barinov, MontaVista Software, Inc. <source@mvista.com>
 *
 * Copyright (C) 2006 Texas Instruments Inc
 *
 * 2007 (c) MontaVista Software, Inc. This file is licensed under
 * the terms of the GNU General Public License version 2. This program
 * is licensed "as is" without any warranty of any kind, whether express
 * or implied.
 */

#include <linux/types.h>
#include <linux/i2c.h>
#include <linux/errno.h>
#include <linux/mutex.h>

#include <asm/arch/i2c-client.h>

static DEFINE_MUTEX(expander_lock);

static int davinci_i2c_expander_read(u8 size, u8 * val, u16 addr)
{
	struct i2c_adapter *adap;
	int err;
	struct i2c_msg msg[1];

        adap = i2c_get_adapter(0);
        if (!adap)
		return -ENODEV;

	msg->addr = addr;
	msg->flags = I2C_M_RD;
	msg->len = size;
	msg->buf = val;

	err = i2c_transfer(adap, msg, 1);
	if (err >= 0)
		return 0;

	return err;
}

static int davinci_i2c_expander_write(u8 size, u8 * val, u16 addr)
{
	struct i2c_adapter *adap;
	int err;
	struct i2c_msg msg[1];

        adap = i2c_get_adapter(0);
        if (!adap)
		return -ENODEV;

	msg->addr = addr;
	msg->flags = 0;
	msg->len = size;
	msg->buf = val;

	err = i2c_transfer(adap, msg, 1);
	if (err >= 0)
		return 0;

	return err;
}

int davinci_i2c_expander_op(u16 client_addr, u35_expander_ops pin, u8 val)
{
	int err = 0;
	char cmd[4] = { 4, 6, 0x00, 0x09 };
	u8 data_to_u35 = 0;

	if (val > 1)
		return -1;

	mutex_lock(&expander_lock);

	err = davinci_i2c_expander_read(1, &data_to_u35, 0x3A);
	if (err < 0)
		return err;

	if (client_addr == 0x3A) {
		switch (pin) {
		case USB_DRVVBUS:
			if (val)
				data_to_u35 |= val;
			else
				data_to_u35 &= (val | 0xFE);
			break;
		case VDDIMX_EN:
			if (val)
				data_to_u35 |= (val << 1);
			else
				data_to_u35 &= (val | 0xFD);
			break;
		case VLYNQ_ON:
			if (val)
				data_to_u35 |= (val << 2);
			else
				data_to_u35 &= (val | 0xFB);
			break;
		case CF_RESET:
			if (val)
				data_to_u35 |= (val << 3);
			else
				data_to_u35 &= (val | 0xF7);
			break;
		case WLAN_RESET:
			if (val)
				data_to_u35 |= (val << 5);
			else
				data_to_u35 &= (val | 0xDF);
			break;
		case ATA_SEL:
			if (val)
				data_to_u35 |= (val << 6);
			else
				data_to_u35 &= (val | 0xBF);
			break;
		case CF_SEL:
			davinci_i2c_expander_write(4, cmd, 0x23);
			if (val)
				data_to_u35 |= (val << 7);
			else
				data_to_u35 &= (val | 0x7F);
			break;
		default:
			break;
		}
	} else {
		printk("Only IO Expander at address 0x3A is suuported\n");
		mutex_unlock(&expander_lock);
		return -EINVAL;
	}

	err = davinci_i2c_expander_write(1, &data_to_u35, 0x3A);

	mutex_unlock(&expander_lock);

	return err;
}
EXPORT_SYMBOL(davinci_i2c_expander_op);
