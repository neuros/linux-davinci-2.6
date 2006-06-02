/*
 *  linux/drivers/davinci/i2c-davinci-client.c
 *
 * Copyright (C) 2006 Texas Instruments Inc
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
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#include <linux/config.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/vmalloc.h>
#include <linux/slab.h>
#include <linux/proc_fs.h>
#include <linux/ctype.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/clk.h>

#include <asm/semaphore.h>
#include <asm/arch/i2c-client.h>

static unsigned long initialized;
static struct semaphore expander_sema;
static struct i2c_client *client_handle;

/* This function is used for internal initialization */
int davinci_i2c_read(u8 size, u8 * val, u16 client_addr)
{
	int err;
	struct i2c_client *client = client_handle;

	struct i2c_msg msg[1];

	if (!client->adapter)
		return -ENODEV;

	msg->addr = client_addr;
	msg->flags = I2C_M_RD;
	msg->len = size;
	msg->buf = val;

	err = i2c_transfer(client->adapter, msg, 1);

	if (err >= 0) {
		return 0;
	}

	return err;
}

EXPORT_SYMBOL(davinci_i2c_read);

/* This function is used for internal initialization */
int davinci_i2c_write(u8 size, u8 * val, u16 client_addr)
{
	int err;
	struct i2c_client *client = client_handle;

	struct i2c_msg msg[1];

	if (!client->adapter)
		return -ENODEV;

	msg->addr = client_addr;
	msg->flags = 0;
	msg->len = size;
	msg->buf = val;

	err = i2c_transfer(client->adapter, msg, 1);
	if (err >= 0)
		return 0;

	return err;
}

EXPORT_SYMBOL(davinci_i2c_write);

int davinci_i2c_expander_op(u16 client_addr, u35_expander_ops pin, u8 val)
{
	int err = 0;
	char cmd[4] = { 4, 6, 0x00, 0x09 };
	u8 data_to_u35 = 0;

	if (val > 1)
		return -1;

	down(&expander_sema);

	err = davinci_i2c_read(1, &data_to_u35, 0x3A);

	if (client_addr == 0x3A) {
		switch (pin) {
		case USB_DRVVBUS:
			if (val)
				data_to_u35 |= val;
			else {
				data_to_u35 &= (val | 0xFE);
			}
			break;
		case VDDIMX_EN:
			if (val)
				data_to_u35 |= (val << 1);
			else {
				data_to_u35 &= (val | 0xFD);
			}
			break;
		case VLYNQ_ON:
			if (val)
				data_to_u35 |= (val << 2);
			else {
				data_to_u35 &= (val | 0xFB);
			}
			break;
		case CF_RESET:
			if (val)
				data_to_u35 |= (val << 3);
			else {
				data_to_u35 &= (val | 0xF7);
			}
			break;
		case WLAN_RESET:
			if (val)
				data_to_u35 |= (val << 5);
			else {
				data_to_u35 &= (val | 0xDF);
			}
			break;
		case ATA_SEL:
			if (val)
				data_to_u35 |= (val << 6);
			else {
				data_to_u35 &= (val | 0xBF);
			}
			break;
		case CF_SEL:
			davinci_i2c_write(4, cmd, 0x23);
			if (val)
				data_to_u35 |= (val << 7);
			else {
				data_to_u35 &= (val | 0x7F);
			}
			break;
		default:
			break;
		}
	} else {
		printk("Only IO Expander at address 0x3A is suuported\n");
		up(&expander_sema);
		return -1;
	}

	err = davinci_i2c_write(1, &data_to_u35, 0x3A);

	up(&expander_sema);

	return err;
}

EXPORT_SYMBOL(davinci_i2c_expander_op);

static struct i2c_driver davinci_i2c_client_driver;

static int davinci_i2c_attach_client(struct i2c_adapter *adap, int addr)
{
	struct i2c_client *client;
	int err;
	u8 data_to_u35 = 0xf6;

	if (!(client = kzalloc(sizeof(struct i2c_client), GFP_KERNEL))) {
		err = -ENOMEM;
		goto exit;
	}
	client_handle = client;

	if (client->adapter)
		return -EBUSY;	/* our client is already attached */

	client->addr = addr;
	client->flags = 0;
	client->driver = &davinci_i2c_client_driver;
	client->adapter = adap;
	strlcpy(client->name, "i2c_davinci_client", I2C_NAME_SIZE);

	err = i2c_attach_client(client);
	if (err) {
		client->adapter = NULL;
		goto exit_kfree;
	}

	err = davinci_i2c_write(1, &data_to_u35, 0x3A);

	return 0;

 exit_kfree:
	kfree(client);
 exit:
	return err;
}

static int davinci_i2c_detach_client(struct i2c_client *client)
{
	int err;

	if (!client->adapter)
		return -ENODEV;	/* our client isn't attached */

	err = i2c_detach_client(client);
	client->adapter = NULL;
	return err;
}

static int davinci_i2c_probe_adapter(struct i2c_adapter *adap)
{
	return davinci_i2c_attach_client(adap, 0x3A);
}

/* This is the driver that will be inserted */
static struct i2c_driver davinci_i2c_client_driver = {
	.driver = {
		.name	= "davinci_i2c_client",
	},
	.attach_adapter	= davinci_i2c_probe_adapter,
	.detach_client	= davinci_i2c_detach_client,
};
  
static int __init davinci_i2c_client_init(void)
{
	return i2c_add_driver(&davinci_i2c_client_driver);
}
  
static void __exit davinci_i2c_client_exit(void)
{
	i2c_del_driver(&davinci_i2c_client_driver);
}
  
module_init(davinci_i2c_client_init);
module_exit(davinci_i2c_client_exit);
