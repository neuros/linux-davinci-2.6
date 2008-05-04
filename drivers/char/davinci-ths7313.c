/* *
 * Copyright (C) 2008 Neuros Technology International LLC
 *
 * This program is free software you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; version 2 of the License.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not,write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307 USA
 */
/* davinci-ths7313.c file */

/*Header files*/

#include <linux/module.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <asm/arch/davinci-ths7313.h>

/* #define DEBUG */

#ifdef DEBUG
#define DPRINTK(x...)  do { \
		printk(KERN_INFO, "<%s>: ", __FUNCTION__); printk(x); \
	} while (0)

#define FN_IN  printk(KERN_INFO, "<%s> start:\n", __FUNCTION__)

#else
#define DPRINTK(x...)
#define FN_IN

#endif

static int ths7313_attach_adapter(struct i2c_adapter *adapter);
static int ths7313_detach_client(struct i2c_client *client);
static int ths7313_detect_client(struct i2c_adapter *adapter,
						int address, int kind);
static int ths7313_write_value(u8 reg, u16 value);
static int ths7313_read_value(u8 reg);

static void ths7313_configure(void);

static __init int ths7313_init(void);
static __exit void ths7313_exit(void);

static struct i2c_driver ths7313_driver = {
	.driver = {
		.name = "THS7313",
	},
	.id = I2C_DRIVERID_THS7313,
	.attach_adapter = ths7313_attach_adapter,
	.detach_client = ths7313_detach_client,
};

/* I2C Addresses to scan */
static unsigned short normal_i2c[] = { THS7313_I2C_ADDR, \
	I2C_CLIENT_END};

/* This makes all addr_data:s */
I2C_CLIENT_INSMOD;

static struct i2c_client *ths7313_client;


static int ths7313_read_value(u8 reg)
{
	return i2c_smbus_read_byte_data(ths7313_client, reg);
}

static int ths7313_write_value(u8 reg, u16 value)
{
	return i2c_smbus_write_byte_data(ths7313_client,
							reg, value);
}

int ths7313_set_input_mode(int channel, int mode)
{
	int origin;

	if (channel > 3 || channel < 0) {
		DPRINTK("Invalidate channel %d \n", channel);
		return -1;
	}

	origin = ths7313_read_value(channel);
	origin = (origin & ~0x07) | (mode & 0x07);
	if (ths7313_write_value(channel, origin)) {
		DPRINTK("ths7313_write_value failed\n");
		return -1;
	}
	DPRINTK("Channel %d set input mode : %x\n", channel, mode & 0x07);

	return 0;
}

int ths7313_set_input_mux(int channel, int select)
{
	int origin;

	if (channel > 3 || channel < 0) {
		DPRINTK("Invalidate channel %d \n", channel);
		return -1;
	}

	origin = ths7313_read_value(channel);
	origin = (origin & ~0x20) | (select & 0x20);
	if (ths7313_write_value(channel, origin)) {
		DPRINTK("ths7313_write_value failed\n");
		return -1;
	}
	DPRINTK("Channel %d set input mux : %x\n", channel, select & 0x20);

	return 0;
}

int ths7313_set_pass_filter(int channel, int filter)
{
	int origin;

	if (channel > 3 || channel < 0) {
		DPRINTK("Invalidate channel %d \n", channel);
		return -1;
	}

	origin = ths7313_read_value(channel);
	origin = (origin & ~0xC0) | (filter & 0xC0);
	if (ths7313_write_value(channel, origin)) {
		DPRINTK("ths7313_write_value failed\n");
		return -1;
	}
	DPRINTK("Channel %d set input mux : %x\n", channel, filter & 0xC0);

	return 0;
}

static void ths7313_configure(void)
{
	/* enable channel 1 and set mode as DC_BIAS_135MV*/
	ths7313_set_input_mode(CHANNEL1_REG, DC_BIAS_135MV);

	/* disbale channel 2 */
	ths7313_set_input_mode(CHANNEL2_REG, DISABLE_CHANNEL);

	/* disbale channel 3 */
	ths7313_set_input_mode(CHANNEL3_REG, DISABLE_CHANNEL);

	/* set input mux :input A select */
	ths7313_set_input_mux(CHANNEL1_REG, INPUT_A_SELECT);
}

static int ths7313_attach_adapter(struct i2c_adapter *adapter)
{
	int res;

	FN_IN;

	res = i2c_probe(adapter, &addr_data, &ths7313_detect_client);
	return res;
}

static int ths7313_detach_client(struct i2c_client *client)
{
	int err;

	FN_IN;

	err = i2c_detach_client(client);
	if (err) {
		DPRINTK("Client deregistration failed, \
		       client not detached.\n");
		return err;
	}
	kfree(client);

	return 0;
}

static int ths7313_detect_client(struct i2c_adapter *adapter,
					int address, int kind)
{
	int err = 0;
	const char *client_name = "THS7313 Video Amplifier";

	FN_IN;

	if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_WORD_DATA |
				     I2C_FUNC_SMBUS_WRITE_BYTE)) {
		DPRINTK("Functinality check failed for %s \n",
				client_name);
		return err;
    }

	ths7313_client = kmalloc(sizeof(struct i2c_client),
							 GFP_KERNEL);
	if (ths7313_client == NULL) {
		err = -ENOMEM;
		DPRINTK("Couldn't allocate memory for %s\n",
				client_name);
		return err;
	}

	memset(ths7313_client, 0x00, sizeof(struct i2c_client));
	ths7313_client->addr = address;
	ths7313_client->adapter = adapter;
	ths7313_client->driver = &ths7313_driver;
	ths7313_client->flags = 0;
	strlcpy(ths7313_client->name, client_name, I2C_NAME_SIZE);

	err = i2c_attach_client(ths7313_client);
	if (err) {
		DPRINTK("Couldn't attach %s\n", client_name);
		kfree(ths7313_client);
		return err;
	}

	return 0;
}

static __init int ths7313_init(void)
{
	FN_IN;

	if (i2c_add_driver(&ths7313_driver)) {
		DPRINTK("Driver registration failed, \
		      module not inserted.\n");
		return -ENODEV;
	}

	ths7313_configure();

	return 0;
}

static __exit void ths7313_exit(void)
{
	FN_IN;

	i2c_del_driver(&ths7313_driver);
}

module_init(ths7313_init);
module_exit(ths7313_exit);

EXPORT_SYMBOL(ths7313_set_input_mode);
EXPORT_SYMBOL(ths7313_set_input_mux);
EXPORT_SYMBOL(ths7313_set_pass_filter);

MODULE_DESCRIPTION("THS7313 SDTV Video Amplifier Driver");
MODULE_AUTHOR("Neuros Technology International LLC");
MODULE_LICENSE("GPL");


