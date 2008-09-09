/*
 *  Copyright(C) 2006-2008 Neuros Technology International LLC. 
 *               <www.neurostechnology.com>
 *
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, version 2 of the License.
 *
 *  This program is distributed in the hope that, in addition to its 
 *  original purpose to support Neuros hardware, it will be useful 
 *  otherwise, but WITHOUT ANY WARRANTY; without even the implied 
 *  warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  
 *  See the GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
 *
 ****************************************************************************
 *
 * tvp7000 driver. 
 *
 */

#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <media/tvp7000.h>
#include <media/v4l2-common.h>
#include <media/davinci_vpfe.h>
#include <asm/arch/gpio.h>

#define DEBUG
#ifdef DEBUG
#define DPRINTK(x...)  do { \
		printk(KERN_INFO "<%s>: ", __FUNCTION__); printk(x); \
	} while (0)

#define FN_IN  printk(KERN_INFO "<%s> start:\n", __FUNCTION__)

#else
#define DPRINTK(x...)
#define FN_IN

#endif

#define HD_CAP_GPIO		GPIO(39)
#define NUM_OF_REGS(x) \
	(sizeof(x) / sizeof(*x))
#define STD(x)  \
    ((x) < TOTAL_STANDARD ? video_std + (x) : NULL)
#define TVP7000_I2C_RETRY 3
#define INIT_DEFAULT 0

/* I2C Addresses to scan */
static unsigned short normal_i2c[] = {
	0xB8 >> 1,
	0xB9 >> 1,
	I2C_CLIENT_END};

/* This makes all addr_data:s */
I2C_CLIENT_INSMOD;

static struct i2c_client *tvp7000_client;

MODULE_DESCRIPTION("Texas Instruments TVP7000 video decoder driver");
MODULE_AUTHOR("Neuros Technology LLC");
MODULE_LICENSE("GPL");

struct i2c_reg_value
{
	u8 reg;
	u8 value;
};

static const struct i2c_reg_value tvp7000_init_component[] = {
	{ /* 0x01 */
		TVP7000_PLL_DIVIDE_MSB, 0x6b
	},
	{ /* 0x02 */
		TVP7000_PLL_DIVIDE_LSB, 0x40
	},
	{ /* 0x03 */
		TVP7000_PLL_CTRL, 0x68
	},
	{ /* 0x04 */
		TVP7000_PHASE_SELECT, 0xb1
	},
	{ /* 0x05 */
		TVP7000_CLAMP_START, 0x06
	},
	{ /* 0x06 */
		TVP7000_CLAMP_WIDTH, 0x10
	},
	{ /* 0x07 */
		TVP7000_HSYNC_OUTPUT_WIDTH, 0x60
	},
	{ /* 0x08 */
		TVP7000_BLUE_FINE_GAIN, 0x80
	},
	{ /* 0x09 */
		TVP7000_GREEN_FINE_GAIN, 0x80
	},
	{ /* 0x0A */
		TVP7000_RED_FINE_GAIN, 0x80
	},
	{ /* 0x0B */
		TVP7000_BLUE_FINE_OFFSET, 0x80
	},
	{ /* 0x0C */
		TVP7000_GREEN_FINE_OFFSET, 0x80
	},
	{ /* 0x0D */
		TVP7000_RED_FINE_OFFSET, 0x80
	},
	{ /* 0x0E */
		TVP7000_SYNC_CTRL_1, 0x20
	},
	{ /* 0x0F */
		TVP7000_PLL_CLAMP_CTRL, 0x2E
	},
	{ /* 0x10 */
		TVP7000_SYNC_ON_GREEN, 0x85
	},
	{ /* 0x11 */
		TVP7000_SYNC_SEPARATOR, 0x47
	},
	{ /* 0x12 */
		TVP7000_PRE_COAST, 0x03
	},
	{ /* 0x13 */
		TVP7000_POST_COAST, 0x0c
	},
	{ /* 0x15 */
		TVP7000_OUTPUT_FORMATTER, 0x02
	},
	{ /* 0x16 */
		TVP7000_TEST_REG, 0xe5
	},
	{ /* 0x19 */
		TVP7000_INPUT_MUX_1, 0x00
	},
	{ /* 0x1A */
		TVP7000_INPUT_MUX_2, 0x85
	},
	{ /* 0x1B */
		TVP7000_BLUE_GREEN_GAIN, 0x55
	},
	{ /* 0x1C */
		TVP7000_RED_COARSE_GAIN, 0x05
	},
	{ /* 0x1D */
		TVP7000_FINE_OFFSET_LSB, 0x00
	},
	{ /* 0x1E */
		TVP7000_BLUE_COARSE_OFFSET, 0x1f
	},
	{ /* 0x1F */
		TVP7000_GREEN_COARSE_OFFSET, 0x1f
	},
	{ /* 0x20 */
		TVP7000_RED_COARSE_OFFSET, 0x1f
	},
	{ /* 0x21 */
		TVP7000_HSOUT_OUTPUT_START, 0x08
	},
	{ /* 0x22 */
		TVP7000_MISC_CTRL, 0x08
	},
	{ /* 0x26 */
		TVP7000_AUTO_CTRL_ENABLE, 0x80
	},
	{ /* 0x28 */
		TVP7000_AUTO_CTRL_FILTER, 0x03
	},
	{ /* 0x2A */
		TVP7000_FINE_CLAMP_CTRL, 0x07
	},
	{ /* 0x2B */
		TVP7000_POWER_CTRL, 0x00
	},
	{ /* 0x2C */
		TVP7000_ADC_SETUP, 0x50
	},
	{ /* 0x2D */
		TVP7000_COARSE_CLAMP_CTRL_1, 0x00
	},
	{ /* 0x2E */
		TVP7000_SOG_CLAMP, 0x80
	},
	{ /* 0x31 */
		TVP7000_ALC_PLACEMENT, 0x18
	},
};

#if INIT_DEFAULT
static const struct i2c_reg_value tvp7000_init_default[] = {
	{ /* 0x01 */
		TVP7000_PLL_DIVIDE_MSB, 0x69
	},
	{ /* 0x02 */
		TVP7000_PLL_DIVIDE_LSB, 0xD0
	},
	{ /* 0x03 */
		TVP7000_PLL_CTRL, 0x48
	},
	{ /* 0x04 */
		TVP7000_PHASE_SELECT, 0x80
	},
	{ /* 0x05 */
		TVP7000_CLAMP_START, 0x80
	},
	{ /* 0x06 */
		TVP7000_CLAMP_WIDTH, 0x80
	},
	{ /* 0x07 */
		TVP7000_HSYNC_OUTPUT_WIDTH, 0x20
	},
	{ /* 0x08 */
		TVP7000_BLUE_FINE_GAIN, 0x80
	},
	{ /* 0x09 */
		TVP7000_GREEN_FINE_GAIN, 0x80
	},
	{ /* 0x0A */
		TVP7000_RED_FINE_GAIN, 0x80
	},
	{ /* 0x0B */
		TVP7000_BLUE_FINE_OFFSET, 0x80
	},
	{ /* 0x0C */
		TVP7000_GREEN_FINE_OFFSET, 0x80
	},
	{ /* 0x0D */
		TVP7000_RED_FINE_OFFSET, 0x80
	},
	{ /* 0x0E */
		TVP7000_SYNC_CTRL_1, 0x40
	},
	{ /* 0x0F */
		TVP7000_PLL_CLAMP_CTRL, 0x4E
	},
	{ /* 0x10 */
		TVP7000_SYNC_ON_GREEN, 0xB8
	},
	{ /* 0x11 */
		TVP7000_SYNC_SEPARATOR, 0x20
	},
	{ /* 0x12 */
		TVP7000_PRE_COAST, 0x00
	},
	{ /* 0x13 */
		TVP7000_POST_COAST, 0x00
	},
	{ /* 0x15 */
		TVP7000_OUTPUT_FORMATTER, 0x00
	},
	{ /* 0x16 */
		TVP7000_TEST_REG, 0x00
	},
	{ /* 0x19 */
		TVP7000_INPUT_MUX_1, 0x00
	},
	{ /* 0x1A */
		TVP7000_INPUT_MUX_2, 0x00
	},
	{ /* 0x1B */
		TVP7000_BLUE_GREEN_GAIN, 0x55
	},
	{ /* 0x1C */
		TVP7000_RED_COARSE_GAIN, 0x05
	},
	{ /* 0x1D */
		TVP7000_FINE_OFFSET_LSB, 0x00
	},
	{ /* 0x1E */
		TVP7000_BLUE_COARSE_OFFSET, 0x20
	},
	{ /* 0x1F */
		TVP7000_GREEN_COARSE_OFFSET, 0x20
	},
	{ /* 0x20 */
		TVP7000_RED_COARSE_OFFSET, 0x20
	},
	{ /* 0x21 */
		TVP7000_HSOUT_OUTPUT_START, 0x09
	},
	{ /* 0x22 */
		TVP7000_MISC_CTRL, 0x00
	},
	{ /* 0x26 */
		TVP7000_AUTO_CTRL_ENABLE, 0x00
	},
	{ /* 0x28 */
		TVP7000_AUTO_CTRL_FILTER, 0x00
	},
	{ /* 0x2A */
		TVP7000_FINE_CLAMP_CTRL, 0x00
	},
	{ /* 0x2B */
		TVP7000_POWER_CTRL, 0x00
	},
	{ /* 0x2C */
		TVP7000_ADC_SETUP, 0x00
	},
	{ /* 0x2D */
		TVP7000_COARSE_CLAMP_CTRL_1, 0x00
	},
	{ /* 0x2E */
		TVP7000_SOG_CLAMP, 0x00
	},
	{ /* 0x31 */
		TVP7000_ALC_PLACEMENT, 0x00
	},
};
#endif

struct tvp7000_video_std
{
	u8 plldiv_msb;
	u8 plldiv_lsb;
	u8 pll_ctrl;
	u8 phase_select_bit0;
	u8 pre_coast;
	u8 post_coast;
	u8 clamp_start;
	u8 clamp_width;
	u8 alc_placement;
};

static const struct tvp7000_video_std video_std[] = {
	{ /* VGA standard: 640*480 resolution,60HZ refresh rate,
			31.5kHZ Horizontal frequency,25.175MHZ pixel rate,
		*/
		.plldiv_msb = 0x64,
		.plldiv_lsb = 0x00,
		.pll_ctrl = 0x68,
		.phase_select_bit0 = 0x01,
	},
	{ /* VGA standard: 640*480 resolution,72HZ refresh rate,
			37.9kHZ Horizontal frequency,31.5MHZ pixel rate,
		*/
		.plldiv_msb = 0x68,
		.plldiv_lsb = 0x00,
		.pll_ctrl = 0x58,
		.phase_select_bit0 = 0x01,
	},
	{ /* VGA standard: 640*480 resolution,75HZ refresh rate,
			37.5kHZ Horizontal frequency,31.5MHZ pixel rate,
		*/
		.plldiv_msb = 0x69,
		.plldiv_lsb = 0x00,
		.pll_ctrl = 0x58,
		.phase_select_bit0 = 0x01,
	},
	{ /* VGA standard: 640*480 resolution,85HZ refresh rate,
			43.3kHZ Horizontal frequency,36MHZ pixel rate,
		*/
		.plldiv_msb = 0x34,
		.plldiv_lsb = 0x00,
		.pll_ctrl = 0x68,
		.phase_select_bit0 = 0x00,
	},
	{ /* SVGA standard: 800*600 resolution,56HZ refresh rate,
			35.1kHZ Horizontal frequency,36MHZ pixel rate,
		*/
		.plldiv_msb = 0x40,
		.plldiv_lsb = 0x00,
		.pll_ctrl = 0x68,
		.phase_select_bit0 = 0x00,
	},
	{ /* SVGA standard: 800*600 resolution,60HZ refresh rate,
			37.9kHZ Horizontal frequency,40MHZ pixel rate,
		*/
		.plldiv_msb = 0x42,
		.plldiv_lsb = 0x00,
		.pll_ctrl = 0x68,
		.phase_select_bit0 = 0x00,
	},
	{ /* SVGA standard: 800*600 resolution,72HZ refresh rate,
			48.1kHZ Horizontal frequency,50MHZ pixel rate,
		*/
		.plldiv_msb = 0x41,
		.plldiv_lsb = 0x00,
		.pll_ctrl = 0x68,
		.phase_select_bit0 = 0x00,
	},
	{ /* SVGA standard: 800*600 resolution,75HZ refresh rate,
			46.9kHZ Horizontal frequency,49.5MHZ pixel rate,
		*/
		.plldiv_msb = 0x42,
		.plldiv_lsb = 0x00,
		.pll_ctrl = 0x68,
		.phase_select_bit0 = 0x00,
	},
	{ /* SVGA standard: 800*600 resolution,85HZ refresh rate,
			53.7kHZ Horizontal frequency,56.25MHZ pixel rate,
		*/
		.plldiv_msb = 0x41,
		.plldiv_lsb = 0x80,
		.pll_ctrl = 0x68,
		.phase_select_bit0 = 0x00,
	},
	{ /* XGA standard: 1024*768 resolution,60HZ refresh rate,
			48.4kHZ Horizontal frequency,65MHZ pixel rate,
		*/
		.plldiv_msb = 0x54,
		.plldiv_lsb = 0x00,
		.pll_ctrl = 0x58,
		.phase_select_bit0 = 0x00,
	},
	{ /* XGA standard: 1024*768 resolution,70HZ refresh rate,
			56.5kHZ Horizontal frequency,75MHZ pixel rate,
		*/
		.plldiv_msb = 0x53,
		.plldiv_lsb = 0x00,
		.pll_ctrl = 0xA8,
		.phase_select_bit0 = 0x00,
	},
	{ /* XGA standard: 1024*768 resolution,75HZ refresh rate,
			60kHZ Horizontal frequency,78.75MHZ pixel rate,
		*/
		.plldiv_msb = 0x52,
		.plldiv_lsb = 0x00,
		.pll_ctrl = 0xA8,
		.phase_select_bit0 = 0x00,
	},
	{ /* XGA standard: 1024*768 resolution,85HZ refresh rate,
			68.7kHZ Horizontal frequency,94.5MHZ pixel rate,
		*/
		.plldiv_msb = 0x56,
		.plldiv_lsb = 0x00,
		.pll_ctrl = 0xA8,
		.phase_select_bit0 = 0x00,
	},
	{ /* SXGA standard: 1280*1024 resolution,60HZ refresh rate,
			64kHZ Horizontal frequency,108MHZ pixel rate,
		*/
		.plldiv_msb = 0x69,
		.plldiv_lsb = 0x80,
		.pll_ctrl = 0xA8,
		.phase_select_bit0 = 0x00,
	},
	{ /* SXGA standard: 1280*1024 resolution,75HZ refresh rate,
			80kHZ Horizontal frequency,135MHZ pixel rate,
		*/
		.plldiv_msb = 0x69,
		.plldiv_lsb = 0x80,
		.pll_ctrl = 0x98,
		.phase_select_bit0 = 0x00,
	},
	{ /* Video standard: 720*480p resolution,60HZ refresh rate,
			31.468kHZ Horizontal frequency,27MHZ pixel rate.
		*/
		.plldiv_msb = 0x6B,
		.plldiv_lsb = 0x40,
		.pll_ctrl = 0x68,
		.phase_select_bit0 = 0x01,
		.pre_coast = 0x03,
		.post_coast = 0x0c,
		.clamp_start = 0x06,
		.clamp_width = 0x10,
		.alc_placement = 0x18,
	},
	{ /* Video standard: 720*576p resolution,50HZ refresh rate,
			31.25kHZ Horizontal frequency,27MHZ pixel rate.
		*/
		.plldiv_msb = 0x6C,
		.plldiv_lsb = 0x00,
		.pll_ctrl = 0x68,
		.phase_select_bit0 = 0x01,
		.pre_coast = 0x03,
		.post_coast = 0x0c,
		.clamp_start = 0x06,
		.clamp_width = 0x10,
		.alc_placement = 0x18,
	},
	{ /* Video standard: 1280*720p resolution,60HZ refresh rate,
			45kHZ Horizontal frequency,74.25MHZ pixel rate.
		*/
		.plldiv_msb = 0x67,
		.plldiv_lsb = 0x20,
		.pll_ctrl = 0xA8,
		.phase_select_bit0 = 0x00,
		.pre_coast = 0x0,
		.post_coast = 0x0,
		.clamp_start = 0x32,
		.clamp_width = 0x20,
		.alc_placement = 0x5a,
	},
	{ /* Video standard: 1280*720p resolution,50HZ refresh rate,
			37.5kHZ Horizontal frequency,74.25MHZ pixel rate.
		*/
		.plldiv_msb = 0x7B,
		.plldiv_lsb = 0xC0,
		.pll_ctrl = 0xA8,
		.phase_select_bit0 = 0x00,
		.pre_coast = 0x0,
		.post_coast = 0x0,
		.clamp_start = 0x32,
		.clamp_width = 0x20,
		.alc_placement = 0x5a,
	},
	{ /* Video standard: 1920*1080i resolution,60HZ refresh rate,
			33.75kHZ Horizontal frequency,74.25MHZ pixel rate.
		*/
		.plldiv_msb = 0x89,
		.plldiv_lsb = 0x80,
		.pll_ctrl = 0xA8,
		.phase_select_bit0 = 0x00,
		.pre_coast = 0x03,
		.post_coast = 0x0,
		.clamp_start = 0x32,
		.clamp_width = 0x20,
		.alc_placement = 0x5a,
	},
	{ /* Video standard: 1920*1080i resolution,50HZ refresh rate,
			28.125kHZ Horizontal frequency,74.25MHZ pixel rate.
		*/
		.plldiv_msb = 0xA5,
		.plldiv_lsb = 0x00,
		.pll_ctrl = 0xA8,
		.phase_select_bit0 = 0x00,
		.pre_coast = 0x03,
		.post_coast = 0x0,
		.clamp_start = 0x32,
		.clamp_width = 0x20,
		.alc_placement = 0x5a,
	},
	{ /* Video standard: 1920*1080p resolution,60HZ refresh rate,
			67.5kHZ Horizontal frequency,148.5MHZ pixel rate.
		*/
		.plldiv_msb = 0x89,
		.plldiv_lsb = 0x80,
		.pll_ctrl = 0xD8,
		.phase_select_bit0 = 0x00,
		.pre_coast = 0x0,
		.post_coast = 0x0,
		.clamp_start = 0x32,
		.clamp_width = 0x20,
		.alc_placement = 0x5a,
	},
	{ /* Video standard: 1920*1080p resolution,50HZ refresh rate,
			56.25kHZ Horizontal frequency,148.5MHZ pixel rate.
		*/
		.plldiv_msb = 0xA5,
		.plldiv_lsb = 0x00,
		.pll_ctrl = 0xD8,
		.phase_select_bit0 = 0x00,
		.pre_coast = 0x0,
		.post_coast = 0x0,
		.clamp_start = 0x32,
		.clamp_width = 0x20,
		.alc_placement = 0x5a,
	},
};

static int tvp7000_attach_adapter(struct i2c_adapter *adapter);
static int tvp7000_detach_client(struct i2c_client *client);

static int tvp7000_detect_client(struct i2c_adapter *adapter,
								 int addr, int kind);

static inline int tvp7000_read_reg(u8 reg);
static inline int tvp7000_write_reg(u8 reg, u8 value);

static int tvp7000_setup_video_standard(const struct tvp7000_video_std *std);

static int tvp7000_device_init(struct vpfe_capture_params *params);
static int tvp7000_device_cmd(u32 cmd, void *arg);

static struct i2c_driver tvp7000_driver = {
	.driver = {
		.name = "TVP7000",
	},
	.id = I2C_DRIVERID_TVP7000,
	.attach_adapter = tvp7000_attach_adapter,
	.detach_client = tvp7000_detach_client,
};

static inline int tvp7000_read_reg(u8 reg)
{
	return i2c_smbus_read_byte_data(tvp7000_client, reg);
}

static inline int tvp7000_write_reg(u8 reg, u8 value)
{
	int ret;

	ret = i2c_smbus_write_byte_data(tvp7000_client,
									reg, value);
	if (ret != 0)
		DPRINTK("Write Error Address = %x\n", reg);

	return ret;
}

static int tvp7000_write_inittab(const struct i2c_reg_value *regs, int num)
{
	int err = 0;
	int i = 0;

	if (regs == NULL)
		return -EINVAL;

	for (i=0; i<num; i++)
		err |= tvp7000_write_reg(regs[i].reg, regs[i].value);

	return err;
}


static int tvp7000_detect_client(struct i2c_adapter *adapter,
								 int addr, int kind)
{
	int err = 0;
	const char *client_name = "TVP7000 Video Capture";

	FN_IN;

	if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_WORD_DATA |
								 I2C_FUNC_SMBUS_WRITE_BYTE))
	{
		DPRINTK("Functionality check failed for %s\n",
				client_name);
		return err;
	}

	tvp7000_client = kmalloc(sizeof(struct i2c_client),
							 GFP_KERNEL);
	if (tvp7000_client == NULL)
	{
		err = -ENOMEM;
		DPRINTK("Couldn't allocate memory for %s\n",
				client_name);
		return err;
	}

	memset(tvp7000_client, 0x00, sizeof(struct i2c_client));
	tvp7000_client->adapter = adapter;
	tvp7000_client->addr = addr;
	tvp7000_client->driver = &tvp7000_driver;
	tvp7000_client->flags = 0;
	strlcpy(tvp7000_client->name, client_name, I2C_NAME_SIZE);

	err = i2c_attach_client(tvp7000_client);
	if (err)
	{
		DPRINTK("Couldn't attach %s\n", client_name);
		kfree(tvp7000_client);
		return err;
	}

	return 0;
}

static int tvp7000_attach_adapter(struct i2c_adapter *adapter)
{
	FN_IN;

	return i2c_probe(adapter, &addr_data, &tvp7000_detect_client);
}

static int tvp7000_detach_client(struct i2c_client *client)
{
	int err = 0;

	FN_IN;

	err = i2c_detach_client(client);
	if (err)
	{
		DPRINTK("Client deregistration failed, \
		       client not detached.\n");
		return err;
	}
	kfree(client);

	return 0;
}

static void tvp7000_device_power_on(bool on)
{
	s8 level = (on == true) ? 0 : 1;
	/* enable the GPIO(39) direction mode as output */
	gpio_direction_output(HD_CAP_GPIO, level);
	/* when on == true */
	/* set the Reset pin level as High for 5ms */
	gpio_set_value(HD_CAP_GPIO, level);
	mdelay(5);
	/* pull down the Reset pin level for 5us,
		reset the chip(Reset pin active in low) */
	gpio_set_value(HD_CAP_GPIO, !level);
	udelay(5);
	/* pull up the Reset pin and delay for 5us,
		now,tvp7000 is power up */
	gpio_set_value(HD_CAP_GPIO, level);
	udelay(5);
}

static int tvp7000_setup_video_standard(const struct tvp7000_video_std *std)
{
	int err = 0;
	int val;

	if (std == NULL)
		return -EINVAL;

	err |= tvp7000_write_reg(TVP7000_PLL_DIVIDE_MSB, std->plldiv_msb);
	err |= tvp7000_write_reg(TVP7000_PLL_DIVIDE_LSB, std->plldiv_lsb);
	err |= tvp7000_write_reg(TVP7000_PLL_CTRL, std->pll_ctrl);

	val = tvp7000_read_reg(TVP7000_PHASE_SELECT);
	val &= ~0x01;
	err |= tvp7000_write_reg(TVP7000_PHASE_SELECT, (std->phase_select_bit0 & 0x01) | val);

	err |= tvp7000_write_reg(TVP7000_PRE_COAST, std->pre_coast);
	err |= tvp7000_write_reg(TVP7000_POST_COAST, std->post_coast);
	err |= tvp7000_write_reg(TVP7000_CLAMP_START, std->clamp_start);
	err |= tvp7000_write_reg(TVP7000_CLAMP_WIDTH, std->clamp_width);
	err |= tvp7000_write_reg(TVP7000_ALC_PLACEMENT, std->alc_placement);

	return err;
}


static int tvp7000_selmux(void)
{
	if (tvp7000_write_reg(TVP7000_INPUT_MUX_1, 0)) // set channel 1
		return -1;
	return 0;
}

static int input_signal_exist(void)
{
	int val;
	val = tvp7000_read_reg(TVP7000_SYNC_DETECT_STATUS);
	if ((val & 0x80) && (val & 0x10))
	{
		return 0;
	}
	return -1;
}

static int tvp7000_device_cmd(u32 cmd, void *arg)
{
	int ret = 0;

	switch (cmd)
	{
	case 0:
	case VIDIOC_INT_RESET:
		tvp7000_device_init(NULL);
		break;
	case VIDIOC_G_INPUT:
		{
			if (!input_signal_exist())
				*(int *)arg = VPFE_AMUX_COMPONENT;
			else
				ret	= -EINVAL;
			break;
		}
	case VIDIOC_S_INPUT:
		{
			int input = *(int *)arg;
			if (input == VPFE_AMUX_COMPONENT)
			{
				if (tvp7000_device_init(NULL))
					ret = -EBUSY;
			}
			else
				ret	= -EINVAL;
			break;
		}
	case VPFE_CMD_CONFIG_CAPTURE:
		{
			struct vpfe_capture_params *params =
			(struct vpfe_capture_params *)arg;

			switch (params->mode)
			{
			case V4L2_STD_HD_480P:
				ret = tvp7000_setup_video_standard(STD(VIDEO480P60HZ));
				break;
			case V4L2_STD_HD_576P:
				ret = tvp7000_setup_video_standard(STD(VIDEO576P50HZ));
				break;
			case V4L2_STD_HD_720P:
				ret = tvp7000_setup_video_standard(STD(VIDEO720P60HZ));
				break;
			case V4L2_STD_HD_1080I:
				ret = tvp7000_setup_video_standard(STD(VIDEO1080I60HZ));
				break;
			default:
				ret	= -1;
			}
			break;
		}
	default:
		ret = -EINVAL;
		break;
	}
	return ret;
}

static int tvp7000_device_init(struct vpfe_capture_params *params)
{
	u8 ver = 0;

	FN_IN;
	ver = tvp7000_read_reg(TVP7000_CHIP_REVISION);
	DPRINTK("TVP7000 detect! Revision: %d\n", ver);

#if INIT_DEFAULT
	/* initialize TVP7000 as its default values */
	tvp7000_write_inittab(tvp7000_init_default, NUM_OF_REGS(tvp7000_init_default));
#endif
	tvp7000_write_inittab(tvp7000_init_component, NUM_OF_REGS(tvp7000_init_component));
	if (tvp7000_selmux())
		return -1;
	return 0;
}

static int tvp7000_device_active(void)
{
	tvp7000_device_power_on(true);
	return 0;
}

static int tvp7000_device_deactive(void)
{
	tvp7000_device_power_on(false);
	return 0;
}

static int tvp7000_device_cleanup(void)
{
	/* do nothing */
	return 0;
}

static struct vpfe_capture_device   tvp7000_capture_device = {
	.name = "TVP7000",
	.id = VPFE_CAPTURE_ID_TVP7000,
	.capture_device_init = tvp7000_device_init,
	.capture_device_cmd = tvp7000_device_cmd,
	.capture_device_active = tvp7000_device_active,
	.capture_device_deactive = tvp7000_device_deactive,
	.capture_device_cleanup = tvp7000_device_cleanup,
};

static __init int tvp7000_init(void)
{
	int i;
	int err = 0;
	FN_IN;

	/* power on the tvp7000 Video decoder*/
	tvp7000_device_power_on(true);
	mdelay(500);

	for (i = 0; i < TVP7000_I2C_RETRY; i++)
	{
		err = i2c_add_driver(&tvp7000_driver);
		if (!err)
		{
			break;
		}
	}
	if (err)
	{
		DPRINTK("I2C driver %s add failed\n",
				tvp7000_driver.driver.name);
		return err;
	}

	err = vpfe_capture_device_register(&tvp7000_capture_device);
	if (err)
	{
		DPRINTK("VPFE Capture Device %s register failed\n",
				tvp7000_capture_device.name);
		return err;
	}
	tvp7000_device_power_on(false);

	return 0;
}

static __exit void tvp7000_exit(void)
{
	FN_IN;

	/* power down the tvp7000 Video decoder*/
	tvp7000_device_power_on(false);

	vpfe_capture_device_unregister(&tvp7000_capture_device);

	i2c_del_driver(&tvp7000_driver);
}

module_init(tvp7000_init);
module_exit(tvp7000_exit);


