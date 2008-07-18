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

struct tvp7000_video_std
{
	u8 plldiv_msb;
	u8 plldiv_lsb;
	u8 pll_ctrl;
	u8 phase_select_bit0;
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
	},
	{ /* Video standard: 720*576p resolution,50HZ refresh rate,
			31.25kHZ Horizontal frequency,27MHZ pixel rate.
		*/
		.plldiv_msb = 0x6C,
		.plldiv_lsb = 0x00,
		.pll_ctrl = 0x68,
		.phase_select_bit0 = 0x01,
	},
	{ /* Video standard: 1280*720p resolution,60HZ refresh rate,
			45kHZ Horizontal frequency,74.25MHZ pixel rate.
		*/
		.plldiv_msb = 0x67,
		.plldiv_lsb = 0x20,
		.pll_ctrl = 0xA8,
		.phase_select_bit0 = 0x00,
	},
	{ /* Video standard: 1280*720p resolution,50HZ refresh rate,
			37.5kHZ Horizontal frequency,74.25MHZ pixel rate.
		*/
		.plldiv_msb = 0x7B,
		.plldiv_lsb = 0xC0,
		.pll_ctrl = 0xA8,
		.phase_select_bit0 = 0x00,
	},
	{ /* Video standard: 1920*1080i resolution,60HZ refresh rate,
			33.75kHZ Horizontal frequency,74.25MHZ pixel rate.
		*/
		.plldiv_msb = 0x89,
		.plldiv_lsb = 0x80,
		.pll_ctrl = 0xA8,
		.phase_select_bit0 = 0x00,
	},
	{ /* Video standard: 1920*1080i resolution,50HZ refresh rate,
			28.125kHZ Horizontal frequency,74.25MHZ pixel rate.
		*/
		.plldiv_msb = 0xA5,
		.plldiv_lsb = 0x00,
		.pll_ctrl = 0xA8,
		.phase_select_bit0 = 0x00,
	},
	{ /* Video standard: 1920*1080p resolution,60HZ refresh rate,
			67.5kHZ Horizontal frequency,148.5MHZ pixel rate.
		*/
		.plldiv_msb = 0x89,
		.plldiv_lsb = 0x80,
		.pll_ctrl = 0xD8,
		.phase_select_bit0 = 0x00,
	},
	{ /* Video standard: 1920*1080p resolution,50HZ refresh rate,
			56.25kHZ Horizontal frequency,148.5MHZ pixel rate.
		*/
		.plldiv_msb = 0xA5,
		.plldiv_lsb = 0x00,
		.pll_ctrl = 0xD8,
		.phase_select_bit0 = 0x00,
	},
};

static __init int tvp7000_init(void);
static __exit void tvp7000_exit(void);

static int tvp7000_attach_adapter(struct i2c_adapter *adapter);
static int tvp7000_detach_client(struct i2c_client *client);

static int tvp7000_detect_client(struct i2c_adapter *adapter,
								 int addr, int kind);

static void tvp7000_device_power_on(bool on);

static inline int tvp7000_read_reg(u8 reg);
static inline int tvp7000_write_reg(u8 reg, u8 value);

static int tvp7000_write_inittab(const struct i2c_reg_value *regs,
								 int num);
static int tvp7000_setup_video_stardard(const struct tvp7000_video_std *std);

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
								 I2C_FUNC_SMBUS_WRITE_BYTE)) {
		DPRINTK("Functionality check failed for %s\n",
				client_name);
		return err;
	}

	tvp7000_client = kmalloc(sizeof(struct i2c_client),
							 GFP_KERNEL);
	if (tvp7000_client == NULL) {
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
	if (err) {
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
	if (err) {
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

static int tvp7000_setup_video_stardard(
    const struct tvp7000_video_std *std)
{
    int err = 0;
    int val;

    if (std == NULL)
        return -EINVAL;

    err |= tvp7000_write_reg(TVP7000_PLL_DIVIDE_MSB,
                             std->plldiv_msb);
    err |= tvp7000_write_reg(TVP7000_PLL_DIVIDE_LSB,
                             std->plldiv_lsb);
    err |= tvp7000_write_reg(TVP7000_PLL_CTRL,
                             std->pll_ctrl);

    val = tvp7000_read_reg(TVP7000_PHASE_SELECT);
    val &= ~0x01;
    err |= tvp7000_write_reg(TVP7000_PHASE_SELECT,
                             (std->phase_select_bit0 & 0x01) | val);

    return err;
}


static int tvp7000_selmux(void)
{
    if(tvp7000_write_reg(TVP7000_INPUT_MUX_1, 0)) // set channel 1
        return -1;
    return 0;
}

static void tvp7000_reset(void)
{
	/* Initializes TVP7000 to its default values */
	tvp7000_write_inittab(tvp7000_init_default, NUM_OF_REGS(tvp7000_init_default));

	/* Selects decoder input */
	tvp7000_selmux();
}

static int input_signal_exist(void)
{
    int val;
    val = tvp7000_read_reg(TVP7000_SYNC_DETECT_STATUS);
    if((val & 0x80) && (val & 0x10))
    {
        return 0;
    }
    return -1;
}

static int tvp7000_device_cmd(u32 cmd, void *arg)
{
    int ret = 0;

    switch (cmd) {
	case 0:
	case VIDIOC_INT_RESET:
		tvp7000_reset();
		break;
	case VIDIOC_G_INPUT:
	{
		if (!input_signal_exist())
			*(int *)arg = VPFE_AMUX_COMPONENT;
		else
            ret = -EINVAL;
		break;
	}
	case VIDIOC_S_INPUT:
	{
		int input = *(int *)arg;
		if (input == VPFE_AMUX_COMPONENT)
        {
            if(tvp7000_selmux())
                ret = -EBUSY;
        }
		else
            ret = -EINVAL;
		break;
	}
    case VIDIOC_S_STD:
        tvp7000_setup_video_stardard(STD(VIDEO480P60HZ));
        break;
	case VPFE_CMD_CONFIG_CAPTURE:
		{
			struct vpfe_capture_params *params =
				(struct vpfe_capture_params *)arg;

			if (params->amuxmode == VPFE_AMUX_COMPONENT) 
            {
                tvp7000_selmux();
                //tvp7000_set_std(params->mode);
            }
            else
                ret = -1;
			break;
		}
    default:
        break;
    }
    return ret;
}

static int tvp7000_device_init(struct vpfe_capture_params *params)
{
	u8 ver = 0;

	FN_IN;

	ver = tvp7000_read_reg(TVP7000_CHIP_REVISION);
	DPRINTK("TVP7000 detect! Revision: %d\n",
			ver);

	/* initialize TVP7000 as its default values */
	tvp7000_write_inittab(tvp7000_init_default, NUM_OF_REGS(tvp7000_init_default));

	//tvp7000_write_reg(TVP7000_PLL_DIVIDE_MSB, 0x6B);
	//tvp7000_write_reg(TVP7000_PLL_DIVIDE_LSB, 0x40);
	//tvp7000_write_reg(TVP7000_PLL_CTRL, 0x68);
	//tvp7000_write_reg(TVP7000_PHASE_SELECT, 0xB9);
//	tvp7000_write_reg(TVP7000_SYNC_ON_GREEN, 0x80);
	//tvp7000_write_reg(TVP7000_PRE_COAST, 0x03);
	//tvp7000_write_reg(TVP7000_POST_COAST, 0x0C);
	tvp7000_write_reg(TVP7000_MISC_CTRL, (1 << 3) | (1 << 1));
	tvp7000_write_reg(TVP7000_SOG_CLAMP, 1<< 7);

	return 0;
}

static struct vpfe_capture_device	tvp7000_capture_device = {
	.name = "TVP7000",
	.id = VPFE_CAPTURE_ID_TVP7000,
	.capture_device_init = tvp7000_device_init,
    .capture_device_cmd = tvp7000_device_cmd,
};

static __init int tvp7000_init(void)
{
	int err = 0;
	FN_IN;

	/* power on the tvp7000 Video decoder*/
	tvp7000_device_power_on(true);

	err = i2c_add_driver(&tvp7000_driver);
	if (err) {
		DPRINTK("I2C driver %s add failed\n",
				tvp7000_driver.driver.name);
		return err;
	}

	err = vpfe_capture_device_register(&tvp7000_capture_device);
	if (err) {
		DPRINTK("VPFE Capture Device %s register failed\n",
				tvp7000_capture_device.name);
		return err;
	}

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


