/*
 *
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
/* tvp5146.c */

#include <linux/init.h>
#include <linux/fs.h>
#include <linux/vmalloc.h>
#include <linux/slab.h>
#include <linux/proc_fs.h>
#include <linux/ctype.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/videodev.h>
#include <media/tvp5146.h>

#define debug_print(x...)	//printk(x)

static struct i2c_client tvp5146_i2c_client;
static struct i2c_driver tvp5146_i2c_driver;

static int i2c_read_reg(struct i2c_client *client, u8 reg, u8 * val);
static int i2c_write_reg(struct i2c_client *client, u8 reg, u8 val);

static int configtvp5146(void *arg);
static int clrtvp5146lostlock(void);
static int enabletvp5146agc(int arg);
static int getctrl(void *arg);
static int gettvp5146status(void *arg);
static int powerdowntvp5146(int powerdownenable);
static int queryctrl(void *arg);
static int resettvp5146(void);
static int setctrl(void *arg);
static int settvp5146amuxmode(int mode);
static int settvp5146brightness(int arg);
static int settvp5146contrast(int arg);
static int settvp5146hue(int arg);
static int settvp5146saturation(int arg);
static int settvp5146std(int arg);
static int setup656sync(int enable);

/*
 * ======== configtvp5146 ========
 */
static int configtvp5146(void *arg)
{
	tvp5146_params *tvp5146params = (tvp5146_params *) arg;
	int ret = 0;

	ret |= setup656sync(tvp5146params->enablebt656sync);
	ret |= settvp5146amuxmode(tvp5146params->amuxmode);
	ret |= settvp5146std(tvp5146params->mode);

	return ret;
}

/*
 * ======== clrtvp5146lostlock  ========
 */
static int clrtvp5146lostlock(void)
{
	int ret = 0;
	u8 clr = 1;
	ret = i2c_write_reg(&tvp5146_i2c_client, 0x39, clr);
	return ret;
}

/*
 * ========  enabletvp5146agc ========
 */
static int enabletvp5146agc(int arg)
{
	int ret = 0;
	int agc;
	if (arg == TRUE) {
		agc = 0xF;
	} else {
		agc = 0xC;
	}
	ret = i2c_write_reg(&tvp5146_i2c_client, 0x01, agc);
	return ret;
}

/*
 * ========  gettvpctrl ========
 */
static int getctrl(void *arg)
{
	struct v4l2_control *ctrl = arg;
	int ret = 0;
	u8 value;

	switch (ctrl->id) {
	case V4L2_CID_BRIGHTNESS:
		ret = i2c_read_reg(&tvp5146_i2c_client, 0x09, &value);
		ctrl->value = value;
		break;
	case V4L2_CID_CONTRAST:
		ret = i2c_read_reg(&tvp5146_i2c_client, 0x0A, &value);
		ctrl->value = value;
		break;
	case V4L2_CID_SATURATION:
		ret = i2c_read_reg(&tvp5146_i2c_client, 0x0B, &value);
		ctrl->value = value;
		break;
	case V4L2_CID_HUE:
		ret = i2c_read_reg(&tvp5146_i2c_client, 0x0C, &value);
		ctrl->value = value;
		break;
	case V4L2_CID_AUTOGAIN:
		ret = i2c_read_reg(&tvp5146_i2c_client, 0x01, &value);
		if ((value & 0x3) == 3) {
			ctrl->value = TRUE;
		} else {
			ctrl->value = FALSE;
		}
		break;
	default:
		ret = -EINVAL;
		break;
	}
	return ret;
}

/*
 * ========  gettvp5146std ========
 */
static int gettvp5146std(tvp5146_mode * mode)
{
	int ret = 0;
	u8 output1;
	u8 std;
	u8 lock_status;

	ret |= i2c_read_reg(&tvp5146_i2c_client, 0x2, &std);
	std &= 0x7;
	if(std == TVP5146_MODE_AUTO){
		ret |= i2c_read_reg(&tvp5146_i2c_client, 0x3F, &std);
	}
	std &= 0x7;
	ret |= i2c_read_reg(&tvp5146_i2c_client, 0x33, &output1);
	*mode = std  | ((output1 & 0x80) >> 4);	/* square pixel status */
	ret |= i2c_read_reg(&tvp5146_i2c_client, 0x3A, &lock_status);
	if ((lock_status & 0xe) != 0xe) {
		/* not quite locked */
		ret = -EAGAIN;
	}

	return ret;
}

/*
 * ========  gettvp5146status ========
 */
static int gettvp5146status(void *arg)
{
	int ret = 0;
	tvp5146_status *status = (tvp5146_status *) arg;
	u8 agc, brightness, contrast, hue, saturation;
	u8 status_byte;
	u8 std;
	u8 output1;

	ret = i2c_read_reg(&tvp5146_i2c_client, 0x01, &agc);
	if ((agc & 0x3) == 3) {
		status->agc_enable = TRUE;
	} else {
		status->agc_enable = FALSE;
	}
	ret |= i2c_read_reg(&tvp5146_i2c_client, 0x09, &brightness);
	status->brightness = brightness;

	ret |= i2c_read_reg(&tvp5146_i2c_client, 0x0A, &contrast);
	status->contrast = contrast;

	ret |= i2c_read_reg(&tvp5146_i2c_client, 0x0B, &saturation);
	status->saturation = saturation;

	ret |= i2c_read_reg(&tvp5146_i2c_client, 0x0C, &hue);
	status->hue = hue;

	ret |= i2c_read_reg(&tvp5146_i2c_client, 0x3A, &status_byte);
	status->field_rate = (status_byte & 0x20) ? 50 : 60;
	status->lost_lock = (status_byte & 0x10) >> 4;
	status->csubc_lock = (status_byte & 0x8) >> 3;
	status->v_lock = (status_byte & 0x4) >> 2;
	status->h_lock = (status_byte & 0x2) >> 1;

	ret |= i2c_read_reg(&tvp5146_i2c_client, 0x3F, &std);
	ret |= i2c_read_reg(&tvp5146_i2c_client, 0x33, &output1);
	if (std | 0x80) {	/* auto switch mode */
		status->video_std = TVP5146_MODE_AUTO;
	} else {
		status->video_std = std;
	}
	status->video_std |= ((output1 & 0x80) >> 4);	/* square pixel status */
	return ret;
}

/*
 * ======== powerdowntvp5146 ========
 */
static int powerdowntvp5146(int powerdownenable)
{
	u8 powerdownsettings = 0x01;

	/*Put _tvp5146 in power down mode */
	if (!powerdownenable) {
		powerdownsettings = 0x00;
	}
	return i2c_write_reg(&tvp5146_i2c_client, 0x03, powerdownsettings);
}

/*
 * ======== resettvp5146========
 */
static int resettvp5146(void)
{
	setup656sync(TRUE);
	settvp5146amuxmode(TVP5146_AMUX_COMPOSITE);
	return powerdowntvp5146(FALSE);
}

/*
 * ======== queryctrl ========
 */
static int queryctrl(void *arg)
{
	struct v4l2_queryctrl *queryctrl = arg;
	int ret = 0;
	__u32 id = queryctrl->id;

	memset(queryctrl, 0, sizeof(*queryctrl));
	switch (id) {
	case V4L2_CTRL_FLAG_NEXT_CTRL:
	case V4L2_CID_BRIGHTNESS:
		queryctrl->id = V4L2_CID_BRIGHTNESS;
		strcpy(queryctrl->name, "BRIGHTNESS");
		queryctrl->type = V4L2_CTRL_TYPE_INTEGER;
		queryctrl->minimum = 0;
		queryctrl->maximum = 255;
		queryctrl->step = 1;
		queryctrl->default_value = 128;
		break;

	case V4L2_CID_BRIGHTNESS|V4L2_CTRL_FLAG_NEXT_CTRL:
	case V4L2_CID_CONTRAST:
		queryctrl->id = V4L2_CID_CONTRAST;
		strcpy(queryctrl->name, "CONTRAST");
		queryctrl->type = V4L2_CTRL_TYPE_INTEGER;
		queryctrl->minimum = 0;
		queryctrl->maximum = 255;
		queryctrl->step = 1;
		queryctrl->default_value = 128;
		break;

	case V4L2_CID_CONTRAST|V4L2_CTRL_FLAG_NEXT_CTRL:
	case V4L2_CID_SATURATION:
		queryctrl->id = V4L2_CID_SATURATION;
		strcpy(queryctrl->name, "SATURATION");
		queryctrl->type = V4L2_CTRL_TYPE_INTEGER;
		queryctrl->minimum = 0;
		queryctrl->maximum = 255;
		queryctrl->step = 1;
		queryctrl->default_value = 128;
		break;

	case V4L2_CID_SATURATION|V4L2_CTRL_FLAG_NEXT_CTRL:
	case V4L2_CID_HUE:
		queryctrl->id = V4L2_CID_HUE;
		strcpy(queryctrl->name, "HUE");
		queryctrl->type = V4L2_CTRL_TYPE_INTEGER;
		queryctrl->minimum = -128;	/* -180 DEGREE */
		queryctrl->maximum = 127;	/* 180  DEGREE */
		queryctrl->step = 1;
		queryctrl->default_value = 0;	/* 0 DEGREE */
		break;

	case V4L2_CID_HUE|V4L2_CTRL_FLAG_NEXT_CTRL:
	case V4L2_CID_AUTOGAIN:
		queryctrl->id = V4L2_CID_AUTOGAIN;
		strcpy(queryctrl->name, "Automatic Gain Control");
		queryctrl->type = V4L2_CTRL_TYPE_BOOLEAN;
		queryctrl->minimum = 0;
		queryctrl->maximum = 1;
		queryctrl->step = 1;
		queryctrl->default_value = 1;
		break;

	default:
		if (id < V4L2_CID_LASTP1)
		{
			queryctrl->flags = V4L2_CTRL_FLAG_DISABLED;
			break;
		}
	case V4L2_CID_AUTOGAIN|V4L2_CTRL_FLAG_NEXT_CTRL:
		ret = -EINVAL;
		break;
	}			/* end switch (id) */
	return ret;
}

/*
 * ======== setctrl ========
 */
static int setctrl(void *arg)
{
	struct v4l2_control *ctrl = arg;
	int ret = 0;

	switch (ctrl->id) {
	case V4L2_CID_BRIGHTNESS:
		ret = settvp5146brightness(ctrl->value);
		break;
	case V4L2_CID_CONTRAST:
		ret = settvp5146contrast(ctrl->value);
		break;
	case V4L2_CID_SATURATION:
		ret = settvp5146saturation(ctrl->value);
		break;
	case V4L2_CID_HUE:
		ret = settvp5146hue(ctrl->value);
		break;
	case V4L2_CID_AUTOGAIN:
		ret = enabletvp5146agc(ctrl->value);
		break;
	default:
		ret = -EINVAL;
		break;
	}
	return ret;
}

/*
 * ======== settvp5146amuxmode ========
 */
static int settvp5146amuxmode(int arg)
{
	u8 input_sel;

	if (arg == TVP5146_AMUX_COMPOSITE) {	/* composite */
		input_sel = 0x05;
	} else if (arg == TVP5146_AMUX_SVIDEO) {	/* s-video */
		input_sel = 0x46;
	} else {
		return -EINVAL;
	}
	return i2c_write_reg(&tvp5146_i2c_client, 0x00, input_sel);
}

/*
 * ======== settvp5146brightness ========
 */
static int settvp5146brightness(int arg)
{
	int ret = 0;
	u8 brightness = (u8) arg;
	ret = i2c_write_reg(&tvp5146_i2c_client, 0x09, brightness);
	return ret;
}

/*
* ======== settvp5146contrast ========
*/
static int settvp5146contrast(int arg)
{
	int ret = 0;
	u8 contrast = (u8) arg;
	ret = i2c_write_reg(&tvp5146_i2c_client, 0x0A, contrast);
	return ret;
}

/*
* ======== settvp5146hue ========
*/
static int settvp5146hue(int arg)
{
	int ret = 0;
	u8 hue = (u8) arg;
	ret = i2c_write_reg(&tvp5146_i2c_client, 0x0C, hue);
	return ret;
}

static int settvp5146saturation(int arg)
{
	int ret = 0;
	u8 saturation = (u8) arg;
	ret = i2c_write_reg(&tvp5146_i2c_client, 0x0B, saturation);
	return ret;
}

static int settvp5146std(int arg)
{
	int ret = 0;
	u8 std = (u8) arg & 0x7;	/* the 4th-bit is for squre pixel sampling */
	u8 output1;

	/* setup the sampling rate: 601 or square pixel */
	debug_print(KERN_INFO "reading i2c registers.\n");
	ret = i2c_read_reg(&tvp5146_i2c_client, 0x33, &output1);
	output1 |= ((arg & 0x8) << 4);
	ret = i2c_write_reg(&tvp5146_i2c_client, 0x33, output1);

	/* setup the video standard */
	ret |= i2c_write_reg(&tvp5146_i2c_client, 0x02, std);
	/* if autoswitch mode, enable all modes for autoswitch */
	if (std == TVP5146_MODE_AUTO) {
		u8 mask = 0x3F;	/* enable autoswitch for  all standards */
		ret = i2c_write_reg(&tvp5146_i2c_client, 0x04, mask);
	}

	return ret;
}

/*
 * ======== setup656sync ========
 */
static int setup656sync(int enable)
{
	int output1, output2, output3, output4;
	int output5, output6;
	int ret = 0;

	if (enable) {
		output1 = 0x40;
		output4 = 0xFF;
		output6 = 0;
	} else {
		output1 = 0x43;
		output4 = 0xAF;
		output6 = 0x1E;
	}

	output2 = 0x11;		/* enable clock, enable Y[9:0] */
	output3 = 0x0;
	output5 = 0x4;

	ret |= i2c_write_reg(&tvp5146_i2c_client, 0x33, output1);
	ret |= i2c_write_reg(&tvp5146_i2c_client, 0x34, output2);
	ret |= i2c_write_reg(&tvp5146_i2c_client, 0x36, output4);
	ret |= i2c_write_reg(&tvp5146_i2c_client, 0x08, output3);
	ret |= i2c_write_reg(&tvp5146_i2c_client, 0x0e, output5);
	ret |= i2c_write_reg(&tvp5146_i2c_client, 0x32, output6);
	return ret;
}

/*
 * ======== tvp5146_ctrl ========
 */
int tvp5146_ctrl(tvp5146_cmd cmd, void *arg)
{
	int ret = 0;
	switch (cmd) {
	case TVP5146_CONFIG:
		ret = configtvp5146(arg);
		break;
	case TVP5146_RESET:
		ret = resettvp5146();
		break;
	case TVP5146_POWERDOWN:
		ret = powerdowntvp5146(*(int *)arg);
		break;
	case TVP5146_SET_AMUXMODE:
		ret = settvp5146amuxmode(*(int *)arg);
		break;
	case TVP5146_SET_BRIGHTNESS:
		ret = settvp5146brightness(*(int *)arg);
		break;
	case TVP5146_SET_CONTRAST:
		ret = settvp5146contrast(*(int *)arg);
		break;
	case TVP5146_SET_HUE:
		ret = settvp5146hue(*(int *)arg);
		break;
	case TVP5146_SET_SATURATION:
		ret = settvp5146saturation(*(int *)arg);
		break;
	case TVP5146_SET_AGC:
		ret = enabletvp5146agc(*(int *)arg);
		break;
	case TVP5146_SET_VIDEOSTD:
		ret = settvp5146std(*(int *)arg);
		break;
	case TVP5146_CLR_LOSTLOCK:
		ret = clrtvp5146lostlock();
		break;
	case TVP5146_GET_STATUS:
		ret = gettvp5146status(arg);
		break;
	case TVP5146_GET_STD:
		ret = gettvp5146std(arg);
		break;
	case VIDIOC_QUERYCTRL:
		ret = queryctrl(arg);
		break;
	case VIDIOC_G_CTRL:
		ret = getctrl(arg);
		break;
	case VIDIOC_S_CTRL:
		ret = setctrl(arg);
		break;
	default:
		ret = -EINVAL;
	}
	return ret;
}

static int i2c_read_reg(struct i2c_client *client, u8 reg, u8 * val)
{
	int err = 0;

	struct i2c_msg msg[1];
	unsigned char data[1];

	if (!client->adapter) {
		err = -ENODEV;
	} else {
		msg->addr = client->addr;
		msg->flags = 0;
		msg->len = 1;
		msg->buf = data;
		data[0] = reg;
		err = i2c_transfer(client->adapter, msg, 1);
		if (err >= 0) {
			msg->flags = I2C_M_RD;
			err = i2c_transfer(client->adapter, msg, 1);
			if (err >= 0) {
				*val = data[0];
			}
		}
	}
	return err;
}

static int i2c_write_reg(struct i2c_client *client, u8 reg, u8 val)
{
	int err = 0;

	struct i2c_msg msg[1];
	unsigned char data[2];

	if (!client->adapter) {
		err = -ENODEV;
	} else {
		msg->addr = client->addr;
		msg->flags = 0;
		msg->len = 2;
		msg->buf = data;
		data[0] = reg;
		data[1] = val;
		err = i2c_transfer(client->adapter, msg, 1);
	}
	debug_print(KERN_INFO " i2c data write \n");

	return err;
}

static int _i2c_attach_client(struct i2c_client *client,
			      struct i2c_driver *driver,
			      struct i2c_adapter *adap, int addr)
{
	int err = 0;

	if (client->adapter) {
		err = -EBUSY;	/* our client is already attached */
	} else {
		client->addr = addr;
/* 		client->flags = I2C_CLIENT_ALLOW_USE; */
		client->driver = driver;
		client->adapter = adap;

		err = i2c_attach_client(client);
		if (err) {
			client->adapter = NULL;
		}
	}
	return err;
}

static int _i2c_detach_client(struct i2c_client *client)
{
	int err = 0;

	if (!client->adapter) {
		return -ENODEV;	/* our client isn't attached */
	} else {
		err = i2c_detach_client(client);
		client->adapter = NULL;
	}
	return err;
}

static int tvp5146_i2c_probe_adapter(struct i2c_adapter *adap)
{
	return _i2c_attach_client(&tvp5146_i2c_client, &tvp5146_i2c_driver,
				  adap, TVP5146_I2C_ADDR);
}

static struct i2c_driver tvp5146_i2c_driver = {
	.driver = {
		.name = "tvp5146",
	},
	.id = I2C_DRIVERID_TVP5150,

	.attach_adapter = tvp5146_i2c_probe_adapter,
	.detach_client = _i2c_detach_client,
};

static int tvp5146_i2c_init(void)
{
	int err;
	struct i2c_driver *driver = &tvp5146_i2c_driver;

/* 	driver->owner = THIS_MODULE; */
/* 	strlcpy(driver->name, "TVP5146 Video Decoder I2C driver", */
/* 		sizeof(driver->name)); */
/* 	driver->id = I2C_DRIVERID_EXP0; */
/* 	driver->flags = I2C_DF_NOTIFY; */
/* 	driver->attach_adapter = tvp5146_i2c_probe_adapter; */
/* 	driver->detach_client = _i2c_detach_client; */

	err = i2c_add_driver(driver);
	if (err) {
		debug_print(KERN_ERR
			"Failed to register TVP5146 I2C client.\n");
	}
	debug_print(KERN_INFO "tvp5146 driver registered.\n");
	return err;
}

static void tvp5146_i2c_cleanup(void)
{
	struct i2c_driver *driver = &tvp5146_i2c_driver;

	i2c_detach_client(&tvp5146_i2c_client);
	i2c_del_driver(driver);
	tvp5146_i2c_client.adapter = NULL;
}

module_init(tvp5146_i2c_init);
module_exit(tvp5146_i2c_cleanup);

EXPORT_SYMBOL(tvp5146_ctrl);
MODULE_LICENSE("GPL");

/**************************************************************************/
/* End of file                                                                            */
/**************************************************************************/
