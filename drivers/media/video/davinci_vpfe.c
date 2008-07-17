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
/* davinci_vpfe.c */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/fs.h>
#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/interrupt.h>
#include <linux/kdev_t.h>
#include <linux/string.h>
#include <linux/videodev.h>
#include <linux/wait.h>
#include <linux/dma-mapping.h>
#include <linux/platform_device.h>

#include <asm/irq.h>
#include <asm/page.h>
#include <asm/io.h>
#include <asm/dma-mapping.h>

#include <media/davinci_vpfe.h>

#define debug_print(x...)	//printk(x)

MODULE_LICENSE("GPL");

/* These routines must be called after device
	 lock is obtained. */
#define ACTIVE_DEVICE() (vpfe_device.active_device)
#define SET_ACTIVE_DEVICE(x) (vpfe_device.active_device = (x))
#define IS_ACTIVE(x) \
	((x) && ACTIVE_DEVICE() && ((x)->id == ACTIVE_DEVICE()->id))
#define DEVICE_DEACTIVATE(x) do { \
	if ((x) && (x)->capture_device_deactive &&			\
	    (x)->capture_device_deactive())				\
	     debug_print(KERN_ERR					\
			 "capture device %s deactivate failed\n",	\
			 (x)->name);					\
} while (0)
#define DEVICE_ACTIVATE(x) do { \
	if ((x) && (x)->capture_device_active &&			\
	    (x)->capture_device_active())				\
	     debug_print(KERN_ERR					\
			 "capture device %s activate failed\n",		\
			 (x)->name);					\
} while (0)
#define DEVICE_INIT(x, params) do { \
	if ((x) && (x)->capture_device_init &&				\
	    (x)->capture_device_init(params))				\
	     debug_print(KERN_ERR					\
			 "capture device %s init failed\n",		\
			 (x)->name);					\
} while (0)
#define DEVICE_CLEANUP(x) do { \
	if ((x) && (x)->capture_device_cleanup &&			\
	    (x)->capture_device_cleanup())				\
	     debug_print(KERN_ERR					\
			 "capture device %s cleanup failed\n",		\
			 (x)->name);					\
} while (0)
#define DEVICE_CMD(dev, cmd, arg) \
	((dev) && (dev)->capture_device_cmd) ?		\
	(dev)->capture_device_cmd(cmd, arg) : -EINVAL

static struct v4l2_rect ntsc_bounds = VPFE_WIN_NTSC;
static struct v4l2_rect pal_bounds = VPFE_WIN_PAL;
static struct v4l2_fract ntsc_aspect = VPFE_PIXELASPECT_NTSC;
static struct v4l2_fract pal_aspect = VPFE_PIXELASPECT_PAL;
static struct v4l2_rect ntscsp_bounds = VPFE_WIN_NTSC_SP;
static struct v4l2_rect palsp_bounds = VPFE_WIN_PAL_SP;
static struct v4l2_fract sp_aspect = VPFE_PIXELASPECT_NTSC_SP;

#define NTOSD_INPUTS	2
static struct v4l2_input ntosd_inputs[NTOSD_INPUTS] = {
     { 0, "COMPOSITE", V4L2_INPUT_TYPE_CAMERA, 2, 0, V4L2_STD_ALL, 0 },
     { 1, "COMPONENT", V4L2_INPUT_TYPE_CAMERA, 2, 0, V4L2_STD_ALL, 0 },
};

static vpfe_obj vpfe_device = {	/* the default format is NTSC */
	.usrs = 0,
	.io_usrs = 0,
	.std = VPFE_STD_AUTO,
	.vwin = VPFE_WIN_PAL,
	.bounds = VPFE_WIN_PAL,
	.pixelaspect = VPFE_PIXELASPECT_NTSC,
	.pixelfmt = V4L2_PIX_FMT_UYVY,
	.field = V4L2_FIELD_INTERLACED,
	.numbuffers = VPFE_DEFNUM_FBUFS,
	.ccdc_params = {
		.pix_fmt = CCDC_PIXFMT_YCBCR_8BIT,
		.frm_fmt = CCDC_FRMFMT_INTERLACED,
		.win = VPFE_WIN_PAL,
		.fid_pol = CCDC_PINPOL_POSITIVE,
		.vd_pol = CCDC_PINPOL_POSITIVE,
		.hd_pol = CCDC_PINPOL_POSITIVE,
		.bt656_enable = TRUE,
		.pix_order = CCDC_PIXORDER_CBYCRY,
		.buf_type = CCDC_BUFTYPE_FLD_INTERLEAVED
	},
	.capture_params = {
		.mode = VPFE_STD_AUTO,
		.amuxmode = VPFE_AMUX_COMPOSITE,
		.enablebt656sync = TRUE,
		.squarepixel = FALSE,
	},
        .irqlock = SPIN_LOCK_UNLOCKED
};

struct v4l2_capability vpfe_drvcap = {
	.driver = "vpfe driver",
	.card = "DaVinci EVM",
	.bus_info = "Platform",
	.version = VPFE_VERSION_CODE,
	.capabilities = V4L2_CAP_VIDEO_CAPTURE | V4L2_CAP_STREAMING
};

static irqreturn_t vpfe_isr(int irq, void *dev_id)
{
	vpfe_obj *vpfe = &vpfe_device;
	int fid;

	/* check which field we are in hardware */
	fid = ccdc_getfid();
	vpfe->field_id ^= 1;	/* switch the software maintained field id */
	debug_print(KERN_INFO "field id = %x:%x.\n", fid, vpfe->field_id);
	if (fid == vpfe->field_id) {	/* we are in-sync here, continue */
		if (fid == 0) {
			/*  One frame is just being captured. If the next frame
			is available, release the current frame and move on */
			if (vpfe->curFrm != vpfe->nextFrm) {
				vpfe->curFrm->state = STATE_DONE;
				wake_up_interruptible(&vpfe->curFrm->done);
				vpfe->curFrm = vpfe->nextFrm;
			}
			/* based on whether the two fields are stored interleavely      */
			/* or separately in memory, reconfigure the CCDC memory address */
			if (vpfe->field == V4L2_FIELD_SEQ_TB) {
				u32 addr =
				    vpfe->curFrm->boff + vpfe->field_offset;
				ccdc_setfbaddr((unsigned long)addr);
			}
		} else if (fid == 1) {
			/* if one field is just being captured */
			/* configure the next frame */
			/* get the next frame from the empty queue */
			/* if no frame is available, hold on to the current buffer */
			if (!list_empty(&vpfe->dma_queue)
			    && vpfe->curFrm == vpfe->nextFrm) {
				vpfe->nextFrm = list_entry(vpfe->dma_queue.next,
					struct videobuf_buffer, queue);
				list_del(&vpfe->nextFrm->queue);
				vpfe->nextFrm->state = STATE_ACTIVE;
				ccdc_setfbaddr(
					(unsigned long)vpfe->nextFrm->boff);
			}
			if (vpfe->mode_changed) {
				ccdc_setwin(&vpfe->ccdc_params);
				/* update the field offset */
				vpfe->field_offset =
				    (vpfe->vwin.height - 2) * vpfe->vwin.width;
				vpfe->mode_changed = FALSE;
			}
		}
	} else if (fid == 0) {
		/* recover from any hardware out-of-sync due to */
		/* possible switch of video source              */
		/* for fid == 0, sync up the two fids           */
		/* for fid == 1, no action, one bad frame will  */
		/* go out, but it is not a big deal             */
		vpfe->field_id = fid;
	}
	debug_print(KERN_INFO "interrupt returned.\n");
	return IRQ_RETVAL(1);
}

/* this is the callback function called from videobuf_qbuf() function */
/* the buffer is prepared and queued into the dma queue */
static int buffer_prepare(struct videobuf_queue *q,
			  struct videobuf_buffer *vb,
			  enum v4l2_field field)
{
	vpfe_obj *vpfe = &vpfe_device;

	if (vb->state == STATE_NEEDS_INIT) {
		vb->width  = vpfe->vwin.width;
		vb->height = vpfe->vwin.height;
		vb->size   = VPFE_MAX_FBUF_SIZE;
		vb->field  = field;
	}
	vb->state = STATE_PREPARED;

	return 0;

}

static void
buffer_config(struct videobuf_queue *q, unsigned int count)
{
	vpfe_obj *vpfe = &vpfe_device;
	int i;

	for(i = 0; i < count; i++) {
		q->bufs[i]->boff = virt_to_phys(vpfe->fbuffers[i]);
		debug_print(KERN_INFO "buffer address: %x\n", q->bufs[i]->boff);
	}
}

static int
buffer_setup(struct videobuf_queue *q, unsigned int *count, unsigned int *size)
{
	vpfe_obj *vpfe = &vpfe_device;
	int i;
	*size = VPFE_MAX_FBUF_SIZE;

	for (i = VPFE_DEFNUM_FBUFS; i < *count; i++) {
		u32 size = PAGE_SIZE << VPFE_MAX_FBUF_ORDER;
		void *mem = (void *)__get_free_pages(GFP_KERNEL |GFP_DMA,
						     VPFE_MAX_FBUF_ORDER);
		if (mem) {
			unsigned long adr = (unsigned long)mem;
			while (size > 0) {
				/* make sure the frame buffers are never
				   swapped out of memory */
				SetPageReserved(virt_to_page(adr));
				adr += PAGE_SIZE;
				size -= PAGE_SIZE;
			}
			vpfe->fbuffers[i] = mem;
		} else {
			break;
		}
	}
	*count = vpfe->numbuffers = i;

	return 0;
}

static void buffer_queue(struct videobuf_queue *q, struct videobuf_buffer *vb)
{
	vpfe_obj *vpfe = &vpfe_device;

        /* add the buffer to the DMA queue */
	list_add_tail(&vb->queue, &vpfe->dma_queue);
	vb->state = STATE_QUEUED;
}

static void buffer_release(struct videobuf_queue *q, struct videobuf_buffer *vb)
{
	/* free the buffer if it is not one of the 3 allocated at initializaiton time */
	if(vb->i < vpfe_device.numbuffers
	   && vb->i >= VPFE_DEFNUM_FBUFS
	   && vpfe_device.fbuffers[vb->i]){
		free_pages((unsigned long)vpfe_device.fbuffers[vb->i],
			   VPFE_MAX_FBUF_ORDER);
		vpfe_device.fbuffers[vb->i] = NULL;
	}
}

static struct videobuf_queue_ops video_qops = {
	.buf_setup    = buffer_setup,
	.buf_prepare  = buffer_prepare,
	.buf_queue    = buffer_queue,
	.buf_release  = buffer_release,
	.buf_config   = buffer_config,
};

static int vpfe_capture_device_active(struct vpfe_capture_device *device)
{
	int ret = 0;

	if (device == NULL)
		return -EINVAL;

	down_interruptible(&vpfe_device.lock);

	/* deactivate the activated device */
	DEVICE_DEACTIVATE(ACTIVE_DEVICE());

	/* set device state as active */
	SET_ACTIVE_DEVICE(device);

	/* call device specific routine to do the active job. */
	DEVICE_ACTIVATE(device);
	up(&vpfe_device.lock);

	return ret;
}

static int vpfe_select_capture_device(int id)
{
	int err = 0;
	struct vpfe_capture_device *device;

	down_interruptible(&vpfe_device.device_list_lock);
	list_for_each_entry(device, &vpfe_device.capture_device_list,
			    device_list){
		if (device->id == id) {
			err = vpfe_capture_device_active(device);
			up(&vpfe_device.device_list_lock);
			return err;
		}
	}
	up(&vpfe_device.device_list_lock);

	return -ENODEV;
}

static int vpfe_doioctl(struct inode *inode, struct file *file,
			unsigned int cmd, void *arg)
{
	vpfe_obj *vpfe = &vpfe_device;
	vpfe_fh *fh = file->private_data;

	int ret = 0;
	switch (cmd) {
	case VIDIOC_S_CTRL:
	case VIDIOC_S_FMT:
	case VIDIOC_S_STD:
	case VIDIOC_S_CROP:
		ret = v4l2_prio_check(&vpfe->prio, &fh->prio);
		if (0 != ret) {
			return ret;
		}
		break;
	}

	switch (cmd) {
	case VIDIOC_QUERYCAP:
	{
		struct v4l2_capability *cap =
			(struct v4l2_capability *)arg;
		memset(cap, 0, sizeof(*cap));
		*cap = vpfe_drvcap;
		break;
	}
	case VIDIOC_ENUM_FMT:
	{
		struct v4l2_fmtdesc *fmt = (struct v4l2_fmtdesc *)arg;
		u32 index = fmt->index;
		memset(fmt, 0, sizeof(*fmt));
		fmt->index = index;
		if (index == 0) {
			/* only yuv4:2:2 format is supported at this point */
			fmt->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
			strcpy(fmt->description,
			       "YCbCr4:2:2 Interleaved UYUV");
			fmt->pixelformat = V4L2_PIX_FMT_UYVY;
		} else if (index == 1) {
			fmt->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
			strcpy(fmt->description,
			       "YCbCr4:2:2 Interleaved YUYV");
			fmt->pixelformat = V4L2_PIX_FMT_YUYV;
		} else {
			ret = -EINVAL;
		}
		break;
	}
	case VIDIOC_G_FMT:
	{
		struct v4l2_format *fmt = (struct v4l2_format *)arg;
		if (fmt->type != V4L2_BUF_TYPE_VIDEO_CAPTURE) {
			ret = -EINVAL;
		} else {
			struct v4l2_pix_format *pixfmt = &fmt->fmt.pix;
			down_interruptible(&vpfe->lock);
			pixfmt->width = vpfe->vwin.width;
			pixfmt->height = vpfe->vwin.height;
			pixfmt->field = vpfe->field;
			pixfmt->pixelformat = vpfe->pixelfmt;
			pixfmt->bytesperline = pixfmt->width * 2;
			pixfmt->sizeimage =
				pixfmt->bytesperline * pixfmt->height;
			pixfmt->colorspace = V4L2_COLORSPACE_SMPTE170M;
			up(&vpfe->lock);
		}
		break;
	}
	case VIDIOC_S_FMT:
	{
		struct v4l2_format *fmt = (struct v4l2_format *)arg;
		struct v4l2_pix_format *pixfmt = &fmt->fmt.pix;
		ccdc_params_ycbcr *params = &vpfe->ccdc_params;
		if (vpfe->started) {	/* make sure streaming is not started */
			ret = -EBUSY;
			break;
		}

		down_interruptible(&vpfe->lock);
		if (fmt->type != V4L2_BUF_TYPE_VIDEO_CAPTURE) {
			ret = -EINVAL;
			up(&vpfe->lock);
			break;
		}
		if ((pixfmt->width + vpfe->vwin.left <=
		     vpfe->bounds.width)
		    & (pixfmt->height + vpfe->vwin.top <=
		       vpfe->bounds.height)) {
			/* this is the case when no scaling is supported */
			/* crop window is directed modified */
			vpfe->vwin.height = pixfmt->height;
			vpfe->vwin.width = pixfmt->width;
			params->win.width = pixfmt->width;
			params->win.height = pixfmt->height;
		} else {
			ret = -EINVAL;
			up(&vpfe->lock);
			break;
		}
		/* setup the CCDC parameters accordingly */
		if (pixfmt->pixelformat == V4L2_PIX_FMT_YUYV) {
			params->pix_order = CCDC_PIXORDER_YCBYCR;
			vpfe->pixelfmt = pixfmt->pixelformat;
		} else if (pixfmt->pixelformat == V4L2_PIX_FMT_UYVY) {
			params->pix_order = CCDC_PIXORDER_CBYCRY;
			vpfe->pixelfmt = pixfmt->pixelformat;
		} else {
			ret = -EINVAL;	/* not supported format */
			up(&vpfe->lock);
			break;
		}
		if (pixfmt->field == V4L2_FIELD_NONE
		    || pixfmt->field == V4L2_FIELD_INTERLACED) {
			params->buf_type = CCDC_BUFTYPE_FLD_INTERLEAVED;
			vpfe->field = pixfmt->field;
		} else if (pixfmt->field == V4L2_FIELD_SEQ_TB) {
			params->buf_type = CCDC_BUFTYPE_FLD_SEPARATED;
			vpfe->field = pixfmt->field;
		} else {
			ret = -EINVAL;
		}

		ret |= DEVICE_CMD(ACTIVE_DEVICE(), VIDIOC_S_FMT, arg);

		up(&vpfe->lock);
		break;
	}
	case VIDIOC_TRY_FMT:
	{
		struct v4l2_format *fmt = (struct v4l2_format *)arg;
		if (fmt->type != V4L2_BUF_TYPE_VIDEO_CAPTURE) {
			ret = -EINVAL;
		} else {
			struct v4l2_pix_format *pixfmt = &fmt->fmt.pix;
			if (pixfmt->width > vpfe->bounds.width
			    || pixfmt->height > vpfe->bounds.height
			    || (pixfmt->pixelformat != V4L2_PIX_FMT_UYVY
				&& pixfmt->pixelformat !=
				V4L2_PIX_FMT_YUYV)) {
				ret = -EINVAL;
			}
		}
		break;
	}
	case VIDIOC_G_STD:
	{
		v4l2_std_id *id = (v4l2_std_id *) arg;
		*id = vpfe->std;
		break;
	}
	case VIDIOC_S_STD:
	{
		v4l2_std_id id = *(v4l2_std_id *) arg;
		int sqp = 0;

		if (vpfe->started) {	/* make sure streaming is not started */
			ret = -EBUSY;
			break;
		}
		down_interruptible(&vpfe->lock);
		if (id & V4L2_STD_625_50) {
			vpfe->std = id;
			vpfe->bounds = vpfe->vwin = pal_bounds;
			vpfe->pixelaspect = pal_aspect;
			vpfe->ccdc_params.win = pal_bounds;

		} else if (id & V4L2_STD_525_60) {
			vpfe->std = id;
			vpfe->bounds = vpfe->vwin = ntsc_bounds;
			vpfe->pixelaspect = ntsc_aspect;
			vpfe->ccdc_params.win = ntsc_bounds;
		} else if (id & VPFE_STD_625_50_SQP) {
			vpfe->std = id;
			vpfe->bounds = vpfe->vwin = palsp_bounds;
			vpfe->pixelaspect = sp_aspect;
			sqp = 1;
			id >>= 32;
		} else if (id & VPFE_STD_525_60_SQP) {
			vpfe->std = id;
			sqp = 1;
			vpfe->std = id;
			id >>= 32;
			vpfe->bounds = vpfe->vwin = ntscsp_bounds;
			vpfe->pixelaspect = sp_aspect;
			vpfe->ccdc_params.win = ntscsp_bounds;
		} else if (id & VPFE_STD_AUTO) {
			vpfe->bounds = vpfe->vwin = pal_bounds;
			vpfe->pixelaspect = pal_aspect;
			vpfe->ccdc_params.win = pal_bounds;
			vpfe->std = id;
		} else if (id & VPFE_STD_AUTO_SQP) {
			vpfe->std = id;
			vpfe->bounds = vpfe->vwin = palsp_bounds;
			vpfe->pixelaspect = sp_aspect;
			sqp = 1;
			vpfe->pixelaspect = sp_aspect;
		} else {
			ret = -EINVAL;
		}

		vpfe->capture_params.mode = id;
		vpfe->capture_params.squarepixel = sqp;

		ret |= DEVICE_CMD(ACTIVE_DEVICE(),
				  VPFE_CMD_CONFIG_CAPTURE,
				  &vpfe->capture_params);

		up(&vpfe->lock);
		break;
	}
	case VIDIOC_ENUMSTD:
	{
		struct v4l2_standard *std = (struct v4l2_standard *)arg;
		u32 index = std->index;
		memset(std, 0, sizeof(*std));
		std->index = index;
		if (index == 0) {
			std->id = V4L2_STD_525_60;
			strcpy(std->name, "SD-525line-30fps");
			std->framelines = 525;
			std->frameperiod.numerator = 1001;
			std->frameperiod.denominator = 30000;
		} else if (index == 1) {
			std->id = V4L2_STD_625_50;
			strcpy(std->name, "SD-625line-25fps");
			std->framelines = 625;
			std->frameperiod.numerator = 1;
			std->frameperiod.denominator = 25;
		} else if (index == 2) {
			std->id = VPFE_STD_625_50_SQP;
			strcpy(std->name,
			       "SD-625line-25fps square pixel");
			std->framelines = 625;
			std->frameperiod.numerator = 1;
			std->frameperiod.denominator = 25;
		} else if (index == 3) {
			std->id = VPFE_STD_525_60_SQP;
			strcpy(std->name,
			       "SD-525line-25fps square pixel");
			std->framelines = 525;
			std->frameperiod.numerator = 1001;
			std->frameperiod.denominator = 30000;
		} else if (index == 4) {
			std->id = VPFE_STD_AUTO;
			strcpy(std->name, "automatic detect");
			std->framelines = 625;
			std->frameperiod.numerator = 1;
			std->frameperiod.denominator = 1;
		} else if (index == 5) {
			std->id = VPFE_STD_AUTO_SQP;
			strcpy(std->name,
			       "automatic detect square pixel");
			std->framelines = 625;
			std->frameperiod.numerator = 1;
			std->frameperiod.denominator = 1;
		} else {
			ret = -EINVAL;
		}
		break;
	}
	case VIDIOC_ENUMINPUT:
	{
		struct v4l2_input *input = (struct v4l2_input *)arg;

		if( input->index < 0 || input->index >= NTOSD_INPUTS)
			ret = -EINVAL;

		memcpy(input, &ntosd_inputs[input->index], sizeof(struct v4l2_input));
		break;
	}
	case VIDIOC_G_INPUT:
	{
		int *index = (int *)arg;

		*index = vpfe->capture_params.amuxmode;
		break;
	}
	case VIDIOC_S_INPUT:
	{
		int *index = (int *)arg;

		if (*index == VPFE_AMUX_COMPOSITE)
			vpfe_select_capture_device(VPFE_CAPTURE_ID_TVP5150);
		else if (*index == VPFE_AMUX_COMPONENT)
			vpfe_select_capture_device(VPFE_CAPTURE_ID_TVP7000);
		else
			return -EINVAL;

		vpfe->capture_params.amuxmode = *index;

		ret = DEVICE_CMD(ACTIVE_DEVICE(), VIDIOC_S_INPUT, index);

		break;
	}
	case VIDIOC_CROPCAP:
	{
		struct v4l2_cropcap *cropcap =
			(struct v4l2_cropcap *)arg;
		cropcap->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		down_interruptible(&vpfe->lock);
		cropcap->bounds = cropcap->defrect = vpfe->vwin;
		cropcap->pixelaspect = vpfe->pixelaspect;
		up(&vpfe->lock);
		break;
	}
	case VIDIOC_G_PARM:
	{
		struct v4l2_streamparm *parm =
		    (struct v4l2_streamparm *)arg;
		if (parm->type != V4L2_BUF_TYPE_VIDEO_CAPTURE) {
			/* only capture is supported */
			ret = -EINVAL;
		} else {
			struct v4l2_captureparm *capparm =
				&parm->parm.capture;
			memset(capparm, 0,
			       sizeof(struct v4l2_captureparm));
			down_interruptible(&vpfe->lock);
			if (vpfe->std & V4L2_STD_625_50) {
				capparm->timeperframe.numerator = 1;
				capparm->timeperframe.denominator = 25;	/* PAL 25fps */
			} else {
				capparm->timeperframe.numerator = 1001;
				capparm->timeperframe.denominator = 30000;	/*NTSC 29.97fps */
			}
			capparm->readbuffers = vpfe->numbuffers;
			up(&vpfe->lock);
		}
		break;
	}
	case VIDIOC_G_CTRL:
		down_interruptible(&vpfe->lock);

		ret |= DEVICE_CMD(ACTIVE_DEVICE(),
				  VIDIOC_G_CTRL, arg);

		up(&vpfe->lock);
		break;
	case VIDIOC_S_CTRL:
		down_interruptible(&vpfe->lock);

		ret |= DEVICE_CMD(ACTIVE_DEVICE(),
				  VIDIOC_S_CTRL, arg);

		up(&vpfe->lock);
		break;
	case VIDIOC_QUERYCTRL:
		down_interruptible(&vpfe->lock);

		ret |= DEVICE_CMD(ACTIVE_DEVICE(),
				  VIDIOC_QUERYCTRL, arg);

		up(&vpfe->lock);
		break;
	case VIDIOC_G_CROP:
	{
		struct v4l2_crop *crop = arg;
		if (crop->type != V4L2_BUF_TYPE_VIDEO_CAPTURE) {
			ret = -EINVAL;
		} else {
			crop->c = vpfe->vwin;
		}
		break;
	}
	case VIDIOC_S_CROP:
	{
		struct v4l2_crop *crop = arg;
		ccdc_params_ycbcr *params = &vpfe->ccdc_params;
		if (vpfe->started) {	/* make sure streaming is not started */
			ret = -EBUSY;
			break;
		}
		/*adjust the width to 16 pixel boundry */
                crop->c.width = ((crop->c.width + 15 )/16 ) * 16;

		/* make sure parameters are valid */
		if (crop->type == V4L2_BUF_TYPE_VIDEO_CAPTURE
		    && (crop->c.left + crop->c.width
			<= vpfe->bounds.left + vpfe->bounds.width)
		    && (crop->c.top + crop->c.height
			<= vpfe->bounds.top + vpfe->bounds.height)) {

			down_interruptible(&vpfe->lock);
			vpfe->vwin = crop->c;
			params->win = vpfe->vwin;
			up(&vpfe->lock);
		} else {
			ret = -EINVAL;
		}
		break;
	}
	case VIDIOC_QUERYSTD:
	{
		v4l2_std_id *id = (v4l2_std_id *) arg;
		down_interruptible(&vpfe->lock);

		ret |= DEVICE_CMD(ACTIVE_DEVICE(),
				  VIDIOC_QUERYSTD, id);

		up(&vpfe->lock);
		break;
	}
	case VIDIOC_G_PRIORITY:
	{
		enum v4l2_priority *p = arg;
		*p = v4l2_prio_max(&vpfe->prio);
		break;
	}
	case VIDIOC_S_PRIORITY:
	{
		enum v4l2_priority *p = arg;
		ret = v4l2_prio_change(&vpfe->prio, &fh->prio, *p);
		break;
	}

	case VIDIOC_REQBUFS:
		if (vpfe->io_usrs != 0) {
			ret = -EBUSY;
			break;
		}
		down_interruptible(&vpfe->lock);
		videobuf_queue_init(&vpfe->bufqueue, &video_qops, NULL,
		&vpfe->irqlock, V4L2_BUF_TYPE_VIDEO_CAPTURE, vpfe->field,
		sizeof(struct videobuf_buffer), fh);

		videobuf_set_buftype(&vpfe->bufqueue, VIDEOBUF_BUF_LINEAR);

		fh->io_allowed = TRUE;
		vpfe->io_usrs = 1;
		INIT_LIST_HEAD(&vpfe->dma_queue);
		ret = videobuf_reqbufs(&vpfe->bufqueue, arg);
		up(&vpfe->lock);
		break;
	case VIDIOC_QUERYBUF:
		ret = videobuf_querybuf(&vpfe->bufqueue, arg);
		break;
	case VIDIOC_QBUF:
		if (!fh->io_allowed)
			ret = -EACCES;
		else
			ret = videobuf_qbuf(&vpfe->bufqueue, arg);
		break;
	case VIDIOC_DQBUF:
		if (!fh->io_allowed)
			ret = -EACCES;
		else
			ret =  videobuf_dqbuf(&vpfe->bufqueue, arg, 0);
		break;
	case VIDIOC_STREAMON:
		if (!fh->io_allowed) {
			ret = -EACCES;
			break;
		}
		if(vpfe->started){
			ret = -EBUSY;
			break;
		}
		ret = videobuf_streamon(&vpfe->bufqueue);
		if(ret) break;

		down_interruptible(&vpfe->lock);
		/* get the current and next frame buffers */
		/* we expect at least one buffer is in driver at this point */
		/* if not, error is returned */
		if (list_empty(&vpfe->dma_queue)) {
			ret = -EIO;
			break;
		}
		debug_print(KERN_INFO "cur frame %x.\n",
			    vpfe->dma_queue.next);
		vpfe->nextFrm = vpfe->curFrm =
			list_entry(vpfe->dma_queue.next,
				   struct videobuf_buffer, queue);
		/* remove the buffer from the queue */
		list_del(&vpfe->curFrm->queue);
		vpfe->curFrm->state = STATE_ACTIVE;

		/* sense the current video input standard */
		ret |= DEVICE_CMD(ACTIVE_DEVICE(),
				  VPFE_CMD_CONFIG_CAPTURE,
				  &vpfe->capture_params);
		/* configure the ccdc and resizer as needed   */
		/* start capture by enabling CCDC and resizer */
		ccdc_config_ycbcr(&vpfe->ccdc_params);
		/* setup the memory address for the frame buffer */
		ccdc_setfbaddr(((unsigned long)(vpfe->curFrm->boff)));
		/* enable CCDC */
		vpfe->field_id = 0;
		vpfe->started = TRUE;
		vpfe->mode_changed = FALSE;
		vpfe->field_offset =
			(vpfe->vwin.height - 2) * vpfe->vwin.width;
		ccdc_enable(TRUE);
		up(&vpfe->lock);
		debug_print(KERN_INFO "started video streaming.\n");
		break;
	case VIDIOC_STREAMOFF:
	{
		if (!fh->io_allowed) {
			ret = -EACCES;
			break;
		}
		if(!vpfe->started){
			ret = -EINVAL;
			break;
		}
		/* disable CCDC */
		down_interruptible(&vpfe->lock);
		ccdc_enable(FALSE);
		vpfe->started = FALSE;
		up(&vpfe->lock);
		ret = videobuf_streamoff(&vpfe->bufqueue);
		break;
	}
	case VPFE_CMD_CONFIG_CCDC:
	{
		/* this can be used directly and bypass the V4L2 APIs */
		ccdc_params_ycbcr *params = &vpfe->ccdc_params;
		if(vpfe->started){
		/* only allowed if streaming is not started */
			ret = -EBUSY;
			break;
		}
		down_interruptible(&vpfe->lock);
		/* make sure the other v4l2 related fields
		   have consistant settings */
		*params = (*(ccdc_params_ycbcr *) arg);
		vpfe->vwin = params->win;
		if (params->buf_type == CCDC_BUFTYPE_FLD_INTERLEAVED) {
			vpfe->field = V4L2_FIELD_INTERLACED;
		} else if (params->buf_type ==
			   CCDC_BUFTYPE_FLD_SEPARATED) {
			vpfe->field = V4L2_FIELD_SEQ_TB;
		}
		if (params->pix_order == CCDC_PIXORDER_YCBYCR) {
			vpfe->pixelfmt = V4L2_PIX_FMT_YUYV;
		} else if (params->pix_order == CCDC_PIXORDER_CBYCRY) {
			vpfe->pixelfmt = V4L2_PIX_FMT_UYVY;
		}
		up(&vpfe->lock);
		break;
	}
	case VPFE_CMD_CONFIG_CAPTURE:
	/* this can be used directly and bypass the V4L2 APIs */
	{
		/* the settings here must be consistant with that of the CCDC's,
		   driver does not check the consistancy */
		struct vpfe_capture_params *params =
			(struct vpfe_capture_param *) arg;
		v4l2_std_id std = 0;
		if(vpfe->started){
		/* only allowed if streaming is not started */
			ret = -EBUSY;
			break;
		}
		down_interruptible(&vpfe->lock);

		std = params->mode;
		if (params->squarepixel) {	/* square pixel mode */
			std <<= 32;
		}

		if (std & V4L2_STD_625_50) {
			vpfe->bounds = pal_bounds;
			vpfe->pixelaspect = pal_aspect;
		} else if (std & V4L2_STD_525_60) {
			vpfe->bounds = ntsc_bounds;
			vpfe->pixelaspect = ntsc_aspect;
		} else if (std & VPFE_STD_625_50_SQP) {
			vpfe->bounds = palsp_bounds;
			vpfe->pixelaspect = sp_aspect;
		} else if (std & VPFE_STD_525_60_SQP) {
			vpfe->bounds = ntscsp_bounds;
			vpfe->pixelaspect = sp_aspect;
		}
		vpfe->std = std;
		ret |= DEVICE_CMD(ACTIVE_DEVICE(),
				  VPFE_CMD_CONFIG_CAPTURE,
				  params);
		vpfe->capture_params = *params;
		up(&vpfe->lock);
		break;
	}
	default:
		ret = -ENOIOCTLCMD;
		break;
	}			/* end switch(cmd) */
	return ret;
}

static int vpfe_ioctl(struct inode *inode, struct file *file,
		      unsigned int cmd, unsigned long arg)
{
	int ret;
	ret =  video_usercopy(inode, file, cmd, arg, vpfe_doioctl);
	if( cmd == VIDIOC_S_FMT || cmd == VIDIOC_TRY_FMT ){
		ret = video_usercopy(inode, file, VIDIOC_G_FMT,
				     arg, vpfe_doioctl);
	}
	return ret;
}

static int vpfe_mmap(struct file *file, struct vm_area_struct *vma)
{
	return videobuf_mmap_mapper(&vpfe_device.bufqueue, vma);
}

static int vpfe_open(struct inode *inode, struct file *filep)
{
	int minor = iminor(inode);
	vpfe_obj *vpfe = NULL;
	vpfe_fh *fh = NULL;

	debug_print(KERN_INFO "vpfe: open minor=%d\n", minor);

	/* check to make sure the minor numbers match */
	if (vpfe_device.video_dev && vpfe_device.video_dev->minor == minor) {
		vpfe = &vpfe_device;
	} else {		/* device not found here */
		return -ENODEV;
	}

	/* allocate per filehandle data */
	if ((fh = kmalloc(sizeof(*fh), GFP_KERNEL)) == NULL) {
		return -ENOMEM;
	}
	filep->private_data = fh;
	fh->dev = vpfe;
	fh->io_allowed = FALSE;
	fh->prio = V4L2_PRIORITY_UNSET;
	v4l2_prio_open(&vpfe->prio, &fh->prio);
	vpfe->usrs++;

	/* active the tvp5150 */
	vpfe_select_capture_device(VPFE_CAPTURE_ID_TVP5150);
	/* detect if there is valid signal input */
	{
		v4l2_std_id *id;

		*id = 0;
		down_interruptible(&vpfe->lock);
		DEVICE_CMD(ACTIVE_DEVICE(), VIDIOC_QUERYSTD, id);
		up(&vpfe->lock);
		if (*id != V4L2_STD_UNKNOWN)
			vpfe->capture_params.amuxmode = VPFE_AMUX_COMPOSITE;
		else
		{
			/*  no valid input for tvp5150 then try tvp7000
			 *  activate the tvp7000, and detect if there is valid input
			 *  Todo after tvp7000 driver available.
			 */
			debug_print(KERN_INFO "no valid signal input\n");
			/*  the device can be opened even without valid input
			 *  so if no valid input, use a default one
			 */
			vpfe->capture_params.amuxmode = VPFE_AMUX_COMPOSITE;
		}
	}

	return 0;
}

static int vpfe_release(struct inode *inode, struct file *filep)
{
	vpfe_fh *fh = filep->private_data;
	vpfe_obj *vpfe = fh->dev;

	down_interruptible(&vpfe->lock);
	if (fh->io_allowed) {
		vpfe->io_usrs = 0;
		ccdc_enable(FALSE);
		vpfe->started = FALSE;
		videobuf_queue_cancel(&vpfe->bufqueue);
		vpfe->numbuffers = VPFE_DEFNUM_FBUFS;
	}
	vpfe->usrs--;
	v4l2_prio_close(&vpfe->prio, &fh->prio);
	filep->private_data = NULL;
	kfree(fh);
	up(&vpfe->lock);

	return 0;
}

static struct file_operations vpfe_fops = {
	.owner = THIS_MODULE,
	.open = vpfe_open,
	.release = vpfe_release,
	.ioctl = vpfe_ioctl,
	.mmap = vpfe_mmap
};

static struct video_device vpfe_video_template = {
	.name = "vpfe",
	.type = VID_TYPE_CAPTURE | VID_TYPE_CLIPPING | VID_TYPE_SCALES,
	.hardware = 0,
	.fops = &vpfe_fops,
	.minor = -1,
};

static void vpfe_platform_release(struct device *device)
{
	/* This is called when the reference count goes to zero. */
}

static int __init vpfe_probe(struct device *device)
{
	struct video_device *vfd;
	vpfe_obj *vpfe = &vpfe_device;

	/* alloc video device */
	if ((vfd = video_device_alloc()) == NULL) {
		return -ENOMEM;
	}
	*vfd = vpfe_video_template;
	vfd->dev = device;
	vfd->release = video_device_release;
	snprintf(vfd->name, sizeof(vfd->name), "DM644X_VPFE_DRIVER_V%d.%d.%d",
		 (VPFE_VERSION_CODE >> 16) & 0xff,
		 (VPFE_VERSION_CODE >> 8) & 0xff, (VPFE_VERSION_CODE) & 0xff);

	vpfe->video_dev = vfd;
	vpfe->usrs = 0;
	vpfe->io_usrs = 0;
	vpfe->started = FALSE;
	vpfe->latest_only = TRUE;

	v4l2_prio_init(&vpfe->prio);
	init_MUTEX(&vpfe->lock);
	init_MUTEX(&vpfe->device_list_lock);
	INIT_LIST_HEAD(&vpfe->capture_device_list);
	SET_ACTIVE_DEVICE(NULL);

	/* register video device */
	debug_print(KERN_INFO "trying to register vpfe device.\n");
	debug_print(KERN_INFO "vpfe=%x,vpfe->video_dev=%x\n", (int)vpfe,
		    (int)&vpfe->video_dev);
	if (video_register_device(vpfe->video_dev, VFL_TYPE_GRABBER, -1) < 0) {
		video_device_release(vpfe->video_dev);
		vpfe->video_dev = NULL;
		return -1;
	}

	debug_print(KERN_INFO "DM644X vpfe: driver version V%d.%d.%d loaded\n",
		    (VPFE_VERSION_CODE >> 16) & 0xff,
		    (VPFE_VERSION_CODE >> 8) & 0xff,
		    (VPFE_VERSION_CODE) & 0xff);

	debug_print(KERN_INFO "vpfe: registered device video%d\n",
		    vpfe->video_dev->minor & 0x1f);

	/* all done */
	return 0;
}

int vpfe_capture_device_register(struct vpfe_capture_device *device)
{
	if (device == NULL)
		return -EINVAL;

	/* register capture device to vpfe. */
	down_interruptible(&vpfe_device.device_list_lock);
	list_add_tail(&device->device_list, &vpfe_device.capture_device_list);
	up(&vpfe_device.device_list_lock);

	/* call capture device specific init routine */
	down_interruptible(&vpfe_device.lock);
	DEVICE_INIT(device, &vpfe_device.capture_params);
	up(&vpfe_device.lock);

	debug_print(KERN_INFO "VPFE Capture device %s registered, id = %d.\n",
		    device->name, device->id);
	return 0;
}
EXPORT_SYMBOL(vpfe_capture_device_register);

int vpfe_capture_device_unregister(struct vpfe_capture_device *device)
{
	if (device == NULL)
		return -EINVAL;

	/* unregister it from vpfe. */
	down_interruptible(&vpfe_device.device_list_lock);
	list_del(&device->device_list);
	up(&vpfe_device.device_list_lock);

	down_interruptible(&vpfe_device.lock);
	/* if the device to be unregistered is active,
		deactivate it! */
	if (IS_ACTIVE(device)) {
		DEVICE_DEACTIVATE(device);
		SET_ACTIVE_DEVICE(NULL);
	}
	/* call device specific routine to do the clean up. */
	DEVICE_CLEANUP(device);
	up(&vpfe_device.lock);

	debug_print(KERN_INFO "VPFE Capture device %s unregistered, id = %d\n",
		    device->name, device->id);
	return 0;
}
EXPORT_SYMBOL(vpfe_capture_device_unregister);

static int capture_device_all_unregister(void)
{
	int ret = 0;
	struct vpfe_capture_device *device;

	down_interruptible(&vpfe_device.lock);
	list_for_each_entry(device, &vpfe_device.capture_device_list,
			    device_list){
		ret |= vpfe_capture_device_unregister(device);
	}
	up(&vpfe_device.lock);

	return ret;
}

static int vpfe_remove(struct device *device)
{
	/* un-register device */
	video_unregister_device(vpfe_device.video_dev);

	return 0;
}

#ifdef NEW
static struct platform_driver vpfe_driver = {
	.driver = {
		.name		= "VPFE",
		.owner		= THIS_MODULE,
	},
	.probe			= vpfe_probe,
	.remove			= vpfe_remove,
};

#else
static struct device_driver vpfe_driver = {
	.name = "vpfe",
	.bus = &platform_bus_type,
	.probe = vpfe_probe,
	.remove = vpfe_remove,
};
#endif

static struct platform_device _vpfe_device = {
	.name = "vpfe",
	.id = 1,
	.dev = {
		.release = vpfe_platform_release,
	}
};

static int vpfe_init(void)
{
	int i = 0;
	int result = 0;
	void *mem;
	/* allocate memory at initialization time to guarentee availability */
	for (i = 0; i < VPFE_DEFNUM_FBUFS; i++) {
		mem = (void *)__get_free_pages(GFP_KERNEL | GFP_DMA,
					       VPFE_MAX_FBUF_ORDER);
		if (mem) {
			unsigned long adr = (unsigned long)mem;
			u32 size = PAGE_SIZE << VPFE_MAX_FBUF_ORDER;
			while (size > 0) {
				/* make sure the frame buffers
				   are never swapped out of memory */
				SetPageReserved(virt_to_page(adr));
				adr += PAGE_SIZE;
				size -= PAGE_SIZE;
			}
			vpfe_device.fbuffers[i] = (u8 *) mem;
			debug_print(KERN_INFO "memory address %d\t%x\n", i,
				    mem);
		} else {
			while (--i >= 0) {
				free_pages((unsigned long)vpfe_device.fbuffers[i],
					   VPFE_MAX_FBUF_ORDER);
			}
			debug_print(KERN_INFO
				    "frame buffer memory allocation failed.\n");
			return -ENOMEM;
		}
	}
	if (driver_register(&vpfe_driver) != 0) {
		debug_print(KERN_INFO "driver registration failed\n");
		return -1;
	}
	if (platform_device_register(&_vpfe_device) != 0) {
		driver_unregister(&vpfe_driver);
		debug_print(KERN_INFO "device registration failed\n");
		return -1;
	}

	ccdc_reset();
	/* setup interrupt handling */
	result = request_irq(IRQ_VDINT0, vpfe_isr, IRQF_DISABLED,
			     "dm644xv4l2", (void *)&vpfe_device);
	if (result < 0) {
		printk(KERN_ERR "DaVinci v4l2 capture driver: cannot initialize IRQ\n");
		return result;
	}

	printk(KERN_INFO "DaVinci v4l2 capture driver V1.0 loaded\n");
	return 0;
}

static void vpfe_cleanup(void)
{
	int i = vpfe_device.numbuffers;
	platform_device_unregister(&_vpfe_device);
	driver_unregister(&vpfe_driver);

	capture_device_all_unregister();

	/* disable interrupt */
	free_irq(IRQ_VDINT0, &vpfe_device);

	while (--i >= 0) {
		free_pages((unsigned long)vpfe_device.fbuffers[i],
			   VPFE_MAX_FBUF_ORDER);
	}
	debug_print(KERN_INFO "vpfe: un-registered device video.\n");
}

module_init(vpfe_init);
module_exit(vpfe_cleanup);
