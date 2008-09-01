/*
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
/* davinci_vpfe.h */

#ifndef DAVINCI_VPFE_H
#define DAVINCI_VPFE_H
#ifdef __KERNEL__
#include <media/v4l2-dev.h>
#endif

#include <media/ccdc_davinci.h>

#define TRUE 1
#define FALSE 0

/* vpfe specific video standards */
#define VPFE_STD_625_50_SQP ((V4L2_STD_625_50)<<32)
#define VPFE_STD_525_60_SQP ((V4L2_STD_525_60)<<32)
#define VPFE_STD_AUTO ((v4l2_std_id)(0x1000000000000000ULL))
#define VPFE_STD_AUTO_SQP ((v4l2_std_id)(0x2000000000000000ULL))

#define VPFE_CMD_CONFIG_CCDC _IOW('V',BASE_VIDIOC_PRIVATE + 1,ccdc_params_ycbcr)
#define VPFE_CMD_LATEST_FRM_ONLY   _IOW('V',BASE_VIDIOC_PRIVATE + 2,int)
#define VPFE_CMD_CONFIG_CAPTURE _IOW('V', BASE_VIDIOC_PRIVATE + 3,\
		struct vpfe_capture_params)
#define VPFE_CMD_CAPTURE_ACTIVE _IOW('V', BASE_VIDIOC_PRIVATE + 4, long int)

#define    VPFE_AMUX_COMPOSITE0 0
#define    VPFE_AMUX_COMPOSITE1 1
#define    VPFE_AMUX_COMPONENT  2
#define    VPFE_AMUXES 3

/* settings for commonly used video formats */
#define VPFE_WIN_NTSC    {0,0,720,480}
#define VPFE_WIN_PAL     {0,0,720,576}
#define VPFE_WIN_NTSC_SP {0,0,640,480}	/* ntsc square pixel */
#define VPFE_WIN_PAL_SP  {0,0,768,576}	/* pal square pixel */
#define VPFE_WIN_CIF     {0,0,352,288}
#define VPFE_WIN_QCIF    {0,0,176,144}
#define VPFE_WIN_QVGA    {0,0,320,240}
#define VPFE_WIN_SIF     {0,0,352,240}
#define VPFE_WIN_HD480P    {69,0,720,480}
#define VPFE_WIN_HD576P    {72,0,720,576}
#define VPFE_WIN_HD720P    {185,0,1280,720}
#define VPFE_WIN_HD1080I    {140,0,1920,1080}

#define VPFE_CAPTURE_ID_TVP5150		0
#define VPFE_CAPTURE_ID_TVP7000		1


#ifdef __KERNEL__

#include <media/video-buf.h>

#define VPFE_MAJOR_RELEASE 0
#define VPFE_MINOR_RELEASE 0
#define VPFE_BUILD         1

#define VPFE_VERSION_CODE \
     (VPFE_MAJOR_RELEASE<<16)  | (VPFE_MINOR_RELEASE<<8) | VPFE_BUILD

/* By default, the driver is setup for auto-swich mode */
#define VPFE_DEFAULT_STD VPFE_STD_AUTO

#define VPFE_PIXELASPECT_NTSC {11, 10}
#define VPFE_PIXELASPECT_PAL  {54, 59}
#define VPFE_PIXELASPECT_NTSC_SP    {1, 1}
#define VPFE_PIXELASPECT_PAL_SP     {1, 1}
#define VPFE_PIXELASPECT_DEFAULT    {1, 1}

#define VPFE_MAX_FRAME_WIDTH      1920	/* account for PAL Square pixel mode */
#define VPFE_MAX_FRAME_HEIGHT     1080	/* account for PAL                   */
/* 4:2:2 data */
#define VPFE_MAX_FBUF_SIZE       (VPFE_MAX_FRAME_WIDTH*VPFE_MAX_FRAME_HEIGHT*2)
/* frame buffers allocate at driver initialization time */
#define VPFE_DEFNUM_FBUFS             3

#define VPFE_MAX_FBUF_ORDER \
   get_order(roundup_pow_of_two(VPFE_MAX_FBUF_SIZE))

struct vpfe_capture_params{
	v4l2_std_id mode;
	int amuxmode;
	int enablebt656sync;
	int squarepixel;
};

struct vpfe_capture_device
{
	const char *name;
	int id;
	struct list_head	device_list;
	int (*capture_device_init)(struct vpfe_capture_params *param);
	int (*capture_device_active)(void);
	int (*capture_device_deactive)(void);
	int (*capture_device_cmd)(u32 cmd, void *arg);
	int (*capture_device_cleanup)(void);
};


/* device object */
typedef struct vpfe_obj {
	struct video_device *video_dev;
	struct videobuf_queue bufqueue;/* queue with frame buffers      */
	struct list_head dma_queue;
	u32 latest_only;		/* indicate whether to return the most */
					/* recent captured buffers only        */
	u32 usrs;
	u32 io_usrs;
	struct v4l2_prio_state prio;
	v4l2_std_id std;
	struct v4l2_rect vwin;
	struct v4l2_rect bounds;
	struct v4l2_fract pixelaspect;
       	spinlock_t irqlock;
	struct semaphore lock;
	enum v4l2_field field;
	u32 pixelfmt;
	u32 numbuffers;
	u8* fbuffers[VIDEO_MAX_FRAME];
	struct videobuf_buffer *curFrm;
	struct videobuf_buffer *nextFrm;
	int field_id;
	int mode_changed;
	int started;
	int field_offset;
	struct semaphore device_list_lock;
	struct vpfe_capture_params capture_params;
	struct list_head capture_device_list;
	struct vpfe_capture_device *active_device;
	ccdc_params_ycbcr ccdc_params;
} vpfe_obj;

/* file handle */
typedef struct vpfe_fh {
	struct vpfe_obj *dev;
	int io_allowed;
	enum v4l2_priority prio;
} vpfe_fh;

/**
 *  register the capture device
 * @param device
 * 		device that to be registered.
 * @return int
 * 		0:sucess, otherwise, failed.
 */
int vpfe_capture_device_register(struct vpfe_capture_device *device);

/**
 * 	unregister the capture device
 * @param device
 * 		device that to be unregistered.
 * @return int
 * 	0:sucess, otherwise, failed.
 */
int vpfe_capture_device_unregister(struct vpfe_capture_device *device);
#endif

#endif /* DAVINCI_VPFE_H */
