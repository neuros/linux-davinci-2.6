/*
 * drivers/video/davincifb.c
 *
 * Framebuffer driver for Texas Instruments DaVinci display controller.
 *
 * Copyright (C) 2006 Texas Instruments, Inc.
 * Rishi Bhattacharya <support@ti.com>
 *
 * Leveraged from the framebuffer driver for OMAP24xx
 * written by Andy Lowe (source@mvista.com)
 * Copyright (C) 2004 MontaVista Software, Inc.
 *
 * This file is licensed under the terms of the GNU General Public License
 * version 2. This program is licensed "as is" without any warranty of any
 * kind, whether express or implied.
 */

/*
 * TODO:
 *
 * + Make the module load/unload *DONE*
 * + Add support to change resolution at runtime *DONE*
 * + Add support for different bpp per plane *DONE*
 * + Remove all the dobule/triple buffering thing, make it variable based on
 * + Add support to change video output at runtime *DONE*
 * + Add support to list avilable modes
 * the virtual size, support only multiple of real res (2x, 3x, 4x, etc)
 * + Changing the resolution on vid0 should change the output size too
 * + x and y aren't reserved[0] or reserved[1], fix all referecnes to it
 * + The hardware has several bugs, use defines to handle that, current chips
 * i own are revision 1.3
 * + Rename all window functions to be on the same form
 * + Split the check_var we might need to add more unsupported cases and that
 * function is getting big
 * + Remove dparams completely and redo the parser of the parameters, also
 * add module parameters
 * + The status of each window isnt stored and every mode set resets every
 * window configuration, whcih is really wrong
 *
 * Notes:
 * + Every framebuffer but the vid0 can have a size different to the output size
 * + Chaing thr size on vid0 will change the output clocks
 * + The memory is already allocated
 * + To change the output port (composite, component, whatever) just do a
 * echo X > /sys/class/video_output/venc where X can be a number from the enum
 * at davincifb.h Note that for some output modes, some ports might not be
 * available, like 720p over composite
 */

#define DEBUG
/* TODO we should replace this VID0FIX define with a way to get the chip
 * revision
 */
#define VID0FIX

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/string.h>
#include <linux/mm.h>
#include <linux/tty.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/fb.h>
#include <linux/init.h>
#include <linux/dma-mapping.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/video_output.h>

#include <asm/irq.h>
#include <asm/uaccess.h>
#include <asm/arch/hardware.h>

#include <video/davincifb.h>
#include <asm/system.h>

#ifdef CONFIG_THS8200
#include <asm/arch/davinci-ths8200.h>
#endif /* CONFIG_THS8200 */

#if 0
/* TODO call the right function to enable LCD */
#include <asm/arch/gpio.h>
#include <asm/arch/mux.h>
#endif

#define MODULE_NAME "davincifb"

/* needed prototypes */
static int davincifb_check_var(struct fb_var_screeninfo *var,
			       struct fb_info *info);
static int davincifb_set_par(struct fb_info *info);
static struct fb_ops davincifb_ops;

/*============================================================================*
 *               Display controller register I/O routines                     *
 *============================================================================*/
/*
 * Read a register
 */
static __inline__ u32 dispc_reg_in(u32 offset)
{
	return inl(offset);
}
/*
 * Write a register
 */
static __inline__ u32 dispc_reg_out(u32 offset, u32 val)
{
	outl(val, offset);
	return val;
}
/*
 * Write a register and merge it with the current value
 */
static __inline__ u32 dispc_reg_merge(u32 offset, u32 val, u32 mask)
{
	u32 addr = offset;
	u32 new_val = (inl(addr) & ~mask) | (val & mask);
	outl(new_val, addr);
	return new_val;
}

struct dm_win_info {
	struct fb_info info;

	/* TODO this can be infered from the registers */
	/* X and Y position */
	unsigned int x, y;
	unsigned char enabled;

	/* framebuffer area */
	dma_addr_t fb_base_phys;
	unsigned long fb_base;
	unsigned long fb_size;

	u32 pseudo_palette[17];

	/* flag to identify if framebuffer area is fixed already or not */
	unsigned long sdram_address;
	struct dm_info *dm;
	unsigned int win;
};

struct dm_extended_mode {
	char *name;
	unsigned int basex;
	unsigned int basey;
	unsigned int vstarta;
};

struct dm_info {
	/* to map the registers */
	dma_addr_t mmio_base_phys;
	unsigned long mmio_base;
	unsigned long mmio_size;

	wait_queue_head_t vsync_wait;
	unsigned long vsync_cnt;
	int timeout;

	/* this is the function that configures the output device (NTSC/PAL/LCD)
	 * for the required output format (composite/s-video/component/rgb)
	 */
	void (*output_device_config) (int on); /* FIXME, remove this */

	struct device *dev;
	struct dm_win_info *windows[DAVINCIFB_WINDOWS];
	unsigned int windows_mask;
	struct output_device *output;
	unsigned int output_sel;
	struct davincifb_mach_info *mach_info;
	int curr_mode; /* index on the modedb that we are using */
};

/* All window widths have to be rounded up to a multiple of 32 bytes */

/* The DAVINCIFB_WIN_OSD0 window has to be always within DAVINCIFB_WIN_VID0. Plus, since it is in RGB565
 * mode, it _cannot_ overlap with DAVINCIFB_WIN_VID1.
 * For defaults, we are setting the DAVINCIFB_WIN_OSD0 window to be displayed in the top
 * left quadrant of the screen, and the DAVINCIFB_WIN_VID1 in the bottom right quadrant.
 * So the default 'xres' and 'yres' are set to  half of the screen width and
 * height respectively. Note however that the framebuffer size is allocated
 * for the full screen size so the user can change the 'xres' and 'yres' by
 * using the FBIOPUT_VSCREENINFO ioctl within the limits of the screen size.
 */
#define round_32(width)	((((width) + 31) / 32) * 32 )
/* get FB window buffer size */
static inline int fb_window_size(int width, int height, int bufnum)
{
	return round_32(width * 16 / 8) * height * bufnum;
}

static char *dm_win_names[DAVINCIFB_WINDOWS] = {
	[DAVINCIFB_WIN_OSD0] = "dm_osd0_fb",
	[DAVINCIFB_WIN_OSD1] = "dm_osd1_fb",
	[DAVINCIFB_WIN_VID0] = "dm_vid0_fb",
	[DAVINCIFB_WIN_VID1] = "dm_vid1_fb"
};


static inline int is_win(const struct dm_win_info *w, unsigned int win)
{
	if (w->win == win)
		return 1;
	else
		return 0;
}

/* TODO remove this static default vars */
#define BASEX		0x80
#define BASEY		0x12

#define DISP_XRES	720
#define DISP_YRES	480
#define DISP_MEMY	576

#define BASEX480P 0x50
#define BASEY480P 0x5
#define DISP_XRES480P  720
#define DISP_YRES480P  480

#define BASEX720P 0x50
#define BASEY720P 0x5

#define DISP_XRES720P   1280
#define DISP_YRES720P   720
#define DISP_MEMY720P   720

#define BASEX1080I   0x58
#define BASEY1080I   0x5
#define DISP_XRES1080I   1920
#define DISP_YRES1080I   1080
#define DISP_MEMY1080I   1080

/* Random value chosen for now. Should be within the panel's supported range */
#define LCD_PANEL_CLOCK	180000

static const struct dm_extended_mode dm_extended_modedb[] = {
	{ "480i", 0x80, 0x12, 0 },
	{ "576i", 0x80, 0x18, 0 },
	{ "480p", 0x50, 0x5, 0 },
	{ "720p", 300, 30, 0 },
	{ "1080i", 243, 20, 13 },
};

static const struct fb_videomode dmfb_modedb[] = {
	/* name, refresh, xres, yres, pixclock, left_margin, right_margin
	 * upper_margin, lower_margin, hsync_len, vsync_len, sync,
	 *  vmode, flag
	 */
	/* Standard Modes */
	{ "480i", 50, 720, 480, LCD_PANEL_CLOCK, 0, 0, 0, 0, 127, 5, FB_SYNC_BROADCAST,	FB_VMODE_INTERLACED, 0},
	{ "576i", 50, 720, 576, LCD_PANEL_CLOCK, 0, 0, 0, 0, 127, 6, FB_SYNC_BROADCAST,	FB_VMODE_INTERLACED, 0},
	/* Modes provided by THS8200 */
	{ "480p", 30, 720, 480, LCD_PANEL_CLOCK, 122, 15, 36, 8, 0x50, 0x5, FB_SYNC_BROADCAST, FB_VMODE_NONINTERLACED, 0},
	//{ "720p", 30, 1280, 720, LCD_PANEL_CLOCK, 300, 69, 26, 3, 0x50, 0x5, FB_SYNC_BROADCAST, FB_VMODE_NONINTERLACED, 0},
	{ "720p", 30, 1280, 720, LCD_PANEL_CLOCK, 300, 69, 26, 3, 0x50, 0x5, FB_SYNC_BROADCAST, FB_VMODE_NONINTERLACED, 0},
	{ "1080i", 30, 1920, 1080, LCD_PANEL_CLOCK, 200, 79, 13, 31, 0x58, 0x5, FB_SYNC_BROADCAST, FB_VMODE_INTERLACED, 0},

};
/*============================================================================*
 *                              VENC interface                                *
 *============================================================================*/
/* slow down the VCLK to 27MHZ from
 * 74MHZ */
static inline void slow_down_vclk(void)
{
	/* set DCLKPTN works as the clock enable for ENC
	 * clock. */
	outl(VENC_DCKCTL_DCKEC, VENC_DCLKCTL);

	/* DCLK pattern. The specified bit pattern is output in
	  * resolution of ENC clock units.*/
	outl(0x01, VENC_DCLKPTN0);

	/* select MXI mode. Use 27 MHz (from MXI27)
	 * (DAC clock = 27 MHz).VPBE/Video encoder clock
	 * is enabled*/
	outl(VPSS_CLKCTL_ENABLE_VPBE_CLK, VPSS_CLKCTL);
}
/**
 * Enables de digital output on venc
 */
static void enable_digital_output(bool on)
{
	if (on) {

		/* Set PINMUX0 reg to enable LCD
		   (all other settings are kept per u-boot) */
		dispc_reg_merge(PINMUX0, PINMUX0_LOEEN, PINMUX0_LOEEN);

		/* Set PCR register for FULL clock */
		dispc_reg_out(VPBE_PCR, 0);

		/* Enable video clock output and non-inverse clock polarity */
		dispc_reg_out(VENC_VIDCTL, VENC_VIDCTL_VLCKE);

		/* Setting DRGB Matrix registers back to default values */
		dispc_reg_out(VENC_DRGBX0, 0x00000400);
		dispc_reg_out(VENC_DRGBX1, 0x00000576);
		dispc_reg_out(VENC_DRGBX2, 0x00000159);
		dispc_reg_out(VENC_DRGBX3, 0x000002cb);
		dispc_reg_out(VENC_DRGBX4, 0x000006ee);

		/* Enable DCLOCK */
		dispc_reg_out(VENC_DCLKCTL, VENC_DCKCTL_DCKEC);

		/* Set DCLOCK pattern */
		dispc_reg_out(VENC_DCLKPTN0, 1);
		dispc_reg_out(VENC_DCLKPTN1, 0);
		dispc_reg_out(VENC_DCLKPTN2, 0);
		dispc_reg_out(VENC_DCLKPTN3, 0);
		dispc_reg_out(VENC_DCLKPTN0A, 2);
		dispc_reg_out(VENC_DCLKPTN1A, 0);
		dispc_reg_out(VENC_DCLKPTN2A, 0);
		dispc_reg_out(VENC_DCLKPTN3A, 0);
		dispc_reg_out(VENC_DCLKHS, 0);
		dispc_reg_out(VENC_DCLKHSA, 1);
		dispc_reg_out(VENC_DCLKHR, 0);
		dispc_reg_out(VENC_DCLKVS, 0);
		dispc_reg_out(VENC_DCLKVR, 0);

		/* Enable LCD output control (accepting default polarity) */
		dispc_reg_out(VENC_LCDOUT, 0x1);

		/* Set brightness start position and pulse width to zero */
		dispc_reg_out(VENC_BRTS, 0);
		dispc_reg_out(VENC_BRTW, 0);

		/* Set LCD AC toggle interval and horizontal position to zero */
		dispc_reg_out(VENC_ACCTL, 0);

		/* Set PWM period and width to zero */
		dispc_reg_out(VENC_PWMP, 0);
		dispc_reg_out(VENC_PWMW, 0);

		/* Clear component and composite mode
			registers (applicable to Analog DACS) */
		dispc_reg_out(VENC_CVBS, 0);
		dispc_reg_out(VENC_CMPNT, 0);

		/* turning on horizontal and vertical syncs */
		dispc_reg_out(VENC_SYNCCTL,
			      (VENC_SYNCCTL_SYEV|VENC_SYNCCTL_SYEH));

		/* Set OSD clock and OSD Sync Adavance registers */
		dispc_reg_out(VENC_OSDCLK0, 0);
		dispc_reg_out(VENC_OSDCLK1, 1);
		dispc_reg_out(VENC_OSDHAD, 0);
		/* set VPSS clock */
		dispc_reg_out(VPSS_CLKCTL, 0x0a);

	} else {

		/* Set PINMUX0 reg to disable LCD
		   (all other settings are kept per u-boot) */
		dispc_reg_merge(PINMUX0, 0, PINMUX0_LOEEN);

		/* disable VCLK output pin enable */
		dispc_reg_out(VENC_VIDCTL, 0x1101);

		/* Disable DCLOCK */
		dispc_reg_out(VENC_DCLKCTL, 0);

		dispc_reg_out(VENC_LCDOUT, 0x0);

		/* Initialize the VPSS Clock Control register */
		dispc_reg_out(VPSS_CLKCTL, 0x18);

		/* Disable output sync pins */
		dispc_reg_out(VENC_SYNCCTL, 0);

		dispc_reg_out(VENC_DRGBX1, 0x0000057C);

		/* Disable LCD output control
		   (accepting default polarity) */
		dispc_reg_out(VENC_LCDOUT, 0);
		dispc_reg_out(VENC_CMPNT, 0x100);
		dispc_reg_out(VENC_HSPLS, 0);
		dispc_reg_out(VENC_VSPLS, 0);
		dispc_reg_out(VENC_HINT, 0);
		dispc_reg_out(VENC_HSTART, 0);
		dispc_reg_out(VENC_HVALID, 0);
		dispc_reg_out(VENC_VINT, 0);
		dispc_reg_out(VENC_VSTART, 0);
		dispc_reg_out(VENC_VVALID, 0);
		dispc_reg_out(VENC_HSDLY, 0);
		dispc_reg_out(VENC_VSDLY, 0);
		dispc_reg_out(VENC_YCCCTL, 0);
		dispc_reg_out(VENC_VSTARTA, 0);

		/* Set OSD clock and OSD Sync Adavance registers */
		dispc_reg_out(VENC_OSDCLK0, 1);
		dispc_reg_out(VENC_OSDCLK1, 2);
	}
}

/* Find the mode that matches exactly this var */
static int dm_venc_find_mode(const struct fb_var_screeninfo *var)
{
	unsigned int i;

        /* only check the mode that has the same displayable size */
	for (i = 0; i < ARRAY_SIZE(dmfb_modedb); i++) {
		if (var->xres == dmfb_modedb[i].xres &&
			var->yres == dmfb_modedb[i].yres &&
			var->vmode == dmfb_modedb[i].vmode)
			return i;
	}
	return -EINVAL;
}

/**
 * Checks if a mode is a HD mode
 * @return 0 if mode is SD, 1 if mode is HD
 */
static int dm_venc_mode_is_hd(struct fb_videomode *mode)
{
	if (!strcmp(mode->name, "576i") || !strcmp(mode->name, "480i"))
		return 0;
	else
		return 1;
}
/**
 * Set the timmings of the VENC
 */
static void dm_venc_timmings_set(struct dm_info *dm, const struct fb_videomode *mode, const struct dm_extended_mode *extmode)
{
	dispc_reg_out(VENC_HSPLS, mode->hsync_len);
	dispc_reg_out(VENC_VSPLS, mode->vsync_len);
	dispc_reg_out(VENC_HINT, mode->xres + mode->left_margin +
		      mode->right_margin);
	dispc_reg_out(VENC_HSTART, mode->left_margin);
	dispc_reg_out(VENC_HVALID, mode->xres);
	dispc_reg_out(VENC_VINT, mode->yres + mode->upper_margin +
		      mode->lower_margin);
	dispc_reg_out(VENC_VSTART, mode->upper_margin);
	dispc_reg_out(VENC_VVALID, mode->vmode == FB_VMODE_INTERLACED ? mode->yres / 2 : mode->yres);
	/* TODO check vmode (interlaced / progressive)*/
	/* set the window field / frame mode */
	dispc_reg_out(VENC_YCCCTL, 0x0);
	dispc_reg_out(VENC_VSTARTA, extmode->vstarta);
	dispc_reg_out(OSD_BASEPX, extmode->basex);
	dispc_reg_out(OSD_BASEPY, extmode->basey);
}

/**
 * Set the venc mode
 */
static void dm_venc_mode_set(struct dm_info *dm, const struct fb_videomode *mode,
	const struct dm_extended_mode *extmode)
{
	/* shutdown previous mode */
	dispc_reg_out(VENC_VMOD, 0);
	enable_digital_output(false);
	/* set the timmings */
	dm_venc_timmings_set(dm, mode, extmode);
	/* for standard modes */
	if (!strcmp(mode->name, "480i")) {
		/* Enable all DACs  */
		dispc_reg_out(VENC_DACTST, 0);
		/* Set REC656 Mode */
		dispc_reg_out(VENC_YCCCTL, 0x1);
		/* Enable output mode and NTSC  */
		dispc_reg_out(VENC_VMOD, 0x1003);
	}
	else if (!strcmp(mode->name, "576i")) {
		/* Enable all DACs  */
		dispc_reg_out(VENC_DACTST, 0);
		/* Set REC656 Mode */
		dispc_reg_out(VENC_YCCCTL, 0x1);
		/* Enable output mode and PAL  */
		dispc_reg_out(VENC_VMOD, 0x1043);
	}
	/* for non standard modes */
	else {
		printk("Setting mode %s\n", mode->name);
		enable_digital_output(true);
#ifdef CONFIG_THS8200
		if (!strcmp(mode->name, "480p")) {
			/* slow down the vclk as 27MHZ */
			slow_down_vclk();
			ths8200_set_480p_mode();
			dispc_reg_merge(PINMUX0, 0, PINMUX0_LFLDEN);
			/* Enable all VENC, non-standard timing mode,
			 * master timing, HD, progressive */
			dispc_reg_out(VENC_VMOD,
				      (VENC_VMOD_VENC | VENC_VMOD_VMD | VENC_VMOD_HDMD));
		}
		else if (!strcmp(mode->name, "720p")) {
			ths8200_set_720p_mode();
			dispc_reg_merge(PINMUX0, 0, PINMUX0_LFLDEN);
			/* Enable all VENC, non-standard timing mode,
			 * master timing, HD, progressive */
			dispc_reg_out(VENC_VMOD,
				      (VENC_VMOD_VENC |	VENC_VMOD_VMD | VENC_VMOD_HDMD));
		}
		else if (!strcmp(mode->name, "1080i")) {
			ths8200_set_1080i_mode();
			dispc_reg_merge(PINMUX0, PINMUX0_LFLDEN, PINMUX0_LFLDEN);
			/* Enable all VENC, non-standard timing mode,
			 * master timing, HD, interlaced */
			dispc_reg_out(VENC_VMOD,
				      (VENC_VMOD_VENC |	VENC_VMOD_VMD |
				       VENC_VMOD_HDMD |	VENC_VMOD_NSIT));
		}
#endif
	}
}

/* Select the output on the venc mode. This outputs are only
 * available on standard moed timings, both SD or HD.
 * DACSEL.DAnS CMPNT.MRGB DAC Output
 *      0           -        CVBS
 *      1           -      S-Video Y
 *      2           -      S-Video C
 *      3          0           Y
 *                 1           G
 *      4          0          Pb
 *                 1           B
 *      5          0           Pr
 *                 1           R
 */
int dm_venc_set_state(struct output_device *od)
{
	struct dm_info *dm = (struct dm_info *)class_get_devdata(&od->class_dev);
	unsigned long state = od->request_state;

	/* TODO check that the output is in standard mode */
	switch (state)
	{
	case DAVINCIFB_OUT_COMPOSITE:
		dispc_reg_out(VENC_DACSEL, 0x0);
		break;

	case DAVINCIFB_OUT_COMPONENT:
		/* Enable Component output; DAC A: Y, DAC B: Pb, DAC C: Pr  */
		dispc_reg_out(VENC_DACSEL, 0x543);
		break;

	case DAVINCIFB_OUT_SVIDEO:
		/* Enable S-Video Output; DAC B: S-Video Y, DAC C: S-Video C  */
		dispc_reg_out(VENC_DACSEL, 0x210);
		break;

	case DAVINCIFB_OUT_RGB:
		/* TODO handle rgb */
		printk("rgb!\n");
		break;

	default:
		return -EINVAL;

	}
	dm->output_sel = state;
	return 0;
}
/* Returns the current output mode selcted */
int dm_venc_get_status(struct output_device *od)
{
	struct dm_info *dm = (struct dm_info *)class_get_devdata(&od->class_dev);

	return dm->output_sel;
}

struct output_properties dm_venc_props = {
	.set_state = &dm_venc_set_state,
	.get_status = &dm_venc_get_status,
};
#if 0
static int dm_vout_probe(struct dm_info *info)
{
	int ret;

	dm->output = video_output_register("venc", dm->dev, dm, &dm_venc_props);
	if (!dm->output)
		return -EINVAL;
	return 0;
}
#endif
/*============================================================================*
 *                             Mode definitions                               *
 *============================================================================*/
#if 0
static void davincifb_480p_component_config(int on)
{
	if (on) {

#ifdef CONFIG_THS8200
		/* Enable THS8200 DAC output mode as 480P */
		ths8200_set_480p_mode();
#endif/* CONFIG_THS8200 */

		dispc_reg_out(VENC_VMOD, 0);

		/* Set new baseX and baseY */
		dispc_reg_out(OSD_BASEPX, BASEX480P);
		dispc_reg_out(OSD_BASEPY, BASEY480P);

		/* Enable the digtal output */
		enable_digital_output(true);

		/* slow down the vclk as 27MHZ */
		slow_down_vclk();

		dispc_reg_merge(PINMUX0, 0, PINMUX0_LFLDEN);

		/* Enable DAVINCIFB_WIN_OSD0 Window */
		dispc_reg_out(OSD_OSDWIN0MD, 0x00002001);

		/* Enable DAVINCIFB_WIN_OSD1 Window */
		dispc_reg_out(OSD_OSDWIN1MD, 0x00008000);

		/* Set Timing parameters for 480P frame
		   (must match what THS8200 expects) */
		dispc_reg_out(VENC_HSPLS, BASEX480P);
		dispc_reg_out(VENC_VSPLS, BASEY480P);
		dispc_reg_out(VENC_HINT, 858 - 1);
		dispc_reg_out(VENC_HSTART, 122);
		dispc_reg_out(VENC_HVALID, DISP_XRES480P);
		dispc_reg_out(VENC_VINT, 525 - 1);
		dispc_reg_out(VENC_VSTART, 36);
		dispc_reg_out(VENC_VVALID, DISP_YRES480P);
		dispc_reg_out(VENC_HSDLY, 0);
		dispc_reg_out(VENC_VSDLY, 0);
		dispc_reg_out(VENC_YCCCTL, 0);
		dispc_reg_out(VENC_VSTARTA, 0);

		/* Set DAVINCIFB_WIN_VID0 window  origin and size */
		dispc_reg_out(OSD_VIDWIN0XP, 20);
		dispc_reg_out(OSD_VIDWIN0YP, 25);
		dispc_reg_out(OSD_VIDWIN0XL, DISP_XRES480P);
		dispc_reg_out(OSD_VIDWIN0YL, DISP_YRES480P);

		/* Set DAVINCIFB_WIN_VID1 window  origin and size */
		dispc_reg_out(OSD_VIDWIN1XP, 20);
		dispc_reg_out(OSD_VIDWIN1YP, 25);
		dispc_reg_out(OSD_VIDWIN1XL, DISP_XRES480P);
		dispc_reg_out(OSD_VIDWIN1YL, DISP_YRES480P);

		/* Set DAVINCIFB_WIN_OSD0 window  origin and size */
		dispc_reg_out(OSD_OSDWIN0XP, 20);
		dispc_reg_out(OSD_OSDWIN0YP, 25);
		dispc_reg_out(OSD_OSDWIN0XL, DISP_XRES480P);
		dispc_reg_out(OSD_OSDWIN0YL, DISP_YRES480P);

		/* Set DAVINCIFB_WIN_OSD1 window  origin and size */
		dispc_reg_out(OSD_OSDWIN1XP, 20);
		dispc_reg_out(OSD_OSDWIN1YP, 25);
		dispc_reg_out(OSD_OSDWIN1XL, DISP_XRES480P);
		dispc_reg_out(OSD_OSDWIN1YL, DISP_YRES480P);

		/* Set DAVINCIFB_WIN_OSD1 window  origin and size */
		dispc_reg_out(OSD_CURXP, 20);
		dispc_reg_out(OSD_CURYP, 25);
		dispc_reg_out(OSD_CURXL, DISP_XRES480P);
		dispc_reg_out(OSD_CURYL, DISP_YRES480P);

		/* Enable all VENC, non-standard timing mode,
		   master timing, HD, progressive */
		dispc_reg_out(VENC_VMOD,
			      (VENC_VMOD_VENC | VENC_VMOD_VMD | VENC_VMOD_HDMD));

		printk(KERN_INFO "Davinci set video mode as 480p\n");
	} else {
		/* Reset video encoder module */
		dispc_reg_out(VENC_VMOD, 0);
	}
}
#endif

/*============================================================================*
 *                          OSD controller interface                          *
 *============================================================================*/
/* All this functions should setup some global register of the osd controller
 */
/* Wait for a vsync interrupt.  This routine sleeps so it can only be called
 * from process context.
 */
static int davincifb_wait_for_vsync(struct dm_win_info *w)
{
	struct dm_info *dm = w->dm;
	wait_queue_t wq;
	unsigned long cnt;
	int ret;

	init_waitqueue_entry(&wq, current);

	cnt = dm->vsync_cnt;
	ret = wait_event_interruptible_timeout(dm->vsync_wait,
					       cnt != dm->vsync_cnt,
					       dm->timeout);
	if (ret < 0)
		return ret;
	if (ret == 0)
		return -ETIMEDOUT;

	return 0;
}
/* set the global transparency color for all osd windows */
static void dm_transp_color_set(struct dm_info *i, int color)
{
	struct device *dev = i->dev;

	dev_dbg(dev, "Setting color transparency to %x\n", color);
	printk("OSDTRANSPVA %x\n", dispc_reg_in(OSD_TRANSPVA));
	dispc_reg_out(OSD_TRANSPVA, color);
	printk("OSDTRANSPVA %x\n", dispc_reg_in(OSD_TRANSPVA));
}
/* enable color bar test mode */
static void dm_cbtest_enable(struct dm_info *i, int enable)
{
	struct device *dev = i->dev;

	dev_dbg(dev, "Setting color bar mode %d\n", enable);
	dispc_reg_merge(VENC_VDPRO, VENC_VDPRO_CBTYPE, VENC_VDPRO_CBTYPE);
	dispc_reg_merge(VENC_VDPRO, enable << 8, VENC_VDPRO_CBMD);
	printk("VDPRO %x\n", dispc_reg_in(VENC_VDPRO));
}

/*============================================================================*
 *                           OSD windows interface                            *
 *============================================================================*/
/* TODO OK, we should move all the logic here on the windows interfae */
static void set_sdram_params(const struct dm_win_info *w, u32 addr, u32 line_length);
/* Sets a uniform attribute value over a rectangular area on the attribute
 * window. The attribute value (0 to 7) is passed through the fb_fillrect's
 * color parameter.
 */
static int davincifb_set_attr_blend(struct dm_win_info *w,
				    struct fb_fillrect *r)
{
	struct fb_info *info = &w->info;
	struct fb_var_screeninfo *var = &info->var;
	unsigned long start = 0;
	u8 blend;
	u32 width_bytes;

	if (r->dx + r->width > var->xres_virtual)
		return -EINVAL;
	if (r->dy + r->height > var->yres_virtual)
		return -EINVAL;
	if (r->color < 0 || r->color > 7)
		return -EINVAL;

	/* since bits_per_pixel = 4, this will truncate the width if it is
	 * not even. Similarly r->dx will be rounded down to an even pixel.
	 * ... Do we want to return an error otherwise?
	 */
	width_bytes = r->width * var->bits_per_pixel / 8;
	start = w->fb_base + r->dy * info->fix.line_length + r->dx * var->bits_per_pixel / 8;

	blend = (((u8) r->color & 0xf) << 4) | ((u8) r->color);
	while (r->height--) {
		start += info->fix.line_length;
		memset((void *)start, blend, width_bytes);
	}

	return 0;
}
/* Interlaced = Frame mode, Non-interlaced = Field mode */
static void set_interlaced(struct dm_win_info *w, unsigned int on)
{
	struct device *dev = w->dm->dev;

	on = (on == 0) ? 0 : ~0;

	dev_dbg(dev, "Setting window interlaced %s %c\n", dm_win_names[w->win], on ? 'I' : 'P');

	if (is_win(w, DAVINCIFB_WIN_VID0))
		dispc_reg_merge(OSD_VIDWINMD, on, OSD_VIDWINMD_VFF0);
	else if (is_win(w, DAVINCIFB_WIN_VID1))
		dispc_reg_merge(OSD_VIDWINMD, on, OSD_VIDWINMD_VFF1);
	else if (is_win(w, DAVINCIFB_WIN_OSD0))
		dispc_reg_merge(OSD_OSDWIN0MD, on, OSD_OSDWIN0MD_OFF0);
	else if (is_win(w, DAVINCIFB_WIN_OSD1))
		dispc_reg_merge(OSD_OSDWIN1MD, on, OSD_OSDWIN1MD_OFF1);
}

/* For zooming, we just have to set the start of framebuffer, the zoom factors
 * and the display size. The hardware will then read only
 * (display size / zoom factor) area of the framebuffer and  zoom and
 * display it. In the following function, we assume that the start of
 * framebuffer and the display size parameters are set already.
 */
static void set_zoom(struct dm_win_info *w, int h_factor, int v_factor)
{
	/* TODO remove DAVINCIFB_WIN_VID0, DAVINCIFB_WIN_OSD0, etc */
	switch (w->win) {
	case DAVINCIFB_WIN_VID0:
		dispc_reg_merge(OSD_VIDWINMD,
				h_factor << OSD_VIDWINMD_VHZ0_SHIFT,
				OSD_VIDWINMD_VHZ0);
		dispc_reg_merge(OSD_VIDWINMD,
				v_factor << OSD_VIDWINMD_VVZ0_SHIFT,
				OSD_VIDWINMD_VVZ0);
		break;
	case DAVINCIFB_WIN_VID1:
		dispc_reg_merge(OSD_VIDWINMD,
				h_factor << OSD_VIDWINMD_VHZ1_SHIFT,
				OSD_VIDWINMD_VHZ1);
		dispc_reg_merge(OSD_VIDWINMD,
				v_factor << OSD_VIDWINMD_VVZ1_SHIFT,
				OSD_VIDWINMD_VVZ1);
		break;
	case DAVINCIFB_WIN_OSD0:
		dispc_reg_merge(OSD_OSDWIN0MD,
				h_factor << OSD_OSDWIN0MD_OHZ0_SHIFT,
				OSD_OSDWIN0MD_OHZ0);
		dispc_reg_merge(OSD_OSDWIN0MD,
				v_factor << OSD_OSDWIN0MD_OVZ0_SHIFT,
				OSD_OSDWIN0MD_OVZ0);
		break;
	case DAVINCIFB_WIN_OSD1:
		dispc_reg_merge(OSD_OSDWIN1MD,
				h_factor << OSD_OSDWIN1MD_OHZ1_SHIFT,
				OSD_OSDWIN1MD_OHZ1);
		dispc_reg_merge(OSD_OSDWIN1MD,
				v_factor << OSD_OSDWIN1MD_OVZ1_SHIFT,
				OSD_OSDWIN1MD_OVZ1);
		break;
	}
}

/* Chooses the ROM CLUT for now. Can be extended later. */
static void set_bg_color(u8 clut, u8 color_offset)
{
	clut = 0;		/* 0 = ROM, 1 = RAM */

	dispc_reg_merge(OSD_MODE, OSD_MODE_BCLUT & clut, OSD_MODE_BCLUT);
	dispc_reg_merge(OSD_MODE, color_offset << OSD_MODE_CABG_SHIFT,
			OSD_MODE_CABG);
}

static void set_sdram_params(const struct dm_win_info *w, u32 addr, u32 line_length)
{
	/* The parameters to be written to the registers should be in
	 * multiple of 32 bytes
	 */
	addr = addr;		/* div by 32 */
	line_length = line_length / 32;

	if (is_win(w, DAVINCIFB_WIN_VID0)) {
#ifndef VID0FIX
		dispc_reg_out(OSD_VIDWIN0ADR, addr);
        	dispc_reg_out(OSD_VIDWIN0OFST, line_length);
#else
		struct dm_info *dm = w->dm;
		/* BUG */
		if (dmfb_modedb[w->dm->curr_mode].vmode == FB_VMODE_INTERLACED) {
			u32 length = line_length * 32;
			dispc_reg_out(OSD_VIDWIN0ADR, addr - length);
			dispc_reg_out(OSD_VIDWIN0OFST, line_length);
			dispc_reg_out(OSD_PPVWIN0AD, addr + length);
			dispc_reg_merge(OSD_VIDWINMD, OSD_VIDWINMD_VFF0, OSD_VIDWINMD_VFF0);
			dispc_reg_merge(OSD_MISCCT, OSD_MISCCT_PPRV, OSD_MISCCT_PPRV);
		}
		else {
			dispc_reg_out(OSD_VIDWIN0ADR, addr);
			dispc_reg_out(OSD_VIDWIN0OFST, line_length);
			dispc_reg_merge(OSD_MISCCT, 0, OSD_MISCCT_PPRV);
		}
#endif
	} else if (is_win(w, DAVINCIFB_WIN_VID1)) {
		dispc_reg_out(OSD_VIDWIN1ADR, addr);
		dispc_reg_out(OSD_VIDWIN1OFST, line_length);
	} else if (is_win(w, DAVINCIFB_WIN_OSD0)) {
		dispc_reg_out(OSD_OSDWIN0ADR, addr);
		dispc_reg_out(OSD_OSDWIN0OFST, line_length);
	} else if (is_win(w, DAVINCIFB_WIN_OSD1)) {
		dispc_reg_out(OSD_OSDWIN1ADR, addr);
		dispc_reg_out(OSD_OSDWIN1OFST, line_length);
	}
}
/**
 * Calculates the window bandwith:
 * (x0 × y0) × b0 x FR
 */
static int dm_win_bandwith_get(struct dm_win_info *w)
{
	int bandwith = 0;

	if (w) {
		struct fb_var_screeninfo *var;

		var = &w->info.var;
		bandwith = var->xres * var->yres * var->bits_per_pixel;
	}
	return bandwith;
}

static inline void dm_win_position_get(const struct dm_win_info *w,
				    u32 * xp, u32 * yp, u32 * xl, u32 * yl)
{
	struct fb_var_screeninfo *v = &(w->info.var);

	*xp = w->x;
	*yp = w->y;
	*xl = v->xres;
	*yl = v->yres;
}

static void dm_win_clear(struct dm_win_info *w)
{
	struct fb_var_screeninfo *v = &(w->info.var);
	int bg_color = 0x00;

	switch (v->bits_per_pixel) {
		case 16:
		/* yuv422 */
		if (is_win(w, DAVINCIFB_WIN_VID0) || is_win(w, DAVINCIFB_WIN_VID1))
			bg_color = 0x88;
		break;

		/* attribute */
		case 4:
		if (is_win(w, DAVINCIFB_WIN_OSD1))
			bg_color = 0x77;
		break;
	}
	memset((void *)w->fb_base, bg_color, w->fb_size);
}
/**
 * Checks if the rectangle formed by the first four coordinates is inside the
 * rectangle formed by the second four coordinates
 * @return 1 if is inside, 0 if is outside
 */
static inline int rectangle_inside(int x1, int y1, int w1, int h1, int x2,
		int y2, int w2, int h2)
{
	if ((x1 >= x2) && (y1 >= y2) && (x2 + w2 >= x1 + w1) && (y2 + h2 >= y1 + h1))
		return 1;
	else
		return 0;
}

/*
 * Checks if the coordinates are inside vid0 size
 * @return 1 if the window parameters are within vid0, 0 otherwise
 */
static int dm_win_vid0_within(struct dm_win_info *vid0, int x, int y, int w, int h)
{
	int vx, vy, vw, vh;

	/* if vid0 window is disabled, it is ok */
	if (!vid0 || !vid0->enabled)
		return 1;
	dm_win_position_get(vid0, &vx, &vy, &vw, &vh);
	return rectangle_inside(x, y, w, h, vx, vy, vw, vh);
}

/* Checks if the new size of vid0 include all other window sizes
 * @param
 * @param
 * @param
 * @param
 * @return 1 if the new size includes the others, 0 otherwise
 * @note If a window is disabled it wont check agains it
 */
static int dm_win_vid0_size_new(struct dm_win_info *vid0, int x, int y, int w, int h)
{
	int i, vx, vy, vw, vh;

	if (!vid0 || !vid0->enabled)
		return 1;
	dm_win_position_get(vid0, &vx, &vy, &vw, &vh);
	for (i = 0; i < DAVINCIFB_WINDOWS; i++) {
		struct dm_win_info *owin = vid0->dm->windows[i];
		int x, y, w, h;

		if (!owin || !owin->enabled)
			continue;
		dm_win_position_get(owin, &x, &y, &w, &h);
		if (!rectangle_inside(x, y, w, h, vx, vy, vw, vh)) {
			printk("%s is too big %dx%d at %d %d\n", dm_win_names[owin->win],
					x, y, w, h);
			return 0;
		}
	}
	return 1;
}

/**
 * Enables a window
 * @param w The window to enable
 * @param on 1 to enable, 0 to disable
 * @note To enable a window we should follow several rules:
 * 1. In HD mode, vid1 must be disabled
 * 2. In HD mode, osd0 and osd1 total bandwith can't be greater than 25MB/s
 * [(x0 × y0) × b0 + (x1 × y1) × b1] × FR < 25 Mbytes/second
 * FR = Frame rate, 60 in progressive mode
 * 3. The attribute window can't be disabled, you need to set the mode to RGB565
 * and the disable it
 * 4. A window to be enabled must be inside vid0
 */
static int dm_win_enable(struct dm_win_info *win, unsigned int on)
{
#define SZ_25MB (25 * 1024 * 1024)
	struct dm_info *dm = win->dm;
	struct fb_videomode *mode = &dmfb_modedb[dm->curr_mode];
	int x, y, w, h;

	on = (on == 0) ? 0 : ~0;
	dm_win_position_get(win, &x, &y, &w, &h);

	if (is_win(win, DAVINCIFB_WIN_VID0)) {
		/* if you enable vid0, all the other windows must be inside */
		if (on && dm_win_vid0_size_new(win, x, y, w, h)) {
			/* Turning off DAVINCIFB_WIN_VID0 use due to field inversion issue */
			dispc_reg_merge(OSD_VIDWINMD, on, OSD_VIDWINMD_ACT0);
		}
		else {
			dispc_reg_merge(OSD_VIDWINMD, 0, OSD_VIDWINMD_ACT0);
		}
	} else {
		/* the other windows must be inside vid0 */
		struct dm_win_info *vid0 = dm->windows[DAVINCIFB_WIN_VID0];

		if (on && !dm_win_vid0_within(vid0, x, y, w, h)) {
			printk("Enabling a window that is not inside vid0\n");
			return 0;
		}
	}
	if (is_win(win, DAVINCIFB_WIN_VID1)) {
		if (dm_venc_mode_is_hd(mode) && on) {
			printk("Is not possible to enable vid1 window in hd mode\n");
			return 0;
		}
		dispc_reg_merge(OSD_VIDWINMD, on, OSD_VIDWINMD_ACT1);
	} else if (is_win(win, DAVINCIFB_WIN_OSD0)) {
		int bandwith;

		bandwith = dm_win_bandwith_get(win);
		bandwith += dm_win_bandwith_get(dm->windows[DAVINCIFB_WIN_OSD1]);
		if (bandwith < SZ_25MB) {
			dispc_reg_merge(OSD_OSDWIN0MD, on, OSD_OSDWIN0MD_OACT0);
		}
		else {
			printk("Bandwith too high\n");
			return 0;
		}

	} else if (is_win(win, DAVINCIFB_WIN_OSD1)) {
		/* The OACT1 bit is applicable only if DAVINCIFB_WIN_OSD1 is not used as
		 * the attribute window
		 */
		if (!(dispc_reg_in(OSD_OSDWIN1MD) & OSD_OSDWIN1MD_OASW)) {
			int bandwith;

			bandwith = dm_win_bandwith_get(win);
			bandwith += dm_win_bandwith_get(dm->windows[DAVINCIFB_WIN_OSD1]);
			if (bandwith < SZ_25MB) {
				dispc_reg_merge(OSD_OSDWIN1MD, on, OSD_OSDWIN1MD_OACT1);
			}
			else {
				printk("Bandwith too high\n");
				return 0;
			}
		} else {
			printk("Can't disable/enable the attribute window\n");
			return 0;
		}
	}
	win->enabled = on;
	return 1;
#undef SZ_25MB
}

/*
 * Sets the depth of the window plane
 * TODO fix this, isnt flexible enough
 */
static void set_win_mode(const struct dm_win_info *w)
{
	if (is_win(w, DAVINCIFB_WIN_VID0)) ;
	else if (is_win(w, DAVINCIFB_WIN_VID1)) {
		if (w->info.var.bits_per_pixel == 32)
			dispc_reg_merge(OSD_MISCCT, ~0,
					OSD_MISCCT_RGBWIN | OSD_MISCCT_RGBEN);
	} else if (is_win(w, DAVINCIFB_WIN_OSD0))
		/* Set RGB565 mode */
		dispc_reg_merge(OSD_OSDWIN0MD, OSD_OSDWIN0MD_RGB0E,
				OSD_OSDWIN0MD_RGB0E);
	else if (is_win(w, DAVINCIFB_WIN_OSD1)) {
		/* Set as attribute window */
		dispc_reg_merge(OSD_OSDWIN1MD, OSD_OSDWIN1MD_OASW,
				OSD_OSDWIN1MD_OASW);
	}
}

/*
 * TODO remove this function to only receive the position not the window size
 * These position parameters are given through fb_var_screeninfo.
 * xp = var.reserved[0], yp = var.reserved[1],
 * xl = var.xres, yl = var.yres
 */
static void set_win_position(const struct dm_win_info *w, u32 xp, u32 yp, u32 xl,
	u32 yl)
{
	int i = 0;
	struct device *dev = w->dm->dev;

	dev_dbg(dev, "Setting window position %s %u %u %u %u\n", dm_win_names[w->win], xp, yp, xl, yl);
	if (is_win(w, DAVINCIFB_WIN_VID0)) {
		i = 0;
	} else if (is_win(w, DAVINCIFB_WIN_VID1)) {
		i = 1;
	} else if (is_win(w, DAVINCIFB_WIN_OSD0)) {
		i = 2;
	} else if (is_win(w, DAVINCIFB_WIN_OSD1)) {
		i = 3;
	}

	dispc_reg_out(OSD_WINXP(i), xp);
	dispc_reg_out(OSD_WINYP(i), yp);
	dispc_reg_out(OSD_WINXL(i), xl);
	dispc_reg_out(OSD_WINYL(i), yl);
}

/* Returns 1 if the windows overlap, 0 otherwise */
static int window_overlap(struct dm_win_info *w, u32 xp, u32 yp, u32 xl, u32 yl)
{
	u32 _xp = 0, _yp = 0, _xl = 0, _yl = 0;

#define OVERLAP(x1, y1, x2, y2, x3, y3, x4, y4)		\
(!(	((x1)<(x3) && (x2)<(x3)) || ((x1)>(x4) && (x2)>(x4)) ||	\
	((y1)<(y3) && (y2)<(y3)) || ((y1)>(y4) && (y2)>(y4)) )	\
)

	if (!w)
		return 0;

	dm_win_position_get(w, &_xp, &_yp, &_xl, &_yl);

	return OVERLAP(xp, yp, xp + xl, yp + yl,
		       _xp, _yp, _xp + _xl, _yp + _yl);
#undef OVERLAP
}

/* enable the transparency on the plane based on the transparency color */
static int dm_win_transp_enable(struct dm_win_info *w,
		struct dmfb_transparency *transp)
{
	struct device *dev = w->dm->dev;

	dev_dbg(dev, "Enabling transparency on plane %s %d with level %d\n", dm_win_names[w->win], transp->on, transp->level);
	if (is_win(w, DAVINCIFB_WIN_VID0) || is_win(w, DAVINCIFB_WIN_VID1)) {
		return -EINVAL;
	}
	else if (is_win(w, DAVINCIFB_WIN_OSD0)) {
		dispc_reg_merge(OSD_OSDWIN0MD, transp->on << 2, OSD_OSDWIN0MD_TE0);
		dispc_reg_merge(OSD_OSDWIN0MD, transp->level << OSD_OSDWIN0MD_BLND0_SHIFT, OSD_OSDWIN0MD_BLND0);
		printk("WINO0MD %x\n", dispc_reg_in(OSD_OSDWIN0MD));
	}
	else {
		/* TODO check that is not set in attribute mode */
		dispc_reg_merge(OSD_OSDWIN1MD, transp->on << 2, OSD_OSDWIN1MD_TE1);
		dispc_reg_merge(OSD_OSDWIN1MD, transp->level << OSD_OSDWIN1MD_BLND1_SHIFT, OSD_OSDWIN1MD_BLND1);
	}
	return 0;
}

/**
 * Sets a window mode
 *
 */
static void dm_win_mode_set(struct dm_win_info *w, struct fb_var_screeninfo *v)
{
	u32 start = 0, offset = 0;
	int interlaced = v->vmode == FB_VMODE_INTERLACED ? 1 : 0;

	/* Memory offsets */
	w->info.fix.line_length = v->xres_virtual * v->bits_per_pixel / 8;
	offset = v->yoffset * w->info.fix.line_length + v->xoffset * v->bits_per_pixel / 8;
	start = (u32) w->fb_base_phys + offset;
	set_sdram_params(w, start, w->info.fix.line_length);
	/* set interlaced mode */
	set_interlaced(w, interlaced);
	/* set window position and size */
	set_win_position(w, w->x, w->y, v->xres, v->yres / (interlaced + 1));
	set_win_mode(w);
	dm_win_clear(w);
}

/* Initialize the fb_info structure with common values and values that wont
 * change if var changes
 */
static void dm_win_common_info_set(struct dm_win_info *w)
{
	struct fb_info *info = &w->info;

	info->flags = FBINFO_DEFAULT;
	info->fbops = &davincifb_ops;
	info->screen_base = (char *)(w->fb_base);
	info->pseudo_palette = w->pseudo_palette;
	info->par = w;

	strlcpy(info->fix.id, dm_win_names[w->win], sizeof(info->fix.id));
	info->fix.mmio_start = w->dm->mmio_base_phys;
	info->fix.mmio_len = w->dm->mmio_size;
	info->fix.accel = FB_ACCEL_NONE;
	w->sdram_address = 0;
}

/* Initialize fixed screeninfo.
 * The fixed screeninfo cannot be directly specified by the user, but
 * it may change to reflect changes to the var info.
 */
static void dm_win_fix_set(struct dm_win_info *w)
{
	struct fb_info *info = &w->info;

	info->fix.smem_start = w->fb_base_phys;
	info->fix.smem_len = w->fb_size;
	info->fix.type = FB_TYPE_PACKED_PIXELS;
	info->fix.visual = (info->var.bits_per_pixel <= 8) ?
	     FB_VISUAL_PSEUDOCOLOR : FB_VISUAL_TRUECOLOR;
	/* some values might change based on check_var, set_par and pan */
	info->fix.xpanstep = 0;
	info->fix.ypanstep = 1;
	info->fix.ywrapstep = 0;
	info->fix.type_aux = 0;
}

static void dm_win_mem_free(struct dm_win_info *w)
{
	struct device *dev  = w->dm->dev;

	dev_dbg(dev, "Freeing allocated at memory\n");
	free_pages_exact((void *)w->fb_base, w->fb_size);
}

static int dm_win_mem_alloc(struct dm_win_info *w)
{
	struct device *dev  = w->dm->dev;

	w->fb_size = w->dm->mach_info->size[w->win];
	dev_dbg(dev, "Trying to allocate %lu bytes of buffer\n", w->fb_size);
	w->fb_base = (unsigned long)alloc_pages_exact(w->fb_size, GFP_DMA);
	w->fb_base_phys = virt_to_phys((void *)w->fb_base);

	if (!w->fb_base)
		return -ENOMEM;

	dev_dbg(dev, "Framebuffer allocated at 0x%x "
		"mapped to 0x%x, size %dk\n",
		(unsigned)w->fb_base_phys, (unsigned)w->fb_base,
		(unsigned)w->fb_size / 1024);
	return 0;
}

static void dm_win_remove(struct dm_win_info *w)
{
	struct device *dev  = w->dm->dev;

	dev_dbg(dev, "Removing framebuffer %s\n", dm_win_names[w->win]);
	if (!unregister_framebuffer(&w->info)) {
		dm_win_mem_free(w);
	}
	/* TODO disable the plane */
}

static int dm_wins_remove(struct dm_info *info)
{
	int windows;
	int i = 0;

	windows = info->windows_mask;
	while (windows)
	{
		if (windows & 1)
		{
			dm_win_remove(info->windows[i]);
			kfree(info->windows[i]);
			info->windows[i] = NULL;
		}
		windows >>= 1;
		i++;
	}
	return 0;
}

static int dm_win_probe(struct dm_win_info *w)
{
	int ret;
	struct fb_info *info = &w->info;
	struct device *dev  = w->dm->dev;
	struct fb_var_screeninfo *vinfo = &info->var;

	dev_dbg(dev, "Probing device %s\n", dm_win_names[w->win]);
	/* alloc the memory */
	if (dm_win_mem_alloc(w) < 0) {
		dev_err(dev, "%s: Cannot allocate framebuffer of size %lu\n",
			dm_win_names[w->win], w->fb_size);
		return -EINVAL;
	}
	/* setup common values */
	dm_win_common_info_set(w);
	dm_win_fix_set(w);
	/* append the list modes */
	fb_videomode_to_modelist(dmfb_modedb, ARRAY_SIZE(dmfb_modedb),
				 &info->modelist);
	if (!fb_find_mode(vinfo, &w->info, NULL, dmfb_modedb,
			  ARRAY_SIZE(dmfb_modedb), NULL,
			  is_win(w, DAVINCIFB_WIN_OSD1) ? 4 : 16)) {
		return -EINVAL;
	}
	/* clear the memory */
	dm_win_clear(w);
	/* create the fb device */
	if (register_framebuffer(info) < 0) {
		dev_err(dev, "Unable to register %s framebuffer\n",
			dm_win_names[w->win]);
		ret = -EINVAL;
		goto register_error;
	}
	davincifb_set_par(info);
	dm_win_enable(w, 1);
	return 0;

register_error:
	dm_win_mem_free(w);
	return ret;
}

static int dm_wins_probe(struct dm_info *info)
{
	int windows;
	int windows_ok = 0;
	int i = 0;

	/* Set an initial invalid mode */
	info->curr_mode = -1;
	/* Setup DAVINCIFB_WIN_VID0 framebuffer */
	if (!(info->windows_mask & (1 << DAVINCIFB_WIN_VID0))) {
		printk(KERN_WARNING "No video/osd windows will be enabled "
		       "because Video0 is disabled\n");
		return 0;	/* background will still be shown */
	}
	/* Probe all requested windows */
	windows = info->windows_mask;
	while (windows)
	{
		if (windows & 1) {
			struct dm_win_info *w;

			w = kzalloc(sizeof(struct dm_win_info), GFP_KERNEL);
			if (!w) {
				dev_err(info->dev, "%s: could not allocate\
					memory\n", dm_win_names[i]);
				break;
			}
			/* necessary information for checkvar */
			w->win = i;
			w->dm = info;
			w->info.par = w;
			if (dm_win_probe(w) < 0) {
				kfree(w);
				break;
			}
			else {
				info->windows[i] = w;
				windows_ok |= (1 << i);
			}
		}
		windows >>= 1;
		i++;
	}
	/* Something has failed, remove all the probed wins */
	if (windows_ok != info->windows_mask) {
		dev_err(info->dev, "Not all windows were correctly setup "
			"removing all framebuffers (%x %x)\n", windows_ok,
			info->windows_mask);
		info->windows_mask = windows_ok;
		dm_wins_remove(info);
		return -EINVAL;
	}
	return 0;
}

static int parse_win_params(char *wp,
			    int *xres, int *yres, int *xpos, int *ypos)
{
	char *s;

	if ((s = strsep(&wp, "x")) == NULL)
		return -EINVAL;
	*xres = simple_strtoul(s, NULL, 0);

	if ((s = strsep(&wp, "@")) == NULL)
		return -EINVAL;
	*yres = simple_strtoul(s, NULL, 0);

	if ((s = strsep(&wp, ",")) == NULL)
		return -EINVAL;
	*xpos = simple_strtoul(s, NULL, 0);

	if ((s = strsep(&wp, ":")) == NULL)
		return -EINVAL;
	*ypos = simple_strtoul(s, NULL, 0);

	return 0;
}
/*============================================================================*
 *                             Framebuffer API                                *
 *============================================================================*/
/**
 *      davincifb_check_var - Validates a var passed in.
 *      @var: frame buffer variable screen structure
 *      @info: frame buffer structure that represents a single frame buffer
 *
 *	Checks to see if the hardware supports the state requested by
 *	var passed in. This function does not alter the hardware state!!!
 *	This means the data stored in struct fb_info and struct xxx_par do
 *      not change. This includes the var inside of struct fb_info.
 *	Do NOT change these. This function can be called on its own if we
 *	intent to only test a mode and not actually set it.
 *	If the var passed in is slightly off by what the hardware can support
 *	then we alter the var PASSED in to what we can do.
 *
 *	Returns negative errno on error, or zero on success.
 */
static int davincifb_check_var(struct fb_var_screeninfo *var,
			       struct fb_info *info)
{
	const struct dm_win_info *w = (const struct dm_win_info *)info->par;
	struct fb_var_screeninfo v;
	struct device *dev = w->dm->dev;
	unsigned int mod;
	unsigned int line_length;
	int mode;

/* Rules:
 * 1) Vid1, DAVINCIFB_WIN_OSD0, DAVINCIFB_WIN_OSD1 and Cursor must be fully contained inside of Vid0.
 * 2) Vid0 and Vid1 are both set to accept YUV 4:2:2 (for now).
 * 3) OSD window data is always packed into 32-bit words and left justified.
 * 4) Each horizontal line of window data must be a multiple of 32 bytes.
 *    32 bytes = 32 bytes / 2 bytes per pixel = 16 pixels.
 *    This implies that 'xres' must be a multiple of 32 bytes.
 * 5) The offset registers hold the distance between the start of one line and
 *    the start of the next. This offset value must be a multiple of 32 bytes.
 *    This implies that 'xres_virtual' is also a multiple of 32 bytes. Note
 *    that 'xoffset' needn't be a multiple of 32 bytes.
 * 6) DAVINCIFB_WIN_OSD0 is set to accept RGB565.
 * 	dispc_reg_merge(OSD_OSDWIN0ND, OSD_OSDWIN0ND_RGB0E, OSD_OSDWIN0ND_RGB0E)
 * 7) DAVINCIFB_WIN_OSD1 is set to be the attribute window.
 * 8) Vid1 startX = Vid0 startX + N * 16 pixels (32 bytes)
 * 9) Vid1 width = (16*N - 8) pixels
 * 10) When one of the OSD windows is in RGB565, it cannot overlap with Vid1.
 * 11) Vid1 start X position must be offset a multiple of 16 pixels from the
 * left edge of Vid0.
 */
	memcpy(&v, var, sizeof(v));

	mode = dm_venc_find_mode(var);
	if (mode >= 0) {
		printk("%s mode = %s\n", dm_win_names[w->win], dmfb_modedb[mode].name);
	}
	else {
		printk("%s Setting a specific mode\n", dm_win_names[w->win]);
	}
	dev_dbg(dev, "Trying to set <%d>%dx%d<%d>@%dbpp\n", v.xres_virtual, v.xres, v.yres,
		v.yres_virtual, v.bits_per_pixel);

	/* always force VENC interlaced mode */
	if (w->dm->curr_mode >= 0 && (!is_win(w, DAVINCIFB_WIN_VID0))) {
		v.vmode = dmfb_modedb[w->dm->curr_mode].vmode;
	}
	/* virtual size < display size */
	if (v.xres_virtual < v.xres || v.yres_virtual < v.yres) {
		dev_dbg(dev, "Virtual size > displayed size\n");
		return -EINVAL;
	}
	/* virtual offset > virtual size */
	if (v.xoffset > v.xres_virtual - v.xres) {
		dev_dbg(dev, "Virtual x offset > vitual size\n");
		return -EINVAL;
	}
	if (v.yoffset > v.yres_virtual - v.yres) {
		dev_dbg(dev, "Virtual y offset > vitual size\n");
		return -EINVAL;
	}
	/* get the mode */
	if (is_win(w, DAVINCIFB_WIN_VID0)) {
		if (dm_venc_find_mode(&v) < 0)
			return -EINVAL;
	}
	/* Align the line_length to 32 bytes */
	mod = (v.xres * v.bits_per_pixel / 8) % 32;
	if (mod) {
		v.xres_virtual = v.xres + (((32 - mod) * 8) / v.bits_per_pixel);
	}
	else
		v.xres_virtual = v.xres;
	v.yres_virtual = (w->fb_size / v.bits_per_pixel * 8) /
		(v.xres_virtual ? v.xres_virtual : 1);
	if (v.yres_virtual < v.yres)
		v.yres = v.yres_virtual;

	line_length = v.xres_virtual * v.bits_per_pixel / 8;

	dev_dbg(dev, "<%d>%dx%d<%d>@%dbpp-%c\n", v.xres_virtual, v.xres, v.yres,
		v.yres_virtual, v.bits_per_pixel, v.vmode == FB_VMODE_INTERLACED ? 'I' : 'P');

	/* Rules 4, 5 */
	if (line_length % 32) {
		dev_dbg(dev, "%s Offset isnt 32 byte aligned xres = %d "
			"xres_virtual = %d bpp = %d\n", dm_win_names[w->win],
			v.xres, v.xres_virtual, v.bits_per_pixel);
		return -EINVAL;
	}
	if ((w->fb_size) &&
	    (v.xres_virtual * v.yres_virtual * v.bits_per_pixel / 8 > w->fb_size)) {
		dev_dbg(dev, "Requested resolution too big\n");
		goto error;
	}
	/* positions relative to vid0 */
	if (!is_win(w, DAVINCIFB_WIN_VID0)) {
		/* Rule 1 */
		if (!dm_win_vid0_within(w->dm->windows[DAVINCIFB_WIN_VID0],
					w->x, w->y, v.xres, v.yres)) {
			dev_dbg(dev, "Window %s isnt fully contained on vid0\n",
				dm_win_names[w->win]);
			goto error;
		}
	} else {
		if (!dm_win_vid0_size_new(w, w->x, w->y, v.xres, v.yres)) {
			dev_dbg(dev, "vid0 isn't large enough to handle all windows\n");
			goto error;
		}
	}
	if (is_win(w, DAVINCIFB_WIN_OSD0)) {
		/* Rule 10 */
		/*if (window_overlap(w->dm->windows[DAVINCIFB_WIN_VID1],
				   x_pos(w), y_pos(w), v.xres, v.yres)) {
			dev_dbg(dev, "osd0 window overlaps with vid1\n");
			return -EINVAL;
		}*/
		/* Rule 5 */
		v.bits_per_pixel = 16;
		v.red.offset = 11;
		v.green.offset = 5;
		v.blue.offset = 0;
		v.red.length = 5;
		v.green.length = 6;
		v.blue.length = 5;
		v.transp.offset = v.transp.length = 0;
		v.red.msb_right = v.green.msb_right = v.blue.msb_right
		    = v.transp.msb_right = 0;
		v.nonstd = 0;
		v.accel_flags = 0;
	} else if (is_win(w, DAVINCIFB_WIN_OSD1)) {
		v.bits_per_pixel = 4;
	} else if (is_win(w, DAVINCIFB_WIN_VID0)) {
		v.bits_per_pixel = 16;
	} else if (is_win(w, DAVINCIFB_WIN_VID1)) {
		/* Rule 11 */
#if 0
		if (w->dm->windows[DAVINCIFB_WIN_VID0] &&
		    ((w->dm->windows[DAVINCIFB_WIN_VID0]->x - w->x) % 16)) {
			dev_dbg(dev, "vid1 x should be multiple of 16 from vid0\n");
			return -EINVAL;
		}
#endif
		/* Video1 may be in YUV or RGB888 format */
		if ((v.bits_per_pixel != 16) && (v.bits_per_pixel != 32))
			return -EINVAL;
	} else
		return -EINVAL;

	/* if we are on hd mode, we should not enable vid1 */
	/* normalize values */
	/* check specific window contraints */
	/* chek that the mode is valid */
	if (is_win(w, DAVINCIFB_WIN_VID0)) {
		int m;

		m = dm_venc_find_mode(&v);
		if (m < 0) {
			dev_dbg(dev, "No such VENC mode\n");
			return -EINVAL;
		}
		if (dm_venc_mode_is_hd(&dmfb_modedb[m])) {
			//printk("ok hd mode\n");
			if (w->dm->windows[DAVINCIFB_WIN_VID1] &&
				w->dm->windows[DAVINCIFB_WIN_VID1]->enabled) {

				dev_dbg(dev, "vid1 is enabled, disable it first\n");
				return -EINVAL;
			}
		}
	} else {
		/* set timmings from video output mode */
		v.vmode = dmfb_modedb[w->dm->curr_mode].vmode;
	}

	memcpy(var, &v, sizeof(v));
	return 0;
error:
	return -EINVAL;
}

/**
 *      davincifb_set_par - Optional function. Alters the hardware state.
 *      @info: frame buffer structure that represents a single frame buffer
 *
 *	Using the fb_var_screeninfo in fb_info we set the resolution of the
 *	this particular framebuffer. This function alters the par AND the
 *	fb_fix_screeninfo stored in fb_info. It doesn't not alter var in
 *	fb_info since we are using that data. This means we depend on the
 *	data in var inside fb_info to be supported by the hardware.
 *	davincifb_check_var is always called before dmfb_set_par to ensure this.
 *	Again if you can't can't the resolution you don't need this function.
 *
 */
static int davincifb_set_par(struct fb_info *info)
{
	struct dm_win_info *w = (struct dm_win_info *)info->par;
	struct fb_var_screeninfo *v = &info->var;
	int mode;
	int i;

	if (!is_win(w, DAVINCIFB_WIN_VID0)) {
		dm_win_mode_set(w, v);
		return 0;
	}
	/* we are going to change the mode */
	mode = dm_venc_find_mode(v);
	if (mode == w->dm->curr_mode) {
		printk("same mode, do nothing!!\n");
		return 0;
	}
	if (mode < 0)
	{
		printk("errorrrr\n");
		return 0;
	}
	dm_venc_mode_set(w->dm, &dmfb_modedb[mode], &dm_extended_modedb[mode]);
	w->dm->curr_mode = mode;
	dm_win_mode_set(w, v);
	/* different interlaced mode, update all other planes */
	for (i = 0; i < DAVINCIFB_WINDOWS; i++) {
		struct dm_win_info *w_tmp = w->dm->windows[i];

		if (!w_tmp || w_tmp == w)
			continue;
		w_tmp->info.var.vmode = dmfb_modedb[mode].vmode;
		dm_win_mode_set(w_tmp, &w_tmp->info.var);
	}
	return 0;
}

/**
 *	davincifb_ioctl - handler for private ioctls.
 */
static int davincifb_ioctl(struct fb_info *info, unsigned int cmd,
			   unsigned long arg)
{
	struct dm_win_info *w = (struct dm_win_info *)info->par;
	void __user *argp = (void __user *)arg;
	struct fb_fillrect rect;
	struct zoom_params zoom;
	unsigned int enable = 0;
	unsigned int pos = 0;
	unsigned int color = 0;
	struct dmfb_transparency transp;

	switch (cmd) {
	case FBIO_WAITFORVSYNC:
		/* This ioctl accepts an integer argument to specify a
		 * display.  We only support one display, so we will
		 * simply ignore the argument.
		 */
		return (davincifb_wait_for_vsync(w));
		break;
	case FBIO_SETATTRIBUTE:
		if (copy_from_user(&rect, argp, sizeof(rect)))
			return -EFAULT;
		if (!is_win(w, DAVINCIFB_WIN_OSD1))
			return -EINVAL;
		return (davincifb_set_attr_blend(w, &rect));
		break;
	/* FIXME this is wrong, we can't use the info.var as parameter
	 * to check var, it will be overwritten
	 */
	case FBIO_SETPOSX:
		if (copy_from_user(&pos, argp, sizeof(u_int32_t)))
			return -EFAULT;
		{
			struct fb_var_screeninfo v;
			int old_pos;

			memcpy(&v, &w->info.var, sizeof(v));
			old_pos = w->x;
			w->x = pos;
			if (davincifb_check_var(&v, &w->info) < 0) {
				w->x = old_pos;
				return -EINVAL;
			}
			memcpy(&w->info.var, &v, sizeof(v));
			davincifb_set_par(&w->info);
			return 0;
		}
		break;
	case FBIO_SETPOSY:
		if (copy_from_user(&pos, argp, sizeof(u_int32_t)))
			return -EFAULT;
		{
			struct fb_var_screeninfo v;
			int old_pos;

			memcpy(&v, &w->info.var, sizeof(v));
			old_pos = w->y;
			w->y = pos;
			if (davincifb_check_var(&v, &w->info) < 0) {
				w->y = old_pos;
				return -EINVAL;
			}
			memcpy(&w->info.var, &v, sizeof(v));
			davincifb_set_par(&w->info);
			return 0;
		}
		break;
	case FBIO_SETZOOM:
		if (copy_from_user(&zoom, argp, sizeof(zoom)))
			return -EFAULT;
		if ((zoom.zoom_h == 2) || (zoom.zoom_h == 0) ||
		    (zoom.zoom_h == 1) || (zoom.zoom_v == 2) ||
		    (zoom.zoom_v == 0) || (zoom.zoom_v == 1)) {
			if (!is_win(w, zoom.window_id))
				return -EINVAL;
			set_zoom(w, zoom.zoom_h, zoom.zoom_v);
			return 0;
		} else {
			return -EINVAL;
		}
		break;
	case FBIO_ENABLE:
		if (copy_from_user(&enable, argp, sizeof(u_int32_t)))
			return -EFAULT;
		if (!dm_win_enable(w, enable))
			return -EINVAL;
		else
			return 0;
		break;
	/* set the transparent rgb value this will affect all osd windows */
	case FBIO_TRANSP_COLOR:
		if (copy_from_user(&color, argp, sizeof(u_int32_t)))
			return -EFAULT;
		dm_transp_color_set(w->dm, color);
		return 0;
		break;
	/* enable the blending based on the transparent value for the specified
	 * plane
	 */
	case FBIO_TRANSP:
		if (copy_from_user(&transp, argp, sizeof(struct dmfb_transparency)))
			return -EFAULT;
		return dm_win_transp_enable(w, &transp);
		break;
	case FBIO_CBTEST:
		if (copy_from_user(&enable, argp, sizeof(u_int32_t)))
			return -EFAULT;
		dm_cbtest_enable(w->dm, enable);
		return 0;
		break;
	}

	return -EINVAL;
}

/**
 *  	davincifb_setcolreg - Optional function. Sets a color register.
 *      @regno: Which register in the CLUT we are programming
 *      @red: The red value which can be up to 16 bits wide
 *	@green: The green value which can be up to 16 bits wide
 *	@blue:  The blue value which can be up to 16 bits wide.
 *	@transp: If supported the alpha value which can be up to 16 bits wide.
 *      @info: frame buffer info structure
 *
 *  	Set a single color register. The values supplied have a 16 bit
 *  	magnitude which needs to be scaled in this function for the hardware.
 *	Things to take into consideration are how many color registers, if
 *	any, are supported with the current color visual. With truecolor mode
 *	no color palettes are supported. Here a psuedo palette is created
 *	which we store the value in pseudo_palette in struct fb_info. For
 *	pseudocolor mode we have a limited color palette. To deal with this
 *	we can program what color is displayed for a particular pixel value.
 *	DirectColor is similar in that we can program each color field. If
 *	we have a static colormap we don't need to implement this function.
 *
 *	Returns negative errno on error, or zero on success.
 */
static int davincifb_setcolreg(unsigned regno, unsigned red, unsigned green,
			       unsigned blue, unsigned transp,
			       struct fb_info *info)
{
	/* only pseudo-palette (16 bpp) allowed */
	if (regno >= 16)	/* maximum number of palette entries */
		return 1;

	if (info->var.grayscale) {
		/* grayscale = 0.30*R + 0.59*G + 0.11*B */
		red = green = blue = (red * 77 + green * 151 + blue * 28) >> 8;
	}

	/* Truecolor has hardware-independent 16-entry pseudo-palette */
	if (info->fix.visual == FB_VISUAL_TRUECOLOR) {
		u32 v;

		if (regno >= 16)
			return 1;

		red >>= (16 - info->var.red.length);
		green >>= (16 - info->var.green.length);
		blue >>= (16 - info->var.blue.length);

		v = (red << info->var.red.offset) |
		    (green << info->var.green.offset) |
		    (blue << info->var.blue.offset);

		switch (info->var.bits_per_pixel) {
		case 16:
			((u16 *) (info->pseudo_palette))[regno] = v;
			break;
		default:
			return 1;
		}
		return 0;
	}
	return 0;
}

/**
 *      davincifb_pan_display - NOT a required function. Pans the display.
 *      @var: frame buffer variable screen structure
 *      @info: frame buffer structure that represents a single frame buffer
 *
 *	Pan (or wrap, depending on the `vmode' field) the display using the
 *  	`xoffset' and `yoffset' fields of the `var' structure.
 *  	If the values don't fit, return -EINVAL.
 *
 *      Returns negative errno on error, or zero on success.
 */
static int davincifb_pan_display(struct fb_var_screeninfo *var,
				 struct fb_info *info)
{
	struct dm_win_info *w = (struct dm_win_info *)info->par;
	u32 start = 0, offset = 0;

	if (var->xoffset > var->xres_virtual - var->xres)
		return -EINVAL;
	if (var->yoffset > var->yres_virtual - var->yres)
		return -EINVAL;
	if ((var->xres_virtual * var->bits_per_pixel / 8) % 32)
		return -EINVAL;

	offset = var->yoffset * info->fix.line_length + var->xoffset * var->bits_per_pixel / 8;
	start = (u32) w->fb_base_phys + offset;

	if ((dispc_reg_in(VENC_VSTAT) & 0x00000010)==0x10)
		set_sdram_params(w, start, info->fix.line_length);
	else
		w->sdram_address = start;

	return 0;
}

/**
 *      davincifb_blank - NOT a required function. Blanks the display.
 *      @blank_mode: the blank mode we want.
 *      @info: frame buffer structure that represents a single frame buffer
 *
 *      Blank the screen if blank_mode != 0, else unblank. Return 0 if
 *      blanking succeeded, != 0 if un-/blanking failed due to e.g. a
 *      video mode which doesn't support it. Implements VESA suspend
 *      and powerdown modes on hardware that supports disabling hsync/vsync:
 *      blank_mode == 2: suspend vsync
 *      blank_mode == 3: suspend hsync
 *      blank_mode == 4: powerdown
 *
 *      Returns negative errno on error, or zero on success.
 *
 */
static int davincifb_blank(int blank_mode, struct fb_info *info)
{
	return 0;
}

static inline void fix_default_var(struct dm_win_info *w,
				   u32 xres, u32 yres, u32 xpos, u32 ypos,
				   int n_buf)
{
	struct fb_var_screeninfo *v = &w->info.var;

	v->xres = xres;
	v->yres = yres;
	v->xres_virtual = v->xres;
	v->yres_virtual = v->yres * n_buf;
	w->x = xpos;
	w->y = ypos;
}


/*
 *  Frame buffer operations
 */
static struct fb_ops davincifb_ops = {
	.owner = THIS_MODULE,
	.fb_check_var = davincifb_check_var,
	.fb_set_par = davincifb_set_par,
	.fb_setcolreg = davincifb_setcolreg,
	.fb_blank = davincifb_blank,
	.fb_pan_display = davincifb_pan_display,
	.fb_fillrect = cfb_fillrect,
	.fb_copyarea = cfb_copyarea,
	.fb_imageblit = cfb_imageblit,
	.fb_rotate = NULL,
	.fb_sync = NULL,
	.fb_ioctl = davincifb_ioctl,
};
/*============================================================================*
 *                            Platform device                                 *
 *============================================================================*/
/**
 *
 */
static irqreturn_t davincifb_isr(int irq, void *arg)
{
	struct dm_info *dm = (struct dm_info *)arg;
	int win;

#ifdef VID0FIX
	win = dm->windows[DAVINCIFB_WIN_VID0];
	if ((win) && (dmfb_modedb[dm->curr_mode].vmode == FB_VMODE_INTERLACED)) {
		int fld;
		int curr_fld;

		curr_fld = (dispc_reg_in(OSD_MISCCT) & OSD_MISCCT_PPSW) >> 2;
 		fld = ((dispc_reg_in(VENC_VSTAT) & 0x00000010) >> 4);
		if (fld != curr_fld)
			dispc_reg_merge(OSD_MISCCT, fld << 2, OSD_MISCCT_PPSW);
	}
#else
	if ((dispc_reg_in(VENC_VSTAT) & 0x00000010) == 0x10) {
		for (win = 0; win < DAVINCIFB_WINDOWS; win++)
		{
			struct dm_win_info *w;
			unsigned long addr = 0;

			w = dm->windows[win];
			if (!w)
				continue;
			xchg(&addr, w->sdram_address);
			if (addr) {
				set_sdram_params(w, w->sdram_address,
						 w->info.fix.line_length);
				w->sdram_address = 0;
			}
		}
		return IRQ_HANDLED;
	}
#endif
	 else {
		++dm->vsync_cnt;
		wake_up_interruptible(&dm->vsync_wait);
		return IRQ_HANDLED;
  	}

	return IRQ_HANDLED;
}
/*
 *  Cleanup
 */
static int davincifb_remove(struct platform_device *pdev)
{
	struct dm_info *dm;

	dm = platform_get_drvdata(pdev);
	free_irq(IRQ_VENCINT, dm);

	/* Cleanup all framebuffers */
	dm_wins_remove(dm);
	/* Reset OSD registers to default. */
	dispc_reg_out(OSD_MODE, 0);
	dispc_reg_out(OSD_VIDWINMD, 0);
	dispc_reg_out(OSD_OSDWIN0MD, 0);
	dispc_reg_out(OSD_OSDWIN1MD, 0);
	/* TODO remove this */
	/* Turn OFF the output device */
	//dm->output_device_config(0);
	video_output_unregister(dm->output);

	if (dm->mmio_base)
		iounmap((void *)dm->mmio_base);
	release_mem_region(dm->mmio_base_phys, dm->mmio_size);

	kfree(dm);
	return 0;
}
/*
 *  Initialization
 */
static int davincifb_probe(struct platform_device *pdev)
{
	struct dm_info *dm;

	printk(MODULE_NAME " Initializing\n");
	dm = kzalloc(sizeof(struct dm_info), GFP_KERNEL);

	dm->dev = &pdev->dev;
	dm->mach_info = pdev->dev.platform_data;
	dm->mmio_base_phys = OSD_REG_BASE;
	dm->mmio_size = OSD_REG_SIZE;

	if (!request_mem_region
	    (dm->mmio_base_phys, dm->mmio_size, MODULE_NAME)) {
		dev_err(dm->dev, ": cannot reserve MMIO region\n");
		goto free_dm;
	}

	/* map the regions */
	dm->mmio_base = (unsigned long)ioremap(dm->mmio_base_phys, dm->mmio_size);
	if (!dm->mmio_base) {
		dev_err(dm->dev, ": cannot map MMIO\n");
		goto release_mmio;
	}

	/* initialize the vsync wait queue */
	init_waitqueue_head(&dm->vsync_wait);
	dm->timeout = HZ / 5;

	printk("Setting Up Clocks for DM420 OSD rev = %x\n", dispc_reg_in(VPBE_PID));
	/* Initialize the VPSS Clock Control register */
	dispc_reg_out(VPSS_CLKCTL, 0x18);

	/* Set Base Pixel X and Base Pixel Y */
	dispc_reg_out(OSD_BASEPX, BASEX);
	dispc_reg_out(OSD_BASEPY, BASEY);

	/* Reset OSD registers to default. */
	dispc_reg_out(OSD_MODE, 0);
	dispc_reg_out(OSD_OSDWIN0MD, 0);
	dispc_reg_out(OSD_OSDWIN1MD, 0);
	dispc_reg_out(OSD_VIDWINMD, 0);

	/* Set blue background color */
	set_bg_color(0, 162);

	/* Field Inversion Workaround */
	dispc_reg_out(OSD_MODE, 0x200);

	dm->windows_mask = (1 << DAVINCIFB_WIN_OSD0) |(1 << DAVINCIFB_WIN_OSD1) |
			(1 << DAVINCIFB_WIN_VID0) | (1 << DAVINCIFB_WIN_VID1);
	if (dm_wins_probe(dm) < 0)
		goto probe_error;
	/* install our interrupt service routine */
	if (request_irq(IRQ_VENCINT, davincifb_isr, IRQF_SHARED, MODULE_NAME, dm)) {
		dev_err(dm->dev, MODULE_NAME ": could not install interrupt service routine\n");
		goto irq_error;
	}
	/* TODO remove this */
	//dm->output_device_config(1);

	dm->output = video_output_register("venc", dm->dev, dm, &dm_venc_props);
	if (!dm->output)
		goto venc_error;
	platform_set_drvdata(pdev, dm);
	return 0;

venc_error:
	free_irq(IRQ_VENCINT, dm);
irq_error:
	dm_wins_remove(dm);
probe_error:
	iounmap((void *)dm->mmio_base);
release_mmio:
	release_mem_region(dm->mmio_base_phys, dm->mmio_size);
free_dm:
	kfree(dm);
	return -ENODEV;
}

static struct platform_driver davincifb_driver = {
	.probe		= davincifb_probe,
	.remove		= davincifb_remove,
	.driver		= {
		.name	= MODULE_NAME,
		.owner	= THIS_MODULE,
	},
};

/*
 * Pass boot-time options by adding the following string to the boot params:
 * 	video=davincifb:[option[:option]]
 * Valid options:
 * 	output=[lcd|ntsc|pal]
 * 	format=[composite|s-video|component|rgb]
 * 	vid0=[off|MxN@X,Y]
 * 	vid1=[off|MxN@X,Y]
 * 	osd0=[off|MxN@X,Y]
 * 	osd1=[off|MxN@X,Y]
 * 		MxN specify the window resolution (displayed size)
 * 		X,Y specify the window position
 * 		M, N, X, Y are integers
 * 		M, X should be multiples of 16
 */

#ifndef MODULE
int __init davincifb_setup(char *options)
{
	char *this_opt;
	u32 xres, yres, xpos, ypos;
	int format_xres = 720;
	int format_yres = 480;

	pr_debug("davincifb: Options \"%s\"\n", options);

	if (!options || !*options)
		return 0;
#if 0
	/* This will go away soon */
	while ((this_opt = strsep(&options, ":")) != NULL) {

		if (!*this_opt)
			continue;

		if (!strncmp(this_opt, "output=", 7)) {
			if (!strncmp(this_opt + 7, "lcd", 3)) {
				dmparams.output = LCD;
				dmparams.format = 0;
			} else if (!strncmp(this_opt + 7, "ntsc", 4))
				dmparams.output = NTSC;
			else if (!strncmp(this_opt + 7, "pal", 3))
				dmparams.output = PAL;
			else if (!strncmp(this_opt + 7, "720p", 4)) {
				dmparams.output = HD720P;
				dmparams.format = COMPONENT;
			} else if (!strncmp(this_opt + 7, "1080i", 5)) {
				dmparams.output = HD1080I;
				dmparams.format = COMPONENT;
			} else if (!strncmp(this_opt + 7, "480p", 4)) {
				dmparams.output = HD480P;
				dmparams.format = COMPONENT;
			}
		} else if (!strncmp(this_opt, "format=", 7)) {
			if (dmparams.output == LCD || dmparams.output == HD720P ||
			    dmparams.output == HD1080I || dmparams.output == HD480P)
				continue;
			if (!strncmp(this_opt + 7, "composite", 9))
				dmparams.format = COMPOSITE;
			else if (!strncmp(this_opt + 7, "s-video", 7))
				dmparams.format = SVIDEO;
			else if (!strncmp(this_opt + 7, "component", 9))
				dmparams.format = COMPONENT;
			else if (!strncmp(this_opt + 7, "rgb", 3))
				dmparams.format = RGB;
		} else if (!strncmp(this_opt, "vid0=", 5)) {
			if (!strncmp(this_opt + 5, "off", 3))
				dmparams.windows &= ~(1 << DAVINCIFB_WIN_VID0);
			else if (!parse_win_params(this_opt + 5,
						   &xres, &yres, &xpos, &ypos)) {
				dmparams.vid0_xres = xres;
				dmparams.vid0_yres = yres;
				dmparams.vid0_xpos = xpos;
				dmparams.vid0_ypos = ypos;
			}
		} else if (!strncmp(this_opt, "vid1=", 5)) {
			if (!strncmp(this_opt + 5, "off", 3))
				dmparams.windows &= ~(1 << DAVINCIFB_WIN_VID1);
			else if (!parse_win_params(this_opt + 5,
						   &xres, &yres, &xpos, &ypos)) {
				dmparams.vid1_xres = xres;
				dmparams.vid1_yres = yres;
				dmparams.vid1_xpos = xpos;
				dmparams.vid1_ypos = ypos;
			}
		} else if (!strncmp(this_opt, "osd0=", 5)) {
			if (!strncmp(this_opt + 5, "off", 3))
				dmparams.windows &= ~(1 << DAVINCIFB_WIN_OSD0);
			else if (!parse_win_params(this_opt + 5,
						   &xres, &yres, &xpos, &ypos)) {
				dmparams.osd0_xres = xres;
				dmparams.osd0_yres = yres;
				dmparams.osd0_xpos = xpos;
				dmparams.osd0_ypos = ypos;
			}
		} else if (!strncmp(this_opt, "osd1=", 5)) {
			if (!strncmp(this_opt + 5, "off", 3))
				dmparams.windows &= ~(1 << DAVINCIFB_WIN_OSD1);
			else if (!parse_win_params(this_opt + 5,
						   &xres, &yres, &xpos, &ypos)) {
				dmparams.osd1_xres = xres;
				dmparams.osd1_yres = yres;
				dmparams.osd1_xpos = xpos;
				dmparams.osd1_ypos = ypos;
			}
		}
	}
	printk(KERN_INFO "DaVinci: " "Output on %s%s, Enabled windows: %s %s %s %s\n",
	       (dmparams.output == LCD) ? "LCD" :
	       (dmparams.output == HD720P) ? "HD720P":
	       (dmparams.output == HD1080I) ? "HD1080I":
	       (dmparams.output == HD480P) ? "HD480P":
	       (dmparams.output == NTSC) ? "NTSC" :
	       (dmparams.output == PAL) ? "PAL" : "unknown device!",
	       (dmparams.format == 0) ? "" :
	       (dmparams.format == COMPOSITE) ? " in COMPOSITE format" :
	       (dmparams.format == SVIDEO) ? " in SVIDEO format" :
	       (dmparams.format == COMPONENT) ? " in COMPONENT format" :
	       (dmparams.format == RGB) ? " in RGB format" : "",
	       (dmparams.windows & (1 << DAVINCIFB_WIN_VID0)) ? "Video0" : "",
	       (dmparams.windows & (1 << DAVINCIFB_WIN_VID1)) ? "Video1" : "",
	       (dmparams.windows & (1 << DAVINCIFB_WIN_OSD0)) ? "DAVINCIFB_WIN_OSD0" : "",
	       (dmparams.windows & (1 << DAVINCIFB_WIN_OSD1)) ? "DAVINCIFB_WIN_OSD1" : "");
	if (dmparams.output == NTSC) {
		format_yres = 480;
	} else if (dmparams.output == PAL) {
		format_yres = 576;
	} else if (dmparams.output == HD720P) {
		format_xres = DISP_XRES720P;
		format_yres = DISP_YRES720P;
	} else if (dmparams.output == HD1080I) {
		format_xres = DISP_XRES1080I;
		format_yres = DISP_YRES1080I;
	} else if (dmparams.output == HD480P) {
		format_xres = DISP_XRES480P;
		format_yres = DISP_YRES480P;
	} else {
		printk(KERN_INFO "DaVinci:invalid format..defaulting width to 480\n");
	}
	dmparams.osd0_xres = osd0_default_var.xres = format_xres;
	dmparams.osd1_xres = osd1_default_var.xres = format_xres;
	dmparams.vid0_xres = vid0_default_var.xres = format_xres;
	dmparams.vid1_xres = vid1_default_var.xres = format_xres;

	dmparams.osd0_yres = osd0_default_var.yres = format_yres;
	dmparams.osd1_yres = osd1_default_var.yres = format_yres;
	dmparams.vid0_yres = vid0_default_var.yres = format_yres;
	dmparams.vid1_yres = vid1_default_var.yres = format_yres;

	osd0_default_var.xres_virtual = round_32((format_xres)*16/8) * 8/16;
	osd1_default_var.xres_virtual = round_32((format_xres)*16/8) * 8/16;
	vid0_default_var.xres_virtual = round_32((format_xres)*16/8) * 8/16;
	vid1_default_var.xres_virtual = round_32((format_xres)*16/8) * 8/16;

	osd0_default_var.yres_virtual = format_yres * DOUBLE_BUF;
	osd1_default_var.yres_virtual = format_yres * DOUBLE_BUF;
	vid0_default_var.yres_virtual = format_yres * TRIPLE_BUF;
	vid1_default_var.yres_virtual = format_yres * TRIPLE_BUF;

	dmparams.osd0_phys = DAVINCI_FB_RESERVE_MEM_BASE;
	dmparams.osd1_phys = dmparams.osd0_phys + fb_window_size(format_xres, format_yres, DOUBLE_BUF);
	dmparams.vid0_phys = dmparams.osd1_phys + fb_window_size(format_xres, format_yres, DOUBLE_BUF);
	dmparams.vid1_phys = dmparams.vid0_phys + fb_window_size(format_xres, format_yres, TRIPLE_BUF);

	if (dmparams.windows & (1 << DAVINCIFB_WIN_VID0))
		printk(KERN_INFO "Setting Video0 size %dx%d, "
		       "position (%d,%d)\n",
		       dmparams.vid0_xres, dmparams.vid0_yres,
		       dmparams.vid0_xpos, dmparams.vid0_ypos);
	if (dmparams.windows & (1 << DAVINCIFB_WIN_VID1))
		printk(KERN_INFO "Setting Video1 size %dx%d, "
		       "position (%d,%d)\n",
		       dmparams.vid1_xres, dmparams.vid1_yres,
		       dmparams.vid1_xpos, dmparams.vid1_ypos);
	if (dmparams.windows & (1 << DAVINCIFB_WIN_OSD0))
		printk(KERN_INFO "Setting DAVINCIFB_WIN_OSD0 size %dx%d, "
		       "position (%d,%d)\n",
		       dmparams.osd0_xres, dmparams.osd0_yres,
		       dmparams.osd0_xpos, dmparams.osd0_ypos);
	if (dmparams.windows & (1 << DAVINCIFB_WIN_OSD1))
		printk(KERN_INFO "Setting DAVINCIFB_WIN_OSD1 size %dx%d, "
		       "position (%d,%d)\n",
		       dmparams.osd1_xres, dmparams.osd1_yres,
		       dmparams.osd1_xpos, dmparams.osd1_ypos);
#endif
	return 0;
}
#endif

/*============================================================================*
 *                            Module Interface                                *
 *============================================================================*/
extern void davinci_mux_peripheral(unsigned int mux, unsigned int enable);

/* Register both the driver and the device */
int __init davincifb_init(void)
{
#ifndef MODULE
	/* boot-line options */
	/* handle options for "dm64xxfb" for backwards compatability */
	char *option;
	char *names[] = { "davincifb", "dm64xxfb" };
	int i, num_names = 2, done = 0;

	for (i = 0; i < num_names && !done; i++) {
		if (fb_get_options(names[i], &option)) {
			printk(MODULE_NAME ": Disabled on command-line.\n");
			return -ENODEV;
		} else if (option) {
			davincifb_setup(option);
			done = 1;
		}
	}
#endif
	/* Register the driver with LDM */
	if (platform_driver_register(&davincifb_driver)) {
		pr_debug("failed to register davincifb driver\n");
		return -ENODEV;
	}

	return 0;
}

static void __exit davincifb_cleanup(void)
{
	platform_driver_unregister(&davincifb_driver);
}

module_init(davincifb_init);
module_exit(davincifb_cleanup);

MODULE_DESCRIPTION("Framebuffer driver for TI DaVinci");
MODULE_AUTHOR("Texas Instruments");
MODULE_LICENSE("GPL");
