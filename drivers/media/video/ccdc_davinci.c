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
/* ccdc_davinci.c */

#include <media/ccdc_davinci.h>
#define debug_print(x...)	//printk(x)
void ccdc_reset()
{
	int i;
	/* disable CCDC */
	ccdc_enable(0);
	/* set all registers to default value */
	for (i = 0; i <= 0x94; i += 4) {
		regw(0, i);
	}
	regw(0, PCR);
	regw(0, SYN_MODE);
	regw(0, HD_VD_WID);
	regw(0, PIX_LINES);
	regw(0, HORZ_INFO);
	regw(0, VERT_START);
	regw(0, VERT_LINES);
	regw(0xffff00ff, CULLING);
	regw(0, HSIZE_OFF);
	regw(0, SDOFST);
	regw(0, SDR_ADDR);
	regw(0, VDINT);
	regw(0, REC656IF);
	regw(0, CCDCFG);
	regw(0, FMTCFG);
	regw(0, VP_OUT);
}

void ccdc_setwin(ccdc_params_ycbcr * params)
{
	int horz_start, horz_nr_pixels;
	int vert_start, vert_nr_lines;

	/* configure horizonal and vertical starts and sizes */
	horz_start = params->win.left << 1;
	horz_nr_pixels = (params->win.width <<1) - 1;
	regw((horz_start << 16) | horz_nr_pixels, HORZ_INFO);

	vert_start = params->win.top;

	if (params->frm_fmt == CCDC_FRMFMT_INTERLACED) {
		vert_nr_lines = (params->win.height >> 1) - 1;
		vert_start >>= 1;
	} else {
		vert_nr_lines = params->win.height - 1;
	}
	regw((vert_start << 16) | vert_start, VERT_START);
	regw(vert_nr_lines, VERT_LINES);
}

void ccdc_config_ycbcr(ccdc_params_ycbcr * params)
{
	u32 syn_mode;

	/* first reset the CCDC                                          */
	/* all registers have default values after reset                 */
	/* This is important since we assume default values to be set in */
	/* a lot of registers that we didn't touch                       */
	ccdc_reset();

	/* configure pixel format */
	syn_mode = (params->pix_fmt & 0x3) << 12;

	/* configure video frame format */
	syn_mode |= (params->frm_fmt & 0x1) << 7;

	/* setup BT.656 sync mode */
	if (params->bt656_enable) {
		regw(3, REC656IF);

		/* configure the FID, VD, HD pin polarity */
		/* fld,hd pol positive, vd negative, 8-bit pack mode */
		syn_mode |= 0x00000F04;
	} else {/* y/c external sync mode */
		syn_mode |= ((params->fid_pol & 0x1) << 4);
		syn_mode |= ((params->hd_pol & 0x1) << 3);
		syn_mode |= ((params->vd_pol & 0x1) << 2);
	}

	/* configure video window */
	ccdc_setwin(params);

	/* configure the order of y cb cr in SD-RAM */
	regw((params->pix_order << 11) | 0x8000, CCDCFG);

	/* configure the horizontal line offset */
	/* this is done by rounding up width to a multiple of 16 pixels */
	/* and multiply by two to account for y:cb:cr 4:2:2 data */
	regw(((params->win.width * 2) + 31) & 0xffffffe0, HSIZE_OFF);

	/* configure the memory line offset */
	if (params->buf_type == CCDC_BUFTYPE_FLD_INTERLEAVED) {
		/* two fields are interleaved in memory */
		regw(0x00000249, SDOFST);
	}
	/* enable output to SDRAM */
	syn_mode |= (0x1 << 17);
	/* enable internal timing generator */
	syn_mode |= (0x1 << 16);

	regw(syn_mode, SYN_MODE);
}
