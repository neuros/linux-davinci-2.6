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
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307 USA
 */

/* davinci_previewer_hw.c file */

#include <linux/errno.h>

#include <asm/arch/davinci_previewer_hw.h>
#include <asm/arch/davinci_previewer.h>

#include <linux/device.h>
#ifdef __KERNEL__

extern struct device *prev_dev;

/* previewer_hw_setup:It is used for Hardware Setup */
int previewer_hw_setup(struct prev_params *config)
{
	u32 utemp = 0, pcr = 0;
	s32 temp = 0;
	int i, j;
	/* Setting Down Sampling rate for input averager */
	if (!config) {
		dev_err(prev_dev, "\nError in prev_params");
		return -EINVAL;
	}
	/* check whether down sampling rate is one of the supported */
	if (config->sample_rate != DOWN_SAMPLE_RATE1
	    && config->sample_rate != DOWN_SAMPLE_RATE2
	    && config->sample_rate != DOWN_SAMPLE_RATE3
	    && config->sample_rate != DOWN_SAMPLE_RATE4)
		/* if not return error */
		return -EINVAL;

	utemp |= AVE_ODD_PIXEL_DIST;
	utemp |= AVE_EVEN_PIXEL_DIST;

	switch (config->sample_rate) {
	case DOWN_SAMPLE_RATE1:
		utemp |= 0;
		break;
	case DOWN_SAMPLE_RATE2:
		utemp |= 1;
		break;
	case DOWN_SAMPLE_RATE3:
		utemp |= 2;
		break;
	case DOWN_SAMPLE_RATE4:
		utemp |= 3;
		break;
	default:
		utemp |= 0;
	}

	/* set sampling rate in register */
	regw(utemp, AVE);
	utemp = 0;

	/* Setting white balancing parameters */

	/* Set the common gain for white balancing in register */
	regw(((config->white_balance_params.wb_dgain) & 0x03FF), WB_DGAIN);

	/* Set individual color gains in register for white balancing */
	utemp = (int)(config->white_balance_params.wb_gain[0]);
	utemp |= ((int)(config->white_balance_params.wb_gain[1]) << 8);
	utemp |= ((int)(config->white_balance_params.wb_gain[2]) << 16);
	utemp |= ((int)(config->white_balance_params.wb_gain[3]) << 24);

	/* Write individual color gains to the register */
	regw(utemp, WBGAIN);

	/* Setting position of the colors in 4x4 grid */
	for (utemp = 0, i = 0; i < WB_GAIN_MAX; i++) {
		for (j = 0; j < WB_GAIN_MAX; j++) {
			utemp |=
			    ((config->white_balance_params.
			      wb_coefmatrix[i][j] & 0x03) << ((i * 8) +
							      (j * 2)));
		}
	}
	regw(utemp, WBSEL);

	/* setting RGB2RGB blending */
	temp = 0;

	temp = config->rgbblending_params.blending[0][0] & 0x0FFF;
	temp |= ((config->rgbblending_params.blending[0][1] & 0x0FFF) << 16);

	regw(temp, RGB_MAT1);

	temp = 0;

	temp = config->rgbblending_params.blending[0][2] & 0x0FFF;
	temp |= ((config->rgbblending_params.blending[1][0] & 0x0FFF) << 16);

	regw(temp, RGB_MAT2);

	temp = 0;

	temp = config->rgbblending_params.blending[1][1] & 0x0FFF;
	temp |= ((config->rgbblending_params.blending[1][2] & 0x0FFF) << 16);

	regw(temp, RGB_MAT3);

	temp = 0;

	temp = config->rgbblending_params.blending[2][0] & 0x0FFF;
	temp |= ((config->rgbblending_params.blending[2][1] & 0x0FFF) << 16);

	regw(temp, RGB_MAT4);

	temp = 0;

	temp = config->rgbblending_params.blending[2][2] & 0x0FFF;

	regw(temp, RGB_MAT5);

	/* Writing offset of RGB2RGB blending */

	temp = 0;
	temp = config->rgbblending_params.offset[1] & 0x03FF;
	temp |= ((config->rgbblending_params.offset[0] & 0x03FF) << 16);

	regw(temp, RGB_OFF1);

	temp = 0;
	temp = config->rgbblending_params.offset[2] & 0x03FF;
	regw(temp, RGB_OFF2);

	/* Setting RGB 2 YCbCr matrix gains and offsets */
	temp = 0;
	temp = ((config->rgb2ycbcr_params.coeff[0][0]) & 0x03FF);
	temp |= (((config->rgb2ycbcr_params.coeff[0][1]) & 0x03FF) << 10);
	temp |= (((config->rgb2ycbcr_params.coeff[0][2]) & 0x03FF) << 20);
	regw(temp, CSC0);

	temp = 0;
	temp = (config->rgb2ycbcr_params.coeff[1][0] & 0x03FF);
	temp |= (((config->rgb2ycbcr_params.coeff[1][1]) & 0x03FF) << 10);
	temp |= (((config->rgb2ycbcr_params.coeff[1][2]) & 0x03FF) << 20);
	regw(temp, CSC1);

	temp = 0;
	temp = (config->rgb2ycbcr_params.coeff[2][0] & 0x03FF);
	temp |= (((config->rgb2ycbcr_params.coeff[2][1]) & 0x03FF) << 10);
	temp |= (((config->rgb2ycbcr_params.coeff[2][2]) & 0x03FF) << 20);
	regw(temp, CSC2);

	temp = 0;
	temp = (config->rgb2ycbcr_params.offset[2] & 0x00FF);
	temp |= ((config->rgb2ycbcr_params.offset[1] & 0x00FF) << 8);
	temp |= ((config->rgb2ycbcr_params.offset[0] & 0x00FF) << 16);
	regw(temp, CSC_OFFSET);

	/* Setting black adjustment offsets */
	temp = 0;
	temp = config->black_adjst_params.blueblkadj;
	temp |= (config->black_adjst_params.greenblkadj << 8);
	temp |= (config->black_adjst_params.redblkadj << 16);
	regw(temp, BLKADJOFF);

	temp = (config->contrast & 0x0FF) << 8;
	temp |= (config->brightness & 0xFF);
	regw(temp, CNT_BRT);

	/* Enable dark frame capture if it is enabled in configuration */
	if (config->features & PREV_DARK_FRAME_CAPTURE) {
		SETBIT(pcr, DARK_FRAME_CAPTURE_BIT);
	}
	/* Enable Inverse A-Law if it is enabled in configuration */
	if (config->features & PREV_INVERSE_ALAW) {
		SETBIT(pcr, INVALAW_BIT);
	}

	/* Enable HMF and set its threshold if it is enabled in 
	   configuration */
	if (config->features & PREV_HORZ_MEDIAN_FILTER) {
		SETBIT(pcr, HMF_BIT);
		regw(((config->hmf_threshold & 0xFF) | 0x300), HMED);
	}

	/* Enable Noise filter and set its coefficients if it is 
	   enabled in configuration */
	if (config->features & PREV_NOISE_FILTER) {
		SETBIT(pcr, NOISE_FILTER_BIT);
		/* Set coefficients of NF */
		/* Set table address */
		regw(NOISE_FILTER_START_ADDR, SET_TBL_ADDR);
		/* set data */
		for (i = NOISE_FILTER_START_ADDR;
		     i <= NOISE_FILTER_END_ADDR; i++) {
			regw(config->nf_coeffs.
			     noise[i - NOISE_FILTER_START_ADDR], SET_TBL_DATA);
			/* Address is auto incremented */
		}
		/* write the strength of the weighted average */
		regw((config->nf_coeffs.strength & 0x0F), NF);
	}

	/*  Set CFA Coefficients */
	if (config->features & PREV_CFA) {
		/* enable CFA interpolation in pcr */
		SETBIT(pcr, CFA_BIT);

		/* Set coefficients of Gamma correction */
		/* Set table address for red gamma */
		regw(CFA_COEFF_START_ADDR, SET_TBL_ADDR);
		/* set data */
		for (i = CFA_COEFF_START_ADDR; i <= CFA_COEFF_END_ADDR; i++) {
			regw(config->cfa_coeffs.
			     coeffs[i - CFA_COEFF_START_ADDR], SET_TBL_DATA);
			/* Address is auto incremented */
		}

		/* set horizontal and vertical threshold */
		temp = (config->cfa_coeffs.hthreshold & 0xFF);
		temp |= ((config->cfa_coeffs.vthreshold & 0xFF) << 8);
		regw(temp, CFA);
	}

	/* Set gamma correction Coefficients if it is enabled in 
	   configuration */
	if (config->features & PREV_GAMMA) {
		/* disable gamma bypass in PCR */
		RESETBIT(pcr, GAMMA_BYPASS_BIT);
		/* Set coefficients of Gamma correction */
		/* Set table address for red gamma */
		regw(RED_GAMMA_START_ADDR, SET_TBL_ADDR);
		/* set data */
		for (i = RED_GAMMA_START_ADDR; i <= RED_GAMMA_END_ADDR; i++) {
			regw(config->gamma_coeffs.
			     red[i - RED_GAMMA_START_ADDR], SET_TBL_DATA);
			/* Address is auto incremented */
		}
		/* Set table start address for green gamma */
		regw(GREEN_GAMMA_START_ADDR, SET_TBL_ADDR);
		/* set data */
		for (i = GREEN_GAMMA_START_ADDR; i <= GREEN_GAMMA_END_ADDR; i++) {
			regw(config->gamma_coeffs.
			     green[i - GREEN_GAMMA_START_ADDR], SET_TBL_DATA);
			/* Address is auto incremented */
		}
		/* Set table address for red gamma */
		regw(BLUE_GAMMA_START_ADDR, SET_TBL_ADDR);
		/* set data */
		for (i = BLUE_GAMMA_START_ADDR; i <= BLUE_GAMMA_END_ADDR; i++) {
			regw(config->gamma_coeffs.
			     blue[i - BLUE_GAMMA_START_ADDR], SET_TBL_DATA);
			/* Address is auto incremented */
		}
	} else {
		/* else enable gamma bypassing */
		SETBIT(pcr, GAMMA_BYPASS_BIT);
	}

	/* Set luma enhancement Coefficients if it is enabled in 
	   configuration */
	if (config->features & PREV_LUMA_ENHANCE) {
		/* enable Luma enhancement in PCR */
		SETBIT(pcr, LUMA_ENHANCE_BIT);

		/* Set the start address for luma enhancement */
		regw(LUMA_ENHANCE_START_ADDR, SET_TBL_ADDR);
		/* set data */
		for (i = LUMA_ENHANCE_START_ADDR;
		     i <= LUMA_ENHANCE_END_ADDR; i++) {
			regw(config->
			     luma_enhance[i - LUMA_ENHANCE_START_ADDR],
			     SET_TBL_DATA);
		}
	}

	/* Set luma enhancement Coefficients if it is enabled in 
	   configuration */
	if (config->features & PREV_CHROMA_SUPPRESS) {
		/* enable Luma enhancement in PCR */
		SETBIT(pcr, CHROMA_SUPPRESS_BIT);
		temp = 0;
		if (config->chroma_suppress_params.hpfy) {
			SETBIT(temp, CHROMA_HPFY);
		} else {
			RESETBIT(temp, CHROMA_HPFY);
		}

		temp |= (config->chroma_suppress_params.gain & 0x00FF);
		temp |= (config->chroma_suppress_params.threshold << 8);

		regw(temp, CSUP);
	}

	/* enable dark frame subtract if it is enabled in configuration */
	if (config->features & PREV_DARK_FRAME_SUBTRACT) {
		/* enable dark frame subtract in PCR */
		SETBIT(pcr, DARK_FRAME_WRITE_BIT);

		/* set the dark frame address and line offset */
		regw(config->dark_frame_addr, DSDR_ADDR);
		regw(config->dark_frame_pitch, DRKF_OFFSET);
	}

	/* enable lens shading if it is enabled in configuration */
	if (config->features & PREV_LENS_SHADING) {
		/* enable lens shading in PCR */
		SETBIT(pcr, SHADECOMP_BIT);

		/* set the dark frame address and line offset */
		regw(config->dark_frame_addr, DSDR_ADDR);
		regw(config->dark_frame_pitch, DRKF_OFFSET);

		/* set lens shading shift parameter */
		pcr |=
		    (((config->
		       lens_shading_sift) & 0x7) << (SHADECOMP_BIT + 1));
	}

	/* Set previewer source to DDRAM */
	SETBIT(pcr, PREV_SOURCE_BIT);
	RESETBIT(pcr, PREV_SOURCE_BIT);

	/* Set one shot mode */
	SETBIT(pcr, PREV_ONESHOT_BIT);
	RESETBIT(pcr, PREV_ONESHOT_BIT);

	/* Set pixel width */
	if (config->size_params.pixsize == PREV_INWIDTH_8BIT)
		SETBIT(pcr, PREV_WIDTH_BIT);
	else
		RESETBIT(pcr, PREV_WIDTH_BIT);

	/* Enable writing to DDRAM */
	SETBIT(pcr, DDRAMPORT_BIT);

	/* set output format in PCR */
	pcr |= ((config->pix_fmt & 0x3) << PIXEL_ORDER_BIT);

	/* Write PCR register */
	regw(pcr, PCR);

	return 0;
}

#endif				/* End of #ifdef __KERNEL__ */
