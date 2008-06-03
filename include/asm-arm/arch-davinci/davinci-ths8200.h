/* *
 * Copyright (C) 2008 Neuros Technology International LLC
 *
 * This program is free software you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation version 2 of the License.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not,write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307 USA
 */
/* davinci-ths8200.h file */

#ifndef DAVINCI_THS8200
#define DAVINCI_THS8200

/* THS8200 Register File */
/* System Control Registers 0x02-0x03 */
#define VERSION_REG					0x02
#define CHIP_CTL_REG			    0x03

/* Color Space Conversion Control Registers 0x04-0x19 */
#define CSC_R11_REG					 0x04
#define CSC_R21_REG					 0x06
#define CSC_R31_REG					 0x08
#define CSC_G11_REG					 0x0A
#define CSC_G21_REG					 0x0C
#define CSC_G31_REG					 0x0E
#define CSC_B11_REG					 0x10
#define CSC_B21_REG					 0x12
#define CSC_B31_REG					 0x14
#define CSC_OFFS1_REG			  0x16
#define CSC_OFFS12_REG		 	 0x17
#define CSC_OFFS23_REG			 0x18
#define CSC_OFFS3_REG			  0x19

/* Test Control Registers 0x1A-0x1B */
#define TST_CNTL1_REG						0x1A
#define TST_CNTL2_REG						0x1B

/* Data Path Control Register 0x1C */
#define DATA_CNTL_REG						0x1C

/* Display Timing Generator Control Part1 Registers 0x1D-0x3C */
#define DTG1_Y_SYNC1_LSB_REG		0x1D
#define DTG1_Y_SYNC2_LSB_REG		0x1E
#define DTG1_Y_SYNC3_LSB_REG		0x1F
#define DTG1_CBCR_SYNC1_LSB_REG		0x20
#define DTG1_CBCR_SYNC2_LSB_REG		0x21
#define DTG1_CBCR_SYNC3_LSB_REG		0x22
#define DTG1_Y_SYNC_MSB_REG				 0x23
#define DTG1_CBCR_SYNC_MSB_REG		0x24
#define DTG1_SPEC_A_REG							0x25
#define DTG1_SPEC_B_REG							0x26
#define DTG1_SPEC_C_REG							0x27
#define DTG1_SPEC_D_LSB_REG		  		   0x28
#define DTG1_SPEC_D1_REG					   0x29
#define DTG1_SPEC_E_LSB_REG					0x2A
#define DTG1_SPEC_DEH_MSB_REG		  0x2B
#define DTG1_SPEC_H_LSB_REG				   0x2C
#define DTG1_SPEC_I_MSB_REG					0x2D
#define DTG1_SPEC_I_LSB_REG					 0x2E
#define DTG1_SPEC_K_LSB_REG					0x2F
#define DTG1_SPEC_K_MSB_REG				  0x30
#define DTG1_SPEC_K1_REG						0x31
#define DTG1_SPEC_G_LSB_REG					0x32
#define DTG1_SPEC_G_MSB_REG				  0x33
#define DTG1_TOTAL_PIXELS_MSB_REG				  0x34
#define DTG1_TOTAL_PIXELS_LSB_REG					0x35
#define DTG1_FIELDFLIP_LINECNT_MSB_REG		   0x36
#define DTG1_FIELDFLIP_LINECNT_LSB_REG			0x37
#define DTG1_MODE_REG			0x38
#define DTG1_FRAME_FILED_SIZE_MSB_REG		  0x39
#define DTG1_FRAMESIZE_LSB_REG			0x3A
#define DTG1_FIELDSIZE_LSB_REG			 0x3B
#define DTG1_VESA_CBAR_SIZE_REG			0x3C

/* DAC control Registers 0x3D-0x40 */

/* Clip/Scale/Multiplier Control Registers 0x41-0x4F */
#define CSM_GY_CNTL_MULT_MSB_REG   0x4A

/* Display Timing Generator Control Part2 Registers 0x50-0x82 */
#define DTG2_HLENGTH_LSB_REG			 0x70
#define DTG2_HLENGTH_LSB_HDLY_MSB	0x71
#define DTG2_HDLY_LSB		0x72
#define DTG2_VLENGTH1_LSB		0x73
#define DTG2_VLENGTH1_MSB_VDLY1_MSB		0x74
#define DTG2_VDLY1_LSB		0x75
#define DTG2_VLENGTH2_LSB		0x76
#define DTG2_VLENGTH2_MSB_VDLY2_MSB  0x77
#define DTG2_VDLY2_LSB		0x78
#define DTG2_HS_IN_DLY_MSB_REG			0x79
#define DTG2_HS_IN_DLY_LSB_REG			 0x7A
#define DTG2_VS_IN_DLY_MSB_REG			 0x7B
#define DTG2_VS_IN_DLY_LSB_REG			  0x7C
#define DTG2_PIXEL_CNT_MSB_REG			 0x7D
#define DTG2_PIXEL_CNT_LSB_REG			   0x7E
#define DTG2_LINE_CNT_MSB_REG			  0x7F
#define DTG2_LINE_CNT_LSB_REG				0x80
#define DTG2_CNTL_REG				0x82

/* used by System Control Register 0x02-0x03 */
#define CHIP_SOFTWARE_RESET				0x00
#define CHIP_SOFTWARE_OUT_OF_RESET               0x01
#define CHIP_LOW_FREQUENCY			0x10

/* used by Color Space Conversion Control 0x04-0x19 */
#define CSC_BYPASSED				0x02
#define CSC_PROTECTION_ON	0x01

/* used by Data Patch Control 0x1C */
#define DATA_30BIT_YCBCR_MODE		0x00
#define DATA_16BIT_RGB_MODE			  0x01
#define DATA_15BIT_RGB_MODE			  0x02
#define DATA_20BIT_YCBCR_MODE		0x03
#define DATA_10BIT_YCBCR_MODE		0x04

/* used by Display Timing Generator Control Part1 */
#define DTG_ON			 0x80
#define VIDEO_DATA_PASSED	    0x10
#define ATSC_MODE_1080P			  0x00
#define ATSC_MODE_1080I		 	   0x01
#define ATSC_MODE_720P		 	   0x02
#define GENERIC_MODE_FOR_HDTV	0x03
#define ATSC_MODE_480I			0x04
#define ATSC_MODE_480P			0x05
#define VESA_MASTER			0x06
#define VESA_SLAVE			0x07
#define SDTV_625_INTERLACED			0x08
#define GENERIC_MODE_FOR_SDTV  0x09

/* used by DTG2_CNTL_REG */
#define HS_IN_POSITIVE_POLARITY					0x01
#define VS_IN_POSITIVE_POLARITY					 0x02
#define FID_POLARITY		0x04
#define HS_OUT_POSITIVE_POLARITY			 0x08
#define VS_OUT_POSITIVE_POLARITY			 0x10

/**
 * set THS8200 DAC mode as 720p
 *
 * @return int
 * 0: success, other error.
 */
int ths8200_set_720p_mode(void);

/**
 * set THS8200 DAC mode as 1080i
 *
 * @return int
 * 0: success, other error.
 */
int ths8200_set_1080i_mode(void);

/**
 * set THS8200 DAC mode as 480p
 *
 * @return int
 * 0: success, other error.
 */
int ths8200_set_480p_mode(void);
#endif /* End of DAVINCI_THS8200 */
