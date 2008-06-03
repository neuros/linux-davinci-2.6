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
/* davinci-ths8200.c file */

/*Header files*/
#include <linux/module.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <asm/arch/davinci-ths8200.h>

/* #define DEBUG */
#ifdef DEBUG
#define DPRINTK(x...)  do { \
		printk(KERN_INFO "<%s>: ", __FUNCTION__); printk(x); \
	} while (0)

#define FN_IN  printk(KERN_INFO "<%s> start:\n", __FUNCTION__)

#else
#define DPRINTK(x...)
#define FN_IN

#endif

static int ths8200_attach_adapter(struct i2c_adapter *adapter);
static int ths8200_detach_client(struct i2c_client *client);
static int ths8200_detect_client(struct i2c_adapter *adapter,
						int address, int kind);
static inline int ths8200_write_value(u8 reg, u8 value);
static inline int ths8200_read_value(u8 reg);

static __init int ths8200_init(void);
static __exit void ths8200_exit(void);

static struct i2c_driver ths8200_driver = {
	.driver = {
		.name = "THS8200",
	},
	.id = 101,
	.attach_adapter = ths8200_attach_adapter,
	.detach_client = ths8200_detach_client,
};

/* I2C Addresses to scan */
static unsigned short normal_i2c[] = { 0x20, \
	I2C_CLIENT_END};

/* This makes all addr_data:s */
I2C_CLIENT_INSMOD;

static struct i2c_client *ths8200_client;


static inline int ths8200_read_value(u8 reg)
{
	return i2c_smbus_read_byte_data(ths8200_client, reg);
}

static inline int ths8200_write_value(u8 reg, u8 value)
{
	int ret;

	ret = i2c_smbus_write_byte_data(ths8200_client,
							reg, value);
	if (ret != 0)
		DPRINTK("Write Error Address = %x\n", reg);

	return ret;
}

static int ths8200_attach_adapter(struct i2c_adapter *adapter)
{
	int res;

	FN_IN;

	res = i2c_probe(adapter, &addr_data, &ths8200_detect_client);
	return res;
}

static int ths8200_detach_client(struct i2c_client *client)
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

static int ths8200_detect_client(struct i2c_adapter *adapter,
					int address, int kind)
{
	int err = 0;
	const char *client_name = "THS8200 Video DAC";

	FN_IN;

	if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_WORD_DATA |
				     I2C_FUNC_SMBUS_WRITE_BYTE)) {
		DPRINTK("Functinality check failed for %s \n",
				client_name);
		return err;
    }

	ths8200_client = kmalloc(sizeof(struct i2c_client),
							 GFP_KERNEL);
	if (ths8200_client == NULL) {
		err = -ENOMEM;
		DPRINTK("Couldn't allocate memory for %s\n",
				client_name);
		return err;
	}

	memset(ths8200_client, 0x00, sizeof(struct i2c_client));
	ths8200_client->addr = address;
	ths8200_client->adapter = adapter;
	ths8200_client->driver = &ths8200_driver;
	ths8200_client->flags = 0;
	strlcpy(ths8200_client->name, client_name, I2C_NAME_SIZE);

	err = i2c_attach_client(ths8200_client);
	if (err) {
		DPRINTK("Couldn't attach %s\n", client_name);
		kfree(ths8200_client);
		return err;
	}

	return 0;
}

int ths8200_set_480p_mode(void)
{
    /* place ths8200 in reset state */
	ths8200_write_value(CHIP_CTL_REG, CHIP_SOFTWARE_RESET);

    /* take ths8200 out of reset and in normal operation mode */
	ths8200_write_value(CHIP_CTL_REG, CHIP_SOFTWARE_OUT_OF_RESET |
						CHIP_LOW_FREQUENCY);

    /* place color space conversion control in reset state */
	ths8200_write_value(CSC_R11_REG, 0x00);
	ths8200_write_value(CSC_R21_REG, 0x00);
	ths8200_write_value(CSC_R31_REG, 0x00);
	ths8200_write_value(CSC_G11_REG, 0x00);
	ths8200_write_value(CSC_G21_REG, 0x00);
	ths8200_write_value(CSC_G31_REG, 0x00);
	ths8200_write_value(CSC_B11_REG, 0x00);
	ths8200_write_value(CSC_B21_REG, 0x00);
	ths8200_write_value(CSC_B31_REG, 0x00);
	ths8200_write_value(CSC_OFFS1_REG, 0x00);
	ths8200_write_value(CSC_OFFS12_REG, 0x00);
	ths8200_write_value(CSC_OFFS23_REG, 0x00);
	ths8200_write_value(CSC_OFFS3_REG,
						CSC_BYPASSED |
						CSC_PROTECTION_ON);

    /* set YCx20 External Sync */
	ths8200_write_value(DTG2_CNTL_REG, HS_IN_POSITIVE_POLARITY |
						VS_IN_POSITIVE_POLARITY |
						HS_OUT_POSITIVE_POLARITY |
						VS_OUT_POSITIVE_POLARITY);

	/* select the format for the input data manager */
	ths8200_write_value(DATA_CNTL_REG, DATA_20BIT_YCBCR_MODE);

	/* set the amplitude of the blanking level for the Y channel */
	ths8200_write_value(DTG1_Y_SYNC1_LSB_REG, 0xFF);

	/* set the amplitude of the negative sync and
		equalization/serration/broad pulses for the Y channel */
	ths8200_write_value(DTG1_Y_SYNC2_LSB_REG, 0x49);

	/* set the amplitude of the positive sync for the Y channel */
	ths8200_write_value(DTG1_Y_SYNC3_LSB_REG, 0xFF);

	/* set msb for sync1 sync2 and sync3 */
	ths8200_write_value(DTG1_Y_SYNC_MSB_REG, 0x13);

	/* set the amplitude of the blanking level for the Cb and Cr channels */
	ths8200_write_value(DTG1_CBCR_SYNC1_LSB_REG, 0xFF);

	/* set the amplitude of the negative sync and
		equalization/serration/broad pulses for the
		Cb and Cr channels */
	ths8200_write_value(DTG1_CBCR_SYNC2_LSB_REG, 0xFF);

	/* set the amplitude of the positive sync for the Cb and Cr channels */
	ths8200_write_value(DTG1_CBCR_SYNC3_LSB_REG, 0xFF);

	/* set msb for sync1 sync2 and sync3 */
	ths8200_write_value(DTG1_CBCR_SYNC_MSB_REG, 0x15);

    /* set negative hsync width (half of total width) */
	ths8200_write_value(DTG1_SPEC_A_REG, 0x28);

    /* set end of active video to start of negative sync */
	ths8200_write_value(DTG1_SPEC_B_REG, 0x10);

    /* set positive hsync width (half of total width) */
	ths8200_write_value(DTG1_SPEC_C_REG, 0x28);

	/* set LSBs of sync to broad pulse */
	ths8200_write_value(DTG1_SPEC_D_LSB_REG, 0x7A);

	/* set LSBs of sync to active video */
	ths8200_write_value(DTG1_SPEC_E_LSB_REG, 0x7A);

    /* set MSB bit of sync to active video width[6]/sync to broad pulse [7] */
	ths8200_write_value(DTG1_SPEC_DEH_MSB_REG, 0x00);

	/* set broad pulse duration for SDTV (NA) */
	ths8200_write_value(DTG1_SPEC_H_LSB_REG, 0x00);

    /* set end of active video to sync LSBs [7:0] */
	ths8200_write_value(DTG1_SPEC_K_LSB_REG, 0x10);

	/* set end of active video to sync MSBs [2:0] */
	ths8200_write_value(DTG1_SPEC_K_MSB_REG, 0x00);

	/* set MSB bit of total number of pixels per line */
	ths8200_write_value(DTG1_TOTAL_PIXELS_MSB_REG, 0x03);

	/* set LSB bit of total number of pixels per line */
	ths8200_write_value(DTG1_TOTAL_PIXELS_LSB_REG, 0x5A);

	/* set MSB and LSB bit of the starting line number
		for the DTG when Vsync input or V-bit is
		asserted(vertical display control) */
	ths8200_write_value(DTG1_FIELDFLIP_LINECNT_MSB_REG, 0x00);
	ths8200_write_value(DTG1_FIELDFLIP_LINECNT_LSB_REG, 0x01);

	/* set DTG on and set DTG operation mode to
		ATSC mode 720P(SMPTE296M progressive)*/
	ths8200_write_value(DTG1_MODE_REG, DTG_ON | ATSC_MODE_480P);

	/* set MSB bit of number of lines per frame and
		number of lines in field 1when in generic mode */
	ths8200_write_value(DTG1_FRAME_FILED_SIZE_MSB_REG, 0x27);

	/* set LSB bit of number of lines per frame when in generic mode */
	ths8200_write_value(DTG1_FRAMESIZE_LSB_REG, 0x0D);

	/* set LSB bit of number of lines in field 1 when in generic mode */
	ths8200_write_value(DTG1_FIELDSIZE_LSB_REG, 0xFF);

	/* set MSB and LSB bit of the number of pixels that the DTG
		startup is horizontally delayed with respect to HS input for
		dedicated timing modes or EAV input for embedded
		timing modes. */
	ths8200_write_value(DTG2_HS_IN_DLY_MSB_REG, 0x00);
	ths8200_write_value(DTG2_HS_IN_DLY_LSB_REG, 0x40);

	/* set MSB and LSB bit of the number of lines that the DTG
		startup is vertically delayed with respect to VS input for
		dedicated timing modes or the line counter value for
		embedded timing.*/
	ths8200_write_value(DTG2_VS_IN_DLY_MSB_REG, 0x00);
	ths8200_write_value(DTG2_VS_IN_DLY_LSB_REG, 0x00);

   /* place ths8200 in reset state */
	ths8200_write_value(CHIP_CTL_REG, CHIP_SOFTWARE_RESET);

	/* take ths8200 out of reset and in normal operation mode */
	ths8200_write_value(CHIP_CTL_REG, CHIP_SOFTWARE_OUT_OF_RESET |
						CHIP_LOW_FREQUENCY);

	printk(KERN_INFO "THS8200 set video mode as 480p\n");

	return 0;
}

int ths8200_set_720p_mode(void)
{
	FN_IN;

    /* place ths8200 in reset state */
	ths8200_write_value(CHIP_CTL_REG, CHIP_SOFTWARE_RESET);

    /* take ths8200 out of reset and in normal operation mode */
	ths8200_write_value(CHIP_CTL_REG, CHIP_SOFTWARE_OUT_OF_RESET);

    /* place color space conversion control in reset state */
	ths8200_write_value(CSC_R11_REG, 0x00);
	ths8200_write_value(CSC_R21_REG, 0x00);
	ths8200_write_value(CSC_R31_REG, 0x00);
	ths8200_write_value(CSC_G11_REG, 0x00);
	ths8200_write_value(CSC_G21_REG, 0x00);
	ths8200_write_value(CSC_G31_REG, 0x00);
	ths8200_write_value(CSC_B11_REG, 0x00);
	ths8200_write_value(CSC_B21_REG, 0x00);
	ths8200_write_value(CSC_B31_REG, 0x00);
	ths8200_write_value(CSC_OFFS1_REG, 0x00);
	ths8200_write_value(CSC_OFFS12_REG, 0x00);
	ths8200_write_value(CSC_OFFS23_REG, 0x00);
	ths8200_write_value(CSC_OFFS3_REG,
						CSC_BYPASSED |
						CSC_PROTECTION_ON);

    /* set YCx20 External Sync */
	ths8200_write_value(DTG2_CNTL_REG, HS_IN_POSITIVE_POLARITY |
						VS_IN_POSITIVE_POLARITY |
						HS_OUT_POSITIVE_POLARITY |
						VS_OUT_POSITIVE_POLARITY);

	/* select the format for the input data manager */
	ths8200_write_value(DATA_CNTL_REG, DATA_20BIT_YCBCR_MODE);

	/* set the amplitude of the blanking level for the Y channel */
	ths8200_write_value(DTG1_Y_SYNC1_LSB_REG, 0xFF);

	/* set the amplitude of the negative sync and
		equalization/serration/broad pulses for the Y channel */
	ths8200_write_value(DTG1_Y_SYNC2_LSB_REG, 0x49);

	/* set the amplitude of the positive sync for the Y channel */
	ths8200_write_value(DTG1_Y_SYNC3_LSB_REG, 0xB6);

	/* set msb for sync1 sync2 and sync3 */
	ths8200_write_value(DTG1_Y_SYNC_MSB_REG, 0x13);

	/* set the amplitude of the blanking level for the Cb and Cr channels */
	ths8200_write_value(DTG1_CBCR_SYNC1_LSB_REG, 0xFF);

	/* set the amplitude of the negative sync and
		equalization/serration/broad pulses for the
		Cb and Cr channels */
	ths8200_write_value(DTG1_CBCR_SYNC2_LSB_REG, 0xFF);

	/* set the amplitude of the positive sync for the Cb and Cr channels */
	ths8200_write_value(DTG1_CBCR_SYNC3_LSB_REG, 0xFF);

	/* set msb for sync1 sync2 and sync3 */
	ths8200_write_value(DTG1_CBCR_SYNC_MSB_REG, 0x15);

    /* set negative hsync width (half of total width) */
	ths8200_write_value(DTG1_SPEC_A_REG, 0x28);

    /* set end of active video to start of negative sync */
	ths8200_write_value(DTG1_SPEC_B_REG, 0x46);

    /* set positive hsync width (half of total width) */
	ths8200_write_value(DTG1_SPEC_C_REG, 0x28);

	/* set LSBs of sync to broad pulse */
	ths8200_write_value(DTG1_SPEC_D_LSB_REG, 0x2C);

	/* set LSBs of sync to active video */
	ths8200_write_value(DTG1_SPEC_E_LSB_REG, 0x2C);

    /* set MSB bit of sync to active video width[6]/sync to broad pulse [7] */
	ths8200_write_value(DTG1_SPEC_DEH_MSB_REG, 0xC0);

	/* set broad pulse duration for SDTV (NA) */
	ths8200_write_value(DTG1_SPEC_H_LSB_REG, 0x00);

    /* set end of active video to sync LSBs [7:0] */
	ths8200_write_value(DTG1_SPEC_K_LSB_REG, 0x46);

	/* set end of active video to sync MSBs [2:0] */
	ths8200_write_value(DTG1_SPEC_K_MSB_REG, 0x00);

	/* set MSB bit of total number of pixels per line */
	ths8200_write_value(DTG1_TOTAL_PIXELS_MSB_REG, 0x06);

	/* set LSB bit of total number of pixels per line */
	ths8200_write_value(DTG1_TOTAL_PIXELS_LSB_REG, 0x72);

	/* set MSB and LSB bit of the starting line number
		for the DTG when Vsync input or V-bit is
		asserted(vertical display control) */
	ths8200_write_value(DTG1_FIELDFLIP_LINECNT_MSB_REG, 0x00);
	ths8200_write_value(DTG1_FIELDFLIP_LINECNT_LSB_REG, 0x01);

	/* set DTG on and set DTG operation mode to
		ATSC mode 720P(SMPTE296M progressive)*/
	ths8200_write_value(DTG1_MODE_REG, DTG_ON | ATSC_MODE_720P);

	/* set MSB bit of number of lines per frame and
		number of lines in field 1when in generic mode */
	ths8200_write_value(DTG1_FRAME_FILED_SIZE_MSB_REG, 0x27);

	/* set LSB bit of number of lines per frame when in generic mode */
	ths8200_write_value(DTG1_FRAMESIZE_LSB_REG, 0xEE);

	/* set LSB bit of number of lines in field 1 when in generic mode */
	ths8200_write_value(DTG1_FIELDSIZE_LSB_REG, 0xFF);

	/* set MSB and LSB bit of the number of pixels that the DTG
		startup is horizontally delayed with respect to HS input for
		dedicated timing modes or EAV input for embedded
		timing modes. */
	ths8200_write_value(DTG2_HS_IN_DLY_MSB_REG, 0x00);
	ths8200_write_value(DTG2_HS_IN_DLY_LSB_REG, 0x60);

	/* set MSB and LSB bit of the number of lines that the DTG
		startup is vertically delayed with respect to VS input for
		dedicated timing modes or the line counter value for
		embedded timing.*/
	ths8200_write_value(DTG2_VS_IN_DLY_MSB_REG, 0x08);
	ths8200_write_value(DTG2_VS_IN_DLY_LSB_REG, 0x06);

    /* place ths8200 in reset state */
	ths8200_write_value(CHIP_CTL_REG, CHIP_SOFTWARE_RESET);

    /* take ths8200 out of reset and in normal operation mode */
	ths8200_write_value(CHIP_CTL_REG, CHIP_SOFTWARE_OUT_OF_RESET);

	printk(KERN_INFO "THS8200 set video mode as 720p\n");
	return 0;
}

int ths8200_set_1080i_mode(void)
{
	FN_IN;

    /* place ths8200 in reset state */
	ths8200_write_value(CHIP_CTL_REG,CHIP_SOFTWARE_RESET |
						CHIP_LOW_FREQUENCY);

    /* take ths8200 out of reset and in normal operation mode */
	ths8200_write_value(CHIP_CTL_REG, CHIP_SOFTWARE_OUT_OF_RESET |
						CHIP_LOW_FREQUENCY);

    /* place color space conversion control in reset state */
	ths8200_write_value(CSC_R11_REG, 0x00);
	ths8200_write_value(CSC_R21_REG, 0x00);
	ths8200_write_value(CSC_R31_REG, 0x00);
	ths8200_write_value(CSC_G11_REG, 0x00);
	ths8200_write_value(CSC_G21_REG, 0x00);
	ths8200_write_value(CSC_G31_REG, 0x00);
	ths8200_write_value(CSC_B11_REG, 0x00);
	ths8200_write_value(CSC_B21_REG, 0x00);
	ths8200_write_value(CSC_B31_REG, 0x00);
	ths8200_write_value(CSC_OFFS1_REG, 0x00);
	ths8200_write_value(CSC_OFFS12_REG, 0x00);
	ths8200_write_value(CSC_OFFS23_REG, 0x00);
	ths8200_write_value(CSC_OFFS3_REG,
						CSC_BYPASSED |
						CSC_PROTECTION_ON);

    /* Turn off THS8200 Test Modes */
	ths8200_write_value(TST_CNTL1_REG, 0x00);
	ths8200_write_value(TST_CNTL2_REG, 0x00);

    /* Turn CSM Off */
	ths8200_write_value(CSM_GY_CNTL_MULT_MSB_REG, 0x00);

    /* set YCx20 External Sync */
	ths8200_write_value(DTG2_CNTL_REG, HS_IN_POSITIVE_POLARITY |
						VS_IN_POSITIVE_POLARITY |
						HS_OUT_POSITIVE_POLARITY |
						VS_OUT_POSITIVE_POLARITY |
						FID_POLARITY);

	/* select the format for the input data manager */
	ths8200_write_value(DATA_CNTL_REG, DATA_20BIT_YCBCR_MODE);

	/* set the amplitude of the blanking level for the Y channel */
	ths8200_write_value(DTG1_Y_SYNC1_LSB_REG, 0xFF);

	/* set the amplitude of the negative sync and
		equalization/serration/broad pulses for the Y channel */
	ths8200_write_value(DTG1_Y_SYNC2_LSB_REG, 0x49);

	/* set the amplitude of the positive sync for the Y channel */
	ths8200_write_value(DTG1_Y_SYNC3_LSB_REG, 0xB6);

	/* set the amplitude of the blanking level for the Cb and Cr channels */
	ths8200_write_value(DTG1_CBCR_SYNC1_LSB_REG, 0xFF);

	/* set the amplitude of the negative sync and
		equalization/serration/broad pulses for the
		Cb and Cr channels */
	ths8200_write_value(DTG1_CBCR_SYNC2_LSB_REG, 0xFF);

	/* set the amplitude of the positive sync for the Cb and Cr channels */
	ths8200_write_value(DTG1_CBCR_SYNC3_LSB_REG, 0xFF);

	/* set msb for sync1 sync2 and sync3 */
	ths8200_write_value(DTG1_Y_SYNC_MSB_REG, 0x13);

	/* set msb for sync1 sync2 and sync3 */
	ths8200_write_value(DTG1_CBCR_SYNC_MSB_REG, 0x15);

	/* set negative hsync width (half of total width) */
	ths8200_write_value(DTG1_SPEC_A_REG, 0x2C);

    /* set end of active video to start of negative sync */
	ths8200_write_value(DTG1_SPEC_B_REG, 0x58);

    /* set positive hsync width (half of total width) */
	ths8200_write_value(DTG1_SPEC_C_REG, 0x2C);

	/* set LSBs of sync to broad pulse */
	ths8200_write_value(DTG1_SPEC_D_LSB_REG, 0x84);

	/* set distance from equalization pulse at center
		of line to active video*/
	ths8200_write_value(DTG1_SPEC_D1_REG, 0x00);

	/* set LSBs of sync to active video */
	ths8200_write_value(DTG1_SPEC_E_LSB_REG, 0xC0);

    /* set MSB bit of sync to active video width[6]/sync to broad pulse [7] */
	ths8200_write_value(DTG1_SPEC_DEH_MSB_REG, 0x00);

	/* set broad pulse duration for SDTV (NA) */
	ths8200_write_value(DTG1_SPEC_H_LSB_REG, 0x00);

    /* set end of active video to sync LSBs [7:0] */
	ths8200_write_value(DTG1_SPEC_K_LSB_REG, 0x58);

	/* set end of active video to sync MSBs [2:0] */
	ths8200_write_value(DTG1_SPEC_K_MSB_REG, 0x00);

	/* set LSB bit of half the line length */
	ths8200_write_value(DTG1_SPEC_G_LSB_REG, 0x58);

	/* set MSB bit of half the line length */
	ths8200_write_value(DTG1_SPEC_G_MSB_REG, 0x00);

	/* set MSB bit of total number of pixels per line */
	ths8200_write_value(DTG1_TOTAL_PIXELS_MSB_REG, 0x08);

	/* set LSB bit of total number of pixels per line */
	ths8200_write_value(DTG1_TOTAL_PIXELS_LSB_REG, 0x98);

	/* set MSB and LSB bit of the starting line number
		for the DTG when Vsync input or V-bit is
		asserted(vertical display control) */
	ths8200_write_value(DTG1_FIELDFLIP_LINECNT_MSB_REG, 0x00);
	ths8200_write_value(DTG1_FIELDFLIP_LINECNT_LSB_REG, 0x01);

	/* set DTG on and set DTG operation mode to
		ATSC mode 1080I(SMPTE274M Interlaced)*/
	ths8200_write_value(DTG1_MODE_REG, DTG_ON | ATSC_MODE_1080I);

	/* set MSB bit of number of lines per frame and
		number of lines in field 1when in generic mode */
	ths8200_write_value(DTG1_FRAME_FILED_SIZE_MSB_REG, 0x42);

	/* set LSB bit of number of lines per frame when in generic mode */
	ths8200_write_value(DTG1_FRAMESIZE_LSB_REG, 0x65);

	/* set LSB bit of number of lines in field 1 when in generic mode */
	ths8200_write_value(DTG1_FIELDSIZE_LSB_REG, 0x33);

	/* set LSB bit of vlength */
	ths8200_write_value(DTG2_HLENGTH_LSB_REG, 0x58);

	/* set MSB bit of pixel value that the HS_OUT signal is asserted on */
	ths8200_write_value(DTG2_HLENGTH_LSB_HDLY_MSB, 0x00);

	/* set LSB bit of pixel value that the HS_OUT signal is asserted on */
	ths8200_write_value(DTG2_HDLY_LSB, 0x00);

	/* set LSB bit of the duration of the VS_OUT output signal during
		progressive scan video modes or during the vertical blank interval
		of field 1 in interlaced video modes. */
	ths8200_write_value(DTG2_VLENGTH1_LSB, 0x05);

	/* set MSB bit of the VS_OUT output signal during progressive scan
		video modes or during the vertical blank interval of field 1 in
		interlaced video modes. */
	ths8200_write_value(DTG2_VLENGTH1_MSB_VDLY1_MSB, 0x00);

	/* set LSB bit of line number that VS_OUT signal is asserted on for
		progressive video modes or for field 1 of interlaced video modes. */
	ths8200_write_value(DTG2_VDLY1_LSB, 0x00);

	/* set LSB bit of the duration of the VS_OUT output signal during
		the vertical blank interval of field 2 in interlaced video modes.
		In progressive video modes, this register must be set to all 0. */
	ths8200_write_value(DTG2_VLENGTH2_LSB, 0x05);

	/* set the MSB bit of the duration of the VS_OUT output signal
		during the vertical blank interval of field2 in interlaced video
		modes.In progressive video modes, this register must be set to all 0.*/
	ths8200_write_value(DTG2_VLENGTH2_MSB_VDLY2_MSB, 0x77);

	/* set LSB bit of the line number that the VS_OUT signal is asserted on
		for field 2 of interlaced scan video modes.For progressive scan video
		modes, this register must be set to all 1. */
	ths8200_write_value(DTG2_VDLY2_LSB, 0x00);

	/* set MSB and LSB bit of the number of pixels that the DTG
		startup is horizontally delayed with respect to HS input for
		dedicated timing modes or EAV input for embedded
		timing modes. */
	ths8200_write_value(DTG2_HS_IN_DLY_MSB_REG, 0x00);
	ths8200_write_value(DTG2_HS_IN_DLY_LSB_REG, 0x44);

	/* set MSB and LSB bit of the number of lines that the DTG
		startup is vertically delayed with respect to VS input for
		dedicated timing modes or the line counter value for
		embedded timing.*/
	ths8200_write_value(DTG2_VS_IN_DLY_MSB_REG, 0x00);
	ths8200_write_value(DTG2_VS_IN_DLY_LSB_REG, 0x01);

	/* place ths8200 in reset state */
	ths8200_write_value(CHIP_CTL_REG, CHIP_SOFTWARE_RESET);

	/* take ths8200 out of reset and in normal operation mode */
	ths8200_write_value(CHIP_CTL_REG, CHIP_SOFTWARE_OUT_OF_RESET);

	printk(KERN_INFO "THS8200 set video mode as 1080i\n");

  return 0;
}

static __init int ths8200_init(void)
{
	FN_IN;

	if (i2c_add_driver(&ths8200_driver)) {
		DPRINTK("Driver registration failed, \
		      module not inserted.\n");
		return -ENODEV;
	}

	return 0;
}

static __exit void ths8200_exit(void)
{
	FN_IN;

	i2c_del_driver(&ths8200_driver);
}


module_init(ths8200_init);
module_exit(ths8200_exit);

EXPORT_SYMBOL(ths8200_set_720p_mode);
EXPORT_SYMBOL(ths8200_set_1080i_mode);
EXPORT_SYMBOL(ths8200_set_480p_mode);

MODULE_DESCRIPTION("THS8200 Video DAC Convert Driver");
MODULE_AUTHOR("Neuros Technology International LLC");
MODULE_LICENSE("GPL");
