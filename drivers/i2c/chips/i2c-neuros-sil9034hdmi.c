/*
 *  Copyright(C) 2006-2007 Neuros Technology International LLC. 
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
 * Silicon Image SIL9034 HDMI driver.
 *
 * REVISION:
 * 1) Initial creation. --------------------------------- 2008-06-13 JChen
 * 2) I2c control interface. ---------------------------- 2008-06-23 JChen
 *
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/spinlock.h>
#include <linux/interrupt.h>
#include <linux/kthread.h>
#include <linux/jiffies.h>
#include <linux/poll.h>
#include <linux/workqueue.h>
#include <linux/miscdevice.h>


#include <asm/uaccess.h>
#include <asm/io.h>
#include <asm/irq.h>

#include <asm/arch/hardware.h>
#include <linux/neuros_sil9034.h> /* Silicon Image 9034 register definition */

#if 1
    #define sil9034_dbg(fmt, arg...) \
        printk(KERN_INFO "%s:%d> " fmt, __func__, __LINE__ , ## arg)
#else
    #define sil9034_dbg(fmt, arg...)
#endif

#define I2C_RETRY_SLEEP 200 
#define I2C_RETRY_COUNT 3

#define MOD_DESC "Sil9034 HDMI Driver (c) 2007"
#define SIL9034_HDMI_NAME "sil9034hdmi"
/* Si9034 use 2 i2c to control the chip 0x39 & 0x3A */
#define SLAVE_SIZE 2
#define	TIMER_JIFFIES	(1 * HZ)

/* Timer only could cause the kernel busy, we push it into schedule */
#define SIL9034_TIMER 1
#define SIL9034_SCHED 1

/* YCBCR MODE */
#define YCBCR_480I 0
#define YCBCR_480P 0
#define YCBCR_720P 1

/* Silicon Image provide 2 slave address to control. */
static int slave_num = 0 ;
static int sil9034_attach_adapter(struct i2c_adapter * adapter);
static int sil9034_detach_client(struct i2c_client * client);
static struct input_dev *sil9034_hdmi_dev;

static unsigned short normal_i2c[] = {
	/* Jchen: should probe this address if real physical device is mount */
	TX_SLV0,
	TX_SLV1,
       	I2C_CLIENT_END
};

/* Macro need by addr_data */
I2C_CLIENT_INSMOD;

/* bus mapping struct */
typedef struct bus_mapping_sil9034
{
	u8 bus_reg ;
	u16 value ;
} bus_mapping_sil9034 ;

/* bus mapping for 480p YCbCr 4:2:2 Separate Sync Input*/
bus_mapping_sil9034 sil9034_480p_setting[] =
{
	{DE_CTRL_ADDR,0x41},
	{DE_DELAY_ADDR,0x04},
	{DE_TOP_ADDR,0x19},
	{DE_CNTH_ADDR,0x5},
	{DE_CNTL_ADDR,0x00},
	{DEL_H_ADDR,0x2},
	{DEL_L_ADDR,0xD0},
	{TX_VID_CTRL_ADDR,0x20},
	{TX_VID_MODE_ADDR,0x00}
};

/* bus mapping for 720p YCbCr 4:2:2 Separate Sync Input*/
bus_mapping_sil9034 sil9034_720p_setting[] =
{
	{DE_CTRL_ADDR,0x41},
	{DE_DELAY_ADDR,0x04},
	{DE_TOP_ADDR,0x19},
	{DE_CNTH_ADDR,0x5},
	{DE_CNTL_ADDR,0x00},
	{DEL_H_ADDR,0x2},
	{DEL_L_ADDR,0xD0},
	{TX_VID_CTRL_ADDR,0x20},
	{TX_VID_MODE_ADDR,0x00}
};
/* bus mapping for 1080i YCbCr 4:2:2 Separate Sync Input*/
bus_mapping_sil9034 sil9034_1080i_setting[] =
{
	{DE_CTRL_ADDR,0x41},
	{DE_DELAY_ADDR,0x04},
	{DE_TOP_ADDR,0x19},
	{DE_CNTH_ADDR,0x5},
	{DE_CNTL_ADDR,0x00},
	{DEL_H_ADDR,0x2},
	{DEL_L_ADDR,0xD0},
	{TX_VID_CTRL_ADDR,0x20},
	{TX_VID_MODE_ADDR,0x00}
};

/* i2c private data */
typedef struct davinci6446_sil9034
{
       	struct i2c_client* sil9034_client[SLAVE_SIZE] ;
#ifdef SIL9034_SCHED
	struct work_struct      work;
#endif
#ifdef SIL9034_TIMER
	struct timer_list timer ;
#endif
	spinlock_t              lock;
	/* pointer to different setting according to system */
	bus_mapping_sil9034 *sil9034_setting ;
	unsigned char sil9034_setting_num ;
} davinci6446_sil9034 ;

static davinci6446_sil9034 ds ;

static struct i2c_driver sil9034_driver = {
       	.driver	    = {
	       	.owner          = THIS_MODULE,
	       	.name           = "SIL9034 HDMI Driver",
	       	//.flags          = I2C_DF_NOTIFY,
		},
       	.id             = I2C_DRIVERID_SIL9034, /*define in i2c-id.h */
       	.attach_adapter = &sil9034_attach_adapter,
       	.detach_client  = &sil9034_detach_client,
};
static const char * pname = "SIL9034 HDMI Driver" ;

static int sil9034_fops_open(struct inode * inode, struct file * file)
{
	return 0 ;
}

static ssize_t sil9034_fops_read(struct file *filp, char __user *buff, size_t count, loff_t *ppos)
{
	return 0 ;
}

static int sil9034_fops_ioctl(struct inode *inode, struct file *file,
		     unsigned int cmd, unsigned long arg)
{
	switch (cmd)
	{
		default:
			return -EINVAL;
	}
	return 0 ;
}

static struct file_operations sil9034_fops = {
    .open    = sil9034_fops_open,
    .ioctl   = sil9034_fops_ioctl,
    .read    = sil9034_fops_read,
};

static struct miscdevice hdmi_dev = {
	MISC_DYNAMIC_MINOR,
	SIL9034_HDMI_NAME,
	&sil9034_fops
};


static int sil9034_detect_client(struct i2c_adapter * adapter, int address, int kind)
{
       	int ret = 0;
       	struct i2c_client* client = NULL;

	/* Jchen: i use a little trick here to make it register 2 i2c in 1 driver */
       	if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_WRITE_BYTE))
       	{
	       	sil9034_dbg("ERROR returned from i2c_check_functionality");
	       	ret = -1;
	       	goto out;
       	} 

	if (!(client = kmalloc(sizeof(*client), GFP_KERNEL)))
       	{
	       	sil9034_dbg("ERROR error returned from kmalloc");
	       	ret = -ENOMEM;
	       	goto out;
       	}

	memset(client, 0, sizeof(struct i2c_client));

	client->addr = address;
       	client->adapter = adapter;
       	client->driver = &sil9034_driver;
       	/* Jchen: it seem like ten bit probe doesn't support here, but we don't need
	 * it, since we know that the i2c address is fix in embedded.
	 * client->flags = I2C_M_IGNORE_NAK|I2C_M_TEN;
	 */
	client->flags = I2C_M_IGNORE_NAK;

	strcpy(client->name, "sil9034");

	/* JChen: i use force mode, no need this */
       	if ((ret = i2c_attach_client(client)) != 0)
       	{
	       	sil9034_dbg("%s Unable to attach client.\n", pname);
	       	kfree(client);
	       	ret = -1;
	       	goto out;
       	} 
	ds.sil9034_client[slave_num++] = client;

out:
       	return ret;
}

static int sil9034_attach_adapter(struct i2c_adapter * adapter)
{
	/* JChen : force to support 10-bit address, i2c-core need to
	 * fix to follow the linux i2c game.
	sil9034_detect_client(adapter,TX_SLV0,I2C_DRIVERID_SIL9034) ;
	sil9034_detect_client(adapter,TX_SLV1,I2C_DRIVERID_SIL9034) ;
	 */
       	return(i2c_probe(adapter, &addr_data, &sil9034_detect_client));
}

static int sil9034_detach_client(struct i2c_client * client)
{
       	int ret = 0;

	if ((ret = i2c_detach_client(client)) != 0)
       	{
	       	sil9034_dbg("%s Unable to detach client.\n", pname);
	       	goto out;
       	}

#if SIL9034_TIMER
       	del_timer_sync(&ds.timer);
#endif
#if SIL9034_SCHED
       	flush_scheduled_work();
#endif
       	kfree(client);

out:
       	return ret;
}

static int sil9034_write(davinci6446_sil9034 *priv,u8 slave,u8 reg, u16 value)
{
       	int retry = I2C_RETRY_COUNT;
       	int ret = -1;

	if(priv->sil9034_client[slave])
       	{
	       	while(retry--)
	       	{
		       	ret = i2c_smbus_write_byte_data(priv->sil9034_client[slave], reg, value);
		       	if (-1 != ret) break;
		       	printk("sil9034_write retry [%d]\n", retry);
		       	mdelay(I2C_RETRY_SLEEP);
	       	}
	       	return ret;
       	}
       	else
	       	return 0xff;
}
/* JChen: this should be call by others driver or 
 * create new char dev to ioctl the control.
EXPORT_SYMBOL(sil9034_write);
 */

static int sil9034_read(davinci6446_sil9034 *priv,u8 slave,u8 reg)
{
	int retry = I2C_RETRY_COUNT;
	int dat;

	if(priv->sil9034_client[slave])
       	{
		while(retry--)
		{
			dat = i2c_smbus_read_byte_data(priv->sil9034_client[slave], reg);
		       	if (-1 != dat) break;
			printk("(sil9034_read) retry [%d]\n", retry);
		       	mdelay(I2C_RETRY_SLEEP);
	       	}
	       	return dat;
       	}
       	else
	       	return 0xff;
}
//EXPORT_SYMBOL(sil9034_read);

//--------------------------  INIT / EXIT ---------------------------------------------------------
static int sil9034_chipInfo(davinci6446_sil9034 *priv)
{
	u8 device_info[3] = {255,255,255} ;

	device_info[1] = sil9034_read(priv,SLAVE0,DEV_IDL) ;
	device_info[0] = sil9034_read(priv,SLAVE0,DEV_IDH) ;
	device_info[2] = sil9034_read(priv,SLAVE0,DEV_REV) ;
	printk(KERN_INFO "Silicon Image Device Driver Id 0x%02X%02X. Rev %02i.\n",device_info[0],device_info[1],device_info[2]) ;

	return 0 ;
}

static int sil9034_audioInfoFrameSetting(davinci6446_sil9034 *priv)
{
	u8 aud_info_addr ;

	sil9034_dbg("----------%s----------\n",__FUNCTION__) ;
	/* set the audio info frame type according to EIA-CEA-861-B datasheet */
	aud_info_addr = AUD_IF_ADDR ;

	/* Audio type , pg 78 on EIA_CEA_861_B.pdf */
	sil9034_write(priv,SLAVE1,aud_info_addr++,0x04) ;

	/* Audio version */
	sil9034_write(priv,SLAVE1,aud_info_addr++,0x01) ;

	/* Audio info frame length */
	sil9034_write(priv,SLAVE1,aud_info_addr++,0x10) ;

	/* Audio info frame chsum */
	sil9034_write(priv,SLAVE1,aud_info_addr++,(0x04+0x01+0x10)) ;

	/* AUDIO INFO DATA BYTE , according to Sil FAE, 5 byte is enought.
	 * page 56
	 */
	/* CT3 | CT2 | CT1 | CT0 | Rsvd | CC2 | CC1 | CC0| */
	sil9034_write(priv,SLAVE1,aud_info_addr++,0x11) ;

	/* Reserved (shall be 0) | SF2 | SF1 | SF0 | SS1 | SS0 |*/
	/* I should provide ioctl to re-sampling the frequence according
	 * to audio header type in user space program.
	 */
	sil9034_write(priv,SLAVE1,aud_info_addr++,0x1D) ;

	/* format depend on data byte 1 */
	sil9034_write(priv,SLAVE1,aud_info_addr++,0x11) ;

	/* CA7 | CA6 | CA5 | CA4 | CA3 | CA2 | CA1 | CA0 | */
	sil9034_write(priv,SLAVE1,aud_info_addr++,0) ;

	/* DM_I NH | LSV3 | LSV2 | LSV1 | LSV0 | Reserved (shall be 0)| */
	sil9034_write(priv,SLAVE1,aud_info_addr++,0) ;

	return 0 ;
}

static int sil9034_cea861InfoFrameControl1(davinci6446_sil9034 *priv,u8 enable)
{
	u8 reg_value ;

	sil9034_dbg("----------%s----------\n",__FUNCTION__) ;
	/* enable the avi repeat transmission */
	reg_value = sil9034_read(priv,SLAVE1,INF_CTRL1) ;
	if(enable)
		sil9034_write(priv,SLAVE1,INF_CTRL1,(reg_value | (BIT_AVI_REPEAT |BIT_AUD_ENABLE |BIT_AUD_REPEAT))) ;
	else
		sil9034_write(priv,SLAVE1,INF_CTRL1,0) ;
	reg_value = sil9034_read(priv,SLAVE1,INF_CTRL1) ;
	sil9034_dbg("InfoFrame control#1 register 0x%x = 0x%x\n",INF_CTRL1,reg_value) ;


	return 0 ;
}

static int sil9034_cea861InfoFrameControl2(davinci6446_sil9034 *priv,u8 enable)
{
	u8 reg_value ;

	sil9034_dbg("----------%s----------\n",__FUNCTION__) ;
	/* Generic packet transmittion & repeat mode enable */
	reg_value = sil9034_read(priv,SLAVE1,INF_CTRL2) ;
	if(enable)
	{
		/* enable GCP_RPT , GCP_EN */
		sil9034_write(priv,SLAVE1,INF_CTRL2,(reg_value |(GCP_EN|GCP_RPT))) ;
	}
	else
	{
		sil9034_write(priv,SLAVE1,INF_CTRL2,reg_value & ~(GCP_EN|GCP_RPT)) ;
	}
	reg_value = sil9034_read(priv,SLAVE1,INF_CTRL2) ;
	sil9034_dbg("InfoFrame control#2 register 0x%x = 0x%x\n",INF_CTRL2,reg_value) ;

	return 0 ;
}


static int sil9034_cea861InfoFrameSetting(davinci6446_sil9034 *priv)
{
	u8 avi_info_addr ;

	sil9034_dbg("----------%s----------\n",__FUNCTION__) ;

	/* set the info frame type according to CEA-861 datasheet */
	avi_info_addr = AVI_IF_ADDR ;

	/* AVI type */
	sil9034_write(priv,SLAVE1,avi_info_addr++,0x82) ;

	/* AVI version */
	sil9034_write(priv,SLAVE1,avi_info_addr++,0x02) ;

	/* AVI length */
	sil9034_write(priv,SLAVE1,avi_info_addr++,0x0D) ;

	/* AVI CRC */
	sil9034_write(priv,SLAVE1,avi_info_addr++,(0x82 + 0x02 + 0x0D + 0x3D)) ;

	/* AVI DATA BYTE , according to Sil FAE, 3 byte is enought.
	 * page 102
	 */
	/* 0 | Y1 | Y0 | A0 | B1 | B0 | S1 | S0 */
	sil9034_write(priv,SLAVE1,avi_info_addr++,0x3D) ;

	/* C1 | C0 | M1 | M0 | R3 | R2 | R1 | R0 */
	sil9034_write(priv,SLAVE1,avi_info_addr++,0x68) ;

	/*  0 | 0 | 0 | 0 | 0 | 0 | SC1 | SC0 */
	sil9034_write(priv,SLAVE1,avi_info_addr++,0x3) ;

	return 0 ;
}

static int sil9034_ddcSetting(davinci6446_sil9034 *priv)
{
	sil9034_dbg("----------%s----------\n",__FUNCTION__) ;
	return 0 ;
}

static int sil9034_powerDown(davinci6446_sil9034 *priv,u8 enable)
{
	/* power down internal oscillator
	 * disable internal read of HDCP keys and KSV
	 * disable master DDC block
	 * page 4,50,113
	 */
	u8 reg_value ;

	sil9034_dbg("----------%s----------\n",__FUNCTION__) ;
	reg_value = sil9034_read(priv,SLAVE0,TX_SYS_CTRL1_ADDR) ;
	if(enable)
	{
		sil9034_write(priv,SLAVE1,DIAG_PD_ADDR,~(0x7)) ;
		sil9034_write(priv,SLAVE0,TX_SYS_CTRL1_ADDR,reg_value & ~0x1) ;
	}
	else
	{
		sil9034_write(priv,SLAVE1,DIAG_PD_ADDR,0x7) ;
		sil9034_write(priv,SLAVE0,TX_SYS_CTRL1_ADDR,(reg_value | 0x1)) ;
	}
	reg_value = sil9034_read(priv,SLAVE0,TX_SYS_CTRL1_ADDR) ;
	sil9034_dbg("System control register #1 0x%x = 0x%x\n",TX_SYS_CTRL1_ADDR,reg_value) ;

	reg_value = sil9034_read(priv,SLAVE1,DIAG_PD_ADDR) ;
	sil9034_dbg("Diagnostic power down register 0x%x = 0x%x\n",DIAG_PD_ADDR,reg_value) ;
	return 0 ;
}

static int sil9034_swReset(davinci6446_sil9034 *priv)
{
	/*
	 * audio fifo reset enable
	 * software reset enable
	 */
	sil9034_dbg("----------%s----------\n",__FUNCTION__) ;
	sil9034_write(priv,SLAVE0,TX_SWRST_ADDR,(BIT_TX_SW_RST|BIT_TX_FIFO_RST)) ;
	udelay(100) ;
	sil9034_write(priv,SLAVE0,TX_SWRST_ADDR,~(BIT_TX_SW_RST|BIT_TX_FIFO_RST)) ;
	return 0 ;
}

static int sil9034_generalControlPacket(davinci6446_sil9034 *priv,u8 enable)
{
	u8 reg_value ;
	/*
	 * mute the video & audio
	 */
	sil9034_dbg("----------%s----------\n",__FUNCTION__) ;
	reg_value = sil9034_read(priv,SLAVE1,GCP_BYTE1) ;
	if(enable)
	{
		/* set avmute flag */
		sil9034_write(priv,SLAVE1,GCP_BYTE1,(reg_value | SET_AVMUTE)) ;
	}
	else
	{
		/* clear avmute flag */
		sil9034_write(priv,SLAVE1,GCP_BYTE1,(reg_value | CLR_AVMUTE)) ;
	}
	reg_value = sil9034_read(priv,SLAVE1,GCP_BYTE1) ;
	sil9034_dbg("General control packet register 0x%x = 0x%x\n",GCP_BYTE1,reg_value) ;

	return 0 ;
}

static int sil9034_audioInputConfig(davinci6446_sil9034 *priv)
{
	u8 reg_value ;

	sil9034_dbg("----------%s----------\n",__FUNCTION__) ;
	/* Audio mode register */
	sil9034_write(priv,SLAVE1,AUD_MODE_ADDR,0xF9) ;
	reg_value = sil9034_read(priv,SLAVE1,AUD_MODE_ADDR) ;
	sil9034_dbg("Audio in mode register 0x%x = 0x%x\n",AUD_MODE_ADDR,reg_value) ;

	/* ACR audio frequency register: * MCLK=128 Fs */
	sil9034_write(priv,SLAVE1,FREQ_SVAL_ADDR,0) ;
	reg_value = sil9034_read(priv,SLAVE1,FREQ_SVAL_ADDR) ;
	sil9034_dbg("Audio frequency register 0x%x = 0x%x\n",FREQ_SVAL_ADDR,reg_value) ;
	
	return 0 ;
}

static int sil9034_hdmiVideoEmbSyncDec(davinci6446_sil9034 *priv)
{
	u8 reg_value ;

	sil9034_dbg("----------%s----------\n",__FUNCTION__) ;
	reg_value = sil9034_read(priv,SLAVE0,INTERLACE_ADJ_MODE) ;
	sil9034_write(priv,SLAVE0,INTERLACE_ADJ_MODE,(reg_value & ~(0x7))) ;
	reg_value = sil9034_read(priv,SLAVE0,INTERLACE_ADJ_MODE) ;
	sil9034_dbg("Interlace Adjustment register 0x%x = 0x%x\n",INTERLACE_ADJ_MODE,reg_value) ;
	return 0 ;
}

static int sil9034_hdmiOutputConfig(davinci6446_sil9034 *priv)
{
	u8 reg_value ;

	sil9034_dbg("----------%s----------\n",__FUNCTION__) ;
	/* HDMI control register , enable HDMI, disable DVI */
	reg_value = sil9034_read(priv,SLAVE1,HDMI_CTRL_ADDR) ;
	sil9034_write(priv,SLAVE1,HDMI_CTRL_ADDR,(reg_value | 0x1)) ;
	reg_value = sil9034_read(priv,SLAVE1,HDMI_CTRL_ADDR) ;
	sil9034_dbg("Hdmi control register 0x%x = 0x%x\n",HDMI_CTRL_ADDR,reg_value) ;

	return 0 ;
}

static int sil9034_hdmiTmdsConfig(davinci6446_sil9034 *priv)
{
	u8 reg_value ;

	sil9034_dbg("----------%s----------\n",__FUNCTION__) ;
	/* TMDS control register
	 * FPLL is 1.0*IDCK.
	 * Internal source termination enabled.
	 * Driver level shifter bias enabled.
	 * page 27
	 */
	reg_value = sil9034_read(priv,SLAVE0,TX_TMDS_CTRL_ADDR) ;
	sil9034_write(priv,SLAVE0,TX_TMDS_CTRL_ADDR,reg_value|0x5) ;
	reg_value = sil9034_read(priv,SLAVE0,TX_TMDS_CTRL_ADDR) ;
	sil9034_dbg("TMDS control register 0x%x = 0x%x\n",TX_TMDS_CTRL_ADDR,reg_value) ;
}

static int sil9034_hdmiHdcpConfig(davinci6446_sil9034 *priv,u8 enable)
{
	u8 reg_value ;

	sil9034_dbg("----------%s----------\n",__FUNCTION__) ;
	/* HDMI HDCP configuration */
	reg_value = sil9034_read(priv,SLAVE0,HDCP_CTRL_ADDR) ;
	if(enable)
		sil9034_write(priv,SLAVE0,HDCP_CTRL_ADDR,(reg_value | 0x1)) ;
	else
		sil9034_write(priv,SLAVE0,HDCP_CTRL_ADDR,(reg_value & ~(0x7F))) ;

	reg_value = sil9034_read(priv,SLAVE0,HDCP_CTRL_ADDR) ;
	sil9034_dbg("Hdmi hdcp register 0x%x = 0x%x\n",HDCP_CTRL_ADDR,reg_value) ;

	return 0 ;
}

static int sil9034_480i_VideoInputConfig(davinci6446_sil9034 *priv)
{
	/* Output Mode YcbCr 4:2:2 */
	u8 reg_value = 0 ;

	sil9034_dbg("----------%s----------\n",__FUNCTION__) ;

	/* 480i YCbCr 4:2:2 Mux YC Separate Sync Input */
	/* Video DE control register : DE_DLY 3:0=0x0
	 * 			       HS_POL 4 = 1
	 * 			       VS_POL 5 = 1
	 * 			       DE_GEN 6 = 1
	 * 			       Note: DM320 input diverse, so change below:
	 * 			       HS_POL 4 = 0
	 * 			       VS_POL 5 = 0
	 * 			       DE_GEN 6 = 1
	 */
	sil9034_write(priv,SLAVE0,DE_CTRL_ADDR,0x30) ;
	reg_value = sil9034_read(priv,SLAVE0,DE_CTRL_ADDR) ;
	sil9034_dbg("Video DE control register 0x%x = 0x%x\n",DE_CTRL_ADDR,reg_value) ;

	/* Video DE delay register */
	sil9034_write(priv,SLAVE0,DE_DELAY_ADDR,0x77) ;
	reg_value = sil9034_read(priv,SLAVE0,DE_DELAY_ADDR) ;
	sil9034_dbg("Video DE delay register 0x%x = 0x%x\n",DE_DELAY_ADDR,reg_value) ;

	/* Video DE top register, DE_TOP 6:0=0x12*/
	reg_value = sil9034_read(priv,SLAVE0,DE_TOP_ADDR) ;
	sil9034_write(priv,SLAVE0,DE_TOP_ADDR,reg_value|0x12) ;
	reg_value = sil9034_read(priv,SLAVE0,DE_TOP_ADDR) ;
	sil9034_dbg("Video DE top register 0x%x = 0x%x\n",DE_TOP_ADDR,reg_value) ;

	/* Video DE cnt high byte register, DE_CNT 3:0=0x2*/
	reg_value = sil9034_read(priv,SLAVE0,DE_CNTH_ADDR) ;
	sil9034_write(priv,SLAVE0,DE_CNTH_ADDR,(reg_value | 0x2)) ;
	reg_value = sil9034_read(priv,SLAVE0,DE_CNTH_ADDR) ;
	sil9034_dbg("Video DE cnt high register 0x%x = 0x%x\n",DE_CNTH_ADDR,reg_value) ;

	/* Video DE cnt low byte register, DE_CNT 7:0=0xD0*/
	sil9034_write(priv,SLAVE0,DE_CNTL_ADDR,0xD0) ;
	reg_value = sil9034_read(priv,SLAVE0,DE_CNTL_ADDR) ;
	sil9034_dbg("Video DE cnt low register 0x%x = 0x%x\n",DE_CNTL_ADDR,reg_value) ;

	/* Video DE line high byte register, DE_LIN 2:0=0x0*/
	reg_value = sil9034_read(priv,SLAVE0,DEL_H_ADDR) ;
	sil9034_write(priv,SLAVE0,DEL_H_ADDR,(reg_value & ~(0x7))) ;
	reg_value = sil9034_read(priv,SLAVE0,DEL_H_ADDR) ;
	sil9034_dbg("Video DE line high register 0x%x = 0x%x\n",DEL_H_ADDR,reg_value) ;

	/* Video DE line low byte register, DE_LIN 7:0=0xF0*/
	sil9034_write(priv,SLAVE0,DEL_L_ADDR,0xF0) ;
	reg_value = sil9034_read(priv,SLAVE0,DEL_L_ADDR) ;
	sil9034_dbg("Video DE line high register 0x%x = 0x%x\n",DEL_L_ADDR,reg_value) ;

	/* Video control register , ICLK = 00 EXTN = 1*/
	reg_value = sil9034_read(priv,SLAVE0,TX_VID_CTRL_ADDR) ;
	sil9034_write(priv,SLAVE0,TX_VID_CTRL_ADDR,(reg_value|0x20)) ;
	reg_value = sil9034_read(priv,SLAVE0,TX_VID_CTRL_ADDR) ;
	sil9034_dbg("Video control register 0x%x = 0x%x\n",TX_VID_CTRL_ADDR,reg_value) ;

	/* Video mode register , SYNCEXT=0 DEMUX=1 UPSMP=0 CSC=0 DITHER = 0*/
	sil9034_write(priv,SLAVE0,TX_VID_MODE_ADDR,0x2) ;
	reg_value = sil9034_read(priv,SLAVE0,TX_VID_MODE_ADDR) ;
	sil9034_dbg("Video mode register 0x%x = 0x%x\n",TX_VID_MODE_ADDR,reg_value) ;

	return 0 ;
}

static int sil9034_720p_VideoInputConfig(davinci6446_sil9034 *priv)
{
	/* Output Mode YcbCr 4:2:2 */
	u8 reg_value = 0 ;

	sil9034_dbg("----------%s----------\n",__FUNCTION__) ;

	reg_value = sil9034_read(priv,SLAVE0,DE_CTRL_ADDR) ;
	sil9034_write(priv,SLAVE0,DE_CTRL_ADDR,(reg_value|0x41)) ;
	reg_value = sil9034_read(priv,SLAVE0,DE_CTRL_ADDR) ;
	sil9034_dbg("Video DE control register 0x%x = 0x%x\n",DE_CTRL_ADDR,reg_value) ;

	/* Video DE delay register */
	sil9034_write(priv,SLAVE0,DE_DELAY_ADDR,0x04) ;
	reg_value = sil9034_read(priv,SLAVE0,DE_DELAY_ADDR) ;
	sil9034_dbg("Video DE delay register 0x%x = 0x%x\n",DE_DELAY_ADDR,reg_value) ;

	/* Video DE top register */
	reg_value = sil9034_read(priv,SLAVE0,DE_TOP_ADDR) ;
	sil9034_write(priv,SLAVE0,DE_TOP_ADDR,reg_value|0x19) ;
	reg_value = sil9034_read(priv,SLAVE0,DE_TOP_ADDR) ;
	sil9034_dbg("Video DE top register 0x%x = 0x%x\n",DE_TOP_ADDR,reg_value) ;

	/* Video DE cnt high byte register */
	reg_value = sil9034_read(priv,SLAVE0,DE_CNTH_ADDR) ;
	sil9034_write(priv,SLAVE0,DE_CNTH_ADDR,(reg_value | 0x5)) ;
	reg_value = sil9034_read(priv,SLAVE0,DE_CNTH_ADDR) ;
	sil9034_dbg("Video DE cnt high register 0x%x = 0x%x\n",DE_CNTH_ADDR,reg_value) ;

	/* Video DE cnt low byte register */
	sil9034_write(priv,SLAVE0,DE_CNTL_ADDR,0x00) ;
	reg_value = sil9034_read(priv,SLAVE0,DE_CNTL_ADDR) ;
	sil9034_dbg("Video DE cnt low register 0x%x = 0x%x\n",DE_CNTL_ADDR,reg_value) ;

	/* Video DE line high byte register */
	sil9034_write(priv,SLAVE0,DEL_H_ADDR,0x2) ;
	reg_value = sil9034_read(priv,SLAVE0,DEL_H_ADDR) ;
	sil9034_dbg("Video DE line high register 0x%x = 0x%x\n",DEL_H_ADDR,reg_value) ;

	/* Video DE line low byte register */
	sil9034_write(priv,SLAVE0,DEL_L_ADDR,0xD0) ;
	reg_value = sil9034_read(priv,SLAVE0,DEL_L_ADDR) ;
	sil9034_dbg("Video DE line high register 0x%x = 0x%x\n",DEL_L_ADDR,reg_value) ;

	/* Video control register , ICLK = 00 EXTN = 1*/
	reg_value = sil9034_read(priv,SLAVE0,TX_VID_CTRL_ADDR) ;
	sil9034_write(priv,SLAVE0,TX_VID_CTRL_ADDR,(reg_value|0x20)) ;
	reg_value = sil9034_read(priv,SLAVE0,TX_VID_CTRL_ADDR) ;
	sil9034_dbg("Video control register 0x%x = 0x%x\n",TX_VID_CTRL_ADDR,reg_value) ;

	/* Video mode register , SYNCEXT=0 DEMUX=0 UPSMP=0 CSC=0 DITHER = 0*/
	sil9034_write(priv,SLAVE0,TX_VID_MODE_ADDR,0x0) ;
	reg_value = sil9034_read(priv,SLAVE0,TX_VID_MODE_ADDR) ;
	sil9034_dbg("Video mode register 0x%x = 0x%x\n",TX_VID_MODE_ADDR,reg_value) ;

	return 0 ;
}
static int sil9034_Auto_VideoInputConfig(davinci6446_sil9034 *priv)
{
	u8 reg_value = 0 ;
	u8 count = 0 ;

	/* Auto setting by the bus_mapping_sil9034 struct */
	for(count=0 ;count<(priv->sil9034_setting_num) ;count++)
	{
		reg_value = sil9034_read(priv,SLAVE0,\
				priv->sil9034_setting[count].bus_reg) ;
		sil9034_write(priv,SLAVE0,priv->sil9034_setting[count].bus_reg,\
				(reg_value|priv->sil9034_setting[count].value)) ;
		reg_value = sil9034_read(priv,SLAVE0,\
				priv->sil9034_setting[count].bus_reg) ;
		sil9034_dbg("bus mapping regster 0x%x = 0x%x\n",\
				priv->sil9034_setting[count].bus_reg,reg_value) ;
	}

	return 0 ;
}

static int sil9034_480p_VideoInputConfig(davinci6446_sil9034 *priv)
{
	/* Output Mode YcbCr 4:2:2 */
	u8 reg_value = 0 ;

	sil9034_write(priv,SLAVE0,DE_CTRL_ADDR,0x70) ;
	reg_value = sil9034_read(priv,SLAVE0,DE_CTRL_ADDR) ;
	sil9034_dbg("Video DE control register 0x%x = 0x%x\n",DE_CTRL_ADDR,reg_value) ;

	/* Video DE delay register */
	sil9034_write(priv,SLAVE0,DE_DELAY_ADDR,0x7A) ;
	reg_value = sil9034_read(priv,SLAVE0,DE_DELAY_ADDR) ;
	sil9034_dbg("Video DE delay register 0x%x = 0x%x\n",DE_DELAY_ADDR,reg_value) ;

	/* Video DE top register */
	sil9034_write(priv,SLAVE0,DE_TOP_ADDR,0x24) ;
	reg_value = sil9034_read(priv,SLAVE0,DE_TOP_ADDR) ;
	sil9034_dbg("Video DE top register 0x%x = 0x%x\n",DE_TOP_ADDR,reg_value) ;

	/* Video DE cnt high byte register */
	reg_value = sil9034_read(priv,SLAVE0,DE_CNTH_ADDR) ;
	sil9034_write(priv,SLAVE0,DE_CNTH_ADDR,(reg_value|0x2)) ;
	reg_value = sil9034_read(priv,SLAVE0,DE_CNTH_ADDR) ;
	sil9034_dbg("Video DE cnt high register 0x%x = 0x%x\n",DE_CNTH_ADDR,reg_value) ;

	/* Video DE cnt low byte register */
	sil9034_write(priv,SLAVE0,DE_CNTL_ADDR,0xD0) ;
	reg_value = sil9034_read(priv,SLAVE0,DE_CNTL_ADDR) ;
	sil9034_dbg("Video DE cnt low register 0x%x = 0x%x\n",DE_CNTL_ADDR,reg_value) ;

	/* Video DE line high byte register */
	reg_value = sil9034_read(priv,SLAVE0,DEL_H_ADDR) ;
	sil9034_write(priv,SLAVE0,DEL_H_ADDR,(reg_value|0x1)) ;
	reg_value = sil9034_read(priv,SLAVE0,DEL_H_ADDR) ;
	sil9034_dbg("Video DE line high register 0x%x = 0x%x\n",DEL_H_ADDR,reg_value) ;

	/* Video DE line low byte register */
	sil9034_write(priv,SLAVE0,DEL_L_ADDR,0xE0) ;
	reg_value = sil9034_read(priv,SLAVE0,DEL_L_ADDR) ;
	sil9034_dbg("Video DE line high register 0x%x = 0x%x\n",DEL_L_ADDR,reg_value) ;

	/* Video control register , ICLK = 00 EXTN = 1*/
	sil9034_write(priv,SLAVE0,TX_VID_CTRL_ADDR,0x20) ;
	reg_value = sil9034_read(priv,SLAVE0,TX_VID_CTRL_ADDR) ;
	sil9034_dbg("Video control register 0x%x = 0x%x\n",TX_VID_CTRL_ADDR,reg_value) ;

	/* Video mode register , SYNCEXT=0 DEMUX=0 UPSMP=0 CSC=0 DITHER = 0*/
	sil9034_write(priv,SLAVE0,TX_VID_MODE_ADDR,0x0) ;
	reg_value = sil9034_read(priv,SLAVE0,TX_VID_MODE_ADDR) ;
	sil9034_dbg("Video mode register 0x%x = 0x%x\n",TX_VID_MODE_ADDR,reg_value) ;

	return 0 ;
}

int sil9034_dumpSystemStatus(davinci6446_sil9034 *priv)
{
	u8 reg_value ;

	sil9034_dbg("----------%s----------\n",__FUNCTION__) ;
	reg_value = sil9034_read(priv,SLAVE0,TX_STAT_ADDR) ;
	sil9034_dbg("System status register 0x%x = 0x%x\n",TX_STAT_ADDR,reg_value) ;
	return 0 ;
}

int sil9034_dumpDataCtrlStatus(davinci6446_sil9034 *priv)
{
	u8 reg_value ;

	sil9034_dbg("----------%s----------\n",__FUNCTION__) ;
	reg_value = sil9034_read(priv,SLAVE0,DCTL_ADDR) ;
	sil9034_dbg("Data control register 0x%x = 0x%x\n",DCTL_ADDR,reg_value) ;
	return 0 ;
}

int sil9034_unmaskInterruptStatus(davinci6446_sil9034 *priv)
{
	u8 reg_value = 0xFF ;

	sil9034_dbg("----------%s----------\n",__FUNCTION__) ;
	sil9034_write(priv,SLAVE0,HDMI_INT1_MASK,reg_value) ;
	sil9034_write(priv,SLAVE0,HDMI_INT2_MASK,reg_value) ;
	sil9034_write(priv,SLAVE0,HDMI_INT3_MASK,reg_value) ;

	return 0 ;
}

int sil9034_clearInterruptStatus(davinci6446_sil9034 *priv)
{

	sil9034_dbg("----------%s----------\n",__FUNCTION__) ;
	/*
	u8 reg_value ;

	reg_value = sil9034_read(priv,SLAVE0,INT_CNTRL_ADDR) ;
	sil9034_write(priv,SLAVE0,INT_CNTRL_ADDR,reg_value) ;
	reg_value = sil9034_read(priv,SLAVE0,INT_CNTRL_ADDR) ;
	sil9034_dbg("Interrupt control register 0x%x = 0x%x\n",INT_CNTRL_ADDR,reg_value) ;
	*/

	sil9034_write(priv,SLAVE0,INT_SOURCE1_ADDR,0xFF) ;
	sil9034_write(priv,SLAVE0,INT_SOURCE2_ADDR,0xFF) ;
	sil9034_write(priv,SLAVE0,INT_SOURCE3_ADDR,0xFF) ;

	return 0 ;
}

int sil9034_dumpInterruptSourceStatus(davinci6446_sil9034 *priv)
{
	u8 reg_value ;

	sil9034_dbg("----------%s----------\n",__FUNCTION__) ;
	reg_value = sil9034_read(priv,SLAVE0,INT_SOURCE1_ADDR) ;
	sil9034_dbg("Interrupt source 1 register 0x%x = 0x%x\n",INT_SOURCE1_ADDR,reg_value) ;
	reg_value = sil9034_read(priv,SLAVE0,INT_SOURCE2_ADDR) ;
	sil9034_dbg("Interrupt source 2 register 0x%x = 0x%x\n",INT_SOURCE2_ADDR,reg_value) ;
	reg_value = sil9034_read(priv,SLAVE0,INT_SOURCE3_ADDR) ;
	sil9034_dbg("Interrupt source 3 register 0x%x = 0x%x\n",INT_SOURCE3_ADDR,reg_value) ;
	/* Interrupt register will auto clean after read ?*/
	sil9034_clearInterruptStatus(priv) ;

	return 0 ;
}

int sil9034_dumpVideoConfigureStatus(davinci6446_sil9034 *priv)
{
	u8 reg_value ;

	sil9034_dbg("----------%s----------\n",__FUNCTION__) ;
	reg_value = sil9034_read(priv,SLAVE0,HRES_L_ADDR) ;
	sil9034_dbg("H resolution low register 0x%x = 0x%x\n",HRES_L_ADDR,reg_value) ;

	reg_value = sil9034_read(priv,SLAVE0,HRES_H_ADDR) ;
	sil9034_dbg("H resolution high register 0x%x = 0x%x\n",HRES_H_ADDR,reg_value) ;

	reg_value = sil9034_read(priv,SLAVE0,VRES_L_ADDR) ;
	sil9034_dbg("V resolution low register 0x%x = 0x%x\n",VRES_L_ADDR,reg_value) ;

	reg_value = sil9034_read(priv,SLAVE0,VRES_H_ADDR) ;
	sil9034_dbg("V resolution high register 0x%x = 0x%x\n",VRES_H_ADDR,reg_value) ;

	reg_value = sil9034_read(priv,SLAVE0,DEL_L_ADDR) ;
	sil9034_dbg("DE line low register 0x%x = 0x%x\n",DEL_L_ADDR,reg_value) ;

	reg_value = sil9034_read(priv,SLAVE0,DEL_H_ADDR) ;
	sil9034_dbg("DE line high register 0x%x = 0x%x\n",DEL_H_ADDR,reg_value) ;

	reg_value = sil9034_read(priv,SLAVE0,POL_DETECT_ADDR) ;
	sil9034_dbg("Video polarity detect register 0x%x = 0x%x\n",POL_DETECT_ADDR,reg_value) ;

	reg_value = sil9034_read(priv,SLAVE0,HLENGTH1_ADDR) ;
	sil9034_dbg("Video HSYNC length1 register 0x%x = 0x%x\n",HLENGTH1_ADDR,reg_value) ;

	reg_value = sil9034_read(priv,SLAVE0,HLENGTH2_ADDR) ;
	sil9034_dbg("Video HSYNC length2 register 0x%x = 0x%x\n",HLENGTH2_ADDR,reg_value) ;

	reg_value = sil9034_read(priv,SLAVE0,VBIT_TO_VSYNC_ADDR) ;
	sil9034_dbg("Video Vbit to VSync register 0x%x = 0x%x\n",VBIT_TO_VSYNC_ADDR,reg_value) ;

	reg_value = sil9034_read(priv,SLAVE0,VLENGTH_ADDR) ;
	sil9034_dbg("Video VSYNC length register 0x%x = 0x%x\n",VLENGTH_ADDR,reg_value) ;

	reg_value = sil9034_read(priv,SLAVE0,TX_TMDS_CCTRL_ADDR) ;
	sil9034_dbg("TMDS C control register 0x%x = 0x%x\n",TX_TMDS_CCTRL_ADDR,reg_value) ;

	reg_value = sil9034_read(priv,SLAVE0,TX_TMDS_CTRL_ADDR) ;
	sil9034_dbg("TMDS control register 0x%x = 0x%x\n",TX_TMDS_CTRL_ADDR,reg_value) ;

	reg_value = sil9034_read(priv,SLAVE0,TX_TMDS_CTRL2_ADDR) ;
	sil9034_dbg("TMDS control #2 register 0x%x = 0x%x\n",TX_TMDS_CTRL2_ADDR,reg_value) ;

	reg_value = sil9034_read(priv,SLAVE0,TX_TMDS_CTRL3_ADDR) ;
	sil9034_dbg("TMDS control #3 register 0x%x = 0x%x\n",TX_TMDS_CTRL3_ADDR,reg_value) ;

	reg_value = sil9034_read(priv,SLAVE0,TX_TMDS_CTRL4_ADDR) ;
	sil9034_dbg("TMDS control #4 register 0x%x = 0x%x\n",TX_TMDS_CTRL4_ADDR,reg_value) ;

	reg_value = sil9034_read(priv,SLAVE0,TX_VID_ACEN_ADDR) ;
	sil9034_dbg("Video action enable register 0x%x = 0x%x\n",TX_VID_ACEN_ADDR,reg_value) ;

	return 0 ;
}

int sil9034_dumpInterruptStateStatus(davinci6446_sil9034 *priv)
{
	u8 reg_value ;

	sil9034_dbg("----------%s----------\n",__FUNCTION__) ;
	reg_value = sil9034_read(priv,SLAVE0,INT_STATE_ADDR) ;
	sil9034_dbg("Interrupt state register 0x%x = 0x%x\n",INT_STATE_ADDR,reg_value) ;
	if(reg_value & INT_ENABLE)
		sil9034_dumpInterruptSourceStatus(priv) ;
	else
		sil9034_dbg("No unmask interrupt\n") ;
	return 0 ;
}

/* HDCP key exchange for hdmi */
static void sil9034_timer(unsigned long data)
{
#if SIL9034_SCHED
	int status;
#endif
	davinci6446_sil9034 *priv = (void *)data ;

	if(priv)
	{
#if SIL9034_SCHED
		status = schedule_work(&priv->work);
		if(!status)
			printk(KERN_ERR "scheduling work error\n") ;
#else
		mod_timer(&ds.timer, jiffies + TIMER_JIFFIES);
#endif
	}
	return ;
}
#if SIL9034_SCHED
static void sil9034_sched(void *data)
{
	/* This is important, get out of the interrupt context switch trouble */
	davinci6446_sil9034 *priv = container_of(data, davinci6446_sil9034, work);
	
	//printk(KERN_ERR "%s, %d\n",__FUNCTION__,__LINE__) ;
	
	//sil9034_dumpSystemStatus(priv) ;
	//sil9034_dumpDataCtrlStatus(priv) ;
	//sil9034_dumpInterruptStateStatus(priv) ;
	sil9034_dumpVideoConfigureStatus(priv) ;
#if SIL9034_TIMER
	mod_timer(&ds.timer, jiffies + TIMER_JIFFIES);
#endif
	return ;

}
#endif

static int __init sil9034_init(void)
{
	int status = 0 ;
	
	printk(KERN_INFO "\t" MOD_DESC "\n");

	if ((status = i2c_add_driver(&sil9034_driver)) < 0)
	{
	    printk(KERN_INFO "%s Couldn't register SIL9034 I2C driver.\n", pname);
	    goto out;
	} 
	/* Init the priv data
	ds.sil9034_setting = &sil9034_720p_setting ;
	ds.sil9034_setting_num = \
			 sizeof(sil9034_720p_setting)/sizeof(bus_mapping_sil9034) ;
			 */

	
	/* read chip id & revision */
	sil9034_chipInfo(&ds) ;

	/* power down occilator */
	sil9034_powerDown(&ds,ENABLE) ;
	
	/* Tune the video input table according to DM320 hardware spec */
       	sil9034_720p_VideoInputConfig(&ds) ;

	/* Tune the audio input table according to DM320 hardware spec */
       	sil9034_audioInputConfig(&ds) ;

	/* software reset */
       	sil9034_swReset(&ds) ;

	/* power up occilator */
       	sil9034_powerDown(&ds,DISABLE) ;

	/* unmask the interrupt status */
       	sil9034_unmaskInterruptStatus(&ds) ;

	/* Hdmi output setting */
       	sil9034_hdmiOutputConfig(&ds) ;

	/* TMDS control register */
       	sil9034_hdmiTmdsConfig(&ds) ;

	/* ddc master config */
       	sil9034_ddcSetting(&ds) ;

	/* HDCP control handshaking */
       	sil9034_hdmiHdcpConfig(&ds,DISABLE) ;

	/* enable the avi repeat transmission */
       	sil9034_cea861InfoFrameControl1(&ds,ENABLE) ;
	//sil9034_cea861InfoFrameControl2(&ds,ENABLE) ;

	/* CEA-861 Info Frame control setting */
       	sil9034_cea861InfoFrameSetting(&ds) ;

	/* Audio Info Frame control setting */
       	sil9034_audioInfoFrameSetting(&ds) ;

#if SIL9034_SCHED
	INIT_WORK(&ds.work, sil9034_sched);
#endif

#if SIL9034_TIMER
	init_timer(&ds.timer);
	ds.timer.data = (unsigned long)&ds;
	ds.timer.function = sil9034_timer;
	ds.timer.expires = jiffies + TIMER_JIFFIES;
	add_timer(&ds.timer);
#endif

	/* register a device node for user to setting the HDMI param */
	status = misc_register(&hdmi_dev);
	if (status)
	{
		printk(KERN_ERR "%s-Couldn't register device\n",pname);
		goto out ;
	}
out:
	return status;
}

static void __exit sil9034_exit(void)
{
	sil9034_dbg("----------%s----------\n",__FUNCTION__) ;
	i2c_del_driver(&sil9034_driver);
	misc_deregister(&hdmi_dev);
}

MODULE_AUTHOR("Neuros");
MODULE_DESCRIPTION(MOD_DESC);
MODULE_LICENSE("Neuros Technology LLC");

module_init(sil9034_init);
module_exit(sil9034_exit);
