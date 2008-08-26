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
 * 3) system ioctl support. ----------------------------- 2008-06-30 JChen
 * 3) 1080i support. ------------------------------------ 2008-06-30 JChen
 * 4) hdcp support.  ------------------------------------ 2008-07-09 JChen
 * 5) fix hdcp auth. ------------------------------------ 2008-08-11 JChen
 * 6) support line auth. -------------------------------- 2008-08-11 JChen
 * 7) fix var mode into struct. ------------------------- 2008-08-12 JChen
 * 8) hdmi audio support -------------------------------- 2008-08-20 Jchen
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

/* enable DEBUG to 1 if you need more information */
#define DEBUG 0

#if DEBUG
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
#define	TIMER_JIFFIES	(3 * HZ)

/* Timer only could cause the kernel busy, we push it into schedule */
#define SIL9034_TIMER 1
#define SIL9034_SCHED 1

/* Silicon Image provide 2 slave address to control. */
static int slave_num = 0 ;
static int sil9034_attach_adapter(struct i2c_adapter * adapter);
static int sil9034_detach_client(struct i2c_client * client);

static unsigned short normal_i2c[] = {
	/* Jchen: should probe this address if real physical device is mount */
	TX_SLV0,
	TX_SLV1,
       	I2C_CLIENT_END
};

/* Macro need by addr_data */
I2C_CLIENT_INSMOD;

/* video mapping struct */
typedef struct sil9034_video_mapping
{
	u8 video_reg ;
	u16 value ;
} sil9034_video_mapping ;

/* video mapping for 480p YCbCr 4:2:2 Separate Sync Input*/
sil9034_video_mapping sil9034_480p_setting[] =
{
	{DE_CTRL_ADDR,(DE_GEN_ENABLE|0x70)},
	{DE_DELAY_ADDR,0x7A},
	{DE_TOP_ADDR,0x24},
	{DE_CNTH_ADDR,0x2},
	{DE_CNTL_ADDR,0xD0},
	{DEL_H_ADDR,0x1},
	{DEL_L_ADDR,0xE0},
	{TX_VID_CTRL_ADDR,CSCSEL_BT709|SET_EXTN_12BIT},
	{TX_VID_MODE_ADDR,UPSMP_ENABLE|CSC_ENABLE},
	{0,0}
};

/* video mapping for 720p YCbCr 4:2:2 Separate Sync Input*/
sil9034_video_mapping sil9034_720p_setting[] =
{
	{DE_CTRL_ADDR,(DE_GEN_ENABLE|0x1)},
	{DE_DELAY_ADDR,0x04},
	{DE_TOP_ADDR,0x19},
	{DE_CNTH_ADDR,0x5},
	{DE_CNTL_ADDR,0x00},
	{DEL_H_ADDR,0x2},
	{DEL_L_ADDR,0xD0},
	{TX_VID_CTRL_ADDR,CSCSEL_BT709|SET_EXTN_12BIT},
	{TX_VID_MODE_ADDR,UPSMP_ENABLE|CSC_ENABLE},
	{0,0}
};
/* video mapping for 1080i YCbCr 4:2:2 Separate Sync Input*/
sil9034_video_mapping sil9034_1080i_setting[] =
{
	{DE_CTRL_ADDR,(DE_GEN_ENABLE|0x0)},
	{DE_DELAY_ADDR,0xC0},
	{DE_TOP_ADDR,0x14},
	{DE_CNTH_ADDR,0x07},
	{DE_CNTL_ADDR,0x80},
	{DEL_H_ADDR,0x2},
	{DEL_L_ADDR,0x1C},
	{TX_VID_CTRL_ADDR,CSCSEL_BT709|SET_EXTN_12BIT},
	{TX_VID_MODE_ADDR,UPSMP_ENABLE|CSC_ENABLE},
	{0,0}
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
	sil9034_video_mapping *sil9034_setting ;
	unsigned char work_flag ;
	unsigned char auth_state ;
	u8 an_ksv_data[8] ;
	char r0rx[2] ;
	char r0tx[2] ;
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
	int status ;
#if SIL9034_SCHED
	if(ds.work_flag != DO_NOTHING)
		flush_scheduled_work();
#endif
	switch (cmd)
	{
		case SWITCH_480P:
			sil9034_dbg("480p mode.\n") ;
			ds.work_flag = SWITCH_480P ;
			break ;
		case SWITCH_720P:
			sil9034_dbg("720p mode.\n") ;
			ds.work_flag = SWITCH_720P ;
			break ;
		case SWITCH_1080I:
			sil9034_dbg("1080I mode.\n") ;
			ds.work_flag = SWITCH_1080I ;
			break ;
		case HDCP_EXCHANGE:
			sil9034_dbg("hdcp key exchange mode.\n") ;
			if(arg==ENABLE)
				ds.work_flag = HDCP_ENABLE ;
			else
				ds.work_flag = HDCP_DISABLE ;
			break ;
		case SHOW_REGISTER:
#if SIL9034_SCHED
			ds.work_flag = SHOW_REGISTER ;
#endif
			break ;
		case HDCP_RI_STATUS:
			printk(KERN_INFO "HDCP_RI_STATUS\n") ;
			ds.work_flag = HDCP_RI_STATUS ;
			break ;
		default:
			ds.work_flag = DO_NOTHING ;
			return -EINVAL;
	}

	/* task assign */
	if(ds.work_flag != DO_NOTHING)
	{
		status = schedule_work(&ds.work);
		if(!status)
			printk(KERN_ERR "scheduling work error\n") ;
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

static char *sil9034_ddc_write(davinci6446_sil9034 *priv,u8 *value,u8 reg, u8 length)
{
	u8 count = 0 ;

	while(sil9034_read(priv,SLAVE0,DDC_STATUS_ADDR)&BIT_MDDC_ST_IN_PROGR)
		mdelay(10) ;

	sil9034_write(priv,SLAVE0,DDC_ADDR,HDCP_RX_SLAVE) ;
	sil9034_write(priv,SLAVE0,DDC_OFFSET_ADDR,reg) ;
	sil9034_write(priv,SLAVE0,DDC_CNT1_ADDR,length) ;
	sil9034_write(priv,SLAVE0,DDC_CNT2_ADDR,0) ;
	sil9034_write(priv,SLAVE0,DDC_CMD_ADDR,MASTER_CMD_CLEAR_FIFO) ;

	for(count=0 ;count < length ; count++)
	{
		sil9034_write(priv,SLAVE0,DDC_DATA_ADDR,value[count]) ;
	}
	sil9034_write(priv,SLAVE0,DDC_CMD_ADDR,MASTER_CMD_SEQ_WR) ;

	while(sil9034_read(priv,SLAVE0,DDC_STATUS_ADDR)&BIT_MDDC_ST_IN_PROGR)
		mdelay(10) ;

	sil9034_dbg("DDC WRITE FIFO is %d\n",sil9034_read(priv,SLAVE0,DDC_FIFOCNT_ADDR)) ;
	sil9034_write(priv,SLAVE0,DDC_CMD_ADDR,MASTER_CMD_ABORT) ;
	sil9034_write(priv,SLAVE0,DDC_CMD_ADDR,MASTER_CMD_CLOCK) ;
	sil9034_write(priv,SLAVE0,DDC_MAN_ADDR,0) ;
	return NULL ;
}

static char *sil9034_ddc_read(davinci6446_sil9034 *priv,u8 *value,u8 reg, u8 length)
{
	u8 count = 0 ;

	sil9034_write(priv,SLAVE0,DDC_ADDR,HDCP_RX_SLAVE) ;
	sil9034_write(priv,SLAVE0,DDC_OFFSET_ADDR,reg) ;
	sil9034_write(priv,SLAVE0,DDC_CNT1_ADDR,length) ;
	sil9034_write(priv,SLAVE0,DDC_CNT2_ADDR,0) ;
	sil9034_write(priv,SLAVE0,DDC_CMD_ADDR,MASTER_CMD_CLEAR_FIFO) ;
	sil9034_write(priv,SLAVE0,DDC_CMD_ADDR,MASTER_CMD_SEQ_RD) ;

	while(sil9034_read(priv,SLAVE0,DDC_STATUS_ADDR)&BIT_MDDC_ST_IN_PROGR)
		mdelay(10) ;
	sil9034_dbg("DDC READ FIFO is %d\n",sil9034_read(priv,SLAVE0,DDC_FIFOCNT_ADDR)) ;
	for(count=0 ;count < length ; count++)
	{
		value[count] = sil9034_read(priv,SLAVE0,DDC_DATA_ADDR) ;
	}

	while(sil9034_read(priv,SLAVE0,DDC_STATUS_ADDR)&BIT_MDDC_ST_IN_PROGR)
		mdelay(10) ;

	sil9034_write(priv,SLAVE0,DDC_CMD_ADDR,MASTER_CMD_ABORT) ;
	sil9034_write(priv,SLAVE0,DDC_CMD_ADDR,MASTER_CMD_CLOCK) ;
	sil9034_write(priv,SLAVE0,DDC_MAN_ADDR,0) ;
	return NULL ;
}
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
		sil9034_write(priv,SLAVE1,DIAG_PD_ADDR,PDIDCK_NORMAL|PDOSC_NORMAL|PDTOT_NORMAL) ;
		sil9034_write(priv,SLAVE0,TX_SYS_CTRL1_ADDR,reg_value & ~(SET_PD)) ;
	}
	else
	{
		sil9034_write(priv,SLAVE1,DIAG_PD_ADDR,~(PDIDCK_NORMAL|PDOSC_NORMAL|PDTOT_NORMAL)) ;
		sil9034_write(priv,SLAVE0,TX_SYS_CTRL1_ADDR,(reg_value | SET_PD)) ;
	}
	reg_value = sil9034_read(priv,SLAVE0,TX_SYS_CTRL1_ADDR) ;
	sil9034_dbg("System control register #1 0x%x = 0x%x\n",TX_SYS_CTRL1_ADDR,reg_value) ;

	reg_value = sil9034_read(priv,SLAVE1,DIAG_PD_ADDR) ;
	sil9034_dbg("Diagnostic power down register 0x%x = 0x%x\n",DIAG_PD_ADDR,reg_value) ;
	return 0 ;
}

static int sil9034_hdmiTmdsConfig(davinci6446_sil9034 *priv)
{
	u8 reg_value ;

	/* TMDS control register
	 * FPLL is 1.0*IDCK.
	 * Internal source termination enabled.
	 * Driver level shifter bias enabled.
	 * page 27
	 */
	reg_value = sil9034_read(priv,SLAVE0,TX_TMDS_CTRL_ADDR) ;
	sil9034_write(priv,SLAVE0,TX_TMDS_CTRL_ADDR,reg_value|(LVBIAS_ENABLE|STERM_ENABLE)) ;
	reg_value = sil9034_read(priv,SLAVE0,TX_TMDS_CTRL_ADDR) ;
	sil9034_dbg("TMDS control register 0x%x = 0x%x\n",TX_TMDS_CTRL_ADDR,reg_value) ;
}
static int sil9034_audioInputConfig(davinci6446_sil9034 *priv)
{
	u8 reg_value ;

	sil9034_dbg("----------%s----------\n",__FUNCTION__) ;
	/* Audio mode register */
	sil9034_write(priv,SLAVE1,AUD_MODE_ADDR,SD0_ENABLE|AUD_ENABLE) ;
	reg_value = sil9034_read(priv,SLAVE1,AUD_MODE_ADDR) ;
	sil9034_dbg("Audio in mode register 0x%x = 0x%x\n",AUD_MODE_ADDR,reg_value) ;

	/* ACR ctrl */
	sil9034_write(priv,SLAVE1,ACR_CTRL_ADDR,NCTSPKT_ENABLE) ;

	sil9034_write(priv,SLAVE1,I2S_CHST4_ADDR,0x2) ;
	sil9034_write(priv,SLAVE1,I2S_CHST5_ADDR,0x2) ;

	/* ACR audio frequency register: * MCLK=512 Fs */
	sil9034_write(priv,SLAVE1,FREQ_SVAL_ADDR,0x3) ;
	reg_value = sil9034_read(priv,SLAVE1,FREQ_SVAL_ADDR) ;
	sil9034_dbg("Audio frequency register 0x%x = 0x%x\n",FREQ_SVAL_ADDR,reg_value) ;

	/* ACR N software value */
	sil9034_write(priv,SLAVE1,N_SVAL1_ADDR,0x80) ;
	sil9034_write(priv,SLAVE1,N_SVAL2_ADDR,0x2D) ;
	sil9034_write(priv,SLAVE1,N_SVAL3_ADDR,0) ;

	/* ACR CTS hardware value */
	sil9034_write(priv,SLAVE1,N_SVAL1_ADDR,0x51) ;
	sil9034_write(priv,SLAVE1,N_SVAL2_ADDR,0x25) ;
	sil9034_write(priv,SLAVE1,N_SVAL3_ADDR,2) ;

	/* I2S register */
	reg_value = sil9034_read(priv,SLAVE1,I2S_IN_CTRL_ADDR) ;
	sil9034_write(priv,SLAVE1,I2S_IN_CTRL_ADDR,reg_value|(HBRA_ON|VBIT_ON)) ;

	return 0 ;
}

static int sil9034_videoInputConfig(davinci6446_sil9034 *priv)
{
	u8 count = 0 ;

	/* Auto setting by the sil9034_video_mapping struct */
	while(priv->sil9034_setting[count++].video_reg != 0)
	{
		sil9034_dbg("setting count %d 0x%x-0x%x\n",count,\
				priv->sil9034_setting[count].video_reg,\
				priv->sil9034_setting[count].value) ;

		sil9034_write(priv,SLAVE0,priv->sil9034_setting[count].video_reg,\
				priv->sil9034_setting[count].value) ;
	}

	return 0 ;
}

static int sil9034_swReset(davinci6446_sil9034 *priv)
{
	/* use to temporary save inf_ctrl */
	u8 temp1 ;
	u8 temp2 ;

	sil9034_dbg("----------%s----------\n",__FUNCTION__) ;

	temp1 = sil9034_read(priv,SLAVE1,INF_CTRL1) ;
	temp2 = sil9034_read(priv,SLAVE1,INF_CTRL2) ;
	/*
	 * audio fifo reset enable
	 * software reset enable
	 */
	while(!sil9034_read(priv,SLAVE0,TX_STAT_ADDR)&P_STABLE)
		mdelay(10) ;
	sil9034_write(priv,SLAVE0,TX_SWRST_ADDR,(BIT_TX_SW_RST|BIT_TX_FIFO_RST)) ;
	mdelay(10) ;
	sil9034_write(priv,SLAVE0,TX_SWRST_ADDR,0) ;
	mdelay(64) ; // allow TCLK (sent to Rx across the HDMS link) to stabilize

	/* restore */
	sil9034_write(priv,SLAVE1,INF_CTRL1,temp1) ;
	sil9034_write(priv,SLAVE1,INF_CTRL2,temp2) ;
	return 0 ;
}
static int sil9034_triggerRom(davinci6446_sil9034 *priv)
{
	u8 reg_value ;

	sil9034_dbg("----------%s----------\n",__FUNCTION__) ;
	reg_value = sil9034_read(priv,SLAVE0,KEY_COMMAND_ADDR) ;
	sil9034_write(priv,SLAVE0,KEY_COMMAND_ADDR,reg_value & ~LD_KSV) ;
	mdelay(10) ;
	sil9034_write(priv,SLAVE0,KEY_COMMAND_ADDR,reg_value |LD_KSV) ;
	mdelay(10) ;
	sil9034_write(priv,SLAVE0,KEY_COMMAND_ADDR,reg_value & ~LD_KSV) ;
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
	sil9034_write(priv,SLAVE1,aud_info_addr++,0x100-(0x04+0x01+0x10)) ;

	/* AUDIO INFO DATA BYTE , according to Sil FAE, 5 byte is enought.
	 * page 56
	 */
	/* CT3 | CT2 | CT1 | CT0 | Rsvd | CC2 | CC1 | CC0| */
	sil9034_write(priv,SLAVE1,aud_info_addr++,0x01) ;

	/* Reserved (shall be 0) | SF2 | SF1 | SF0 | SS1 | SS0 |*/
	/* I should provide ioctl to re-sampling the frequence according
	 * to audio header type in user space program.
	 */
	sil9034_write(priv,SLAVE1,aud_info_addr++,0xD) ;

	/* format depend on data byte 1 */
	sil9034_write(priv,SLAVE1,aud_info_addr++,0x0) ;

	/* CA7 | CA6 | CA5 | CA4 | CA3 | CA2 | CA1 | CA0 | */
	sil9034_write(priv,SLAVE1,aud_info_addr++,0) ;

	/* DM_I NH | LSV3 | LSV2 | LSV1 | LSV0 | Reserved (shall be 0)| */
	sil9034_write(priv,SLAVE1,aud_info_addr++,0) ;

	return 0 ;
}
static int sil9034_cea861InfoFrameSetting(davinci6446_sil9034 *priv)
{
	u8 avi_info_addr ;
	u8 reg_value ;

	sil9034_dbg("----------%s----------\n",__FUNCTION__) ;

	reg_value = sil9034_read(priv,SLAVE1,INF_CTRL1) ;
	sil9034_write(priv,SLAVE1,INF_CTRL1,reg_value & (~BIT_AVI_REPEAT)) ;
	mdelay(64) ; // Delay VSync for unlock DATA buffer
	if(sil9034_read(priv,SLAVE1,INF_CTRL1)&BIT_AVI_ENABLE)
		sil9034_dbg("Sent AVI error\n") ;
	else
		sil9034_dbg("Silicon Image sending AVI success.\n") ;

	if(sil9034_read(priv,SLAVE1,INF_CTRL1)&BIT_AUD_ENABLE)
		sil9034_dbg("Sent AUD error\n") ;
	else
		sil9034_dbg("Silicon Image sending AUD success.\n") ;


	/* set the info frame type according to CEA-861 datasheet */
	avi_info_addr = AVI_IF_ADDR ;

	/* AVI type */
	sil9034_write(priv,SLAVE1,avi_info_addr++,0x82) ;

	/* AVI version */
	sil9034_write(priv,SLAVE1,avi_info_addr++,0x02) ;

	/* AVI length */
	sil9034_write(priv,SLAVE1,avi_info_addr++,0x13) ;

	/* AVI CRC */
	sil9034_write(priv,SLAVE1,avi_info_addr++,(0x82 + 0x02 + 0x13 + 0x1D)) ;

	/* AVI DATA BYTE , according to Sil FAE, 3 byte is enought.
	 * page 102
	 */
	/* 0 | Y1 | Y0 | A0 | B1 | B0 | S1 | S0 */
	sil9034_write(priv,SLAVE1,avi_info_addr++,0x1D) ;

	/* C1 | C0 | M1 | M0 | R3 | R2 | R1 | R0 */
	sil9034_write(priv,SLAVE1,avi_info_addr++,0x68) ;

	/*  0 | 0 | 0 | 0 | 0 | 0 | SC1 | SC0 */
	sil9034_write(priv,SLAVE1,avi_info_addr++,0x3) ;

	reg_value = sil9034_read(priv,SLAVE1,INF_CTRL1) ;
	sil9034_write(priv,SLAVE1,INF_CTRL1,reg_value | (BIT_AVI_ENABLE|BIT_AVI_REPEAT)) ;
	return 0 ;
}
static int sil9034_wakeupHdmiTx(davinci6446_sil9034 *priv)
{
	u8 reg_value ;

	reg_value = sil9034_read(priv,SLAVE0,TX_SYS_CTRL1_ADDR) ;
	sil9034_write(priv,SLAVE0,TX_SYS_CTRL1_ADDR,reg_value|SET_PD) ;
	sil9034_write(priv,SLAVE0,INT_CNTRL_ADDR,0) ;
	reg_value = sil9034_read(priv,SLAVE1,INF_CTRL1) ;
	sil9034_write(priv,SLAVE1,INF_CTRL1,reg_value |BIT_AVI_REPEAT|BIT_AUD_REPEAT) ;

	return 0 ;
}
static int sil9034_sentCPPackage(davinci6446_sil9034 *priv,u8 enable)
{
	u8 reg_value ;
	u8 timeout = 64 ;

	reg_value = sil9034_read(priv,SLAVE1,INF_CTRL2) ;
	sil9034_write(priv,SLAVE1,INF_CTRL2,reg_value &~BIT_CP_REPEAT) ;
	if(enable)
		sil9034_write(priv,SLAVE1,CP_IF_ADDR,BIT_CP_AVI_MUTE_SET) ;
	else
		sil9034_write(priv,SLAVE1,CP_IF_ADDR,BIT_CP_AVI_MUTE_CLEAR) ;

	while(timeout--)
	{
		if(!sil9034_read(priv,SLAVE1,INF_CTRL2)&BIT_CP_REPEAT)
			break ;
	}

	if(timeout)
		sil9034_write(priv,SLAVE1,INF_CTRL2,reg_value |(BIT_CP_REPEAT|BIT_CP_ENABLE)) ;

	return 0 ;
}
int sil9034_unmaskInterruptStatus(davinci6446_sil9034 *priv)
{
	u8 reg_value = 0xFF ;

	sil9034_write(priv,SLAVE0,HDMI_INT1_MASK,reg_value) ;
	sil9034_write(priv,SLAVE0,HDMI_INT2_MASK,reg_value) ;
	sil9034_write(priv,SLAVE0,HDMI_INT3_MASK,reg_value) ;

	return 0 ;
}
static int sil9034_hdmiOutputConfig(davinci6446_sil9034 *priv)
{
	u8 reg_value ;

	sil9034_dbg("----------%s----------\n",__FUNCTION__) ;
	/* HDMI control register , enable HDMI, disable DVI */
	reg_value = sil9034_read(priv,SLAVE1,HDMI_CTRL_ADDR) ;
	sil9034_write(priv,SLAVE1,HDMI_CTRL_ADDR,(reg_value | HDMI_MODE_ENABLE)) ;
	reg_value = sil9034_read(priv,SLAVE1,HDMI_CTRL_ADDR) ;
	sil9034_dbg("Hdmi control register 0x%x = 0x%x\n",HDMI_CTRL_ADDR,reg_value) ;

	return 0 ;
}

static int sil9034_ddcSetting(davinci6446_sil9034 *priv)
{
	sil9034_write(priv,SLAVE0,DDC_ADDR,HDCP_RX_SLAVE) ;
	return 0 ;
}
static int sil9034_cea861InfoFrameControl2(davinci6446_sil9034 *priv,u8 enable)
{
	u8 reg_value ;

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
static int sil9034_hdmiHdcpConfig(davinci6446_sil9034 *priv,u8 enable)
{
	u8 reg_value ;

	sil9034_dbg("----------%s----------\n",__FUNCTION__) ;
	/* HDMI HDCP configuration */
	reg_value = sil9034_read(priv,SLAVE0,HDCP_CTRL_ADDR) ;
	if(enable)
	{
		sil9034_write(priv,SLAVE0,HDCP_CTRL_ADDR,(reg_value | SET_ENC_EN)) ;
		priv->auth_state = AUTH_NEED ;
	}
	else
	{
		sil9034_write(priv,SLAVE0,HDCP_CTRL_ADDR,(reg_value & ~(SET_ENC_EN))) ;
		priv->auth_state = AUTH_DONE ;
	}

	reg_value = sil9034_read(priv,SLAVE0,HDCP_CTRL_ADDR) ;
	sil9034_dbg("Hdmi hdcp register 0x%x = 0x%x\n",HDCP_CTRL_ADDR,reg_value) ;

	return 0 ;
}
char sil9034_hotplugEvent(davinci6446_sil9034 *priv)
{
	u8 reg_value ;

	reg_value = sil9034_read(priv,SLAVE0,TX_STAT_ADDR) ;
	if(reg_value&SET_HPD)
		return 1 ;
	else
		return 0 ;
}
static int sil9034_checkHdcpDevice(davinci6446_sil9034 *priv)
{
	u8 total = 0 ;
	u8 bits = 0 ;
	u8 count = 0 ;

	/* read 5 byte from ddc */
	sil9034_ddc_read(priv,&priv->an_ksv_data[0],DDC_BKSV_ADDR,5) ;

	/* calculate bits */
	for(count=0 ;count<5 ; count++)
	{
		sil9034_dbg("bksv %d,0x%x\n",count,priv->an_ksv_data[count]) ;
		for(bits=0 ;bits<8 ; bits++)
			if(priv->an_ksv_data[count] & (1<<bits))
				total++ ;
	}

	if(total == HDCP_ACC)
		return TRUE ;
	else
		return FALSE ;
}

static int sil9034_toggleRepeatBit(davinci6446_sil9034 *priv)
{ 
	u8 reg_value ;

	sil9034_dbg("----------%s----------\n",__FUNCTION__) ;
	reg_value = sil9034_read(priv,SLAVE0,HDCP_CTRL_ADDR) ;
	if(reg_value & RX_RPTR_ENABLE)
		sil9034_write(priv,SLAVE0,HDCP_CTRL_ADDR,reg_value&~RX_RPTR_ENABLE) ;
	else
		sil9034_write(priv,SLAVE0,HDCP_CTRL_ADDR,reg_value|RX_RPTR_ENABLE) ;

	return 0 ;
}

static int sil9034_releaseCPReset(davinci6446_sil9034 *priv)
{ 
	u8 reg_value ;

	sil9034_dbg("----------%s----------\n",__FUNCTION__) ;
	reg_value = sil9034_read(priv,SLAVE0,HDCP_CTRL_ADDR) ;
	sil9034_write(priv,SLAVE0,HDCP_CTRL_ADDR,reg_value|SET_CP_RESTN) ;

	return 0 ;
}

static int sil9034_StopRepeatBit(davinci6446_sil9034 *priv)
{ 
	u8 reg_value ;

	sil9034_dbg("----------%s----------\n",__FUNCTION__) ;
	reg_value = sil9034_read(priv,SLAVE0,HDCP_CTRL_ADDR) ;
	sil9034_write(priv,SLAVE0,HDCP_CTRL_ADDR,reg_value&~RX_RPTR_ENABLE) ;
	return 0 ;
}

static int sil9034_writeAnHdcpRx(davinci6446_sil9034 *priv)
{ 
	/* write 8 byte to ddc hdcp rx*/
	sil9034_ddc_write(priv,&priv->an_ksv_data[0],DDC_AN_ADDR,8) ;

	return 0 ;
}
static int sil9034_writeBksvHdcpTx(davinci6446_sil9034 *priv)
{ 
	u8 count = 0 ;

	sil9034_dbg("----------%s----------\n",__FUNCTION__) ;

	for(count=0; count<5; count++)
	{
		 sil9034_write(priv,SLAVE0,HDCP_BKSV1_ADDR+count,priv->an_ksv_data[count]) ;
		 sil9034_dbg("write bksv to tx 0x%x\n",priv->an_ksv_data[count]) ;
	}

	return 0 ;
}
static int sil9034_readBksvHdcpRx(davinci6446_sil9034 *priv)
{ 
	u8 count = 0 ;

	sil9034_dbg("----------%s----------\n",__FUNCTION__) ;

	/* read 5 byte from ddc */
	sil9034_ddc_read(priv,&priv->an_ksv_data[0],DDC_BKSV_ADDR,5) ;
	for(count=0; count<5; count++)
	{
		sil9034_dbg("bksv data %d 0x%x\n",count,priv->an_ksv_data[count]) ;
	}

	return 0 ;
}

static int sil9034_writeAksvHdcpRx(davinci6446_sil9034 *priv)
{ 
	sil9034_dbg("----------%s----------\n",__FUNCTION__) ;
	/* write 5 byte to ddc hdcp rx*/
	sil9034_ddc_write(priv,&priv->an_ksv_data[0],DDC_AKSV_ADDR,5) ;

	return 0 ;
}

static int sil9034_readAksvHdcpTx(davinci6446_sil9034 *priv)
{ 
	u8 count = 0 ;

	sil9034_dbg("----------%s----------\n",__FUNCTION__) ;

	for(count=0; count<5; count++)
	{
		priv->an_ksv_data[count] = sil9034_read(priv,SLAVE0,HDCP_AKSV1_ADDR+count) ;
		sil9034_dbg("aksv data %d 0x%x\n",count,priv->an_ksv_data[count]) ;
	}

	return 0 ;
}

static int sil9034_readAnHdcpTx(davinci6446_sil9034 *priv)
{ 
	u8 count = 0 ;

	sil9034_dbg("----------%s----------\n",__FUNCTION__) ;

	for(count=0; count<8; count++)
	{
		priv->an_ksv_data[count] = sil9034_read(priv,SLAVE0,HDCP_AN1_ADDR+count) ;
		sil9034_dbg("an data %d 0x%x\n",count,priv->an_ksv_data[count]) ;
	}

	return 0 ;
}

static int sil9034_generateAn(davinci6446_sil9034 *priv)
{ 
	u8 reg_value ;

	sil9034_dbg("----------%s----------\n",__FUNCTION__) ;

	reg_value = sil9034_read(priv,SLAVE0,HDCP_CTRL_ADDR) ;
	sil9034_write(priv,SLAVE0,HDCP_CTRL_ADDR,reg_value&~TX_ANSTOP) ;
	mdelay(10) ;
	sil9034_write(priv,SLAVE0,HDCP_CTRL_ADDR,reg_value|TX_ANSTOP) ;

	return 0 ;
}
static int sil9034_isRepeater(davinci6446_sil9034 *priv)
{
	u8 reg_value ;

	sil9034_dbg("----------%s----------\n",__FUNCTION__) ;
	/* read 1 byte from ddc */
	sil9034_ddc_read(priv,&reg_value,DDC_BCAPS_ADDR,1) ;
	if(reg_value&DDC_BIT_REPEATER)
		return TRUE ;

	return FALSE ;
}
static int sil9034_compareR0(davinci6446_sil9034 *priv)
{
	sil9034_dbg("----------%s----------\n",__FUNCTION__) ;

	/* read 2 byte from ddc */
	sil9034_ddc_read(priv,&priv->r0rx[0],DDC_RI_ADDR,2) ;
	priv->r0tx[0] = sil9034_read(priv,SLAVE0,HDCP_RI1) ;
	priv->r0tx[1] = sil9034_read(priv,SLAVE0,HDCP_RI2) ;

	if((priv->r0rx[0]==priv->r0tx[0])&&(priv->r0rx[1]==priv->r0tx[1]))
	{
		printk(KERN_INFO "HDCP handshake complete match.\n") ;
		return TRUE ;
	}

	return FALSE ;
}

static int sil9034_autoRiCheck(davinci6446_sil9034 *priv,u8 enable)
{
	u8 reg_value ;

	reg_value = sil9034_read(priv,SLAVE0,RI_CMD_ADDR) ;
	if(enable)
		sil9034_write(priv,SLAVE0,RI_CMD_ADDR, reg_value|SET_RI_ENABLE);
	else
		sil9034_write(priv,SLAVE0,RI_CMD_ADDR, reg_value&~SET_RI_ENABLE);
	return 0 ;
}

int sil9034_dumpSystemStatus(davinci6446_sil9034 *priv)
{
	u8 reg_value ;

	sil9034_dbg("----------%s----------\n",__FUNCTION__) ;
	reg_value = sil9034_read(priv,SLAVE0,TX_STAT_ADDR) ;
	printk(KERN_INFO "System status register 0x%x = 0x%x\n",TX_STAT_ADDR,reg_value) ;
	return 0 ;
}

int sil9034_dumpDataCtrlStatus(davinci6446_sil9034 *priv)
{
	u8 reg_value ;

	sil9034_dbg("----------%s----------\n",__FUNCTION__) ;
	reg_value = sil9034_read(priv,SLAVE0,DCTL_ADDR) ;
	printk(KERN_INFO "Data control register 0x%x = 0x%x\n",DCTL_ADDR,reg_value) ;
	return 0 ;
}

int sil9034_dumpVideoConfigureStatus(davinci6446_sil9034 *priv)
{
	u8 reg_value ;

	sil9034_dbg("----------%s----------\n",__FUNCTION__) ;
	reg_value = sil9034_read(priv,SLAVE0,HRES_L_ADDR) ;
	printk(KERN_INFO "H resolution low register 0x%x = 0x%x\n",HRES_L_ADDR,reg_value) ;

	reg_value = sil9034_read(priv,SLAVE0,HRES_H_ADDR) ;
	printk(KERN_INFO "H resolution high register 0x%x = 0x%x\n",HRES_H_ADDR,reg_value) ;

	reg_value = sil9034_read(priv,SLAVE0,VRES_L_ADDR) ;
	printk(KERN_INFO "V resolution low register 0x%x = 0x%x\n",VRES_L_ADDR,reg_value) ;

	reg_value = sil9034_read(priv,SLAVE0,VRES_H_ADDR) ;
	printk(KERN_INFO "V resolution high register 0x%x = 0x%x\n",VRES_H_ADDR,reg_value) ;

	reg_value = sil9034_read(priv,SLAVE0,DEL_L_ADDR) ;
	printk(KERN_INFO "DE line low register 0x%x = 0x%x\n",DEL_L_ADDR,reg_value) ;

	reg_value = sil9034_read(priv,SLAVE0,DEL_H_ADDR) ;
	printk(KERN_INFO "DE line high register 0x%x = 0x%x\n",DEL_H_ADDR,reg_value) ;

	reg_value = sil9034_read(priv,SLAVE0,POL_DETECT_ADDR) ;
	printk(KERN_INFO "Video polarity detect register 0x%x = 0x%x\n",POL_DETECT_ADDR,reg_value) ;

	reg_value = sil9034_read(priv,SLAVE0,HLENGTH1_ADDR) ;
	printk(KERN_INFO "Video HSYNC length1 register 0x%x = 0x%x\n",HLENGTH1_ADDR,reg_value) ;

	reg_value = sil9034_read(priv,SLAVE0,HLENGTH2_ADDR) ;
	printk(KERN_INFO "Video HSYNC length2 register 0x%x = 0x%x\n",HLENGTH2_ADDR,reg_value) ;

	reg_value = sil9034_read(priv,SLAVE0,VBIT_TO_VSYNC_ADDR) ;
	printk(KERN_INFO "Video Vbit to VSync register 0x%x = 0x%x\n",VBIT_TO_VSYNC_ADDR,reg_value) ;

	reg_value = sil9034_read(priv,SLAVE0,VLENGTH_ADDR) ;
	printk(KERN_INFO "Video VSYNC length register 0x%x = 0x%x\n",VLENGTH_ADDR,reg_value) ;

	reg_value = sil9034_read(priv,SLAVE0,TX_TMDS_CCTRL_ADDR) ;
	printk(KERN_INFO "TMDS C control register 0x%x = 0x%x\n",TX_TMDS_CCTRL_ADDR,reg_value) ;

	reg_value = sil9034_read(priv,SLAVE0,TX_TMDS_CTRL_ADDR) ;
	printk(KERN_INFO "TMDS control register 0x%x = 0x%x\n",TX_TMDS_CTRL_ADDR,reg_value) ;

	reg_value = sil9034_read(priv,SLAVE0,TX_TMDS_CTRL2_ADDR) ;
	printk(KERN_INFO "TMDS control #2 register 0x%x = 0x%x\n",TX_TMDS_CTRL2_ADDR,reg_value) ;

	reg_value = sil9034_read(priv,SLAVE0,TX_TMDS_CTRL3_ADDR) ;
	printk(KERN_INFO "TMDS control #3 register 0x%x = 0x%x\n",TX_TMDS_CTRL3_ADDR,reg_value) ;

	reg_value = sil9034_read(priv,SLAVE0,TX_TMDS_CTRL4_ADDR) ;
	printk(KERN_INFO "TMDS control #4 register 0x%x = 0x%x\n",TX_TMDS_CTRL4_ADDR,reg_value) ;

	reg_value = sil9034_read(priv,SLAVE0,TX_VID_ACEN_ADDR) ;
	printk(KERN_INFO "Video action enable register 0x%x = 0x%x\n",TX_VID_ACEN_ADDR,reg_value) ;

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
	printk(KERN_INFO "Interrupt source 1 register 0x%x = 0x%x\n",INT_SOURCE1_ADDR,reg_value) ;
	reg_value = sil9034_read(priv,SLAVE0,INT_SOURCE2_ADDR) ;
	printk(KERN_INFO "Interrupt source 2 register 0x%x = 0x%x\n",INT_SOURCE2_ADDR,reg_value) ;
	reg_value = sil9034_read(priv,SLAVE0,INT_SOURCE3_ADDR) ;
	printk(KERN_INFO "Interrupt source 3 register 0x%x = 0x%x\n",INT_SOURCE3_ADDR,reg_value) ;
	/* Interrupt register will auto clean after read ?*/
	sil9034_clearInterruptStatus(priv) ;

	return 0 ;
}
int sil9034_dumpInterruptStateStatus(davinci6446_sil9034 *priv)
{
	u8 reg_value ;

	sil9034_dbg("----------%s----------\n",__FUNCTION__) ;
	reg_value = sil9034_read(priv,SLAVE0,INT_STATE_ADDR) ;
	printk(KERN_INFO "Interrupt state register 0x%x = 0x%x\n",INT_STATE_ADDR,reg_value) ;
	if(reg_value & INT_ENABLE)
		sil9034_dumpInterruptSourceStatus(priv) ;
	else
		sil9034_dbg("No unmask interrupt\n") ;
	return 0 ;
}

int sil9034_dumpHdcpRiStatus(davinci6446_sil9034 *priv)
{
	u8 reg_value ;

	sil9034_dbg("----------%s----------\n",__FUNCTION__) ;
	reg_value = sil9034_read(priv,SLAVE0,HDCP_CTRL_ADDR) ;
	printk(KERN_INFO "Hdcp register is %x,%s\n",reg_value,(reg_value&SET_ENC_EN)?"Enable":"Disable") ;
	reg_value = sil9034_read(priv,SLAVE0,HDCP_RI1) ;
	printk(KERN_INFO "Hdcp ri 1 status register 0x%x = 0x%x\n",HDCP_RI1,reg_value) ;
	reg_value = sil9034_read(priv,SLAVE0,HDCP_RI2) ;
	printk(KERN_INFO "Hdcp ri 2 status register 0x%x = 0x%x\n",HDCP_RI2,reg_value) ;
	reg_value = sil9034_read(priv,SLAVE0,HDCP_RI128_COMP) ;
	printk(KERN_INFO "Hdcp ri 128 status register 0x%x = 0x%x\n",HDCP_RI128_COMP,reg_value) ;
	reg_value = sil9034_read(priv,SLAVE0,RI_CMD_ADDR) ;
	printk(KERN_INFO "ri cmd status register 0x%x = 0x%x\n",RI_CMD_ADDR,reg_value) ;
	reg_value = sil9034_read(priv,SLAVE0,DDC_MAN_ADDR) ;
	printk(KERN_INFO "ddc man status register 0x%x = 0x%x\n",DDC_MAN_ADDR,reg_value) ;
	reg_value = sil9034_read(priv,SLAVE0,RI_STAT_ADDR) ;
	printk(KERN_INFO "ri start status register 0x%x = 0x%x\n",RI_STAT_ADDR,reg_value) ;
	reg_value = sil9034_read(priv,SLAVE0,RI_RX_L_ADDR) ;
	printk(KERN_INFO "ri rx low status register 0x%x = 0x%x\n",RI_RX_L_ADDR,reg_value) ;
	reg_value = sil9034_read(priv,SLAVE0,RI_RX_H_ADDR) ;
	printk(KERN_INFO "ri rx hight status register 0x%x = 0x%x\n",RI_RX_H_ADDR,reg_value) ;
	reg_value = sil9034_read(priv,SLAVE0,DDC_ADDR) ;
	printk(KERN_INFO "ddc status register 0x%x = 0x%x\n",DDC_ADDR,reg_value) ;

	reg_value = sil9034_read(priv,SLAVE1,INF_CTRL2) ;
	printk(KERN_INFO "InfoFrame ctrl2 register 0x%x = 0x%x\n",INF_CTRL2,reg_value) ;

	reg_value = sil9034_read(priv,SLAVE1,INF_CTRL1) ;
	printk(KERN_INFO "InfoFrame ctrl1 register 0x%x = 0x%x\n",INF_CTRL1,reg_value) ;
	return 0 ;
}

int sil9034_hdcpAuthentication(davinci6446_sil9034 *priv)
{
	sil9034_autoRiCheck(priv,DISABLE) ;
	if(sil9034_hotplugEvent(priv))
	{

		sil9034_sentCPPackage(priv,ENABLE) ;
		if(sil9034_checkHdcpDevice(priv) == TRUE)
		{
			sil9034_dbg("got 20's 1\n") ;
			/* Random key */
			sil9034_toggleRepeatBit(priv) ;
			sil9034_releaseCPReset(priv) ;
			sil9034_StopRepeatBit(priv) ;
			sil9034_generateAn(priv) ;
			/* Handshake start */
			sil9034_readAnHdcpTx(priv) ;
			sil9034_writeAnHdcpRx(priv) ;
			sil9034_readAksvHdcpTx(priv) ;
			sil9034_writeAksvHdcpRx(priv) ;
			sil9034_readBksvHdcpRx(priv) ;
			sil9034_writeBksvHdcpTx(priv) ;
			if(sil9034_isRepeater(priv)==TRUE)
				printk(KERN_ERR "This is repeater,not support.\n") ;
			/* Finally, compare key */
			mdelay(100) ; //delay for R0 calculation
			if(sil9034_compareR0(priv)==FALSE)
			{
				priv->auth_state = REAUTH_NEED ;
			}
			else
			{
				/* unmute */
				sil9034_sentCPPackage(priv,DISABLE) ;
				sil9034_autoRiCheck(priv,ENABLE) ;
				priv->auth_state = AUTH_DONE ;
			}
		}
		else // no 20 ones and zeros
		{
			/* mute */
			sil9034_sentCPPackage(priv,ENABLE) ;
			sil9034_dbg("TV not send 20's 1,retry!!\n") ;
		}
	}
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
		priv->work_flag = EVENT_NOTIFY ;
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
	u8 intr1_isr ;
	u8 intr3_isr ;
	
	switch(priv->work_flag)
	{
		case SWITCH_480P:
			priv->sil9034_setting = sil9034_480p_setting ;
			priv->work_flag = VIDEO_CHANGE ;
			break ;
		case SWITCH_720P:
			priv->sil9034_setting = sil9034_720p_setting ;
			priv->work_flag = VIDEO_CHANGE ;
			break ;
		case SWITCH_1080I:
			priv->sil9034_setting = sil9034_1080i_setting ;
			priv->work_flag = VIDEO_CHANGE ;
			break ;
		case HDCP_ENABLE:
			sil9034_hdmiHdcpConfig(priv,ENABLE) ;
			break ;
		case HDCP_DISABLE:
			sil9034_hdmiHdcpConfig(priv,DISABLE) ;
			break ;
		case SHOW_REGISTER:
			sil9034_dumpSystemStatus(priv) ;
			sil9034_dumpDataCtrlStatus(priv) ;
			sil9034_dumpInterruptStateStatus(priv) ;
			sil9034_dumpVideoConfigureStatus(priv) ;
			break ;
		case HDCP_RI_STATUS:
			sil9034_dumpHdcpRiStatus(priv) ;
			break ;
		case EVENT_NOTIFY:
			intr3_isr = sil9034_read(priv,SLAVE0,INT_SOURCE3_ADDR) ;
			intr1_isr = sil9034_read(priv,SLAVE0,INT_SOURCE1_ADDR) ;
			/* ri frame error occur */
			if(intr3_isr & (INTR3_STAT7|INTR3_STAT5|INTR3_STAT4))
			{
				printk(KERN_INFO "Ri frame error 0x%x\n",intr3_isr) ;
				priv->auth_state = REAUTH_NEED ;
			}
			/* line plug */
			if(intr1_isr & (INTR1_HPD|INTR1_RSEN))
			{
				printk(KERN_INFO "line plug 0x%x\n",intr1_isr) ;
				priv->auth_state = REAUTH_NEED ;
			}

			break ;
		default:
			break ;
	}

	/* Check if video setting change */
	if((priv->work_flag==VIDEO_CHANGE))
		sil9034_videoInputConfig(priv) ;

	/* Check if hdcp authentication need */
	if((priv->auth_state == AUTH_NEED) || (priv->auth_state == REAUTH_NEED))
	{
		sil9034_hdcpAuthentication(priv) ;
	}

	/* clean the work flag */
	priv->work_flag = DO_NOTHING ;
	
#if SIL9034_TIMER
	mod_timer(&ds.timer, jiffies + TIMER_JIFFIES);
#endif
	sil9034_clearInterruptStatus(priv) ;
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

	/* Initial default as 720p */
	ds.sil9034_setting = sil9034_720p_setting ;
	
	/* read chip id & revision */
	sil9034_chipInfo(&ds) ;

	/* power down occilator */
	sil9034_powerDown(&ds,ENABLE) ;

	/* TMDS control register */
	sil9034_hdmiTmdsConfig(&ds) ;

	/* Tune the audio input table according to DM320 hardware spec */
	sil9034_audioInputConfig(&ds) ;

	/* read flag from env and tune the video input table 
	 * according to DM320 hardware spec */
	sil9034_videoInputConfig(&ds) ;

	/* software reset */
	sil9034_swReset(&ds) ;

	/* Trigger ROM */
	sil9034_triggerRom(&ds) ;

	/* Audio Info Frame control setting */
	sil9034_audioInfoFrameSetting(&ds) ;
       	sil9034_audioInputConfig(&ds) ;

	/* software reset */
	sil9034_swReset(&ds) ;

	/* CEA-861 Info Frame control setting */
	sil9034_cea861InfoFrameSetting(&ds) ;

	/* Wake up HDMI TX */
	sil9034_wakeupHdmiTx(&ds) ;

	/* Sent CP package */
	sil9034_sentCPPackage(&ds,ENABLE) ;

	/* unmask the interrupt status */
	sil9034_unmaskInterruptStatus(&ds) ;

	/* Hdmi output setting */
	sil9034_hdmiOutputConfig(&ds) ;

	/* ddc master config */
	sil9034_ddcSetting(&ds) ;

	/* General control packet */
	sil9034_sentCPPackage(&ds,DISABLE) ;

	sil9034_cea861InfoFrameControl2(&ds,ENABLE) ;

	/* HDCP control handshaking */
	sil9034_hdmiHdcpConfig(&ds,ENABLE) ;

	/* Disable Auto RI check here, enable it until hdcp complete. */
	sil9034_autoRiCheck(&ds,DISABLE) ;

	sil9034_clearInterruptStatus(&ds) ;

#if SIL9034_SCHED
	INIT_WORK(&ds.work, sil9034_sched);
	ds.work_flag = EVENT_NOTIFY ;
	schedule_work(&ds.work);
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
