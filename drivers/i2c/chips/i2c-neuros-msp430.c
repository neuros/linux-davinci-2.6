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
 * MSP430 i2c driver. 
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

#include <asm/uaccess.h>
#include <asm/io.h>
#include <asm/irq.h>

#include <asm/arch/hardware.h>

#if 0
    #define dbg(fmt, arg...) \
        printk(KERN_INFO "%s:%d> " fmt, __func__, __LINE__ , ## arg)
#else
    #define dbg(fmt, arg...)
#endif


//TODO: the sleep time should be tweaked to some value between 100 and 200 ms. 200ms sure works, but maybe it's too much. 100 doesn't work well. --nerochiaro
#define I2C_RETRY_SLEEP 200 
#define I2C_RETRY_COUNT 3

#define MOD_DESC "MSP430 I2C Driver (c) 2008"

static int irrtc_attach_adapter(struct i2c_adapter * adapter);
static int irrtc_detach_client(struct i2c_client * client);

static unsigned short normal_i2c[] = {
  0x59,       /* 7-bit address does not require >> 1 */
  I2C_CLIENT_END
};

/*
static unsigned short normal_i2c_range[] = {
    I2C_CLIENT_END
};
*/
I2C_CLIENT_INSMOD;

static struct i2c_client* irrtc_client = NULL;

static struct i2c_driver irrtc_driver = {
	.driver = {
        .owner  = THIS_MODULE,
		.name	= "MSP430 I2C v.$Revision: 1.24 $",
		//.flags	= I2C_DF_NOTIFY,
	},
	.id		= I2C_DRIVERID_IRRTC,
	.attach_adapter	= irrtc_attach_adapter,
	.detach_client	= irrtc_detach_client,
};

static const char * pname = "MPS430_I2C(KM):";

int i2c_write(u8 reg, u16 value)
{
    int retry = I2C_RETRY_COUNT;
    int ret = -1;

    if(irrtc_client)
    {
        while(retry--)
        {
            ret = i2c_smbus_write_byte_data(irrtc_client, reg, value & 0xFF);
            if (-1 != ret) break;
            printk("(msp430(i2c_write)) retry [%d]\n", retry);
            msleep(I2C_RETRY_SLEEP);
        }
        return ret;
    }
    else
        return 0xff;
}
EXPORT_SYMBOL(i2c_write);

int i2c_read(u8 reg)
{
    int retry = I2C_RETRY_COUNT;
    int dat;

    if(irrtc_client)
    {
        while(retry--)
        {
            dat = i2c_smbus_read_byte_data(irrtc_client, reg);
            if (-1 != dat) break;
            printk("(msp430(i2c_read)) retry [%d]\n", retry);
            msleep(I2C_RETRY_SLEEP);
        }
        return dat;
    }
    else
        return 0xff;
}
EXPORT_SYMBOL(i2c_read);

static int irrtc_detect_client(struct i2c_adapter * adapter, int address, int kind)
{
    int ret = 0;
    struct i2c_client* client = NULL;

    if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_WRITE_BYTE))
    {
        dbg("ERROR returned from i2c_check_functionality");
        ret = -1;
        goto out;
    }

    if (!(client = kmalloc(sizeof(*client), GFP_KERNEL)))
    {
        dbg("ERROR error returned from kmalloc");
        ret = -ENOMEM;
        goto out;
    }

    memset(client, 0, sizeof(struct i2c_client));

    client->addr = address;
    client->adapter = adapter;
    client->driver = &irrtc_driver;
    client->flags = I2C_M_IGNORE_NAK;

    strcpy(client->name, "irrtc");

    if ((ret = i2c_attach_client(client)) != 0)
    {
        dbg("%s Unable to attach client.\n", pname);
        //kfree(client);
        ret = -1;
        goto out;
    }

    irrtc_client = client;

#if 0
    printk("Reading remote ID\n");
    printk("Register:Value 0x%02X:0x%02X\n", 0x36, i2c_read(0x36));
#endif
out:
    return ret;
}

static int irrtc_attach_adapter(struct i2c_adapter * adapter)
{
    return(i2c_probe(adapter, &addr_data, &irrtc_detect_client));
}

static int irrtc_detach_client(struct i2c_client * client)
{
    int ret = 0;

    if ((ret = i2c_detach_client(client)) != 0)
    {
        dbg("%s Unable to detach client.\n", pname);
        goto out;
    }

    kfree(client);

out:
    return ret;
}

//----------------------------------------------------------  INIT / EXIT ---------------------------------------------------------

static int __init irrtc_init(void)
{
    int status = 0;

    printk(KERN_INFO "\t" MOD_DESC "\n");

    if ((status = i2c_add_driver(&irrtc_driver)) < 0)
    {
        printk(KERN_INFO "%s Couldn't register MSP430 I2C driver.\n", pname);
        goto out;
    }

out:
    return status;
}

static void __exit irrtc_exit(void)
{
    i2c_del_driver(&irrtc_driver);
}

MODULE_AUTHOR("Neuros");
MODULE_DESCRIPTION(MOD_DESC);
MODULE_LICENSE("Neuros Technology LLC");

module_init(irrtc_init);
module_exit(irrtc_exit);


