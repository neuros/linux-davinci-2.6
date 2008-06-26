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
 * IR Receiver driver. 
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
#include <linux/input.h>

#include <asm/uaccess.h>
#include <asm/io.h>
#include <asm/irq.h>
#include <linux/interrupt.h>

#include <asm/arch/hardware.h>

#include <linux/neuros_ir.h>
#include <linux/neuros_ir_blaster.h>
#include "neuros_keymaps.h"

#define MOD_DESC "Neuros IR Driver (c) 2008"

#if 0
    #define dbg(fmt, arg...) \
        printk(KERN_INFO "%s:%d> " fmt, __func__, __LINE__ , ## arg)
#else
    #define dbg(fmt, arg...)
#endif

#define USE_WORKQUEUE 1
#define READ_ONLY_ONE_KEY 1
#define IR_RETRY_COUNT 3
#define FACTORY_TEST_DELAY 6

struct irrtc_device {
    int    key;
};
static struct irrtc_device device;

static wait_queue_head_t wait;
static wait_queue_head_t poll_queue;

static char *devname = "neuros_ir";
static struct input_dev *ir_input_dev;
static int factory_test=0;
static int osd_key=0;
static int is_learning=0;

#if USE_WORKQUEUE
#define KEYBUF_SIZE 2
static int keybuf[KEYBUF_SIZE];
static int roffset=0;
static int woffset=0;
static int numOfKeys=0;
#else
static int keyin=0;
#endif

static void irrtc_report_key(int ir_key);

DECLARE_MUTEX(keybuf_sem);

static spinlock_t data_protect = SPIN_LOCK_UNLOCKED;

static void lock_data_protect(void)
{
	spin_lock(&data_protect);
}
EXPORT_SYMBOL(lock_data_protect);

static void unlock_data_protect(void)
{
	spin_unlock(&data_protect);        	
}
EXPORT_SYMBOL(unlock_data_protect);

static void set_factory_test(int value)
{
	factory_test=value;
}
EXPORT_SYMBOL(set_factory_test);

static int get_osd_key(void)
{
	return osd_key;
}
EXPORT_SYMBOL(get_osd_key);

static void set_osd_key(int value)
{
	osd_key=value;
}
EXPORT_SYMBOL(set_osd_key);

static void set_is_learning(int value)
{
    is_learning=value;
}
EXPORT_SYMBOL(set_is_learning);

static int read_keybuf(void)
{
    int key = -1;//none key

    if (-EINTR == down_interruptible(&keybuf_sem))
        return -1;

    dbg("numOfKeys = %d", numOfKeys);

    if (numOfKeys > 0)
    {
        numOfKeys--;
        key = keybuf[roffset];
	if(++roffset >= KEYBUF_SIZE) roffset = 0;
    }

    up(&keybuf_sem);

    return key;
}

#if USE_WORKQUEUE

static void write_keybuf(int key)
{
    if (-EINTR == down_interruptible(&keybuf_sem))
	return;

	// Display some more sober debug message on key presses but don't bother with key releases.
	if (key != 0x00) printk("{IR:key:%02x}\n", key);

    if(++numOfKeys > KEYBUF_SIZE) 
      {
	  numOfKeys = KEYBUF_SIZE;
	  if(++roffset >= KEYBUF_SIZE) roffset = 0;
      }

    keybuf[woffset]=key;
    if(++woffset >= KEYBUF_SIZE) woffset = 0;

    up(&keybuf_sem);
}
#endif

static void report_key(int key)
{
	irrtc_report_key(key); //before adding to keybuf we report this key to the input system
	write_keybuf(key);
	wake_up_interruptible(&poll_queue);
}
EXPORT_SYMBOL(report_key);
//----------------------------------------------------------- INTERRUPTS -------------------------------------------------------

#if USE_WORKQUEUE
static void irrtc_do_wq(struct work_struct *work)
{       
	int key;
	int retry = IR_RETRY_COUNT;

	if (is_learning == 1)
	{
		disable_irq(IRQ_TINT1_TINT34);
        TIMER1_TCR &= ~(3<<22); //disable timer1 34
        CLR_GPIO01_RIS_INT |= (1<<7); // gpio 7 rising edge IRQ disable
        CLR_GPIO01_FAL_INT |= (1<<7); // gpio 7 falling edge IRQ disable
		lock_data_protect();
		set_osd_key(1);
        set_is_learning(0);
		unlock_data_protect();
	}
	//HACK: we KNOW that key should never become 0xFF!!
	do 
	{
		key = KEY_MASK & i2c_read(regIR);
	} while ((key == NULL_KEY) && (retry--));

	dbg("do tasklet: key = [%x]\n", key);
	if (key <= TEST_KEY && key >= UP_KEY)
	{
		report_key(key);
	}
}

DECLARE_WORK(irrtc_wq, irrtc_do_wq); 
DECLARE_DELAYED_WORK(irrtc_delay_wq, irrtc_do_wq); 
#endif

static irqreturn_t handle_irrtc_irqs(int irq, void * dev_id)
{
#if USE_WORKQUEUE
	if (!factory_test)
		schedule_work(&irrtc_wq);
	else 
	{
		schedule_delayed_work(&irrtc_delay_wq,FACTORY_TEST_DELAY);
		set_factory_test(0);
	}
#else
	keyin=1;
#endif
	return IRQ_HANDLED;
}

static void irqs_irrtc_init( void )
{
    GPIO01_DIR |= 0x04;  //gio 2 direction input
    SET_GPIO01_RIS_INT |= 0x04; // gio 2 rising edge IRQ enable
    request_irq(IRQ_GPIO2, handle_irrtc_irqs, 0, "irrtc", &device); 
}

static void irqs_irrtc_exit( void )
{
    free_irq(IRQ_GPIO2, &device);
}

// ---------------------------------------------------------- DEVICE -----------------------------------------------------------

static int irrtc_open(struct inode * inode, struct file * file)
{
#ifdef ONLYOPENONE
    if (opened)
        return -EBUSY;
    opened = 1;
#endif

    return 0;
}

static int irrtc_release(struct inode * inode, struct file * file)
{
#ifdef ONLYOPENONE
    if (!opened)
        return -ERESTARTSYS;

    opened = 0;
#endif

    return 0;
}

static ssize_t irrtc_read(struct file *filp, char __user *buff, size_t count, loff_t *ppos)
{ 
#if READ_ONLY_ONE_KEY        
    int key,r;
    key = read_keybuf();
    dbg("key=%d\n", key);

    if (key!=-1)  //none key
        r=copy_to_user(buff, &key, sizeof(int));
    else
        count=0;
#else
    if (-EINTR == down_interruptible(&keybuf_sem))
        return 0; 

    if (count > KEYBUF_SIZE * sizeof(int)) {
        if (roffset <= woffset)
            count = (woffset - roffset) * sizeof(int);
        else
            count = (KEYBUF_SIZE - (roffset - woffset + 1)) * sizeof(int);
    }

    if (count == 0) {
        printk("buffer empty\n");
        goto out;
    }

    if (roffset < woffset)
        r=copy_to_user(buff, keybuf + roffset, count);
    else {
        r=copy_to_user(buff, &keybuf[roffset], (KEYBUF_SIZE - roffset + 1) * sizeof(int));
        r=copy_to_user(buff + (KEYBUF_SIZE - roffset + 1) * sizeof(int), keybuf, woffset * sizeof(int));
    }

out:
    up(&keybuf_sem);
#endif
    return count;
}

static ssize_t irrtc_write(struct file * file, const char __user * buf, size_t count, loff_t *ppos)
{
    int key;
    if (copy_from_user(&key, buf, sizeof(int)))
        return -1;

    //printk("-----------------%x\n",key);
	report_key(key);
    return count;
}

static unsigned int irrtc_poll(struct file*filp, poll_table*wait)
{
    unsigned int mask = 0;
    poll_wait(filp, &poll_queue, wait);
    if (numOfKeys)
        mask |= POLLIN | POLLRDNORM;
    return mask;
}

static struct file_operations irrtc_fops = {
    .open    = irrtc_open,
    .release = irrtc_release,
    .read    = irrtc_read,
    .poll    = irrtc_poll,
    .write   = irrtc_write,
};


// setup all input device information and actually create the device
static void irrtc_inputdev_init(void)
{
    int i;

    ir_input_dev = input_allocate_device();
    ir_input_dev->name = devname;
    ir_input_dev->evbit[0] = BIT(EV_KEY); //this is the type of events we will generate. only keys.

    // set a bit in the input device structure for each key that we are able to generate.
    for (i = 0; i < NUM_KEYS; i++)
        if (keymap[i] != 0) 
            set_bit(keymap[i], ir_input_dev->keybit);

    input_register_device(ir_input_dev); //done. we can now send events into the system.
}

// close the input device
static void irrtc_inputdev_close(void)
{
    input_unregister_device(ir_input_dev);
}

// gets an IR key code and reports it to the input system
static void irrtc_report_key(int key)
{
    int code;

    // Repeat keypresses on IR remote have different codes, but we treat all them as normal key 
    // presses by clearing away the modifiers.
    //if ((key & 0x80) == 0x80) key ^= 0x80;        //START_REPEAT_MOD
    //if ((key & 0x40) == 0x40) key ^= 0x40;        //REPEAT_MOD

    // Map the code to one of the standard keboard codes accepted by the input system.
    // todo: allow keymap switching via ioctl (maybe custom remapping too)
    code = (key > NUM_KEYS) ? 0 : keymap[key];
    if (code == 0) code = KEY_UNKNOWN;

    // Report a full key press + release for each even received from IR. I see no other way to do this
    // until MSP430 can send us properly flagged "key release" codes.
    input_report_key(ir_input_dev, code, 1);
    input_report_key(ir_input_dev, code, 0);
}

//-------------------------------------------------- INIT / EXIT ----------------------------------------------------------

static const char * pname = "NEUROS_IR(KM):";

static int __init irrtc_init(void)
{
    int status = 0;

    init_waitqueue_head (&wait);
    init_waitqueue_head (&poll_queue);

    printk(KERN_INFO "\t" MOD_DESC "\n");

    status = register_chrdev(NEUROS_IR_MAJOR, "neuros_ir", &irrtc_fops);
    if (status != 0)
    {
        if (status == -EINVAL) printk(KERN_ERR "%s Couldn't register device: invalid major number %d.\n", pname, NEUROS_IR_MAJOR);
        else if (status == -EBUSY) printk(KERN_ERR "%s Couldn't register device: major number %d already busy.\n", pname, NEUROS_IR_MAJOR);
        else printk(KERN_ERR "%s Couldn't register device: error %d.\n", pname, status);
        status = -1;

        goto out;
    }

    irrtc_inputdev_init();
    memset(keybuf,-1,sizeof(keybuf));  //init keybuf to none key
    //proc_irrtc_init(); //PROC:
    irqs_irrtc_init();

out:
    return status;
}

static void __exit irrtc_exit(void)
{
    //proc_irrtc_exit(); //PROC
    irqs_irrtc_exit();
    irrtc_inputdev_close();
    unregister_chrdev(NEUROS_IR_MAJOR, "neuros_ir");
}

MODULE_AUTHOR("Neuros");
MODULE_DESCRIPTION(MOD_DESC);
MODULE_LICENSE("Neuros Technology LLC");

module_init(irrtc_init);
module_exit(irrtc_exit);
