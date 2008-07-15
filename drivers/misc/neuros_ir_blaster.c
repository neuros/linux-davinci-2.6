/*
 *  Copyright(C) 2006-2008 Neuros Technology International LLC. 
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
 * IR Blaster Receiver driver. 
 *
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/poll.h>
#include <linux/interrupt.h>
#include <asm/io.h>
#include <asm/arch/irqs.h>

#include <linux/neuros_ir.h>
#include <linux/neuros_ir_blaster.h>

#define MOD_DESC "Neuros IR Blaster Driver (c) 2008"

#if 0
    #define dbg(fmt, arg...) \
        printk(KERN_INFO "%s:%d> " fmt, __func__, __LINE__ , ## arg)
#else
    #define dbg(fmt, arg...)
#endif

#define WAIT_BLASTER_TIME 10
#define DELAY_FOR_IR 200

#define LEANRING_COMPLETE_DELAY (HZ)
#define POLL_RELEASE_DELAY (HZ/20)
#define KEY_WAVE_PRESENT 1
#define WAIT_LEARN_COMPLETE 2
#define WAIT_RELEASE_REMOTE 4
#define BLS_TIMER_PRESCALE 15
#define BLASTER 0
#define CAPTURE 1
#define INT_TIMER1_TIME_WASTE 3
#define INT_CAPTURE_TIME_WASTE 2
#define MAX_COUNTER (0xffffffff)
#define WAIT_HARDWARE_RESET 50 //50ms
#define GIO_BLS (1<<15)
#define GIO_CAP (1<<7)
static int learning_status = 0;
static int bls_wave_count;
static int bls_status = BLS_COMPLETE;
static int timer_int_counter = 0;
static int wave_len = 0;
static int int_type;
static struct timer_list learning_key_timer;
static struct blaster_data_type *BlsKey;
static struct blaster_data_pack *bls_data_pack;

struct irrtc_device
{
    int    key;
};
static struct irrtc_device device;
static wait_queue_head_t wait;

extern void set_osd_key(int);
extern int get_osd_key(void);
extern void lock_data_protect(void);
extern void unlock_data_protect(void);
extern void set_factory_test(int);
extern void report_key(int key);
extern void set_is_learning(int value);

static void enable_learning(void)
{
    enable_irq(IRQ_GPIO7);
    lock_data_protect();
    set_is_learning(1);
    unlock_data_protect();
}

static void disable_learning(void)
{
    disable_irq(IRQ_GPIO7);
    lock_data_protect();
    set_is_learning(0);
    unlock_data_protect();
}

static void set_timer1_div(struct blaster_data_pack *blsdat)
{
    unsigned int div;
    int count;
    static int sbit;
    if (wave_len)
    {
        div = wave_len;
        if (div > MAX_COUNTER)
            div = MAX_COUNTER;
        wave_len -= div;
    }
    else
    {
        count = blsdat->bitstimes&BITS_COUNT_MASK;
        if (count == bls_wave_count)
            sbit = 0;
        if (keybit_get(blsdat->mbits, (count - bls_wave_count)))
        {
            if (keybit_get(blsdat->dbits, (count - bls_wave_count)))
            {
                div = blsdat->specbits[sbit];
                sbit++;
            }
            else
            {
                div = blsdat->bit2;
            }
        }
        else
        {
            if (keybit_get(blsdat->dbits, (count - bls_wave_count)))
            {
                div = blsdat->bit1;
            }
            else
            {
                div = blsdat->bit0;
            }
        }
    }
    //div-=div>>5;   // The ideal situation is that set the timer prescale to let the timer cycle is 1us 
    // but the nearest setting make the cycle 1.032us so do this adjustment with 
    // divider value
    div -= INT_TIMER1_TIME_WASTE;
    if (div > MAX_COUNTER)
    {
        wave_len = div - MAX_COUNTER;
        div = MAX_COUNTER;
    }
    div--;
    TIMER1_TCR &= ~(3<<22); //disable timer1 34
    TIMER1_TGCR &= ~(1<<1); //reset timer1 34
    TIMER1_TGCR &= ~(3<<2); //clear timer1 mode
    TIMER1_TGCR |= (1<<2); //set timer1 unchained mode
    TIMER1_TGCR |= (1<<1); //set timer1 34 in used
    TIMER1_TIM34 = 0;
    TIMER1_PRD34 = div; // set timer period
    TIMER1_TGCR &= ~(15<<8); //clean the prescale value
    TIMER1_TGCR |= (BLS_TIMER_PRESCALE<<8); //set prescale value
    TIMER1_TCR |= (1<<22); //enable timer1 34 as one time mode
}

static void blaster_key(struct blaster_data_pack* blsdat)
{ 
    uint16_t bitset2;
    wave_len = 0;
    bls_status = BLS_START;
    bls_wave_count = blsdat->bitstimes & BITS_COUNT_MASK;
    if (bls_wave_count > BLASTER_MAX_CHANGE || bls_wave_count <= 0)
    {
        bls_status = BLS_ERROR;
        if (bls_data_pack)
        {
            kfree(bls_data_pack);
            bls_data_pack = NULL;
        }
        return;
    }
    /*check if the io port status correct if not correct set it's logic to reverse of start level and hold for a momemt*/
    bitset2 = GPIO23_OUT_DATA;
    if (((bitset2 & GIO_BLS) != 0) && ((bls_data_pack->bitstimes & FIRST_LEVEL_BIT_MASK) != 0))
    {
        GPIO23_CLR_DATA |= GIO_BLS;
        msleep(WAIT_HARDWARE_RESET);
    }
    else if (((bitset2 & GIO_BLS) == 0) && ((bls_data_pack->bitstimes & FIRST_LEVEL_BIT_MASK) == 0))
    {
        GPIO23_SET_DATA |= GIO_BLS;
        msleep(WAIT_HARDWARE_RESET);
    }
    if (blsdat->bitstimes & FIRST_LEVEL_BIT_MASK)
        GPIO23_SET_DATA |= GIO_BLS;
    else
        GPIO23_CLR_DATA |= GIO_BLS;
    set_timer1_div(blsdat);
    enable_irq(IRQ_TINT1_TINT34);
}

void timer_handle(unsigned long data)
{
    int osd_key;
    dbg("bitstimes=%d\n", BlsKey->bitstimes);
    if (learning_status & WAIT_LEARN_COMPLETE)
    {
        lock_data_protect();
        osd_key=get_osd_key();
        unlock_data_protect();
        if (!osd_key )
        {
            disable_irq(IRQ_TINT1_TINT34);
            TIMER1_TCR &= ~(3<<22); //disable timer1 34
            TIMER1_TGCR &= ~(1<<1); //reset timer1 34
            report_key(LEARNING_COMPLETE_KEY);
            //report_key(UP_KEY);
        }
        learning_status |= WAIT_RELEASE_REMOTE;
        learning_status &= ~WAIT_LEARN_COMPLETE;
        learning_key_timer.function = timer_handle;
        mod_timer(&learning_key_timer, jiffies + POLL_RELEASE_DELAY);
    }
    else if (learning_status & WAIT_RELEASE_REMOTE)
    {
        if (learning_status & KEY_WAVE_PRESENT)
        {
            learning_status &= ~KEY_WAVE_PRESENT;
            learning_key_timer.function = timer_handle;
            mod_timer(&learning_key_timer, jiffies + POLL_RELEASE_DELAY);
        }
        else
        {
            disable_learning();
            lock_data_protect();
            osd_key = get_osd_key();
            unlock_data_protect();
            if (!osd_key )
            {
                report_key(RELEASE_REMOTE_KEY);
                //report_key(UP_KEY);
            }
        }
    }
}

static int capture_key(struct blaster_data_type* blsdat)
{
    static int times = 0;
    static int old_counter;
    static int old_int_counter;
    int counter;
    int td;

    if (!(learning_status & WAIT_RELEASE_REMOTE))
    {
        counter = TIMER1_TIM34;
        if (old_int_counter == timer_int_counter)
            td = counter - old_counter;
        else
            td = MAX_COUNTER - old_counter - INT_CAPTURE_TIME_WASTE + counter ;
        old_counter = counter;
        old_int_counter = timer_int_counter;
        if (learning_status == 0)
        {
            times = 0;
            learning_key_timer.function = timer_handle;
            mod_timer(&learning_key_timer, jiffies + LEANRING_COMPLETE_DELAY);
            learning_status |= WAIT_LEARN_COMPLETE;
        }

        if (0 == times++)
        {
            if ((GPIO01_IN_DATA & GIO_CAP) == 0)
                blsdat->bitstimes |= FIRST_LEVEL_BIT_MASK;
            return 0;
        }
        blsdat->bits[times-2] = td;
        blsdat->bitstimes++;
        if (times == BLASTER_MAX_CHANGE+1)
        {
            disable_irq(IRQ_TINT1_TINT34);
            TIMER1_TCR &= ~(3<<22); //disable timer1 34
            TIMER1_TGCR &= ~(1<<1); //reset timer1 34
            report_key(LEARNING_COMPLETE_KEY);
            learning_status |= WAIT_RELEASE_REMOTE;
            learning_status &= ~WAIT_LEARN_COMPLETE;
            learning_key_timer.function = timer_handle;
            mod_timer(&learning_key_timer, jiffies + POLL_RELEASE_DELAY);
        }
    }
    else
        learning_status |= KEY_WAVE_PRESENT;
    return(0);
}

static irqreturn_t handle_bls_timer1_irqs(int irq, void * dev_id)
{
    uint32_t bitset2;
    if (int_type == CAPTURE)
    {
        timer_int_counter++;
        return IRQ_HANDLED;
    }
    if (bls_wave_count == 0)
        return IRQ_HANDLED;
    if (!wave_len)
    {
        bitset2 = GPIO23_OUT_DATA;
        if (bitset2 & GIO_BLS)
            GPIO23_CLR_DATA |= GIO_BLS;
        else
            GPIO23_SET_DATA |= GIO_BLS;
        bls_wave_count--;
    }
    if (bls_wave_count == 0)
    {
        disable_irq(IRQ_TINT1_TINT34);
        TIMER1_TCR &= ~(3<<22); //disable timer1 34
        bls_status = BLS_COMPLETE;
        if (bls_data_pack)
        {
            kfree(bls_data_pack);
            bls_data_pack = NULL;
        }
        return IRQ_HANDLED;
    }
    else
    {
        set_timer1_div(bls_data_pack);
    }
    return IRQ_HANDLED;
}

static irqreturn_t handle_capture_irqs(int irq, void * dev_id, struct pt_regs * regs)
{
    capture_key(BlsKey);
    return IRQ_HANDLED;
}

//--------------------------------------------------- DEVICE --------------------------------------------------------------

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

static int irrtc_ioctl(struct inode * inode, struct file * file,
                       unsigned int cmd, unsigned long arg)
{
    int ret = 0;
    int arg_size;
    if (_IOC_TYPE(cmd) != NEUROS_IR_BLASTER_IOC_MAGIC)
    {
        ret = -EINVAL;
        goto bail;
    }

    arg_size = _IOC_SIZE(cmd);
    if (_IOC_DIR(cmd) & _IOC_READ)
        ret = !access_ok(VERIFY_WRITE, (void *)arg, arg_size);
    else if (_IOC_DIR(cmd) & _IOC_WRITE)
        ret = !access_ok(VERIFY_READ, (void *)arg, arg_size);
    if (ret) goto bail;
    switch (cmd)
    {
    case RRB_BLASTER_KEY:
        {
            if (bls_status != BLS_START)
            {
                int_type = BLASTER;
                bls_data_pack = kmalloc(sizeof(struct blaster_data_pack), GFP_KERNEL);
                if (bls_data_pack == NULL)
                {
                    ret = -EINVAL;
                    break;
                }
                copy_from_user(bls_data_pack, (void*)arg, sizeof(struct blaster_data_pack));
                blaster_key(bls_data_pack);
            }
        }
        break;
    case RRB_CAPTURE_KEY:
        {
            BlsKey = kmalloc(sizeof(struct blaster_data_type), GFP_KERNEL);
            if (BlsKey == NULL)
            {
                ret = -EINVAL;
                break;
            }
            memset(BlsKey, 0, sizeof(struct blaster_data_type));
            set_osd_key(0);
            int_type = CAPTURE;
            timer_int_counter = 0;
            learning_status = 0;
            TIMER1_TGCR &= ~(3<<2); //clear timer1 mode
            TIMER1_TGCR |= (1<<2); //set timer1 unchained mode
            TIMER1_TGCR &= ~(1<<1); //reset timer1 34
            TIMER1_TGCR |= (1<<1); //set timer1 34 in used
            TIMER1_PRD34 = MAX_COUNTER; // set timer period 
            TIMER1_TGCR &= ~(15<<8); //clean the prescale value
            TIMER1_TGCR |= (BLS_TIMER_PRESCALE<<8); //set prescale value
            TIMER1_TCR &= ~(3<<22); //disable timer1 34
            TIMER1_TCR |= (2<<22); //enable timer1 34 as continuous mode
            enable_irq(IRQ_TINT1_TINT34);
            enable_learning();
        }
        break;
    case RRB_READ_LEARNING_DATA:
        {
            if (BlsKey)
            {
                ret |= copy_to_user((void*)arg, BlsKey, sizeof(struct blaster_data_type));
                kfree(BlsKey);
                BlsKey = NULL;
            }
        }
        break;
    case RRB_FACTORY_TEST:
        {
            set_factory_test(1);
        }
        break;
    case RRB_GET_BLASTER_STATUS:
        {
            ret |= copy_to_user((void*)arg, &bls_status, sizeof(bls_status));
        }
        break;

    default:
        ret = -EINVAL;
        break;
    }

bail:
    return ret;
}

static struct file_operations irrtc_fops = {
    .open    = irrtc_open,
    .release = irrtc_release,
    .ioctl   = irrtc_ioctl,
};

//-------------------------------------------------- INIT / EXIT ----------------------------------------------------------

static const char * pname = "NEUROS_IRBLAST(KM):";

static int blaster_init( void )
{
    int ret;
    PINMUX1 |= (1<<4);  //set gio45/PWM0 pin works as PWM0
    PWM0_PCR = 1;
    PWM0_CFG &= ~2; //clear PWM0 mode
    PWM0_CFG |= 2; //set PWM0 contiguous mode
    /* PWM0_PER+1=PWM0_PH1D*2 */
    /* PWM config PWM 38kHz PER=709 PH1D=355 */
    PWM0_PER = 709;
    PWM0_PH1D = 355;
    PWM0_START = 1;
    GPIO23_DIR &= ~GIO_BLS;  //gio 47 direction output
    GPIO23_CLR_DATA |= GIO_BLS; //drive the gpio 47 to low
    TIMER1_TCR &= ~(3<<22); //disable timer1 34
    ret = request_irq(IRQ_TINT1_TINT34, handle_bls_timer1_irqs,SA_INTERRUPT , "ir_blaster_timer1", &device); //TIMER__INTERRUPT
    disable_irq(IRQ_TINT1_TINT34);

    GPIO01_DIR |= GIO_CAP;  //gpio 7 direction input
    SET_GPIO01_RIS_INT |= GIO_CAP; // gpio 7 rising edge IRQ enable
    SET_GPIO01_FAL_INT |= GIO_CAP; // gpio 7 falling edge IRQ enable
    request_irq(IRQ_GPIO7, handle_capture_irqs,SA_INTERRUPT , "ir_capture", &device); //SA_SHIRQSA_INTERRUPT
    disable_learning();
}

static int __init irrtc_init(void)
{
    int status = 0;
    init_timer(&learning_key_timer);
    init_waitqueue_head (&wait);

    printk(KERN_INFO "\t" MOD_DESC "\n");

    status = register_chrdev(NEUROS_IR_BLASTER_MAJOR, "neuros_ir_blaster", &irrtc_fops);
    if (status != 0)
    {
        if (status == -EINVAL) printk(KERN_ERR "%s Couldn't register device: invalid major number %d.\n", pname, NEUROS_IR_BLASTER_MAJOR);
        else if (status == -EBUSY) printk(KERN_ERR "%s Couldn't register device: major number %d already busy.\n", pname, NEUROS_IR_BLASTER_MAJOR);
        else printk(KERN_ERR "%s Couldn't register device: error %d.\n", pname, status);
        status = -1;
        goto out;
    }

    blaster_init();

out:
    return status;
}

static void __exit irrtc_exit(void)
{
    //~ irqs_irrtc_exit();

    unregister_chrdev(NEUROS_IR_BLASTER_MAJOR, "ir_blaster");
}

MODULE_AUTHOR("Neuros");
MODULE_DESCRIPTION(MOD_DESC);
MODULE_LICENSE("Neuros Technology LLC");

module_init(irrtc_init);
module_exit(irrtc_exit);


