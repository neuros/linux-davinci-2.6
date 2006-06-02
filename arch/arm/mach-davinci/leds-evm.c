/*
 * <arch/arm/mach-davinci/leds-evm.c>
 *
 * LED support for TI DaVinci EVM
 *
 * 2006 (c) Texas Instruments, Inc. This file is licensed under
 * the terms of the GNU General Public License version 2. This program
 * is licensed "as is" without any warranty of any kind, whether express
 * or implied.
 */

#include <linux/config.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/types.h>
#include <linux/i2c.h>
#include <linux/errno.h>

#include <asm/mach-types.h>
#include <asm/hardware.h>
#include <asm/leds.h>
#include <asm/system.h>


/* eight leds on a pcf8574a I2C gpio expander; 0 == ON, 1 == OFF
 *  - drivers can use leds_event(led_{green,amber,red,blue}_{on,off})
 *  - userspace can do the same with /sys/devices/leds/leds0/event
 */
#define	LED_DS8			(1 << 0)
#define	LED_DS7			(1 << 1)
#define	LED_DS6			(1 << 2)
#define	LED_DS5			(1 << 3)
#define	LED_DS4			(1 << 4)
#define	LED_DS3			(1 << 5)
#define	LED_DS2			(1 << 6)
#define	LED_DS1			(1 << 7)

#define LED_STATE_ENABLED	(1 << 8)
#define LED_STATE_CLAIMED	(1 << 9)

static u16	hw_led_state;
static u8	leds_change;


/* these leds use I2C not GPIO, so we can't change values
 * and remain "idle" ... so there's no "idle" LED.
 */
#define	TIMER_LED		LED_DS8

#define	GREEN_LED		LED_DS1
#define	AMBER_LED		LED_DS2
#define	RED_LED			LED_DS3
#define	BLUE_LED		LED_DS4

#define	APP_LEDS	(GREEN_LED | AMBER_LED | RED_LED | BLUE_LED)

static DEFINE_SPINLOCK(lock);


#define	EVM_I2C_ADDR	0x38

#ifdef CONFIG_PREEMPT_RT
static void pcf_work(unsigned long unused)
#else
static void pcf_work(void *unused)
#endif
{
	struct i2c_adapter	*adap;
	int			err;
	struct i2c_msg		msg;

	adap = i2c_get_adapter(0);
	if (!adap)
		return;

	for (;;) {
		static u8	leds;

		spin_lock_irq(&lock);
		leds = (u8) hw_led_state;
		err= leds_change;
		leds_change = 0;
		spin_unlock_irq(&lock);

		if (!err)
			break;

		msg.addr = EVM_I2C_ADDR;
		msg.flags = 0;
		msg.len = 1;
		msg.buf = &leds;
		err = i2c_transfer(adap, &msg, 1);
		if (err < 0)
			pr_debug("LED: set to %02x, err %d\n", leds, err);
	}

}

/* Under PREEMPT_RT, this code may be called from the timer interrupt
 * with is a real hard IRQ.  Therefore, workqueues cannot be used
 * because 'schedule_work()' can sleep.  OTOH, for non-RT, tasklets
 * cannot be used to call sleeping functions (such as i2c stack) */

#ifdef CONFIG_PREEMPT_RT
static DECLARE_TASKLET(work, pcf_work, 0);
#define doit() tasklet_schedule(&work)
#else
static DECLARE_WORK(work, pcf_work, NULL);
#define doit() schedule_work(&work)
#endif

static void evm_leds_event(led_event_t evt)
{
	unsigned long	flags;
	u16		leds;

	spin_lock_irqsave(&lock, flags);

	if (!(hw_led_state & LED_STATE_ENABLED) && evt != led_start)
		goto done;

	leds = hw_led_state;
	switch (evt) {
	case led_start:
		hw_led_state = LED_STATE_ENABLED | 0xff;
		leds = 0;
		break;

	case led_halted:
	case led_stop:
		hw_led_state = 0xff;
		// NOTE:  work may still be pending!!
		break;

	case led_claim:
		hw_led_state |= LED_STATE_CLAIMED;
		hw_led_state |= APP_LEDS;
		break;

	case led_release:
		hw_led_state &= ~LED_STATE_CLAIMED;
		hw_led_state |= APP_LEDS;
		break;

#ifdef	CONFIG_LEDS_TIMER
	case led_timer:
		hw_led_state ^= TIMER_LED;
		break;
#endif

	/* actually all the LEDs are green */

	case led_green_on:
		if (leds & LED_STATE_CLAIMED)
			hw_led_state &= ~GREEN_LED;
		break;
	case led_green_off:
		if (leds & LED_STATE_CLAIMED)
			hw_led_state |= GREEN_LED;
		break;

	case led_amber_on:
		if (leds & LED_STATE_CLAIMED)
			hw_led_state &= ~AMBER_LED;
		break;
	case led_amber_off:
		if (leds & LED_STATE_CLAIMED)
			hw_led_state |= AMBER_LED;
		break;

	case led_red_on:
		if (leds & LED_STATE_CLAIMED)
			hw_led_state &= ~RED_LED;
		break;
	case led_red_off:
		if (leds & LED_STATE_CLAIMED)
			hw_led_state |= RED_LED;
		break;

	case led_blue_on:
		if (leds & LED_STATE_CLAIMED)
			hw_led_state &= ~BLUE_LED;
		break;
	case led_blue_off:
		if (leds & LED_STATE_CLAIMED)
			hw_led_state |= BLUE_LED;
		break;

	default:
		break;
	}

	leds ^= hw_led_state;
	if (leds & 0xff) {
		leds_change = (u8) leds;
		doit();
	}

done:
	spin_unlock_irqrestore(&lock, flags);
}

static int __init evm_leds_init(void)
{
	if (!machine_is_davinci_evm())
		return 0;

	leds_event = evm_leds_event;
	leds_event(led_start);
	return 0;
}

/* i2c is subsys_initcall, davinci i2c is device_initcall;
 * this needs to follow both of them (sigh)
 */
late_initcall(evm_leds_init);
