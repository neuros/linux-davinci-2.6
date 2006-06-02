/*
 * linux/arch/arm/mach-davinci/time.c
 *
 * DaVinci timer subsystem
 *
 * Author: MontaVista Software, Inc. <source@mvista.com>
 *
 * Copyright 2005 (c) MontaVista Software, Inc. This file is licensed
 * under the terms of the GNU General Public License version 2. This
 * program is licensed "as is" without any warranty of any kind,
 * whether express or implied.
 *
 */
#include <linux/config.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/time.h>
#include <linux/timex.h>
#include <linux/types.h>
#include <linux/sched.h>
#include <linux/interrupt.h>

#include <asm/io.h>
#include <asm/hardware.h>
#include <asm/system.h>
#include <asm/leds.h>
#include <asm/irq.h>
#include <asm/mach/irq.h>
#include <asm/mach/time.h>

#include <asm/arch/timex.h>
#include <asm/arch/irqs.h>
#include <asm/errno.h>

enum {
	T0_BOT = 0, T0_TOP, T1_BOT, T1_TOP, NUM_TIMERS,
};
#define IS_TIMER1(id)    (id & 0x2)
#define IS_TIMER0(id)    (!IS_TIMER1(id))
#define IS_TIMER_TOP(id) ((id & 0x1))
#define IS_TIMER_BOT(id) (!IS_TIMER_TOP(id))

int timer_irqs[NUM_TIMERS] = {
	IRQ_TINT0_TINT12,
	IRQ_TINT0_TINT34,
	IRQ_TINT1_TINT12,
	IRQ_TINT1_TINT34,
};

/*
 * This driver configures the 2 64-bit DaVinci timers as 4 independent
 * 32-bit timers used as follows:
 *
 * T0_BOT: Timer 0, bottom:  free-running counter, used for cycle counter
 * T0_TOP: Timer 0, top   :  reserved for high-res timers
 * T1_BOT: Timer 1, bottom:  reserved for DSP
 * T1_TOP: Timer 1, top   :  Linux system tick
 */
#define TID_SYSTEM  T1_TOP
#define TID_FREERUN T0_BOT
#define TID_HRT     T0_TOP

/* timer regs */
typedef struct davinci_timer_regs_s {
	unsigned int pid12;		/* 0x0 */
	unsigned int emumgt_clksped;	/* 0x4 */
	unsigned int gpint_en;		/* 0x8 */
	unsigned int gpdir_dat;		/* 0xC */
	unsigned int tim12;		/* 0x10 */
	unsigned int tim34;		/* 0x14 */
	unsigned int prd12;		/* 0x18 */
	unsigned int prd34;		/* 0x1C */
	unsigned int tcr;		/* 0x20 */
	unsigned int tgcr;		/* 0x24 */
	unsigned int wdtcr;		/* 0x28 */
	unsigned int tlgc;		/* 0x2C */
	unsigned int tlmr;		/* 0x30 */
} davinci_timer_regs_t;

typedef struct davinci_timer_s {
	char *name;
	unsigned int id;
	unsigned long period;
	unsigned long opts;
	davinci_timer_regs_t *regs;
	struct irqaction irqaction;
} davinci_timer_t;
static davinci_timer_t davinci_timers[];

/* values for 'opts' field of davinci_timer_t */
#define TIMER_DISABLED   0x00
#define TIMER_ONE_SHOT   0x01
#define TIMER_CONTINUOUS 0x02

#define davinci_timer_base(id) \
  (IS_TIMER1(id) ? \
   (volatile davinci_timer_regs_t*)IO_ADDRESS(DAVINCI_TIMER1_BASE) :  \
   (volatile davinci_timer_regs_t*)IO_ADDRESS(DAVINCI_TIMER0_BASE))

/* cycles to nsec conversions taken from arch/i386/kernel/timers/timer_tsc.c,
 * converted to use kHz by Kevin Hilman */
/* convert from cycles(64bits) => nanoseconds (64bits)
 *  basic equation:
 *		ns = cycles / (freq / ns_per_sec)
 *		ns = cycles * (ns_per_sec / freq)
 *		ns = cycles * (10^9 / (cpu_khz * 10^3))
 *		ns = cycles * (10^6 / cpu_khz)
 *
 *	Then we use scaling math (suggested by george at mvista.com) to get:
 *		ns = cycles * (10^6 * SC / cpu_khz / SC
 *		ns = cycles * cyc2ns_scale / SC
 *
 *	And since SC is a constant power of two, we can convert the div
 *  into a shift.
 *			-johnstul at us.ibm.com "math is hard, lets go shopping!"
 */
static unsigned long cyc2ns_scale;
#define CYC2NS_SCALE_FACTOR 10 /* 2^10, carefully chosen */

static inline void set_cyc2ns_scale(unsigned long cpu_khz)
{
	cyc2ns_scale = (1000000 << CYC2NS_SCALE_FACTOR)/cpu_khz;
}

static inline unsigned long long cycles_2_ns(unsigned long long cyc)
{
	return (cyc * cyc2ns_scale) >> CYC2NS_SCALE_FACTOR;
}

static int davinci_timer32_config(davinci_timer_t *t) {
	volatile davinci_timer_regs_t *regs = t->regs;
	u32 enamode_shift, reset_shift;
	int ret = 0;

	if (IS_TIMER_BOT(t->id)) {
		regs->prd12 = t->period;
		enamode_shift = 6;
		reset_shift = 0;
	}
	else {
		regs->prd34 = t->period;
		enamode_shift = 22;
		reset_shift = 1;
	}

	/* reset timer */
	regs->tgcr &= ~(0x1 << reset_shift);

	/* Register interrupt */
	if (t->irqaction.handler != NULL) {
		ret = setup_irq(timer_irqs[t->id], &t->irqaction);
	}

	/* Set enable mode */
	if (t->opts & TIMER_ONE_SHOT) {
		regs->tcr |= 0x1 << enamode_shift;
	}
	else if (t->opts & TIMER_CONTINUOUS) {
		regs->tcr |= 0x2 << enamode_shift;
	}
	else { /* TIMER_DISABLED */
		regs->tcr &= ~(0x3 << enamode_shift);
	}

	/* unreset */
	regs->tgcr |= (0x1 << reset_shift);

	return ret;
}

static inline u32 davinci_timer32_read(davinci_timer_t *t) {
	volatile davinci_timer_regs_t *regs = t->regs;

	if IS_TIMER_TOP(t->id) {
		return regs->tim34;
	}
	else {
		return regs->tim12;
	}
}

/*
 * Last processed system timer interrupt
 */
static unsigned long davinci_timer32_last = 0;
static irqreturn_t system_timer_interrupt(int irq, void *dev_id, struct pt_regs *regs)
{
	unsigned long now, latency;

	write_seqlock(&xtime_lock);
	now = davinci_timer32_read(&davinci_timers[TID_FREERUN]);
	latency = davinci_timer32_read(&davinci_timers[TID_SYSTEM]);
	davinci_timer32_last = now - latency;

	/* Do the Linux timer operations */
	timer_tick(regs);
	write_sequnlock(&xtime_lock);

	return IRQ_HANDLED;
}

unsigned long davinci_gettimeoffset(void)
{
	unsigned long now, elapsed, nsec;

	now = davinci_timer32_read(&davinci_timers[TID_FREERUN]);
	elapsed = now - davinci_timer32_last;

	nsec = (unsigned long)cycles_2_ns(elapsed);
	return nsec / 1000;
}

static irqreturn_t freerun_interrupt(int irq, void *dev_id, struct pt_regs *regs) {
	/* TODO: keep track of roll-overs for 64-bit cycle-count */
	return IRQ_HANDLED;
}

cycles_t davinci_get_cycles(void) {
	return davinci_timer32_read(&davinci_timers[TID_FREERUN]);
}

static davinci_timer_t davinci_timers[NUM_TIMERS] = {
	[TID_SYSTEM] = {
		.name      = "system tick",
		.period    = (CLOCK_TICK_RATE / (HZ)),
		.opts      = TIMER_CONTINUOUS,
		.irqaction = {
			.flags   = SA_INTERRUPT,
			.handler = system_timer_interrupt,
		}
	},
	[TID_FREERUN] = {
		.name       = "free-run counter",
		.period     = 0xffffffff,
		.opts       = TIMER_CONTINUOUS,
		.irqaction = {
			.flags   = SA_INTERRUPT,
			.handler = freerun_interrupt,
		}
	},
};

void __init davinci_timer_init(void)
{
	volatile davinci_timer_regs_t *t0 = davinci_timer_base(T0_BOT);
	volatile davinci_timer_regs_t *t1 = davinci_timer_base(T1_BOT);
	int i;

	/* Disabled, Internal clock source */
	t0->tcr = 0x0;
	t1->tcr = 0x0;

	/* reset both timers, no pre-scaler for timer34 */
	t0->tgcr = 0;
	t1->tgcr = 0;

	/* Set both timers to unchained 32-bit */
	t0->tgcr |= 0x4;
	t1->tgcr |= 0x4;

	/* Unreset timers */
	t0->tgcr |= 0x3;
	t1->tgcr |= 0x3;

	/* Init both counters to zero */
	t0->tim12 = 0;
	t0->tim34 = 0;
	t1->tim12 = 0;
	t1->tim34 = 0;

	set_cyc2ns_scale(CLOCK_TICK_RATE / 1000);

	for(i=0; i<sizeof(davinci_timers)/sizeof(davinci_timer_t); i++) {
		davinci_timer_t *t = &davinci_timers[i];

		if (t->name) {
			t->id = i;
			t->regs =
			    (davinci_timer_regs_t *)davinci_timer_base(t->id);
			t->irqaction.name = t->name;
			t->irqaction.dev_id = (void *)t;

			davinci_timer32_config(&davinci_timers[i]);
		}
	}
}

struct sys_timer davinci_timer = {
	.init   = davinci_timer_init,
	.offset = davinci_gettimeoffset,
};


void davinci_watchdog_reset(void) {
	volatile davinci_timer_regs_t *davinci_wdt = (davinci_timer_regs_t *)IO_ADDRESS(DAVINCI_WDOG_BASE);

        davinci_wdt->tgcr = 0x8;
        davinci_wdt->tgcr |= 0x3;

	davinci_wdt->tim12 = 0;
	davinci_wdt->tim34 = 0;
	davinci_wdt->prd12 =  0;
	davinci_wdt->prd34 =  0;
	davinci_wdt->wdtcr |= 0x4000;
	davinci_wdt->tcr |= 0x40;
	davinci_wdt->wdtcr = 0xA5C64000;
	davinci_wdt->wdtcr = 0xDA7E4000;
}
