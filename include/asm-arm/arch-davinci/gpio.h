/* include/asm-arm/arch-davinci/gpio.h */

#ifndef	__DAVINCI_GPIO_H
#define	__DAVINCI_GPIO_H

/*
 * basic gpio routines
 *
 * board-specific init should be done by arch/.../.../board-XXX.c (maybe
 * initializing banks together) rather than boot loaders; kexec() won't
 * go through boot loaders.
 *
 * the gpio clock will be turned on when gpios are used, and you may also
 * need to pay attention to PINMUX0 and PINMUX1 to be sure those pins are
 * used as gpios, not with other peripherals.
 *
 * GPIOs are numbered 0..(DAVINCI_N_GPIO-1).  For documentation, and maybe
 * for later updates, code should write GPIO(N) or:
 *  - GPIOV18(N) for 1.8V pins, N in 0..53; same as GPIO(0)..GPIO(53)
 *  - GPIOV33(N) for 3.3V pins, N in 0..17; same as GPIO(54)..GPIO(70)
 *
 * GPIO IRQs all use DAVINCI_GPIO_IRQ(N) or DAVINCI_GPIO_IRQ(GPIOV33(N)) etc
 * for now, that's != GPIO(N)
 */
#define	GPIO(X)		(X)		/* 0 <= X <= 71 */
#define	GPIOV18(X)	(X)		/* 1.8V i/o; 0 <= X <= 53 */
#define	GPIOV33(X)	((X)+54)	/* 3.3V i/o; 0 <= X <= 17 */


struct gpio_controller {
	u32	dir;
	u32	out_data;
	u32	set_data;
	u32	clr_data;
	u32	in_data;
	u32	set_rising;
	u32	clr_rising;
	u32	set_falling;
	u32	clr_falling;
	u32	intstat;
};


/* The gpio_to_controller() and gpio_mask() functions inline to constants
 * with constant parameters; or in outlined code they execute at runtime.
 *
 * You'd access the controller directly when reading or writing more than
 * one gpio value at a time, and to support wired logic where the value
 * being driven by the cpu need not match the value read back.
 */
static inline struct gpio_controller *__iomem
gpio_to_controller(unsigned gpio)
{
	void *__iomem ptr;

	if (gpio < 32)
		ptr = (void *__iomem) IO_ADDRESS(DAVINCI_GPIO_BASE + 0x10);
	else if (gpio < 64)
		ptr = (void *__iomem) IO_ADDRESS(DAVINCI_GPIO_BASE + 0x38);
	else if (gpio < DAVINCI_N_GPIO)
		ptr = (void *__iomem) IO_ADDRESS(DAVINCI_GPIO_BASE + 0x60);
	else
		ptr = NULL;
	return ptr;
}

static inline u32 gpio_mask(unsigned gpio)
{
	return 1 << (gpio % 32);
}


/* The get/set/clear functions will inline when called with constant
 * parameters, for low-overhead bitbanging.  Illegal constant parameters
 * cause link-time errors.
 *
 * Otherwise, calls with variable parameters use outlined functions.
 */
extern int __error_inval_gpio(void);

extern int __gpio_set(unsigned gpio, int value);
extern int __gpio_get(unsigned gpio);

static inline int gpio_set(unsigned gpio)
{
	struct gpio_controller	*__iomem g;

	if (!__builtin_constant_p(gpio))
		return __gpio_set(gpio, 1);

	if (gpio >= DAVINCI_N_GPIO)
		return __error_inval_gpio();

	g = gpio_to_controller(gpio);
	__raw_writel(gpio_mask(gpio), &g->set_data);
	return 0;
}

static inline int gpio_clear(unsigned gpio)
{
	struct gpio_controller	*__iomem g;

	if (!__builtin_constant_p(gpio))
		return __gpio_set(gpio, 1);

	if (gpio >= DAVINCI_N_GPIO)
		return __error_inval_gpio();

	g = gpio_to_controller(gpio);
	__raw_writel(gpio_mask(gpio), &g->clr_data);
	return 0;
}

static inline int gpio_set_value(unsigned gpio, int value)
{
	if (__builtin_constant_p(value)) {
		if (value)
			return gpio_set(gpio);
		else
			return gpio_clear(gpio);
	}
	if (__builtin_constant_p(gpio) && gpio >= DAVINCI_N_GPIO)
		return __error_inval_gpio();

	return __gpio_set(gpio, value);
}

/* Returns zero or nonzero, or negative on error; works for gpios
 * configured as inputs OR as outputs.
 *
 * NOTE: changes in reported values are synchronized to the GPIO clock.
 * This is most easily seen after calls to gpio_set() and gpio_clear(),
 * where set-then-get might return the old value.
 */
static inline int gpio_get_value(unsigned gpio)
{
	struct gpio_controller	*__iomem g;

	if (!__builtin_constant_p(gpio))
		return __gpio_get(gpio);

	if (gpio >= DAVINCI_N_GPIO)
		return __error_inval_gpio();

	g = gpio_to_controller(gpio);
	return !!(gpio_mask(gpio) & __raw_readl(&g->in_data));
}

/* powerup default direction is IN */
extern int __init gpio_set_direction(unsigned gpio, int is_in);


/* NOTE:  currently there's no "claim/release" mechanism for GPIOs,
 * so drivers arguing over them will get errors much like they will
 * when the pin isn't muxed properly as gpio ...
 */
#endif	/* __DAVINCI_GPIO_H */
