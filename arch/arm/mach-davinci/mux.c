/*
 * DaVinci pin multiplexing configurations
 *
 * Author: Vladimir Barinov, MontaVista Software, Inc. <source@mvista.com>
 *
 * 2007 (c) MontaVista Software, Inc. This file is licensed under
 * the terms of the GNU General Public License version 2. This program
 * is licensed "as is" without any warranty of any kind, whether express
 * or implied.
 */
#include <linux/io.h>
#include <linux/module.h>
#include <linux/spinlock.h>

#include <asm/hardware.h>

#include <asm/arch/mux.h>

static DEFINE_SPINLOCK(mux_lock);

void davinci_mux_peripheral(unsigned int mux, unsigned int enable)
{
	u32 pinmux, muxreg = PINMUX0;

	if (mux >= DAVINCI_MUX_LEVEL2) {
		muxreg = PINMUX1;
		mux -= DAVINCI_MUX_LEVEL2;
	}

	spin_lock(&mux_lock);
	pinmux = davinci_readl(DAVINCI_SYSTEM_MODULE_BASE + muxreg);
	if (enable)
		pinmux |= (1 << mux);
	else
		pinmux &= ~(1 << mux);
	davinci_writel(pinmux, DAVINCI_SYSTEM_MODULE_BASE + muxreg);
	spin_unlock(&mux_lock);
}
EXPORT_SYMBOL(davinci_mux_peripheral);
