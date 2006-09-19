/*
 * TI DAVINCI I2C adapter driver.
 *
 * Copyright (C) 2006 Texas Instruments.
 *
 * Updated by Vinod & Sudhakar Feb 2005
 *
 * ----------------------------------------------------------------------------
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 * ----------------------------------------------------------------------------
 *
 */
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/clk.h>
#include <linux/errno.h>
#include <linux/sched.h>
#include <linux/err.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>

#include <asm/io.h>
#include <asm/hardware.h>

MODULE_AUTHOR("Texas Instruments India");
MODULE_DESCRIPTION("TI DaVinci I2C bus adapter");
MODULE_LICENSE("GPL");

static int bus_freq;
module_param(bus_freq, int, 0);
MODULE_PARM_DESC(bus_freq,
  "Set I2C bus frequency in KHz: 100 (Standard Mode) or 400 (Fast Mode)");

/* ----- global defines ----------------------------------------------- */

#define DAVINCI_I2C_TIMEOUT (1*HZ)
#define I2C_DAVINCI_INTR_ALL    (DAVINCI_I2C_IMR_AAS | \
				 DAVINCI_I2C_IMR_SCD | \
				 /*DAVINCI_I2C_IMR_XRDY | */\
				 /*DAVINCI_I2C_IMR_RRDY | */\
				 /*DAVINCI_I2C_IMR_ARDY | */\
				 DAVINCI_I2C_IMR_NACK | \
				 DAVINCI_I2C_IMR_AL)

#define DAVINCI_I2C_OAR_REG	0x00
#define DAVINCI_I2C_IMR_REG	0x04
#define DAVINCI_I2C_STR_REG	0x08
#define DAVINCI_I2C_CLKL_REG	0x0c
#define DAVINCI_I2C_CLKH_REG	0x10
#define DAVINCI_I2C_CNT_REG	0x14
#define DAVINCI_I2C_DRR_REG	0x18
#define DAVINCI_I2C_SAR_REG	0x1c
#define DAVINCI_I2C_DXR_REG	0x20
#define DAVINCI_I2C_MDR_REG	0x24
#define DAVINCI_I2C_IVR_REG	0x28
#define DAVINCI_I2C_EMDR_REG	0x2c
#define DAVINCI_I2C_PSC_REG	0x30

#define DAVINCI_I2C_IVR_AAS	(0x07)
#define DAVINCI_I2C_IVR_SCD	(0x06)
#define DAVINCI_I2C_IVR_XRDY	(0x05)
#define DAVINCI_I2C_IVR_RDR	(0x04)
#define DAVINCI_I2C_IVR_ARDY	(0x03)
#define DAVINCI_I2C_IVR_NACK	(0x02)
#define DAVINCI_I2C_IVR_AL	(0x01)

#define DAVINCI_I2C_STR_BB	(1 << 12)
#define DAVINCI_I2C_STR_RSFULL	(1 << 11)
#define DAVINCI_I2C_STR_SCD	(1 << 5)
#define DAVINCI_I2C_STR_ARDY	(1 << 2)
#define DAVINCI_I2C_STR_NACK	(1 << 1)
#define DAVINCI_I2C_STR_AL	(1 << 0)

#define DAVINCI_I2C_MDR_STT	(1 << 13)
#define DAVINCI_I2C_MDR_STP	(1 << 11)
#define DAVINCI_I2C_MDR_MST	(1 << 10)
#define DAVINCI_I2C_MDR_TRX	(1 << 9)
#define DAVINCI_I2C_MDR_XA	(1 << 8)
#define DAVINCI_I2C_MDR_IRS	(1 << 5)

#define DAVINCI_I2C_IMR_AL	(1 << 0)
#define DAVINCI_I2C_IMR_NACK	(1 << 1)
#define DAVINCI_I2C_IMR_RRDY	(1 << 3)
#define DAVINCI_I2C_IMR_XRDY	(1 << 4)
#define DAVINCI_I2C_IMR_SCD	(1 << 5)
#define DAVINCI_I2C_IMR_AAS	(1 << 6)

#define MOD_REG_BIT(val, mask, set) do { \
	if (set) \
		val |= mask; \
	else \
		val &= ~mask; \
} while(0)

/* Following are the default values for the module parameters */
static int bus_freq = 20; /*  Fast Mode = 400 KHz, Standard Mode = 100 KHz */

struct davinci_i2c_dev {
	struct device           *dev;
	void __iomem		*base;
	struct completion	cmd_complete;
	struct clk              *clk;
	int			cmd_err;
	u8			*buf;
	size_t			buf_len;
	int			irq;
	struct i2c_adapter	adapter;
};

static inline void davinci_i2c_write_reg(struct davinci_i2c_dev *i2c_dev,
				      int reg, u16 val)
{
	__raw_writew(val, i2c_dev->base + reg);
}

static inline u16 davinci_i2c_read_reg(struct davinci_i2c_dev *i2c_dev, int reg)
{
	return __raw_readw(i2c_dev->base + reg);
}


/*
 * This functions configures I2C and brings I2C out of reset.
 * This function is called during I2C init function. This function
 * also gets called if I2C encounetrs any errors. Clock calculation portion
 * of this function has been taken from some other driver.
 */
static int i2c_davinci_init(struct davinci_i2c_dev *dev)
{
	u16 psc;
	u32 clk, clkh, clkl;
	u32 input_clock = clk_get_rate(dev->clk);
	u16 w;

	/* put I2C into reset */
	w = davinci_i2c_read_reg(dev, DAVINCI_I2C_MDR_REG);
	MOD_REG_BIT(w, DAVINCI_I2C_MDR_IRS, 0);
	davinci_i2c_write_reg(dev, DAVINCI_I2C_MDR_REG, w);

        /* NOTE: I2C Clock divider programming info
	 * As per I2C specs the following formulas provide prescalar
	 * and low/high divider values
	 * input clk --> PSC Div -----------> ICCL/H Div --> output clock
	 *                       module clk
	 *
	 * output clk = module clk / (PSC + 1) [ (ICCL + d) + (ICCH + d) ]
	 *
	 * Thus,
	 * (ICCL + ICCH) = clk = (input clk / ((psc +1) * output clk)) - 2d;
	 *
	 * where if PSC == 0, d = 7,
	 *       if PSC == 1, d = 6
	 *       if PSC > 1 , d = 5
	 */

	psc = 26; /* To get 1MHz clock */

	clk = ((input_clock/(psc + 1)) / (bus_freq * 1000)) - 10;
	clkh = (27 * clk) / 100;
	clkl = clk - clkh;

	davinci_i2c_write_reg(dev, DAVINCI_I2C_PSC_REG, psc);
	davinci_i2c_write_reg(dev, DAVINCI_I2C_CLKH_REG, clkh);
	davinci_i2c_write_reg(dev, DAVINCI_I2C_CLKL_REG, clkl);

	dev_dbg(dev->dev, "CLK  = %d\n", clk);
	dev_dbg(dev->dev, "PSC  = %d\n", davinci_i2c_read_reg(DAVINCI_I2C_PSC_REG));
	dev_dbg(dev->dev, "CLKL = %d\n", davinci_i2c_read_reg(DAVINCI_I2C_CLKL_REG));
	dev_dbg(dev->dev, "CLKH = %d\n", davinci_i2c_read_reg(DAVINCI_I2C_CLKH_REG));

	/* Take the I2C module out of reset: */
	w = davinci_i2c_read_reg(dev, DAVINCI_I2C_MDR_REG);
	MOD_REG_BIT(w, DAVINCI_I2C_MDR_IRS, 1);
	davinci_i2c_write_reg(dev, DAVINCI_I2C_MDR_REG, w);

	/* Enable interrupts */
	davinci_i2c_write_reg(dev, DAVINCI_I2C_IMR_REG, I2C_DAVINCI_INTR_ALL);

	return 0;
}

/*
 * Waiting on Bus Busy
 */
static int i2c_davinci_wait_for_bb(struct davinci_i2c_dev *dev, char allow_sleep)
{
	unsigned long timeout;

	timeout = jiffies + DAVINCI_I2C_TIMEOUT;
	while ((davinci_i2c_read_reg(dev, DAVINCI_I2C_STR_REG))
	       & DAVINCI_I2C_STR_BB) {
		if (time_after(jiffies, timeout)) {
			return -ETIMEDOUT;
		}
		if (allow_sleep)
			schedule_timeout(1);
	}

	return 0;
}

/*
 * Low level master read/write transaction. This function is called
 * from i2c_davinci_xfer.
 */
static int
i2c_davinci_xfer_msg(struct i2c_adapter *adap, struct i2c_msg *msg, int stop)
{
	struct davinci_i2c_dev *dev = i2c_get_adapdata(adap);
	u32 flag = 0, stat = 0, cnt = 2000;
	u16 w;
	int r;

	if (msg->len == 0)
		return -EINVAL;

	/* Introduce a 20musec delay.  Required for Davinci EVM */
	while (cnt--);

	/* set the slave address */
	davinci_i2c_write_reg(dev, DAVINCI_I2C_SAR_REG, msg->addr);

	dev->buf = msg->buf;
	dev->buf_len = msg->len;

	davinci_i2c_write_reg(dev, DAVINCI_I2C_CNT_REG, dev->buf_len);

	init_completion(&dev->cmd_complete);
	dev->cmd_err = 0;

	/* Clear any pending interrupts by reading the IVR */
	stat = davinci_i2c_read_reg(dev, DAVINCI_I2C_IVR_REG);

	/* Take I2C out of reset, configure it as master and set the
	 * start bit */
	flag = DAVINCI_I2C_MDR_IRS | DAVINCI_I2C_MDR_MST |
		DAVINCI_I2C_MDR_STT;

	/* if the slave address is ten bit address, enable XA bit */
	if (msg->flags & I2C_M_TEN)
		flag |= DAVINCI_I2C_MDR_XA;
	if (!(msg->flags & I2C_M_RD))
		flag |= DAVINCI_I2C_MDR_TRX;
	if (stop)
		flag |= DAVINCI_I2C_MDR_STP;

	/* write the data into mode register */
	davinci_i2c_write_reg(dev, DAVINCI_I2C_MDR_REG, flag);

	/* Enable receive and transmit interrupts */
	if (msg->flags & I2C_M_RD) {
		w = davinci_i2c_read_reg(dev, DAVINCI_I2C_IMR_REG);
		MOD_REG_BIT(w, DAVINCI_I2C_IMR_RRDY, 1);
		davinci_i2c_write_reg(dev, DAVINCI_I2C_IMR_REG, w);
	} else {
		w = davinci_i2c_read_reg(dev, DAVINCI_I2C_IMR_REG);
		MOD_REG_BIT(w, DAVINCI_I2C_IMR_XRDY, 1);
		davinci_i2c_write_reg(dev, DAVINCI_I2C_IMR_REG, w);
	}

	r = wait_for_completion_interruptible_timeout(&dev->cmd_complete,
						      DAVINCI_I2C_TIMEOUT);
	dev->buf_len = 0;
	if (r < 0)
		return r;
	if (r == 0) {
		dev_err(dev->dev, "controller timed out\n");
		i2c_davinci_init(dev);
		return -ETIMEDOUT;
	}

	/* no error */
	if (likely(!dev->cmd_err))
		return msg->len;

	/* We have an error */
	if (dev->cmd_err & DAVINCI_I2C_STR_AL ||
	    dev->cmd_err & DAVINCI_I2C_STR_RSFULL) {
		i2c_davinci_init(dev);
		return -EIO;
	}

	if (dev->cmd_err & DAVINCI_I2C_STR_NACK) {
		if (msg->flags & I2C_M_IGNORE_NAK)
			return msg->len;
		if (stop) {
			w = davinci_i2c_read_reg(dev, DAVINCI_I2C_MDR_REG);
			MOD_REG_BIT(w, DAVINCI_I2C_MDR_STP, 1);
			davinci_i2c_write_reg(dev, DAVINCI_I2C_MDR_REG, w);
		}
		return -EREMOTEIO;
	}
	return msg->len;
}

/*
 * Prepare controller for a transaction and call i2c_davinci_xfer_msg
 */
static int
i2c_davinci_xfer(struct i2c_adapter *adap, struct i2c_msg msgs[], int num)
{
	struct davinci_i2c_dev *dev = i2c_get_adapdata(adap);
	int i;
	int ret = 0;
	char retries = adap->retries;

	dev_dbg(dev->dev, "%s: msgs: %d\n", __FUNCTION__, num);

	if ((ret = i2c_davinci_wait_for_bb(dev, 1)) < 0) {
		dev_warn(dev->dev, "timeout waiting for bus ready");
		return ret;
	}

	for (i = 0; i < num; i++) {
		do {
			ret = i2c_davinci_xfer_msg(adap, &msgs[i],
						   (i == (num - 1)));

			if (ret < 0) {
				dev_dbg(dev->dev, "Retrying ...\n");
				mdelay (1);
				retries--;
			} else
				break;
		} while (retries);

		dev_dbg(dev->dev, "%s:%d ret: %d\n",
			__FUNCTION__, __LINE__, ret);

		if (ret != msgs[i].len)
			break;
	}

	if (ret >= 0 && num > 1)
		ret = num;

	dev_dbg(dev->dev, "%s:%d ret: %d\n", __FUNCTION__, __LINE__, ret);

	return ret;
}

static u32 i2c_davinci_func(struct i2c_adapter *adap)
{
	return I2C_FUNC_I2C | (I2C_FUNC_SMBUS_EMUL & ~I2C_FUNC_SMBUS_QUICK);
}

/*
 * This function marks a transaction as complete.
 */
static inline void i2c_davinci_complete_cmd(struct davinci_i2c_dev *dev)
{
	complete(&dev->cmd_complete);
}

/*
 * Interrupt service routine. This gets called whenever an I2C interrupt
 * occurs.
 */
static irqreturn_t
i2c_davinci_isr(int this_irq, void *dev_id, struct pt_regs *reg)
{
	struct davinci_i2c_dev *dev = dev_id;
	u32 stat;
	int count = 0;
	u16 w;

	while ((stat = davinci_i2c_read_reg(dev, DAVINCI_I2C_IVR_REG)) != 0) {
		dev_dbg(dev->dev, "%s: stat=0x%x\n", __FUNCTION__, stat);
		if (count++ == 100) {
			dev_warn(dev->dev, "Too much work in one IRQ\n");
			break;
		}

		switch (stat) {
		case DAVINCI_I2C_IVR_AL:
			dev->cmd_err |= DAVINCI_I2C_STR_AL;
			i2c_davinci_complete_cmd(dev);
			break;

		case DAVINCI_I2C_IVR_NACK:
			dev->cmd_err |= DAVINCI_I2C_STR_NACK;
			i2c_davinci_complete_cmd(dev);
			break;

		case DAVINCI_I2C_IVR_ARDY:
			w = davinci_i2c_read_reg(dev, DAVINCI_I2C_STR_REG);
			MOD_REG_BIT(w, DAVINCI_I2C_STR_ARDY, 1);
			davinci_i2c_write_reg(dev, DAVINCI_I2C_STR_REG, w);
                        break;

		case DAVINCI_I2C_IVR_RDR:
			if (dev->buf_len) {
				*dev->buf++ = davinci_i2c_read_reg(dev, DAVINCI_I2C_DRR_REG);
				dev->buf_len--;
				if (dev->buf_len) {
					continue;
				} else {
					w = davinci_i2c_read_reg(dev, DAVINCI_I2C_STR_REG);
					MOD_REG_BIT(w, DAVINCI_I2C_IMR_RRDY, 0);
					davinci_i2c_write_reg(dev, DAVINCI_I2C_STR_REG, w);
				}
			} else {
				dev_err(dev->dev, "RDR IRQ while no"
					"data requested\n");
			}
			break;

		case DAVINCI_I2C_IVR_XRDY:
			if (dev->buf_len) {
				davinci_i2c_write_reg(dev, DAVINCI_I2C_DXR_REG, *dev->buf++);
				dev->buf_len--;
				if (dev->buf_len)
					continue;
				else {
					w = davinci_i2c_read_reg(dev, DAVINCI_I2C_IMR_REG);
					MOD_REG_BIT(w, DAVINCI_I2C_IMR_XRDY, 0);
					davinci_i2c_write_reg(dev, DAVINCI_I2C_IMR_REG, w);
				}
			} else {
				dev_err(dev->dev, "TDR IRQ while no data to"
						  "send\n");
			}
			break;

		case DAVINCI_I2C_IVR_SCD:
			w = davinci_i2c_read_reg(dev, DAVINCI_I2C_STR_REG);
			MOD_REG_BIT(w, DAVINCI_I2C_STR_SCD, 1);
			davinci_i2c_write_reg(dev, DAVINCI_I2C_STR_REG, w);
                        i2c_davinci_complete_cmd(dev);
                        break;

		case DAVINCI_I2C_IVR_AAS:
			dev_warn(dev->dev, "Address as slave interrupt");
			break;

		default:
			break;
		}/* switch */
	}/* while */

	return count ? IRQ_HANDLED : IRQ_NONE;
}

static struct i2c_algorithm i2c_davinci_algo = {
	.master_xfer	= i2c_davinci_xfer,
	.functionality	= i2c_davinci_func,
};

static int
davinci_i2c_probe(struct platform_device *pdev)
{
	struct davinci_i2c_dev *dev;
	struct i2c_adapter	*adap;
	struct resource		*mem, *irq, *ioarea;
	int r;

	/* NOTE: driver uses the static register mapping */
	mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!mem) {
		dev_err(&pdev->dev, "no mem resource?\n");
		return -ENODEV;
	}

	irq = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
	if (!irq) {
		dev_err(&pdev->dev, "no irq resource?\n");
		return -ENODEV;
	}

	ioarea = request_mem_region(mem->start, (mem->end - mem->start) + 1,
			pdev->name);
	if (!ioarea) {
		dev_err(&pdev->dev, "I2C region already claimed\n");
		return -EBUSY;
	}

	dev = kzalloc(sizeof(struct davinci_i2c_dev), GFP_KERNEL);
	if (!dev) {
		r = -ENOMEM;
		goto err_release_region;
	}

	dev->dev = &pdev->dev;
	dev->irq = irq->start;
	platform_set_drvdata(pdev, dev);

	dev->clk = clk_get (&pdev->dev, "I2CCLK");
	if (IS_ERR(dev->clk)) {
		r = -ENODEV;
		goto err_free_mem;
	}
	clk_enable(dev->clk);

	dev->base = (void __iomem*)IO_ADDRESS(mem->start);
	i2c_davinci_init(dev);

	r = request_irq(dev->irq, i2c_davinci_isr, 0, pdev->name, dev);
	if (r) {
		dev_err(&pdev->dev, "failure requesting irq %i\n", dev->irq);
		goto err_unuse_clocks;
	}

	adap = &dev->adapter;
	i2c_set_adapdata(adap, dev);
	adap->owner = THIS_MODULE;
	adap->class = I2C_CLASS_HWMON;
	strncpy(adap->name, "DaVinci I2C adapter", sizeof(adap->name));
	adap->algo = &i2c_davinci_algo;
	adap->dev.parent = &pdev->dev;

	/* FIXME */
	adap->timeout = 1;
	adap->retries = 1;

	/* i2c device drivers may be active on return from add_adapter() */
	r = i2c_add_adapter(adap);
	if (r) {
		dev_err(&pdev->dev, "failure adding adapter\n");
		goto err_free_irq;
	}

	return 0;

err_free_irq:
	free_irq(dev->irq, dev);
err_unuse_clocks:
	clk_disable(dev->clk);
	clk_put(dev->clk);
	dev->clk = NULL;
err_free_mem:
	kfree(dev);
err_release_region:
	release_mem_region(mem->start, (mem->end - mem->start) + 1);

	return r;
}

static int
davinci_i2c_remove(struct platform_device *pdev)
{
	struct davinci_i2c_dev *dev = platform_get_drvdata(pdev);
	struct resource		*mem;

	platform_set_drvdata(pdev, NULL);

	clk_disable(dev->clk);
	clk_put(dev->clk);
	dev->clk = NULL;

	davinci_i2c_write_reg(dev, DAVINCI_I2C_MDR_REG, 0);
	free_irq(IRQ_I2C, dev);

	i2c_del_adapter(&dev->adapter);
	mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	release_mem_region(mem->start, (mem->end - mem->start) + 1);
	return 0;
}

static struct platform_driver davinci_i2c_driver = {
	.probe		= davinci_i2c_probe,
	.remove		= davinci_i2c_remove,
	.driver		= {
		.name	= "i2c_davinci",
		.owner	= THIS_MODULE,
	},
};

/* I2C may be needed to bring up other drivers */
static int __init
davinci_i2c_init_driver(void)
{
	return platform_driver_register(&davinci_i2c_driver);
}
subsys_initcall(davinci_i2c_init_driver);

static void __exit davinci_i2c_exit_driver(void)
{
	platform_driver_unregister(&davinci_i2c_driver);
}
module_exit(davinci_i2c_exit_driver);
