/*
 * rtc-davinci-evm.c
 *
 * Copyright (C) 2004 Texas Instruments Inc
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
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#include <linux/init.h>
#include <linux/fs.h>
#include <linux/module.h>
#include <linux/proc_fs.h>
#include <linux/ctype.h>
#include <linux/delay.h>
#include <linux/rtc.h>
#include <linux/bcd.h>
#include <linux/err.h>
#include <linux/platform_device.h>

#include <asm/mach-types.h>
#include <asm/arch/i2c-client.h>


/* REVISIT
 *  - the firmware expects no I2C writes at all, not just no RTC-via-I2C
 *    writes, for 100 usec after i2c read or write... that can't be
 *    assured here.
 *
 *  - this am vs pm thing is bizarre ... firmware should just do a 24 hour
 *    clock, rather than 12 hour with hidden am/pm (we must guess).
 *    similarly with it trying to handle DST for us...
 *
 *  - better (and simpler!!) firmware would support an RTC alarm, and just
 *    count seconds since some UTC instant, letting Linux handle calendar
 *    issues (leapyear, day of week, etc) and DST.
 */
static unsigned char am;


static int evm_read_time(struct device *dev, struct rtc_time *tm)
{
	char rtcdata [9];

	rtcdata[0] = 2;
	rtcdata[1] = 1;
	davinci_i2c_write(2, rtcdata, 0x23);

	msleep(1);
	davinci_i2c_read(9, rtcdata, 0x23);
	msleep(1);

	/* FIXME the RTC reports 12-hour time, without an AM/PM indicator,
	 * but Linux requires that we report 24 hour time...
	 */

	tm->tm_year = BCD_TO_BIN(rtcdata[3]) * 100
			+ BCD_TO_BIN(rtcdata[2])
			- 1900;
	tm->tm_mon = BCD_TO_BIN(rtcdata[4]);
	tm->tm_mday = BCD_TO_BIN(rtcdata[5]);
	tm->tm_hour = BCD_TO_BIN(rtcdata[6]);
	tm->tm_min = BCD_TO_BIN(rtcdata[7]);
	tm->tm_sec = BCD_TO_BIN(rtcdata[8]);

	return 0;
}

static void am_or_pm(struct device *dev)
{
	char rtcdata [9];
	struct rtc_time tm, time, temp;
	unsigned char mon, day, hrs, min, sec;
	unsigned char yr_low, yr_high;
	unsigned int yrs;

	evm_read_time(dev, &tm);

	temp = tm;

	yrs = temp.tm_year + 1900;
	yr_high = yrs / 100;
	yr_low = yrs % 100;

	mon = temp.tm_mon + 1;
	day = temp.tm_mday;
	min = 59;
	sec = 59;
	hrs = 11;

	rtcdata [0] = 9;
	rtcdata [1] = 0;
	rtcdata [2] = BIN_TO_BCD(yr_low);
	rtcdata [3] = BIN_TO_BCD(yr_high);
	mon--;
	rtcdata [4] = BIN_TO_BCD(mon);
	rtcdata [5] = BIN_TO_BCD(day);
	rtcdata [6] = BIN_TO_BCD(hrs);
	rtcdata [7] = BIN_TO_BCD(min);
	rtcdata [8] = BIN_TO_BCD(sec);
	davinci_i2c_write(9, rtcdata, 0x23);
	msleep(1);
	msleep(1000);
	evm_read_time(dev, &time);

	if (time.tm_mday == temp.tm_mday)
		am = 1;
	else
		am = 0;

	davinci_i2c_write(9, rtcdata, 0x23);
	msleep(1);
	msleep(1000);

	yrs = tm.tm_year + 1900;
	yr_high = yrs / 100;
	yr_low = yrs % 100;

	mon = tm.tm_mon + 1;
	day = tm.tm_mday;
	min = tm.tm_min;
	hrs = tm.tm_hour;

	if (tm.tm_sec < 58)
		sec = tm.tm_sec + 2;
	else
		sec = 59;

	davinci_i2c_write(9, rtcdata, 0x23);
	msleep(1);
}

static int evm_set_time(struct device *dev, struct rtc_time *tm)
{
	char rtcdata [9];
	char ampmdata [9];
	unsigned char mon, day, hrs = 0, min, sec, leap_yr;
	unsigned char yr_low, yr_high;
	unsigned int yrs;

	am_or_pm(dev);

	yrs = tm->tm_year + 1900;
	yr_high = yrs / 100;
	yr_low = yrs % 100;

	mon = tm->tm_mon;
	hrs = tm->tm_hour;
	day = tm->tm_mday;
	min = tm->tm_min;
	sec = tm->tm_sec;

        leap_yr = ((!(yrs % 4) && (yrs % 100)) || !(yrs % 400));

	if (am == 1 && tm->tm_hour <= 12) {
		hrs = tm->tm_hour;
		if (tm->tm_hour == 0)
			hrs = tm->tm_hour + 12;

	} else if ((am == 1 && tm->tm_hour > 12)
			|| (am == 0 && tm->tm_hour < 12)) {
		unsigned char mon1 = mon, day1 = day, hrs1 = 11;
		unsigned char min1 = 59, sec1 = 59;
		unsigned char yr_low1 = yr_low, yr_high1 = yr_high;

		ampmdata [0] = 9;
		ampmdata [1] = 0;
		ampmdata [2] = BIN_TO_BCD(yr_low1);
		ampmdata [3] = BIN_TO_BCD(yr_high1);
		ampmdata [4] = BIN_TO_BCD(mon1);
		ampmdata [5] = BIN_TO_BCD(day1);
		ampmdata [6] = BIN_TO_BCD(hrs1);
		ampmdata [7] = BIN_TO_BCD(min1);
		ampmdata [8] = BIN_TO_BCD(sec1);
		davinci_i2c_write(9, ampmdata, 0x23);
		msleep(1);
		msleep(1000);
		am = (am == 1) ? 0 : 1;

		if (!am)
			hrs = tm->tm_hour - 12;
		else if (tm->tm_hour == 0)
			hrs = tm->tm_hour + 12;

	} else if (am == 0 && tm->tm_hour > 12)
		hrs = tm->tm_hour - 12;

	rtcdata [0] = 9;
	rtcdata [1] = 0;
	rtcdata [2] = BIN_TO_BCD(yr_low);
	rtcdata [3] = BIN_TO_BCD(yr_high);
	rtcdata [4] = BIN_TO_BCD(mon);
	rtcdata [5] = BIN_TO_BCD(day);
	rtcdata [6] = BIN_TO_BCD(hrs);
	rtcdata [7] = BIN_TO_BCD(min);
	rtcdata [8] = BIN_TO_BCD(sec);

	davinci_i2c_write(9, rtcdata, 0x23);
	msleep(1);

	return 0;
}

static struct rtc_class_ops evm_rtc_ops = {
	.read_time	= evm_read_time,
	.set_time	= evm_set_time,
};

static int __devinit evm_rtc_probe(struct platform_device *pdev)
{
	struct rtc_device	*rtc;

	/* the 2005-12-05 firmware doesn't issue RTC alarms on GPIO(7);
	 * it only uses IRQ for card detect irqs with removable media.
	 * plus it also hides the am/pm indicator and does magic DST...
	 */
	rtc = rtc_device_register(pdev->name, &pdev->dev,
			&evm_rtc_ops, THIS_MODULE);
	if (IS_ERR(rtc))
		return PTR_ERR(rtc);

	printk(KERN_WARNING "%s: hours 12-23 are misreported as "
			"duplicate hours 00-11\n",
			rtc->class_dev.class_id);

	platform_set_drvdata(pdev, rtc);
	return 0;
}

static int __devexit evm_rtc_remove(struct platform_device *pdev)
{
	rtc_device_unregister(platform_get_drvdata(pdev));
	return 0;
}

static struct platform_driver evm_rtc_driver = {
	.driver = {
		.name		= "rtc_davinci_evm",
	},
	.probe		= evm_rtc_probe,
	.remove		= __devexit_p(evm_rtc_remove),
};

static int evm_rtc_init(void)
{
	if (!machine_is_davinci_evm())
		return -ENODEV;

	return platform_driver_register(&evm_rtc_driver);
}
module_init(evm_rtc_init);

static void evm_rtc_exit(void)
{
	platform_driver_unregister(&evm_rtc_driver);
}
module_exit(evm_rtc_exit);

MODULE_DESCRIPTION("RTC driver for TI DaVinci EVM");
MODULE_LICENSE("GPL");
