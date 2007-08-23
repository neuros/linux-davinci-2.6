/*
 *  linux/drivers/char/rtc.c
 *
 * Copyright (C) 2004 Texas Instruments Inc
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
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 * ----------------------------------------------------------------------------
 Modifications:
 ver. 1.0: Jan 2006, Swaminathan S
 -
 */

#include <linux/init.h>
#include <linux/fs.h>
#include <linux/module.h>
#include <linux/proc_fs.h>
#include <linux/ctype.h>
#include <linux/delay.h>
#include <linux/rtc.h>
#include <linux/bcd.h>
#include <asm/rtc.h>
#include <asm/arch/i2c-client.h>

static unsigned char am;

static int _rtc_read_time(struct rtc_time *tm);
static int _rtc_set_time(struct rtc_time *tm);
static void am_or_pm (void);

static struct rtc_ops davinci_rtc_ops = {
        .owner          = THIS_MODULE,
        .read_time      = _rtc_read_time,
        .set_time       = _rtc_set_time,
};

static unsigned long epoch = 1900;      /* year corresponding to 0x00   */

static const unsigned char days_in_mo[] =
{31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};

static int _rtc_read_time(struct rtc_time *tm)
{
	char rtcdata [9] = { 2, 1, 0, 0, 0, 0,
			     0, 0, 0 };

	davinci_i2c_write (2, rtcdata, 0x23);
	udelay (1000);
	davinci_i2c_read (9, rtcdata, 0x23);
	udelay (1000);

	tm->tm_year = BCD_TO_BIN (rtcdata[3]) * 100 + BCD_TO_BIN (rtcdata[2]) - 1900;
	tm->tm_mon = BCD_TO_BIN (rtcdata[4]);
	tm->tm_mday = BCD_TO_BIN (rtcdata[5]);
	tm->tm_hour = BCD_TO_BIN (rtcdata[6]);
	tm->tm_min = BCD_TO_BIN (rtcdata[7]);
	tm->tm_sec = BCD_TO_BIN (rtcdata[8]);

	return 0;
}

static int _rtc_set_time(struct rtc_time *tm)
{
	char rtcdata [9];
	char ampmdata [9];
	struct timespec tv;
	unsigned char mon, day, hrs = 0, min, sec, leap_yr;
	unsigned char yr_low, yr_high;
	unsigned int yrs;

	am_or_pm ();

	yrs = tm->tm_year + 1900;
	yr_high = yrs/100;
	yr_low = (yrs) % 100;

	mon = tm->tm_mon;
	hrs = tm->tm_hour;
	day = tm->tm_mday;
	min = tm->tm_min;
	sec = tm->tm_sec;

	if (yrs < 1970 || yrs > 2037)
                return -EINVAL;

        leap_yr = ((!(yrs % 4) && (yrs % 100)) || !(yrs % 400));

	if ((mon > 11) || (day == 0))
                return -EINVAL;

        if (day > (days_in_mo[mon] + ((mon == 1) && leap_yr)))
                return -EINVAL;

        if ((hrs >= 24) || (min >= 60) || (sec >= 60))
                return -EINVAL;

        if ((yrs -= epoch) > 255) {    /* They are unsigned */
                return -EINVAL;
        }

	if (am == 1 && tm->tm_hour <= 12) {
		hrs = tm->tm_hour;
		if (tm->tm_hour == 0)
			hrs = tm->tm_hour + 12;
	}

	else if ((am == 1 && tm->tm_hour > 12) ||
	    	 (am == 0 && tm->tm_hour < 12)) {
		unsigned char mon1 = mon, day1 = day, hrs1 = 11, min1 = 59, sec1 = 59;
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
		davinci_i2c_write (9, ampmdata, 0x23);
		udelay (1000);
		mdelay (1000);
		am = (am == 1) ? 0 : 1;

		if (!am)
			hrs = tm->tm_hour - 12;
		else if (tm->tm_hour == 0)
			hrs = tm->tm_hour + 12;
	}

	else if (am == 0 && tm->tm_hour > 12) {
		hrs = tm->tm_hour - 12;
	}

	rtcdata [0] = 9;
	rtcdata [1] = 0;
	rtcdata [2] = BIN_TO_BCD(yr_low);
	rtcdata [3] = BIN_TO_BCD(yr_high);
	rtcdata [4] = BIN_TO_BCD(mon);
	rtcdata [5] = BIN_TO_BCD(day);
	rtcdata [6] = BIN_TO_BCD(hrs);
	rtcdata [7] = BIN_TO_BCD(min);
	rtcdata [8] = BIN_TO_BCD(sec);
	davinci_i2c_write (9, rtcdata, 0x23);
	udelay (1000);

	tv.tv_nsec = 0;
        tv.tv_sec = mktime (tm->tm_year + 1900, tm->tm_mon + 1, tm->tm_mday,
				tm->tm_hour, tm->tm_min, tm->tm_sec+2);
        do_settimeofday (&tv);

	return 0;
}

static void am_or_pm (void)
{
	char rtcdata [9];
	struct rtc_time tm, time, temp;
	unsigned char mon, day, hrs, min, sec;
	unsigned char yr_low, yr_high;
	unsigned int yrs;

	_rtc_read_time (&tm);

	temp = tm;

	yrs = temp.tm_year + 1900;
	yr_high = yrs/100;
	yr_low = (yrs) % 100;

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
	davinci_i2c_write (9, rtcdata, 0x23);
	udelay (1000);
	mdelay (1000);
	_rtc_read_time (&time);

	if (time.tm_mday == temp.tm_mday)
		am = 1;
	else
		am = 0;

	davinci_i2c_write (9, rtcdata, 0x23);
	udelay (1000);
	mdelay (1000);

	yrs = tm.tm_year + 1900;
	yr_high = yrs/100;
	yr_low = (yrs) % 100;

	mon = tm.tm_mon + 1;
	day = tm.tm_mday;
	min = tm.tm_min;
	hrs = tm.tm_hour;
	if (tm.tm_sec < 58)
	{
		sec = tm.tm_sec + 2;
	} else {
		sec = 59;
	}

	davinci_i2c_write (9, rtcdata, 0x23);
	udelay (1000);
}

static int __init davinci_rtc_init(void)
{
	struct timespec tv;
	struct rtc_time tm;
        register_rtc (&davinci_rtc_ops);

	am_or_pm ();

	_rtc_read_time (&tm);

        tv.tv_nsec = 0;
        tv.tv_sec = mktime (tm.tm_year + 1900, tm.tm_mon + 1 , tm.tm_mday,
				tm.tm_hour, tm.tm_min, tm.tm_sec);
        do_settimeofday (&tv);

	return 0;
}

module_init(davinci_rtc_init);
