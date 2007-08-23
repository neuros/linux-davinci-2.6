/*
 *  include/asm-arm/arch-davinci/i2c-client.h
 *
 * Copyright (C) 2006 Texas Instruments Inc
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
/* i2c-client.h */

typedef enum {
        USB_DRVVBUS = 0,
        VDDIMX_EN = 1,
        VLYNQ_ON = 2,
        CF_RESET = 3,
        WLAN_RESET = 4,
        ATA_SEL = 5,
        CF_SEL = 6
} u35_expander_ops;

int davinci_i2c_expander_op (u16 client_addr, u35_expander_ops pin, u8 val);
int davinci_i2c_write(u8 size, u8 * val, u16 client_addr);
int davinci_i2c_read(u8 size, u8 * val, u16 client_addr);
