#ifndef NEUROS_IR__H
#define NEUROS_IR__H
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
 * IR Receiver driver header.
 *
 */
#ifdef __KERNEL__
    #include <linux/types.h>
    extern int i2c_write(uint8_t reg, uint16_t value);
    extern int i2c_read(uint8_t reg);
#else
    #include <stdint.h>
#endif
   
#define regIR                   0xB0  // IR receiver register.
#define NEUROS_IR_MAJOR 110
#define NEUROS_IR_IOC_MAGIC 'i'
#define NULL_KEY 0xff
#define LEARNING_COMPLETE_KEY 0x3e
#define RELEASE_REMOTE_KEY 0x29
#define TEST_KEY 0x3f
#define UP_KEY 0x00

#define GPIO01_DIR	__REG(0x01C67010)
#define SET_GPIO01_IN_DATA	__REG(0x01C67020)
#define SET_GPIO01_RIS_INT	__REG(0x01C67024)
#define CLR_GPIO01_RIS_INT	__REG(0x01C67028)
#define SET_GPIO01_FAL_INT	__REG(0x01C6702C)
#define CLR_GPIO01_FAL_INT	__REG(0x01C67030)
#define GPIO23_DIR	__REG(0x01C67038)
#define GPIO23_OUT_DATA	__REG(0x01C6703C)
#define GPIO23_SET_DATA	__REG(0x01C67040)
#define GPIO23_CLR_DATA	__REG(0x01C67044)
#define PINMUX1 __REG(0x01C40004)
#define PWM0_PCR	__REG(0x01C22004)
#define PWM0_CFG	__REG(0x01C22008)
#define PWM0_START	__REG(0x01C2200C)
#define PWM0_PER    __REG(0x01C22014)
#define PWM0_PH1D    __REG(0x01C22018)
#define TIMER1_TIM34    __REG(0x01C21814)
#define TIMER1_PRD34    __REG(0x01C2181C)
#define TIMER1_TCR    __REG(0x01C21820)
#define TIMER1_TGCR    __REG(0x01C21824)

#endif /* NEUROS_IR__H */

