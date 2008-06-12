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

#endif /* NEUROS_IR__H */

