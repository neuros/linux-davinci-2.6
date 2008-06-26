#ifndef NEUROS_IR_BLASTER__H
#define NEUROS_IR_BLASTER__H
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
 * IR Blaster driver header.
 *
 */
#ifdef __KERNEL__
    #include <linux/types.h>
    extern int i2c_write(uint8_t reg, uint16_t value);
    extern int i2c_read(uint8_t reg);
#else
    #include <stdint.h>
#endif

#define KEY_MASK 0xff
#define NEUROS_IR_BLASTER_MAJOR 111
#define NEUROS_IR_BLASTER_IOC_MAGIC 'b'

#define the_same(x,y,p) \
        ((((x)*100<=(y)*(100+(p)))&&((y)*100<=(x)*(100+(p))))?1:0)

#define SIMILAR_PRECISION 20 /*averaged off x% difference.*/
#define the_similar(x,y) the_same(x,y,SIMILAR_PRECISION)

#define CAPTRUE_PRECISION1 10 /*averaged off x% difference.*/
#define the_same1(x,y) the_same(x,y,CAPTRUE_PRECISION1)

#define CAPTRUE_PRECISION2 13 /*averaged off x% difference.*/
#define the_same2(x,y) the_same(x,y,CAPTRUE_PRECISION2)

#define CAPTRUE_PRECISION3 17 /*averaged off x% difference.*/
#define the_same3(x,y) the_same(x,y,CAPTRUE_PRECISION3)

#define BLS_START 0
#define BLS_COMPLETE 1
#define BLS_ERROR -1

#define BLASTER_MAX_CHANGE	(8*50)  /* maximum edge changes. */
#define BLASTER_MAX_SBITS       20     /*special edge number min is 6*/
#define BITS_COUNT_START        0       /*bits count start. */
#define BITS_COUNT_MASK         (0x3FF<<BITS_COUNT_START)  /*bits count mask. */
#define FIRST_LEVEL_BIT_START   15      /*bits count start. */
#define FIRST_LEVEL_BIT_MASK    (1<<FIRST_LEVEL_BIT_START)    /*end flag mask. */

struct blaster_data_type {
    uint16_t bitstimes;     /*first[15] 1 bit 1=high level  0=low level; 
                                [0-9]10 bit =how many bits;*/

    uint32_t bits[BLASTER_MAX_CHANGE];  	/*each bit length*/
};

struct blaster_data_pack {
    uint16_t bitstimes;       /*first[15] 1 bit 1=high level  0=low level; 
                                [7+BITS_COUNT_START-BITS_COUNT_START]8 bit =how many bits;
                                [END_FLAG_START] 1=have end bits 0=not have end bits
                                [2-0]3 bit=how many repeat times*/
    uint32_t  bit0;
    uint32_t  bit1;
    uint32_t  bit2;

    uint8_t  mbits[BLASTER_MAX_CHANGE/8];
    uint8_t  dbits[BLASTER_MAX_CHANGE/8];

    uint32_t  specbits[BLASTER_MAX_SBITS];
};

#define keybit_set(bi, base, val) \
	do {\
		if (val)\
			*((base) + ((bi) / 8)) |=(1 << ((bi) % 8));\
		else\
			*((base) + ((bi) / 8)) &=~(1 << ((bi) % 8));\
	} while(0)

#define keybit_get(base, bi) \
			*((base) + ((bi) / 8)) & (1 << ((bi) % 8))

#define RRB_CAPTURE_KEY  _IO(NEUROS_IR_BLASTER_IOC_MAGIC, 7)
#define RRB_BLASTER_KEY  _IOW(NEUROS_IR_BLASTER_IOC_MAGIC, 8, \
    struct blaster_data_pack)
#define RRB_FACTORY_TEST _IO(NEUROS_IR_BLASTER_IOC_MAGIC, 9)
#define RRB_READ_LEARNING_DATA  _IOR(NEUROS_IR_BLASTER_IOC_MAGIC, 10, \
    struct blaster_data_type)
#define RRB_GET_BLASTER_STATUS	_IOR(NEUROS_IR_BLASTER_IOC_MAGIC, 11, \
    int)

#endif /* NEUROS_IR_BLASTER__H */

