/*
	Copyright (C) 2004 - 2007 rt2x00 SourceForge Project
	<http://rt2x00.serialmonkey.com>

	This program is free software; you can redistribute it and/or modify
	it under the terms of the GNU General Public License as published by
	the Free Software Foundation; either version 2 of the License, or
	(at your option) any later version.

	This program is distributed in the hope that it will be useful,
	but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
	GNU General Public License for more details.

	You should have received a copy of the GNU General Public License
	along with this program; if not, write to the
	Free Software Foundation, Inc.,
	59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
 */

/*
	Module: rt2x00debug
	Abstract: Data structures for the rt2x00debug.
 */

#ifndef RT2X00DEBUG_H
#define RT2X00DEBUG_H

#include "rt_config.h"

struct rt2x00_dev;

#define RT2X00DEBUGFS_REGISTER_ENTRY(__name, __type)		\
struct reg##__name {						\
	void (*read)(const struct rt2x00_dev *rt2x00dev,	\
		     const unsigned int word, __type *data);	\
	void (*write)(const struct rt2x00_dev *rt2x00dev,	\
		      const unsigned int word, __type data);	\
								\
	unsigned int word_size;					\
	unsigned int word_count;				\
} __name

struct rt2x00debug {
	/*
	 * Reference to the modules structure.
	 */
	struct module *owner;

	/*
	 * Register access entries.
	 */
	RT2X00DEBUGFS_REGISTER_ENTRY(csr, u32);
	RT2X00DEBUGFS_REGISTER_ENTRY(eeprom, u16);
	RT2X00DEBUGFS_REGISTER_ENTRY(bbp, u8);
	RT2X00DEBUGFS_REGISTER_ENTRY(rf, u32);
};

struct rt2x00_ops {
	const struct rt2x00debug *debugfs;
};

struct rt2x00_dev {
	RTMP_ADAPTER *pAd;

	const struct rt2x00debug_intf *debugfs_intf;
	struct rt2x00_ops *ops;
};

void rt2x00debug_register(struct rt2x00_dev *rt2x00dev);
void rt2x00debug_deregister(struct rt2x00_dev *rt2x00dev);

#endif /* RT2X00DEBUG_H */
