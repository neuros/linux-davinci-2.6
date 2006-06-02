/*
 * include/sound/davincisound.h
 *
 * Copyright (C) 2006 Texas Instruments, Inc.
 *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * THIS PACKAGE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
 * WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 * History:
 * --------
 *  2006-03-29 Sudhakar - Created
 */

#include <linux/soundcard.h>

#define SOUND_MIXER_MICBIAS		_IOC_NR(SOUND_MIXER_PRIVATE1)
#define SOUND_MIXER_READ_MICBIAS	_SIOR ('M', SOUND_MIXER_MICBIAS, int)
#define SOUND_MIXER_WRITE_MICBIAS	SOUND_MIXER_PRIVATE1
