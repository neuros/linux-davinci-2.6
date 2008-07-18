#ifndef TVP7000_H__
#define TVP7000_H__
/*
 *  Copyright(C) 2006-2008 Neuros Technology International LLC. 
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
 * tvp7000 driver header. 
 *
 */

#ifdef __KERNEL__
#define TVP7000_CHIP_REVISION		0x00
#define TVP7000_PLL_DIVIDE_MSB		0x01
#define TVP7000_PLL_DIVIDE_LSB		0x02
#define TVP7000_PLL_CTRL		0x03
#define TVP7000_PHASE_SELECT	0x04
#define TVP7000_CLAMP_START		0x05
#define TVP7000_CLAMP_WIDTH		0x06
#define TVP7000_HSYNC_OUTPUT_WIDTH		0x07
#define TVP7000_BLUE_FINE_GAIN		0x08
#define TVP7000_GREEN_FINE_GAIN		0x09
#define TVP7000_RED_FINE_GAIN		0x0A
#define TVP7000_BLUE_FINE_OFFSET		0x0B
#define TVP7000_GREEN_FINE_OFFSET		0x0C
#define TVP7000_RED_FINE_OFFSET		0x0D
#define TVP7000_SYNC_CTRL_1		0x0E
#define TVP7000_PLL_CLAMP_CTRL		0x0F
#define TVP7000_SYNC_ON_GREEN		0x10
#define TVP7000_SYNC_SEPARATOR		0x11
#define TVP7000_PRE_COAST		0x12
#define TVP7000_POST_COAST		0x13
#define TVP7000_SYNC_DETECT_STATUS		0x14
#define TVP7000_OUTPUT_FORMATTER	0x15
#define TVP7000_TEST_REG		0x16
/* Reserved 0x17 - 0x18 */
#define TVP7000_INPUT_MUX_1		0x19
#define TVP7000_INPUT_MUX_2		0x1A
#define TVP7000_BLUE_GREEN_GAIN		0x1B
#define TVP7000_RED_COARSE_GAIN		0x1C
#define TVP7000_FINE_OFFSET_LSB		0x1D
#define TVP7000_BLUE_COARSE_OFFSET		0x1E
#define TVP7000_GREEN_COARSE_OFFSET		0x1F
#define TVP7000_RED_COARSE_OFFSET		0x20
#define TVP7000_HSOUT_OUTPUT_START		0x21
#define TVP7000_MISC_CTRL		0x22
/* Reserved 0x23 - 0x25 */
#define TVP7000_AUTO_CTRL_ENABLE	0x26
/* Reserved 0x27 */
#define TVP7000_AUTO_CTRL_FILTER	0x28
/* Reserved 0x29 */
#define TVP7000_FINE_CLAMP_CTRL		0x2A
#define TVP7000_POWER_CTRL		0x2B
#define TVP7000_ADC_SETUP		0x2C
#define TVP7000_COARSE_CLAMP_CTRL_1		0x2D
#define TVP7000_SOG_CLAMP	0x2E
/* Reserved 0x2F - 0x30 */
#define TVP7000_ALC_PLACEMENT	0x31
#endif /* __KERNEL__ */

/* Video Standards */
typedef enum {
    VGA60HZ = 0,
    VGA72HZ,
    VGA75HZ,
    VGA85HZ,
    SVGA56HZ,
    SVGA60HZ,
    SVGA72HZ,
    SVGA75HZ,
    SVGA85HZ,
    XGA60HZ,
    XGA70HZ,
    XGA75HZ,
    XGA85HZ,
    SXGA60HZ,
    SXGA75HZ,
    VIDEO480P60HZ,
    VIDEO576P50HZ,
    VIDEO720P60HZ,
    VIDEO720P50HZ,
    VIDEO1080I60HZ,
    VIDEO1080I50HZ,
    VIDEO1080P60HZ,
    VIDEO1080P50HZ,

    /* stardards count */
    TOTAL_STANDARD,
} video_standard;

#endif /* TVP7000_H__ */
