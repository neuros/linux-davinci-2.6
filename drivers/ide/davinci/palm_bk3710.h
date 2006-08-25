/*
 *  linux/drivers/ide/davinci/palm_bk3710.h
 *
 *  BRIEF MODULE DESCRIPTION
 *      DAVINCI Virtual memory definitions
 *
 *  Copyright (C) 2006 Texas Instruments.
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 *
 *  THIS  SOFTWARE  IS PROVIDED   ``AS  IS'' AND   ANY  EXPRESS OR IMPLIED
 *  WARRANTIES,   INCLUDING, BUT NOT  LIMITED  TO, THE IMPLIED WARRANTIES OF
 *  MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN
 *  NO  EVENT  SHALL   THE AUTHOR  BE    LIABLE FOR ANY   DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 *  NOT LIMITED   TO, PROCUREMENT OF  SUBSTITUTE GOODS  OR SERVICES; LOSS OF
 *  USE, DATA,  OR PROFITS; OR  BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 *  ANY THEORY OF LIABILITY, WHETHER IN  CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 *  THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *  You should have received a copy of the  GNU General Public License along
 *  with this program; if not, write  to the Free Software Foundation, Inc.,
 *  675 Mass Ave, Cambridge, MA 02139, USA.
 *
 */

#ifndef PALM_BK3710_H
#define PALM_BK3710_H

#include <asm/arch/hardware.h>

/*
 *
 *  PalmChip 3710 IDE Controller Driver Definitions
 *
 */

/*
 *
 *  PalmChip 3710 IDE Controller config Definitions
 *
 */

#define IDE_PALM_FREQ           76	/* Palm Chip operating freq (MHZ) */
#define IDE_PALM_CLK            ((3000000/clk_get_rate (ideclkp)) - 1)	/* In ns */
#define IDE_PALM_REG_MMAP_BASE  DAVINCI_CFC_ATA_BASE /* Register Memory map address */
#define IDE_PALM_ATA_PRI_REG_OFFSET 0x1F0  /**< Offset of the primary interface
registers */
#define IDE_PALM_ATA_PRI_CTL_OFFSET 0x3F6  /**< Primary Control Offset */

/*
 *
 *  PalmChip 3710 IDE Controller PIO cycle timing structure Definition
 */
typedef struct {
	unsigned int activetime;	/* Active Time  */
	unsigned int cycletime;	/* Cycle Time   */
} palm_bk3710_piotiming;

/*
 *
 * PalmChip 3710 IDE Controller DMA cycle timing structure Definition
 */
typedef struct {
	unsigned int activetime;	/* Active Time     */
	unsigned int recoverytime;	/* Recovery Time   */
	unsigned int cycletime;	/* Cycle Time      */
} palm_bk3710_dmatiming;

/*
 *
 * PalmChip 3710 IDE Controller UDMA timing structure Definition
 */
typedef struct {
	unsigned int envtime;	/* Envelope Time        */
	unsigned int rptime;	/* Ready to pause time  */
	unsigned int cycletime;	/* Cycle Time           */
} palm_bk3710_udmatiming;

/**************************************************************************\
* Register Overlay Structure for DmaEngine
\**************************************************************************/
typedef struct {
	unsigned short bmpcp;
	unsigned short bmisp;
	unsigned int bmidtp;
	unsigned short bmics;
	unsigned short bmiss;
	unsigned int bmidtps;
} palm_bk3710_dmaengineregs;

/**************************************************************************\
* Register Overlay Structure for Config
\**************************************************************************/
typedef struct {
	unsigned short idetimp __attribute__ ((packed));
	unsigned short idetims __attribute__ ((packed));
	unsigned char sidetim;
	unsigned short slewctl __attribute__ ((packed));
	unsigned char idestatus;
	unsigned short udmactl __attribute__ ((packed));
	unsigned short udmatim __attribute__ ((packed));
	unsigned char rsvd0[4];
	unsigned int miscctl __attribute__ ((packed));
	unsigned int regstb __attribute__ ((packed));
	unsigned int regrcvr __attribute__ ((packed));
	unsigned int datstb __attribute__ ((packed));
	unsigned int datrcvr __attribute__ ((packed));
	unsigned int dmastb __attribute__ ((packed));
	unsigned int dmarcvr __attribute__ ((packed));
	unsigned int udmastb __attribute__ ((packed));
	unsigned int udmatrp __attribute__ ((packed));
	unsigned int udmaenv __attribute__ ((packed));
	unsigned int iordytmp __attribute__ ((packed));
	unsigned int iordytms __attribute__ ((packed));
} palm_bk3710_ideconfigregs;

/**************************************************************************\
* Register Overlay Structure
\**************************************************************************/
typedef struct {
	palm_bk3710_dmaengineregs dmaengine;
	unsigned char rsvd0[48];
	palm_bk3710_ideconfigregs config;
} palm_bk3710_ideregs;

#endif				/* DDC_BK3710_H */
