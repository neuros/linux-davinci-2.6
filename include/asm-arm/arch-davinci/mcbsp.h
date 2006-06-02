/*
 * linux/include/asm/arch/mcbsp.h
 *
 *  BRIEF MODULE DESCRIPTION
 *      DAVINCI McBSP driver Info
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
#ifndef __ASM_ARCH_DAVINCI_MCBSP_H
#define __ASM_ARCH_DAVINCI_MCBSP_H

#include <asm/arch/hardware.h>
#include <asm/arch/irqs.h>

#define DAVINCI_MCBSP1_BASE	 DAVINCI_MCBSP_BASE
#define DAVINCI_DMA_MCBSP1_RX 3
#define DAVINCI_DMA_MCBSP1_TX 2

#define DAVINCI_McBSP1RX IRQ_MBRINT
#define DAVINCI_McBSP1TX IRQ_MBXINT

#define DRR1	0x00
#define DRR2	0x02
#define DXR1	0x04
#define DXR2	0x06
#define SPCR1	0x08
#define SPCR2	0x0a
#define RCR1	0x0c
#define RCR2	0x0e
#define XCR1	0x10
#define XCR2	0x12
#define SRGR1	0x14
#define SRGR2	0x16
#define MCR1	0x18
#define MCR2	0x1a
#define RCERA	0x1c
#define RCERB	0x1e
#define XCERA	0x20
#define XCERB	0x22
#define PCR0	0x24
#define PCR1	0x26
#define RCERC	0x28
#define RCERD	0x2a
#define XCERC	0x2c
#define XCERD	0x2e
#define RCERE	0x30
#define RCERF	0x32
#define XCERE	0x34
#define XCERF	0x36
#define RCERG	0x38
#define RCERH	0x3a
#define XCERG	0x3c
#define XCERH	0x3e

#define DAVINCI_MAX_MCBSP_COUNT 1

/********************** McBSP SPCR1 bit definitions ***********************/
#define RRST			0x0001
#define RRDY			0x0002
#define RFULL			0x0004
#define RSYNC_ERR		0x0008
#define RINTM(value)		((value)<<4)	/* bits 4:5 */
#define ABIS			0x0040
#define DXENA			0x0080
#define CLKSTP(value)		((value)<<11)	/* bits 11:12 */
#define RJUST(value)		((value)<<13)	/* bits 13:14 */
#define DLB			0x8000

/********************** McBSP SPCR2 bit definitions ***********************/
#define XRST		0x0001
#define XRDY		0x0002
#define XEMPTY		0x0004
#define XSYNC_ERR	0x0008
#define XINTM(value)	((value)<<4)	/* bits 4:5 */
#define GRST		0x0040
#define FRST		0x0080
#define SOFT		0x0100
#define FREE		0x0200

/********************** McBSP PCR bit definitions *************************/
#define CLKRP		0x0001
#define CLKXP		0x0002
#define FSRP		0x0004
#define FSXP		0x0008
#define DR_STAT		0x0010
#define DX_STAT		0x0020
#define CLKS_STAT	0x0040
#define SCLKME		0x0080
#define CLKRM		0x0100
#define CLKXM		0x0200
#define FSRM		0x0400
#define FSXM		0x0800
#define RIOEN		0x1000
#define XIOEN		0x2000
#define IDLE_EN		0x4000

/********************** McBSP RCR1 bit definitions ************************/
#define RWDLEN1(value)		((value)<<5)	/* Bits 5:7 */
#define RFRLEN1(value)		((value)<<8)	/* Bits 8:14 */

/********************** McBSP XCR1 bit definitions ************************/
#define XWDLEN1(value)		((value)<<5)	/* Bits 5:7 */
#define XFRLEN1(value)		((value)<<8)	/* Bits 8:14 */

/*********************** McBSP RCR2 bit definitions ***********************/
#define RDATDLY(value)		(value)	/* Bits 0:1 */
#define RFIG			0x0004
#define RCOMPAND(value)		((value)<<3)	/* Bits 3:4 */
#define RWDLEN2(value)		((value)<<5)	/* Bits 5:7 */
#define RFRLEN2(value)		((value)<<8)	/* Bits 8:14 */
#define RPHASE			0x8000

/*********************** McBSP XCR2 bit definitions ***********************/
#define XDATDLY(value)		(value)	/* Bits 0:1 */
#define XFIG			0x0004
#define XCOMPAND(value)		((value)<<3)	/* Bits 3:4 */
#define XWDLEN2(value)		((value)<<5)	/* Bits 5:7 */
#define XFRLEN2(value)		((value)<<8)	/* Bits 8:14 */
#define XPHASE			0x8000

/********************* McBSP SRGR1 bit definitions ************************/
#define CLKGDV(value)		(value)	/* Bits 0:7 */
#define FWID(value)		((value)<<8)	/* Bits 8:15 */

/********************* McBSP SRGR2 bit definitions ************************/
#define FPER(value)		(value)	/* Bits 0:11 */
#define FSGM			0x1000
#define CLKSM			0x2000
#define CLKSP			0x4000
#define GSYNC			0x8000

/********************* McBSP MCR1 bit definitions *************************/
#define RMCM			0x0001
#define RCBLK(value)		((value)<<2)	/* Bits 2:4 */
#define RPABLK(value)		((value)<<5)	/* Bits 5:6 */
#define RPBBLK(value)		((value)<<7)	/* Bits 7:8 */

/********************* McBSP MCR2 bit definitions *************************/
#define XMCM(value)		(value)	/* Bits 0:1 */
#define XCBLK(value)		((value)<<2)	/* Bits 2:4 */
#define XPABLK(value)		((value)<<5)	/* Bits 5:6 */
#define XPBBLK(value)		((value)<<7)	/* Bits 7:8 */

/* we don't do multichannel for now */
struct davinci_mcbsp_reg_cfg {
	u16 spcr2;
	u16 spcr1;
	u16 rcr2;
	u16 rcr1;
	u16 xcr2;
	u16 xcr1;
	u16 srgr2;
	u16 srgr1;
	u16 mcr2;
	u16 mcr1;
	u16 pcr2;
	u16 pcr0;
	u16 rcerc;
	u16 rcerd;
	u16 xcerc;
	u16 xcerd;
	u16 rcere;
	u16 rcerf;
	u16 xcere;
	u16 xcerf;
	u16 rcerg;
	u16 rcerh;
	u16 xcerg;
	u16 xcerh;
};

typedef enum {
	DAVINCI_MCBSP1 = 0,
} davinci_mcbsp_id;

typedef enum {
	DAVINCI_MCBSP_WORD_8 = 0,
	DAVINCI_MCBSP_WORD_12,
	DAVINCI_MCBSP_WORD_16,
	DAVINCI_MCBSP_WORD_20,
	DAVINCI_MCBSP_WORD_24,
	DAVINCI_MCBSP_WORD_32,
} davinci_mcbsp_word_length;

typedef enum {
	DAVINCI_MCBSP_CLK_RISING = 0,
	DAVINCI_MCBSP_CLK_FALLING,
} davinci_mcbsp_clk_polarity;

typedef enum {
	DAVINCI_MCBSP_FS_ACTIVE_HIGH = 0,
	DAVINCI_MCBSP_FS_ACTIVE_LOW,
} davinci_mcbsp_fs_polarity;

typedef enum {
	DAVINCI_MCBSP_CLK_STP_MODE_NO_DELAY = 0,
	DAVINCI_MCBSP_CLK_STP_MODE_DELAY,
} davinci_mcbsp_clk_stp_mode;

/******* SPI specific mode **********/
typedef enum {
	DAVINCI_MCBSP_SPI_MASTER = 0,
	DAVINCI_MCBSP_SPI_SLAVE,
} davinci_mcbsp_spi_mode;

struct davinci_mcbsp_spi_cfg {
	davinci_mcbsp_spi_mode spi_mode;
	davinci_mcbsp_clk_polarity rx_clock_polarity;
	davinci_mcbsp_clk_polarity tx_clock_polarity;
	davinci_mcbsp_fs_polarity fsx_polarity;
	u8 clk_div;
	davinci_mcbsp_clk_stp_mode clk_stp_mode;
	davinci_mcbsp_word_length word_length;
};

void davinci_mcbsp_config(unsigned int id,
			  const struct davinci_mcbsp_reg_cfg *config);
int davinci_mcbsp_request(unsigned int id);
void davinci_mcbsp_free(unsigned int id);
void davinci_mcbsp_start(unsigned int id);
void davinci_mcbsp_stop(unsigned int id);
void davinci_mcbsp_xmit_word(unsigned int id, u32 word);
u32 davinci_mcbsp_recv_word(unsigned int id);

int davinci_mcbsp_xmit_buffer(unsigned int id, dma_addr_t buffer,
			      unsigned int length);
int davinci_mcbsp_recv_buffer(unsigned int id, dma_addr_t buffer,
			      unsigned int length);

/* SPI specific API */
void davinci_mcbsp_set_spi_mode(unsigned int id,
				const struct davinci_mcbsp_spi_cfg *spi_cfg);

#endif
