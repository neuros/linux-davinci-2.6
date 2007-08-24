/*
 * linux/drivers/mmc/davinci.c
 *
 * TI DaVinci MMC controller file
 *
 * Copyright (C) 2006 Texas Instruments.
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
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 * ----------------------------------------------------------------------------
 Modifications:
 ver. 1.0: Oct 2005, Purushotam Kumar   Initial version
 ver 1.1:  Nov  2005, Purushotam Kumar  Solved bugs
 ver 1.2:  Jan  2066, Purushotam Kumar   Added card remove insert support
 -
 *

 */

#include <linux/module.h>
#include <linux/tty.h>
#include <linux/ioport.h>
#include <linux/init.h>
#include <linux/console.h>
#include <linux/blkdev.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/mmc/host.h>
#include <linux/mmc/card.h>
#include <linux/mmc/mmc.h>

#include <asm/io.h>
#include <asm/irq.h>
#include <asm/hardware.h>
#include <asm/arch/irqs.h>
#include <asm/arch/hardware.h>

#include "davinci_mmc.h"
#include <asm/arch/edma.h>

/* FIXME: old defines  from old mmc.h */
/* #define MMC_RSP_NONE	(0 << 0) */
/* #define MMC_RSP_SHORT	(1 << 0) */
/* #define MMC_RSP_LONG	(2 << 0) */
/* #define MMC_RSP_MASK	(3 << 0) */
/* #define MMC_RSP_CRC	(1 << 3)		/\* expect valid crc *\/ */
/* #define MMC_RSP_BUSY	(1 << 4)		/\* card may send busy *\/ */
#define MMC_RSP_SHORT	MMC_RSP_PRESENT
#define MMC_RSP_LONG    MMC_RSP_136
#define MMC_RSP_MASK    (MMC_RSP_PRESENT | MMC_RSP_136)

extern void davinci_clean_channel(int ch_no);

/* MMCSD Init clock in Hz in opendain mode */
#define MMCSD_INIT_CLOCK		200000
#define DRIVER_NAME			"mmc0"
#define MMCINT_INTERRUPT		IRQ_MMCINT
#define MMCSD_REGS_BASE_ADDR		DAVINCI_MMC_SD_BASE
#define TCINTEN				(0x1<<20)

/* This macro could not be defined to 0 (ZERO) or -ve value.
 * This value is multiplied to "HZ"
 * while requesting for timer interrupt every time for probing card.
 */
#define MULTIPILER_TO_HZ 1

struct device mmc_dev;
struct clk *mmc_clkp = NULL;
mmcsd_config_def mmcsd_cfg = {
/* read write thresholds (in bytes) can be any power of 2 from 2 to 64 */
	32,
/* To use the DMA or not-- 1- Use DMA, 0-Interrupt mode */
	1
};

volatile mmcsd_regs_base *mmcsd_regs;
static unsigned int mmc_input_clk = 0;

/* Used to identify whether card being used currently by linux core or not */
static unsigned int is_card_busy = 0;
/* used to identify whether card probe(detection) is currently in progress */
static unsigned int is_card_detect_progress = 0;
/* used to identify whether core is icurrently initilizing the card or not */
static unsigned int is_init_progress = 0;
/* used to identify whether core request has been queue up or
 * not because request has come when card detection/probe was in progress
 */
static unsigned int is_req_queued_up = 0;
/* data struture to queue one request */
static struct mmc_host *que_mmc_host = NULL;
/* data structure to queue one request */
static struct mmc_request *que_mmc_request = NULL;

/* tells whether card is initizlzed or not */
static unsigned int is_card_initialized = 0;
static unsigned int new_card_state = 0;	/* tells current state of card */

static DEFINE_SPINLOCK(mmc_lock);

static void mmc_davinci_start_command(struct mmc_davinci_host *host,
		struct mmc_command *cmd)
{
	u32 cmd_reg = 0;
	u32 resp_type = 0;
	u32 cmd_type = 0;
	int byte_cnt = 0, i = 0;
	unsigned long flags;

	dev_dbg(&mmc_dev, "\nMMCSD : CMD%d, argument 0x%08x",
		cmd->opcode, cmd->arg);
	if (cmd->flags & MMC_RSP_SHORT)
		dev_dbg(&mmc_dev, ", 32-bit response");
	if (cmd->flags & MMC_RSP_LONG)
		dev_dbg(&mmc_dev, ", 128-bit response");
	if (cmd->flags & MMC_RSP_CRC)
		dev_dbg(&mmc_dev, ", CRC");
	if (cmd->flags & MMC_RSP_BUSY)
		dev_dbg(&mmc_dev, ", busy notification");
	else
		dev_dbg(&mmc_dev, ", No busy notification");
	dev_dbg(&mmc_dev, "\n");
	host->cmd = cmd;

	/* Protocol layer does not provide response type,
	 * but our hardware needs to know exact type, not just size!
	 */
	switch (cmd->flags & MMC_RSP_MASK) {
	case MMC_RSP_NONE:
		/* resp 0 */
		break;
	case MMC_RSP_SHORT:
		/* resp 1, resp 1b */
		/* OR resp 3!! (assume this if bus is set opendrain) */
		if (host->bus_mode == MMC_BUSMODE_OPENDRAIN) {
			resp_type = 3;
			if (cmd->opcode == 3)
				resp_type = 1;
		} else
			resp_type = 1;
		break;
	case MMC_RSP_LONG:
		/* resp 2 */
		resp_type = 2;
		break;
	}

	/* Protocol layer does not provide command type, but our hardware
	 * needs it!
	 * any data transfer means adtc type (but that information is not
	 * in command structure, so we flagged it into host struct.)
	 * However, telling bc, bcr and ac apart based on response is
	 * not foolproof:
	 * CMD0  = bc  = resp0  CMD15 = ac  = resp0
	 * CMD2  = bcr = resp2  CMD10 = ac  = resp2
	 *
	 * Resolve to best guess with some exception testing:
	 * resp0 -> bc, except CMD15 = ac
	 * rest are ac, except if opendrain
	 */

	if (host->data_dir)
		cmd_type = DAVINCI_MMC_CMDTYPE_ADTC;
	else if (resp_type == 0 && cmd->opcode != 15)
		cmd_type = DAVINCI_MMC_CMDTYPE_BC;
	else if (host->bus_mode == MMC_BUSMODE_OPENDRAIN)
		cmd_type = DAVINCI_MMC_CMDTYPE_BCR;
	else
		cmd_type = DAVINCI_MMC_CMDTYPE_AC;

	/* Set command Busy or not */
	if (cmd->flags & MMC_RSP_BUSY) {
		/*
		 * Linux core sending BUSY which is not defined for cmd 24
		 * as per mmc standard
		 */
		if (cmd->opcode != 24)
			cmd_reg = cmd_reg | (1 << 8);
	}

	/* Set command index */
	cmd_reg |= cmd->opcode;

	/* Setting initialize clock */
	if (cmd->opcode == 0)
		cmd_reg = cmd_reg | (1 << 14);

	/* Set for generating DMA Xfer event */
	if ((host->use_dma == 1) && (host->data != NULL)
			&& ((cmd->opcode == 18) || (cmd->opcode == 25)
				|| (cmd->opcode == 24)
				|| (cmd->opcode == 17)))
		cmd_reg = cmd_reg | (1 << 16);

	/* Setting whether command involves data transfer or not */
	if (cmd_type == DAVINCI_MMC_CMDTYPE_ADTC)
		cmd_reg = cmd_reg | (1 << 13);

	/* Setting whether stream or block transfer */
	if (cmd->flags & MMC_DATA_STREAM)
		cmd_reg = cmd_reg | (1 << 12);

	/* Setting whether data read or write */
	if (host->data_dir == DAVINCI_MMC_DATADIR_WRITE)
		cmd_reg = cmd_reg | (1 << 11);

	/* Setting response type */
	cmd_reg = cmd_reg | (resp_type << 9);

	if (host->bus_mode == MMC_BUSMODE_PUSHPULL)
		cmd_reg = cmd_reg | (1 << 7);

	/* set Command timeout */
	mmcsd_regs->mmc_tor = 0xFFFF;

	/* Enable interrupt */
	if (host->data_dir == DAVINCI_MMC_DATADIR_WRITE) {
		if (host->use_dma != 1)
			mmcsd_regs->mmc_im = MMCSD_EVENT_EOFCMD
					| MMCSD_EVENT_WRITE
					| MMCSD_EVENT_ERROR_CMDCRC
					| MMCSD_EVENT_ERROR_DATACRC
					| MMCSD_EVENT_ERROR_CMDTIMEOUT
					| MMCSD_EVENT_ERROR_DATATIMEOUT
					| MMCSD_EVENT_BLOCK_XFERRED;
		else
			mmcsd_regs->mmc_im = MMCSD_EVENT_EOFCMD
					| MMCSD_EVENT_ERROR_CMDCRC
					| MMCSD_EVENT_ERROR_DATACRC
					| MMCSD_EVENT_ERROR_CMDTIMEOUT
					| MMCSD_EVENT_ERROR_DATATIMEOUT
					| MMCSD_EVENT_BLOCK_XFERRED;
	} else if (host->data_dir == DAVINCI_MMC_DATADIR_READ) {
		if (host->use_dma != 1)
			mmcsd_regs->mmc_im = MMCSD_EVENT_EOFCMD
					| MMCSD_EVENT_READ
					| MMCSD_EVENT_ERROR_CMDCRC
					| MMCSD_EVENT_ERROR_DATACRC
					| MMCSD_EVENT_ERROR_CMDTIMEOUT
					| MMCSD_EVENT_ERROR_DATATIMEOUT
					| MMCSD_EVENT_BLOCK_XFERRED;
		else
			mmcsd_regs->mmc_im = MMCSD_EVENT_EOFCMD
					| MMCSD_EVENT_ERROR_CMDCRC
					| MMCSD_EVENT_ERROR_DATACRC
					| MMCSD_EVENT_ERROR_CMDTIMEOUT
					| MMCSD_EVENT_ERROR_DATATIMEOUT
					| MMCSD_EVENT_BLOCK_XFERRED;
	} else
		mmcsd_regs->mmc_im = MMCSD_EVENT_EOFCMD
				| MMCSD_EVENT_ERROR_CMDCRC
				| MMCSD_EVENT_ERROR_DATACRC
				| MMCSD_EVENT_ERROR_CMDTIMEOUT
				| MMCSD_EVENT_ERROR_DATATIMEOUT;

	/*
	 * It is required by controoler b4 WRITE command that
	 * FIFO should be populated with 32 bytes
	 */
	if ((host->data_dir == DAVINCI_MMC_DATADIR_WRITE)
			&& (cmd_type == DAVINCI_MMC_CMDTYPE_ADTC)
			&& (host->use_dma != 1)) {
		byte_cnt = mmcsd_cfg.rw_threshold;
		host->bytes_left -= mmcsd_cfg.rw_threshold;
		for (i = 0; i < (byte_cnt / 4); i++) {
			mmcsd_regs->mmc_dxr = *host->buffer;
			host->buffer++;
		}
	}

	if (cmd->opcode == 7) {
		spin_lock_irqsave(&mmc_lock, flags);
		new_card_state = 1;
		is_card_initialized = 1;
		host->old_card_state = new_card_state;
		is_init_progress = 0;
		spin_unlock_irqrestore(&mmc_lock, flags);
	}
	if (cmd->opcode == 1) {
		spin_lock_irqsave(&mmc_lock, flags);
		is_init_progress = 1;
		spin_unlock_irqrestore(&mmc_lock, flags);
	}

	host->is_core_command = 1;
	mmcsd_regs->mmc_arghl = cmd->arg;
	mmcsd_regs->mmc_cmd = cmd_reg;

}

static void mmc_davinci_dma_cb(int lch, u16 ch_status, void *data)
{
	int sync_dev = 0;
	struct mmc_davinci_host *host = (struct mmc_davinci_host *)data;

	if (DMA_COMPLETE == ch_status) {

		if (host->cmd == NULL && host->data == NULL) {
			if (host->data_dir == DAVINCI_MMC_DATADIR_READ) {
				sync_dev = DAVINCI_DMA_MMCTXEVT;
			} else {
				sync_dev = DAVINCI_DMA_MMCRXEVT;
			}
			dev_dbg(&mmc_dev,
				"Interrupt from DMA when no request has been made\n");
			davinci_stop_dma(sync_dev);
			return;
		}

		if (host->data_dir == DAVINCI_MMC_DATADIR_READ) {
			sync_dev = DAVINCI_DMA_MMCTXEVT;	/* Write */
		} else {
			sync_dev = DAVINCI_DMA_MMCRXEVT;	/* Read */
		}
		davinci_stop_dma(sync_dev);
	} else {
		/* Handing of Event missed interreupt from DMA */
		dev_dbg(&mmc_dev,
			"Event miss interrupt has been generated by DMA\n");
		if (host->data_dir == DAVINCI_MMC_DATADIR_READ) {
			sync_dev = DAVINCI_DMA_MMCTXEVT;	/* Write */
		} else {
			sync_dev = DAVINCI_DMA_MMCRXEVT;	/* Read */
		}
		davinci_clean_channel(sync_dev);
	}
}

static int mmc_davinci_start_dma_transfer(struct mmc_davinci_host *host,
		struct mmc_request *req)
{
	const char *dev_name;
	int sync_dev, r, edma_ch = 0, tcc = 0;
	unsigned char i, j;
	unsigned short acnt, bcnt, ccnt;
	unsigned int src_port, dst_port, temp_ccnt;
	enum address_mode mode_src, mode_dst;
	enum fifo_width fifo_width_src, fifo_width_dst;
	unsigned short src_bidx, dst_bidx;
	unsigned short src_cidx, dst_cidx;
	unsigned short bcntrld;
	enum sync_dimension sync_mode;
	edmacc_paramentry_regs temp;
	enum dma_event_q queue_no = EVENTQ_0;
	int edma_chan_num;
	unsigned int num_eight_words = (req->data->blocks * 512) / 32;
	static unsigned int option_read = 0;
	static unsigned int option_write = 0;
	static unsigned char dma_read_req = 1;
	static unsigned char dma_write_req = 1;

#define MAX_C_CNT		64000

	if ((req->data->flags & MMC_DATA_WRITE)) {
		sync_dev = DAVINCI_DMA_MMCTXEVT;	/* Write */
		dev_name = "MMC_WRITE";

		if (dma_write_req) {
			r = davinci_request_dma(sync_dev, dev_name,
						mmc_davinci_dma_cb, host,
						&edma_ch, &tcc, queue_no);
			if (r != 0) {
				dev_dbg(&mmc_dev,
					"MMC: davinci_request_dma() failed with %d\n",
r);
				return r;
			}
			dma_write_req = 0;
		}
	} else {
		sync_dev = DAVINCI_DMA_MMCRXEVT;	/* Read */
		dev_name = "MMC_READ";
		if (dma_read_req) {
			r = davinci_request_dma(sync_dev, dev_name,
						mmc_davinci_dma_cb, host,
						&edma_ch, &tcc, queue_no);
			if (r != 0) {
				dev_dbg(&mmc_dev,
					"MMC: davinci_request_dma() failed with %d\n",
					r);
				return r;
			}
			dma_read_req = 0;
		}
	}

	if ((req->data->flags & MMC_DATA_WRITE)) {
		/* AB Sync Transfer */
		/* Acnt =32, Bcnt= , Cnt=1 */

		sync_dev = DAVINCI_DMA_MMCTXEVT;	/* Write */
		acnt = 4;
		bcnt = 8;
		if (num_eight_words > MAX_C_CNT) {
			temp_ccnt = MAX_C_CNT;
			ccnt = temp_ccnt;
		} else {
			ccnt = num_eight_words;
			temp_ccnt = ccnt;
		}

		src_port = (unsigned int)virt_to_phys(req->data->mrq->buffer);
		mode_src = INCR;
		fifo_width_src = W8BIT;	/* It's not cared as modeDsr is INCR */
		src_bidx = 4;
		src_cidx = 32;
		dst_port = MMCSD_REGS_BASE_ADDR + 0x2C;
		mode_dst = INCR;
		fifo_width_dst = W8BIT;	/* It's not cared as modeDsr is INCR */
		dst_bidx = 0;
		dst_cidx = 0;
		bcntrld = 8;
		sync_mode = ABSYNC;

	} else {
		sync_dev = DAVINCI_DMA_MMCRXEVT;	/* Read */
		acnt = 4;
		bcnt = 8;
		if (num_eight_words > MAX_C_CNT) {
			temp_ccnt = MAX_C_CNT;
			ccnt = temp_ccnt;
		} else {
			ccnt = num_eight_words;
			temp_ccnt = ccnt;
		}

		src_port = MMCSD_REGS_BASE_ADDR + 0x28;
		mode_src = INCR;
		fifo_width_src = W8BIT;
		src_bidx = 0;
		src_cidx = 0;
		dst_port = (unsigned int)virt_to_phys(req->data->mrq->buffer);
		mode_dst = INCR;
		fifo_width_dst = W8BIT;	/* It's not cared as modeDsr is INCR */
		dst_bidx = 4;
		dst_cidx = 32;
		bcntrld = 8;
		sync_mode = ABSYNC;
	}

	davinci_set_dma_src_params(sync_dev, src_port, mode_src,
			fifo_width_src);
	davinci_set_dma_dest_params(sync_dev, dst_port, mode_dst,
			fifo_width_dst);
	davinci_set_dma_src_index(sync_dev, src_bidx, src_cidx);
	davinci_set_dma_dest_index(sync_dev, dst_bidx, dst_cidx);
	davinci_set_dma_transfer_params(sync_dev, acnt, bcnt, ccnt, bcntrld,
					sync_mode);

	host->edma_ch_details.cnt_chanel = 0;
	davinci_get_dma_params(sync_dev, &temp);
	if (sync_dev == DAVINCI_DMA_MMCTXEVT) {
		if (option_write == 0) {
			option_write = temp.opt;
		} else {
			temp.opt = option_write;
			davinci_set_dma_params(sync_dev, &temp);
		}
	}
	if (sync_dev == DAVINCI_DMA_MMCRXEVT) {
		if (option_read == 0) {
			option_read = temp.opt;
		} else {
			temp.opt = option_read;
			davinci_set_dma_params(sync_dev, &temp);
		}
	}

	if (num_eight_words > MAX_C_CNT) {	/* Linking will be performed */
		davinci_get_dma_params(sync_dev, &temp);
		temp.opt &= ~TCINTEN;
		davinci_set_dma_params(sync_dev, &temp);

		for (i = 0; i < EDMA_MAX_LOGICAL_CHA_ALLOWED; i++) {
			if (i != 0) {
				j = i - 1;
				davinci_get_dma_params(
					host->edma_ch_details.chanel_num[j],
					&temp);
				temp.opt &= ~TCINTEN;
				davinci_set_dma_params(
					host->edma_ch_details.chanel_num[j],
					&temp);
			}

			host->edma_ch_details.cnt_chanel++;
			davinci_request_dma(DAVINCI_EDMA_PARAM_ANY, "LINK",
					NULL, NULL, &edma_chan_num,
					&sync_dev, queue_no);
			host->edma_ch_details.chanel_num[i] = edma_chan_num;
			ccnt = temp.ccnt & 0x0000FFFF;
			if (sync_dev == DAVINCI_DMA_MMCTXEVT) {
				temp.src = temp.src + (acnt * bcnt * ccnt);
			} else {
				temp.dst = temp.dst + (acnt * bcnt * ccnt);
			}
			temp.opt |= TCINTEN;

			if ((num_eight_words - temp_ccnt) > MAX_C_CNT) {
				temp.ccnt = (temp.ccnt & 0xFFFF0000)
					| MAX_C_CNT;
				ccnt = temp.ccnt & 0x0000FFFF;
				temp_ccnt = temp_ccnt + ccnt;
			} else {
				temp.ccnt = (temp.ccnt & 0xFFFF0000)
					| (num_eight_words -temp_ccnt);
				ccnt = temp.ccnt & 0x0000FFFF;
				temp_ccnt = temp_ccnt + ccnt;
			}
			davinci_set_dma_params(edma_chan_num, &temp);
			if (i != 0) {
				j = i - 1;
				davinci_dma_link_lch(host->edma_ch_details.
						chanel_num[j],
						edma_chan_num);
			}
			if (temp_ccnt == num_eight_words)
				break;
		}
		davinci_dma_link_lch(sync_dev,
				host->edma_ch_details.chanel_num[0]);
	}

	davinci_start_dma(sync_dev);
	return 0;
}

static void
mmc_davinci_prepare_data(struct mmc_davinci_host *host, struct mmc_request *req)
{
	int timeout;

	host->data = req->data;
	if (req->data == NULL) {
		host->data_dir = DAVINCI_MMC_DATADIR_NONE;
		mmcsd_regs->mmc_blen = 0;
		mmcsd_regs->mmc_nblk = 0;
		return;
	}
	dev_dbg(&mmc_dev,
		"MMCSD : Data xfer (%s %s), "
		"DTO %d cycles + %d ns, %d blocks of %d bytes\r\n",
		(req->data->flags & MMC_DATA_STREAM) ? "stream" : "block",
		(req->data->flags & MMC_DATA_WRITE) ? "write" : "read",
		req->data->timeout_clks, req->data->timeout_ns,
		req->data->blocks, req->data->blksz);

	/* Convert ns to clock cycles by assuming 20MHz frequency
	 * 1 cycle at 20MHz = 500 ns
	 */
	timeout = req->data->timeout_clks + req->data->timeout_ns / 500;
	if (timeout > 0xffff)
		timeout = 0xffff;

	mmcsd_regs->mmc_tod = timeout;
	mmcsd_regs->mmc_nblk = req->data->blocks;
	mmcsd_regs->mmc_blen = req->data->blksz;
	host->data_dir = (req->data->flags & MMC_DATA_WRITE)
			? DAVINCI_MMC_DATADIR_WRITE
			: DAVINCI_MMC_DATADIR_READ;

	/* Configure the FIFO */
	switch (host->data_dir) {
	case DAVINCI_MMC_DATADIR_WRITE:
		mmcsd_regs->mmc_fifo_ctl = mmcsd_regs->mmc_fifo_ctl | 0x1;
		mmcsd_regs->mmc_fifo_ctl = 0x0;
		mmcsd_regs->mmc_fifo_ctl = mmcsd_regs->mmc_fifo_ctl | (1 << 1);
		mmcsd_regs->mmc_fifo_ctl = mmcsd_regs->mmc_fifo_ctl | (1 << 2);
		break;

	case DAVINCI_MMC_DATADIR_READ:
		mmcsd_regs->mmc_fifo_ctl = mmcsd_regs->mmc_fifo_ctl | 0x1;
		mmcsd_regs->mmc_fifo_ctl = 0x0;
		mmcsd_regs->mmc_fifo_ctl = mmcsd_regs->mmc_fifo_ctl | (1 << 2);
		break;
	default:
		break;
	}

	if ((host->use_dma == 1)
			&& (mmc_davinci_start_dma_transfer(host, req) == 0)) {
		host->buffer = NULL;
		host->bytes_left = 0;
	} else {
		/* Revert to CPU Copy */
		host->buffer = (u32 *) (req->data->mrq->buffer);
		host->bytes_left = req->data->blocks * req->data->blksz;
		host->use_dma = 0;
	}
}

static void mmc_davinci_request(struct mmc_host *mmc, struct mmc_request *req)
{
	struct mmc_davinci_host *host = mmc_priv(mmc);
	unsigned long flags;

	if (!is_card_detect_progress) {
		spin_lock_irqsave(&mmc_lock, flags);
		is_card_busy = 1;
		spin_unlock_irqrestore(&mmc_lock, flags);
		mmc_davinci_prepare_data(host, req);
		mmc_davinci_start_command(host, req->cmd);
	} else {
		/* Queu up the request as card dectection is being excuted */
		que_mmc_host = mmc;
		que_mmc_request = req;
		spin_lock_irqsave(&mmc_lock, flags);
		is_req_queued_up = 1;
		spin_unlock_irqrestore(&mmc_lock, flags);
	}
}

static unsigned int calculate_freq_for_card(unsigned int mmc_req_freq)
{
	unsigned int mmc_freq = 0, cpu_arm_clk = 0, mmc_push_pull = 0;

	cpu_arm_clk = mmc_input_clk;
	if (cpu_arm_clk > (2 * mmc_req_freq))
		mmc_push_pull = ((unsigned int)cpu_arm_clk
				/ (2 * mmc_req_freq)) - 1;
	else
		mmc_push_pull = 0;

	mmc_freq = (unsigned int)cpu_arm_clk / (2 * (mmc_push_pull + 1));

	if (mmc_freq > mmc_req_freq)
		mmc_push_pull = mmc_push_pull + 1;

	return mmc_push_pull;
}

static void mmc_davinci_set_ios(struct mmc_host *mmc, struct mmc_ios *ios)
{
	unsigned short status;
	unsigned int open_drain_freq = 0, cpu_arm_clk = 0;
	unsigned int mmc_push_pull_freq = 0;
	struct mmc_davinci_host *host = mmc_priv(mmc);

	cpu_arm_clk = mmc_input_clk;
	dev_dbg(&mmc_dev, "clock %dHz busmode %d powermode %d Vdd %d.%02d\r\n",
		ios->clock, ios->bus_mode, ios->power_mode,
		ios->vdd / 100, ios->vdd % 100);

	if (ios->bus_mode == MMC_BUSMODE_OPENDRAIN) {
		open_drain_freq = ((unsigned int)cpu_arm_clk
				/ (2 * MMCSD_INIT_CLOCK)) - 1;
		mmcsd_regs->mmc_clk = (mmcsd_regs->mmc_clk & ~(0xFF))
				| open_drain_freq;
	} else {
		mmc_push_pull_freq = calculate_freq_for_card(ios->clock);
		mmcsd_regs->mmc_clk = (mmcsd_regs->mmc_clk & ~(0xFF))
				| mmc_push_pull_freq;
	}
	host->bus_mode = ios->bus_mode;
	if (ios->power_mode == MMC_POWER_UP) {
		/* Send clock cycles, poll completion */
		mmcsd_regs->mmc_arghl = 0x0;
		mmcsd_regs->mmc_cmd = 0x4000;
		status = 0;
		while (!(status & (MMCSD_EVENT_EOFCMD))) {
			status = mmcsd_regs->mmc_st0;
		}
	}
}

static void
mmc_davinci_xfer_done(struct mmc_davinci_host *host, struct mmc_data *data)
{
	unsigned long flags;

	host->data = NULL;
	host->data_dir = DAVINCI_MMC_DATADIR_NONE;
	if (data->error == MMC_ERR_NONE)
		data->bytes_xfered += data->blocks * data->blksz;

	if (data->error == MMC_ERR_TIMEOUT) {
		spin_lock_irqsave(&mmc_lock, flags);
		is_card_busy = 0;
		spin_unlock_irqrestore(&mmc_lock, flags);
		mmc_request_done(host->mmc, data->mrq);
		return;
	}

	if (!data->stop) {
		host->req = NULL;
		spin_lock_irqsave(&mmc_lock, flags);
		is_card_busy = 0;
		spin_unlock_irqrestore(&mmc_lock, flags);
		mmc_request_done(host->mmc, data->mrq);
		return;
	}
	mmc_davinci_start_command(host, data->stop);
}

static void mmc_davinci_cmd_done(struct mmc_davinci_host *host,
				 struct mmc_command *cmd)
{
	unsigned long flags;

	host->cmd = NULL;
	switch (cmd->flags & MMC_RSP_MASK) {
	case MMC_RSP_NONE:
		/* resp 0 */
		break;

	case MMC_RSP_SHORT:
		/* response types 1, 1b, 3, 4, 5, 6 */
		cmd->resp[0] = mmcsd_regs->mmc_rsp67;
		break;

	case MMC_RSP_LONG:
		/* response type 2 */
		cmd->resp[3] = mmcsd_regs->mmc_rsp01;
		cmd->resp[2] = mmcsd_regs->mmc_rsp23;
		cmd->resp[1] = mmcsd_regs->mmc_rsp45;
		cmd->resp[0] = mmcsd_regs->mmc_rsp67;
		break;
	}

	if (host->data == NULL || cmd->error != MMC_ERR_NONE) {
		host->req = NULL;
		if (cmd->error == MMC_ERR_TIMEOUT)
			cmd->mrq->cmd->retries = 0;
		spin_lock_irqsave(&mmc_lock, flags);
		is_card_busy = 0;
		spin_unlock_irqrestore(&mmc_lock, flags);
		mmc_request_done(host->mmc, cmd->mrq);
	}
}

static irqreturn_t mmc_davinci_irq(int irq, void *dev_id)
{
	struct mmc_davinci_host *host = (struct mmc_davinci_host *)dev_id;
	u16 status;
	int end_command;
	int end_transfer;
	int byte_cnt = 0, i = 0;
	unsigned long flags;

	if (host->is_core_command) {
		if (host->cmd == NULL && host->data == NULL) {
			status = mmcsd_regs->mmc_st0;
			dev_dbg(&mmc_dev, "Spurious interrupt 0x%04x\r\n",
				status);
			/* Disable the interrupt from mmcsd */
			mmcsd_regs->mmc_im = 0;
			return IRQ_HANDLED;
		}
	}
	end_command = 0;
	end_transfer = 0;

	status = mmcsd_regs->mmc_st0;
	if (status == 0)
		return IRQ_HANDLED;

	if (host->is_core_command) {
		if (is_card_initialized) {
			if (new_card_state == 0) {
				if (host->cmd) {
					host->cmd->error |= MMC_ERR_TIMEOUT;
					mmc_davinci_cmd_done(host, host->cmd);
				}
				dev_dbg(&mmc_dev,
					"From code segment excuted when card removed\n");
				return IRQ_HANDLED;
			}
		}

		while (status != 0) {
			if (host->data_dir == DAVINCI_MMC_DATADIR_WRITE) {
				if (status & MMCSD_EVENT_WRITE) {
					/* Buffer almost empty */
					if (host->bytes_left > 0) {
						byte_cnt =
							mmcsd_cfg.rw_threshold;
						host->bytes_left -=
							mmcsd_cfg.rw_threshold;
						for (i = 0; i < (byte_cnt / 4);
								i++) {
							mmcsd_regs->mmc_dxr =
								*host->buffer;
							host->buffer++;
						}
					}
				}
			}

			if (host->data_dir == DAVINCI_MMC_DATADIR_READ) {
				if (status & MMCSD_EVENT_READ) {
					/* Buffer almost empty */
					if (host->bytes_left > 0) {
						byte_cnt =
							mmcsd_cfg.rw_threshold;
						host->bytes_left -=
							mmcsd_cfg.rw_threshold;
						for (i = 0; i < (byte_cnt / 4);
								i++) {
							*host->buffer =
								mmcsd_regs->
									mmc_drr;
							host->buffer++;
						}
					}
				}
			}

			if (status & MMCSD_EVENT_BLOCK_XFERRED) {
				/* Block sent/received */
				if (host->data != NULL) {
					end_transfer = 1;
				}
			}

			if (status & MMCSD_EVENT_ERROR_DATATIMEOUT) {
				/* Data timeout */
				if ((host->data) && (new_card_state != 0)) {
					host->data->error |= MMC_ERR_TIMEOUT;
					spin_lock_irqsave(&mmc_lock, flags);
					new_card_state = 0;
					is_card_initialized = 0;
					spin_unlock_irqrestore(&mmc_lock,
						flags);
					dev_dbg(&mmc_dev,
						"MMCSD: Data timeout, CMD%d and status is %x\r\n",
						host->cmd->opcode, status);
					end_transfer = 1;
					host->cmd->error |= MMC_ERR_TIMEOUT;
				}
				dev_dbg(&mmc_dev,
					"MMCSD: Data timeout, CMD%d and status is %x\r\n",
					host->cmd->opcode, status);
			}

			if (status & MMCSD_EVENT_ERROR_DATACRC) {
				/* Data CRC error */
				if (host->data) {
					host->data->error |= MMC_ERR_BADCRC;
					dev_dbg(&mmc_dev,
						"MMCSD: Data CRC error, bytes left %d\r\n",
						host->bytes_left);
					end_transfer = 1;
				} else {
					dev_dbg(&mmc_dev,
						"MMCSD: Data CRC error\r\n");
				}
			}

			if (status & MMCSD_EVENT_ERROR_CMDTIMEOUT) {
				/* Command timeout */
				if (host->cmd) {
					/* Timeouts are normal in case of
					 * MMC_SEND_STATUS
					 */
					if (host->cmd->opcode !=
							MMC_ALL_SEND_CID) {
						dev_dbg(&mmc_dev,
							"MMCSD: CMD%d timeout,"
							" status %x\r\n",
							host->cmd->opcode,
							status);
						spin_lock_irqsave(&mmc_lock,
							flags);
						new_card_state = 0;
						is_card_initialized = 0;
						spin_unlock_irqrestore(
							&mmc_lock, flags);
					}
					host->cmd->error |= MMC_ERR_TIMEOUT;
					end_command = 1;

				}
			}

			if (status & MMCSD_EVENT_ERROR_CMDCRC) {
				/* Command CRC error */
				dev_dbg(&mmc_dev, "Command CRC error\r\n");
				if (host->cmd) {
					host->cmd->error |= MMC_ERR_BADCRC;
					end_command = 1;
				}
			}

			if (status & MMCSD_EVENT_EOFCMD) {
				/* End of command phase */
				end_command = 1;
			}

			if (host->data == NULL) {
				status = mmcsd_regs->mmc_st0;
				if (status != 0) {
					dev_dbg(&mmc_dev,
						"Status is %x at end of ISR when host->data is NULL",
						status);
					status = 0;

				}
			} else {
				status = mmcsd_regs->mmc_st0;
			}
		}

		if (end_command)
			mmc_davinci_cmd_done(host, host->cmd);
		if (end_transfer)
			mmc_davinci_xfer_done(host, host->data);

	} else {
		if (host->cmd_code == 13) {
			if (status & MMCSD_EVENT_EOFCMD) {
				spin_lock_irqsave(&mmc_lock, flags);
				new_card_state = 1;
				spin_unlock_irqrestore(&mmc_lock, flags);

			} else {
				spin_lock_irqsave(&mmc_lock, flags);
				new_card_state = 0;
				is_card_initialized = 0;
				spin_unlock_irqrestore(&mmc_lock, flags);
			}

			spin_lock_irqsave(&mmc_lock, flags);
			is_card_detect_progress = 0;
			spin_unlock_irqrestore(&mmc_lock, flags);

			if (is_req_queued_up) {
				mmc_davinci_request(que_mmc_host,
						que_mmc_request);
				spin_lock_irqsave(&mmc_lock, flags);
				is_req_queued_up = 0;
				spin_unlock_irqrestore(&mmc_lock, flags);
			}

		}

		if (host->cmd_code == 1) {
			if (status & MMCSD_EVENT_EOFCMD) {
				spin_lock_irqsave(&mmc_lock, flags);
				new_card_state = 1;
				is_card_initialized = 0;
				spin_unlock_irqrestore(&mmc_lock, flags);
			} else {

				spin_lock_irqsave(&mmc_lock, flags);
				new_card_state = 0;
				is_card_initialized = 0;
				spin_unlock_irqrestore(&mmc_lock, flags);
			}

			spin_lock_irqsave(&mmc_lock, flags);
			is_card_detect_progress = 0;
			spin_unlock_irqrestore(&mmc_lock, flags);

			if (is_req_queued_up) {
				mmc_davinci_request(que_mmc_host,
						que_mmc_request);
				spin_lock_irqsave(&mmc_lock, flags);
				is_req_queued_up = 0;
				spin_unlock_irqrestore(&mmc_lock, flags);
			}

		}

		if (host->cmd_code == 0) {
			if (status & MMCSD_EVENT_EOFCMD) {
				host->is_core_command = 0;
				host->cmd_code = 1;
				dev_dbg(&mmc_dev,
					"MMC-Probing mmc with cmd1\n");
				/* Issue cmd1 */
				mmcsd_regs->mmc_arghl = 0x80300000;
				mmcsd_regs->mmc_cmd = 0x00000601;

			} else {
				spin_lock_irqsave(&mmc_lock, flags);
				new_card_state = 0;
				is_card_initialized = 0;
				is_card_detect_progress = 0;
				spin_unlock_irqrestore(&mmc_lock, flags);
			}
		}

	}
	return IRQ_HANDLED;
}

static struct mmc_host_ops mmc_davinci_ops = {
	.request = mmc_davinci_request,
	.set_ios = mmc_davinci_set_ios,
};

void mmc_check_card(unsigned long data)
{
	struct mmc_davinci_host *host = (struct mmc_davinci_host *)data;
	unsigned long flags;

	if ((!is_card_detect_progress) || (!is_init_progress)) {
		if (is_card_initialized) {
			host->is_core_command = 0;
			host->cmd_code = 13;
			spin_lock_irqsave(&mmc_lock, flags);
			is_card_detect_progress = 1;
			spin_unlock_irqrestore(&mmc_lock, flags);
			/* Issue cmd13 */
			mmcsd_regs->mmc_arghl = 0x10000;
			mmcsd_regs->mmc_cmd = 0x0000028D;
		} else {
			host->is_core_command = 0;
			host->cmd_code = 0;
			spin_lock_irqsave(&mmc_lock, flags);
			is_card_detect_progress = 1;
			spin_unlock_irqrestore(&mmc_lock, flags);
			/* Issue cmd0 */
			mmcsd_regs->mmc_arghl = 0;
			mmcsd_regs->mmc_cmd = 0x4000;
		}
		mmcsd_regs->mmc_im = MMCSD_EVENT_EOFCMD
				| MMCSD_EVENT_ERROR_CMDCRC
				| MMCSD_EVENT_ERROR_DATACRC
				| MMCSD_EVENT_ERROR_CMDTIMEOUT
				| MMCSD_EVENT_ERROR_DATATIMEOUT;
	}
}

static void davinci_mmc_check_status(unsigned long data)
{
	unsigned long flags;
	struct mmc_davinci_host *host = (struct mmc_davinci_host *)data;

	if (!is_card_busy) {
		if (host->old_card_state ^ new_card_state) {
			mmc_detect_change(host->mmc, 0);
			spin_lock_irqsave(&mmc_lock, flags);
			host->old_card_state = new_card_state;
			spin_unlock_irqrestore(&mmc_lock, flags);
		} else
			mmc_check_card(data);
	}
	mod_timer(&host->timer, jiffies + MULTIPILER_TO_HZ * HZ);
}

static void init_mmcsd_host(void)
{
	/* CMD line portion is diabled and in reset state */
	mmcsd_regs->mmc_ctl = mmcsd_regs->mmc_ctl | 0x1;
	/* DAT line portion is diabled and in reset state */
	mmcsd_regs->mmc_ctl = mmcsd_regs->mmc_ctl | (1 << 1);

	mmcsd_regs->mmc_clk = 0x0;
	mmcsd_regs->mmc_clk = mmcsd_regs->mmc_clk | (1 << 8);

	mmcsd_regs->mmc_tor = 0xFFFF;
	mmcsd_regs->mmc_tod = 0xFFFF;

	mmcsd_regs->mmc_ctl = mmcsd_regs->mmc_ctl & ~(0x1);
	mmcsd_regs->mmc_ctl = mmcsd_regs->mmc_ctl & ~(1 << 1);
}

static int davinci_mmcsd_probe(struct platform_device *pdev)
{
	struct mmc_davinci_host *host;
	struct mmc_host *mmc;
	int ret;

	mmc = mmc_alloc_host(sizeof(struct mmc_davinci_host), &pdev->dev);
	if (!mmc) {
		ret = -ENOMEM;
		goto out;
	}

	mmcsd_regs =
		(volatile mmcsd_regs_base *)IO_ADDRESS(MMCSD_REGS_BASE_ADDR);

	init_mmcsd_host();

	mmc->ops = &mmc_davinci_ops;
	mmc->f_min = 312500;
	mmc->f_max = 20000000;
	mmc->ocr_avail = MMC_VDD_32_33;

	host = mmc_priv(mmc);
	host->mmc = mmc;	/* Important */

	host->use_dma = mmcsd_cfg.use_dma;
	host->irq = MMCINT_INTERRUPT;
	host->sd_support = 1;
	ret = request_irq(MMCINT_INTERRUPT, mmc_davinci_irq, 0, DRIVER_NAME,
			host);

	if (ret)
		goto out;

	platform_set_drvdata(pdev, host);
	mmc_add_host(mmc);

	init_timer(&host->timer);
	host->timer.data = (unsigned long)host;
	host->timer.function = davinci_mmc_check_status;
	host->timer.expires = jiffies + MULTIPILER_TO_HZ * HZ;
	add_timer(&host->timer);

	return 0;

out:
	/* TBD: Free other resources too. */

	return ret;
}

static int davinci_mmcsd_remove(struct platform_device *pdev)
{
	struct mmc_davinci_host *host = platform_get_drvdata(pdev);

	platform_set_drvdata(pdev, NULL);
	mmc_remove_host(host->mmc);
	free_irq(host->irq, host);
	del_timer(&host->timer);
	davinci_free_dma(DAVINCI_DMA_MMCTXEVT);
	davinci_free_dma(DAVINCI_DMA_MMCRXEVT);
	return 0;

}

#ifdef CONFIG_PM
static int davinci_mmcsd_suspend(struct platform_device *pdev, pm_message_t msg)
{
	struct mmc_davinci_host *host = platform_get_drvdata(pdev);

	return mmc_suspend_host(host->mmc, msg);
}

static int davinci_mmcsd_resume(struct platform_device *pdev)
{
	struct mmc_davinci_host *host = platform_get_drvdata(pdev);

	return mmc_resume_host(host->mmc);
}

#else

#define davinci_mmcsd_suspend	NULL
#define davinci_mmcsd_resume	NULL

#endif

static struct platform_driver davinci_mmcsd_driver = {
	.driver = {
		.name = DRIVER_NAME,
		.owner = THIS_MODULE,
	},
	.probe = davinci_mmcsd_probe,
	.remove = davinci_mmcsd_remove,
	.suspend = davinci_mmcsd_suspend,
	.resume = davinci_mmcsd_resume,
};

/* FIXME don't be a legacy driver ... register the device as part
 * of cpu-specific board setup.
 */
static void mmc_release(struct device *dev)
{
	/* Nothing to release? */
}

static u64 mmc_dma_mask = 0xffffffff;

static struct resource mmc_resources[] = {
	{
		.start = IO_ADDRESS(MMCSD_REGS_BASE_ADDR),
		.end = IO_ADDRESS((MMCSD_REGS_BASE_ADDR) + 0x74),
		.flags = IORESOURCE_MEM,
	},
	{
		.start = MMCINT_INTERRUPT,
		.flags = IORESOURCE_IRQ,
	},
};

static struct platform_device mmc_davinci_device = {
	.name = DRIVER_NAME,
	.id = 1,
	.dev = {
		.release = mmc_release,
		.dma_mask = &mmc_dma_mask,
	},
	.num_resources = ARRAY_SIZE(mmc_resources),
	.resource = mmc_resources,
};

static int davinci_mmcsd_init(void)
{
	int ret = 0;
	struct clk *clkp = NULL;

	clkp = clk_get(NULL, "MMCSDCLK");
	if (clkp != NULL) {
		mmc_clkp = clkp;
		clk_enable(mmc_clkp);
		mmc_input_clk = clk_get_rate(mmc_clkp);

		ret = platform_device_register(&mmc_davinci_device);
		if (ret != 0)
			goto free1;

		ret = platform_driver_register(&davinci_mmcsd_driver);
		if (ret == 0)
			return 0;

free1:
		platform_device_unregister(&mmc_davinci_device);
	}

	return -ENODEV;
}

static void __exit davinci_mmcsd_exit(void)
{
	platform_driver_unregister(&davinci_mmcsd_driver);
	platform_device_unregister(&mmc_davinci_device);
	clk_disable(mmc_clkp);
}

module_init(davinci_mmcsd_init);
module_exit(davinci_mmcsd_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("MMCSD driver for Davinci MMC controller");
