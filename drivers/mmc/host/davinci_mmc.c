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
 ver 1.2:  Jan  2006, Purushotam Kumar   Added card remove insert support
 -
 *
 */

#include <linux/module.h>
#include <linux/version.h>
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
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <asm/arch/irqs.h>
#include <asm/arch/hardware.h>

#include "davinci_mmc.h"
#include <asm/arch/edma.h>

extern void davinci_clean_channel(int ch_no);

/* MMCSD Init clock in Hz in opendain mode */
#define MMCSD_INIT_CLOCK 		200000
#define DRIVER_NAME 			"mmc0"
#define MMCINT_INTERRUPT    		IRQ_MMCINT
#define MMCSD_REGS_BASE_ADDR  		DAVINCI_MMC_SD_BASE
#define TCINTEN 			(0x1<<20)

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
	1,
/* flag Indicates 1bit/4bit mode */
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
static unsigned int is_card_removed = 0;

static DEFINE_SPINLOCK(mmc_lock);

#define RSP_TYPE(x)	((x) & ~(MMC_RSP_BUSY|MMC_RSP_OPCODE))
#define MMCST1_BUSY	(1 << 0)

static inline void wait_on_data(void)
{
	int cnt = 900000;
	while (((mmcsd_regs->mmc_st1) & MMCST1_BUSY) && cnt) {
		cnt--;
		udelay(1);
	}
	if (!cnt) {
		dev_warn(&mmc_dev, "ERROR: TOUT waiting for BUSY\n");
	}
}

static void mmc_davinci_start_command(struct mmc_davinci_host *host,
		struct mmc_command *cmd)
{
	u32 cmd_reg = 0;
	u32 resp_type = 0;
	u32 cmd_type = 0;
	unsigned long flags;

#ifdef CONFIG_MMC_DEBUG
	dev_dbg(&mmc_dev, "\nMMCSD : CMD%d, argument 0x%08x",
		cmd->opcode, cmd->arg);
	switch (RSP_TYPE(mmc_resp_type(cmd))) {
	case RSP_TYPE(MMC_RSP_R1):
		dev_dbg(&mmc_dev, ", R1/R1b response");
		break;
	case RSP_TYPE(MMC_RSP_R2):
		dev_dbg(&mmc_dev, ", R2 response");
		break;
	case RSP_TYPE(MMC_RSP_R3):
		dev_dbg(&mmc_dev, ", R3 response");
		break;
	default:
		break;
	}
	dev_dbg(&mmc_dev, "\n");
#endif
	host->cmd = cmd;

	/* Protocol layer does not provide response type,
	 * but our hardware needs to know exact type, not just size!
	 */
	switch (RSP_TYPE(mmc_resp_type(cmd))) {
	case MMC_RSP_NONE:
		/* resp 0 */
		break;
	case RSP_TYPE(MMC_RSP_R1):
		resp_type = 1;
		break;
	case RSP_TYPE(MMC_RSP_R2):
		resp_type = 2;
		break;
	case RSP_TYPE(MMC_RSP_R3):
		resp_type = 3;
		break;
	default:
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

	if (mmc_cmd_type(cmd) == MMC_CMD_ADTC)
		cmd_type = DAVINCI_MMC_CMDTYPE_ADTC;
	else if (mmc_cmd_type(cmd) == MMC_CMD_BC)
		cmd_type = DAVINCI_MMC_CMDTYPE_BC;
	else if (mmc_cmd_type(cmd) == MMC_CMD_BCR)
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
	if ((host->do_dma == 1) && (host->data != NULL)
	    && ((cmd->opcode == 18) || (cmd->opcode == 25)
		|| (cmd->opcode == 24) || (cmd->opcode == 17)))
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
		if (host->do_dma != 1)
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
		if (host->do_dma != 1)
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
			&& (host->do_dma != 1))
		/* Fill the FIFO for Tx */
		davinci_fifo_data_trans(host);

	if (cmd->opcode == 7) {
		spin_lock_irqsave(&mmc_lock, flags);
		is_card_removed = 0;
		new_card_state = 1;
		is_card_initialized = 1;
		host->old_card_state = new_card_state;
		is_init_progress = 0;
		spin_unlock_irqrestore(&mmc_lock, flags);
	}
	if (cmd->opcode == 1 || cmd->opcode == 41) {
		spin_lock_irqsave(&mmc_lock, flags);
		is_card_initialized = 0;
		is_init_progress = 1;
		spin_unlock_irqrestore(&mmc_lock, flags);
	}

	host->is_core_command = 1;
	mmcsd_regs->mmc_arghl = cmd->arg;
	mmcsd_regs->mmc_cmd = cmd_reg;

}

static void mmc_davinci_dma_cb(int lch, u16 ch_status, void *data)
{
	if (DMA_COMPLETE != ch_status) {
		struct mmc_davinci_host *host = (struct mmc_davinci_host *)data;
		dev_warn(&mmc_dev, "[DMA FAILED]");
		davinci_abort_dma(host);
	}
}

static void davinci_fifo_data_trans(struct mmc_davinci_host *host)
{
	int n, i;

	if (host->buffer_bytes_left == 0) {
		host->sg_idx++;
		BUG_ON(host->sg_idx == host->sg_len);
		mmc_davinci_sg_to_buf(host);
	}

	n = mmcsd_cfg.rw_threshold;
	if (n > host->buffer_bytes_left)
		n = host->buffer_bytes_left;
	host->buffer_bytes_left -= n;
	host->bytes_left -= n;

	if (host->data_dir == DAVINCI_MMC_DATADIR_WRITE) {
		for (i = 0; i < (n / 4); i++) {
			mmcsd_regs->mmc_dxr = *host->buffer;
			host->buffer++;
		}
	} else {
		for (i = 0; i < (n / 4); i++) {
			*host->buffer = mmcsd_regs->mmc_drr;
			host->buffer++;
		}
	}
}

static void davinci_reinit_chan(void)
{
	int sync_dev;

	sync_dev = DAVINCI_DMA_MMCTXEVT;
	davinci_stop_dma(sync_dev);
	davinci_clean_channel(sync_dev);

	sync_dev = DAVINCI_DMA_MMCRXEVT;
	davinci_stop_dma(sync_dev);
	davinci_clean_channel(sync_dev);
}

static void davinci_abort_dma(struct mmc_davinci_host *host)
{
	int sync_dev = 0;

	if (host->data_dir == DAVINCI_MMC_DATADIR_READ)
		sync_dev = DAVINCI_DMA_MMCTXEVT;
	else
		sync_dev = DAVINCI_DMA_MMCRXEVT;

	davinci_stop_dma(sync_dev);
	davinci_clean_channel(sync_dev);

}

static int mmc_davinci_start_dma_transfer(struct mmc_davinci_host *host,
		struct mmc_request *req)
{
	int use_dma = 1, i;
	struct mmc_data *data = host->data;
	int block_size = data->blksz;

	host->sg_len = dma_map_sg(mmc_dev(host->mmc), data->sg, host->sg_len,
				((data->flags & MMC_DATA_WRITE)
				? DMA_TO_DEVICE
				: DMA_FROM_DEVICE));

	/* Decide if we can use DMA */
	for (i = 0; i < host->sg_len; i++) {
		if ((data->sg[i].length % block_size) != 0) {
			use_dma = 0;
			break;
		}
	}

	if (!use_dma) {
		dma_unmap_sg(mmc_dev(host->mmc), data->sg, host->sg_len,
				  (data->flags & MMC_DATA_WRITE)
				? DMA_TO_DEVICE
				: DMA_FROM_DEVICE);
		return -1;
	}

	host->do_dma = 1;

	host->dma_state = 0;

	mmc_davinci_send_dma_request(host, req);

	return 0;

}

static int davinci_release_dma_channels(struct mmc_davinci_host *host)
{
	davinci_free_dma(DAVINCI_DMA_MMCTXEVT);
	davinci_free_dma(DAVINCI_DMA_MMCRXEVT);

	if (host->edma_ch_details.cnt_chanel) {
		davinci_free_dma(host->edma_ch_details.chanel_num[0]);
		host->edma_ch_details.cnt_chanel = 0;
	}

	return 0;
}

static int davinci_acquire_dma_channels(struct mmc_davinci_host *host)
{
	int edma_chan_num, tcc = 0, r, sync_dev;
	enum dma_event_q queue_no = EVENTQ_0;

	/* Acquire master DMA write channel */
	if ((r = davinci_request_dma(DAVINCI_DMA_MMCTXEVT, "MMC_WRITE",
			mmc_davinci_dma_cb, host, &edma_chan_num, &tcc,
			queue_no)) != 0) {
		dev_warn(&mmc_dev,
				"MMC: davinci_request_dma() failed with %d\n",
				r);
		return r;
	}

	/* Acquire master DMA read channel */
	if ((r = davinci_request_dma(DAVINCI_DMA_MMCRXEVT, "MMC_READ",
			mmc_davinci_dma_cb, host, &edma_chan_num, &tcc,
			queue_no)) != 0) {
		dev_warn(&mmc_dev,
				"MMC: davinci_request_dma() failed with %d\n",
				r);
		goto free_master_write;
	}

	host->edma_ch_details.cnt_chanel = 0;

	/* currently data Writes are done using single block mode,
	 * so no DMA slave write channel is required for now */

	/* Create a DMA slave read channel
	 * (assuming max segments handled is 2) */
	sync_dev = DAVINCI_DMA_MMCRXEVT;
	if ((r = davinci_request_dma(DAVINCI_EDMA_PARAM_ANY, "LINK",
			NULL, NULL, &edma_chan_num, &sync_dev,
			queue_no)) != 0) {
		dev_warn(&mmc_dev,
			"MMC: davinci_request_dma() failed with %d\n", r);
		goto free_master_read;
	}

	host->edma_ch_details.cnt_chanel++;
	host->edma_ch_details.chanel_num[0] = edma_chan_num;

	return 0;

free_master_read:
	davinci_free_dma(DAVINCI_DMA_MMCRXEVT);
free_master_write:
	davinci_free_dma(DAVINCI_DMA_MMCTXEVT);

	return r;
}

static int mmc_davinci_send_dma_request(struct mmc_davinci_host *host,
					struct mmc_request *req)
{
	int sync_dev;
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
	int edma_chan_num;
	static unsigned int option_read = 0;
	static unsigned int option_write = 0;
	struct mmc_data *data = host->data;
	struct scatterlist *sg = &data->sg[0];
	unsigned int count;
	int num_frames, frame;

#define MAX_C_CNT		64000

	frame = data->blksz;
	count = sg_dma_len(sg);

	if ((data->blocks == 1) && (count > data->blksz)) {
		count = frame;
	}

	if (count % 32 == 0) {
		acnt = 4;
		bcnt = 8;
		num_frames = count / 32;
	} else {
		acnt = count;
		bcnt = 1;
		num_frames = 1;
	}

	if (num_frames > MAX_C_CNT) {
		temp_ccnt = MAX_C_CNT;
		ccnt = temp_ccnt;
	} else {
		ccnt = num_frames;
		temp_ccnt = ccnt;
	}

	if (host->data_dir == DAVINCI_MMC_DATADIR_WRITE) {
		/*AB Sync Transfer */
		sync_dev = DAVINCI_DMA_MMCTXEVT;

		src_port = (unsigned int)sg_dma_address(sg);
		mode_src = INCR;
		fifo_width_src = W8BIT;	/* It's not cared as modeDsr is INCR */
		src_bidx = acnt;
		src_cidx = acnt * bcnt;
		dst_port = MMCSD_REGS_BASE_ADDR + 0x2C;
		mode_dst = INCR;
		fifo_width_dst = W8BIT;	/* It's not cared as modeDsr is INCR */
		dst_bidx = 0;
		dst_cidx = 0;
		bcntrld = 8;
		sync_mode = ABSYNC;

	} else {
		sync_dev = DAVINCI_DMA_MMCRXEVT;

		src_port = MMCSD_REGS_BASE_ADDR + 0x28;
		mode_src = INCR;
		fifo_width_src = W8BIT;
		src_bidx = 0;
		src_cidx = 0;
		dst_port = (unsigned int)sg_dma_address(sg);
		mode_dst = INCR;
		fifo_width_dst = W8BIT;	/* It's not cared as modeDsr is INCR */
		dst_bidx = acnt;
		dst_cidx = acnt * bcnt;
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

	if (host->sg_len > 1) {
		davinci_get_dma_params(sync_dev, &temp);
		temp.opt &= ~TCINTEN;
		davinci_set_dma_params(sync_dev, &temp);

		for (i = 0; i < host->sg_len - 1; i++) {
			sg = &data->sg[i + 1];

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

			edma_chan_num = host->edma_ch_details.chanel_num[0];

			frame = data->blksz;
			count = sg_dma_len(sg);

			if ((data->blocks == 1) && (count > data->blksz))
				count = frame;

			ccnt = count / 32;

			if (sync_dev == DAVINCI_DMA_MMCTXEVT)
				temp.src = (unsigned int)sg_dma_address(sg);
			else
				temp.dst = (unsigned int)sg_dma_address(sg);
			temp.opt |= TCINTEN;

			temp.ccnt = (temp.ccnt & 0xFFFF0000) | (ccnt);

			davinci_set_dma_params(edma_chan_num, &temp);
			if (i != 0) {
				j = i - 1;
				davinci_dma_link_lch(host->edma_ch_details.
						chanel_num[j],
						edma_chan_num);
			}
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
	int timeout, sg_len;
	host->data = req->data;
	if (req->data == NULL) {
		host->data_dir = DAVINCI_MMC_DATADIR_NONE;
		mmcsd_regs->mmc_blen = 0;
		mmcsd_regs->mmc_nblk = 0;
		return;
	}
	/* Init idx */
	host->sg_idx = 0;

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

	sg_len = (req->data->blocks == 1) ? 1 : req->data->sg_len;
	host->sg_len = sg_len;

	host->bytes_left = req->data->blocks * req->data->blksz;

	if ((host->use_dma == 1) && (host->bytes_left % 32 == 0)
	    && (mmc_davinci_start_dma_transfer(host, req) == 0)) {
		host->buffer = NULL;
		host->bytes_left = 0;
	} else {
		/* Revert to CPU Copy */

		host->do_dma = 0;
		mmc_davinci_sg_to_buf(host);
	}
}

/* PIO only */
static void mmc_davinci_sg_to_buf(struct mmc_davinci_host *host)
{
	struct scatterlist *sg;

	sg = host->data->sg + host->sg_idx;
	host->buffer_bytes_left = sg->length;
	host->buffer = page_address(sg->page) + sg->offset;
	if (host->buffer_bytes_left > host->bytes_left) {
		host->buffer_bytes_left = host->bytes_left;
	}
}

static void mmc_davinci_request(struct mmc_host *mmc, struct mmc_request *req)
{
	struct mmc_davinci_host *host = mmc_priv(mmc);
	unsigned long flags;

	if (is_card_removed) {
		if (req->cmd) {
			req->cmd->error |= MMC_ERR_TIMEOUT;
			mmc_request_done(mmc, req);
		}
		dev_dbg(&mmc_dev,
			"From code segment excuted when card removed\n");
		return;
	}

	wait_on_data();

	if (!is_card_detect_progress) {
		spin_lock_irqsave(&mmc_lock, flags);
		is_card_busy = 1;
		spin_unlock_irqrestore(&mmc_lock, flags);
		host->do_dma = 0;
		mmc_davinci_prepare_data(host, req);
		mmc_davinci_start_command(host, req->cmd);
	} else {
		/* Queue up the request as card dectection is being excuted */
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
	if (ios->bus_width == MMC_BUS_WIDTH_4) {
		dev_dbg(&mmc_dev, "\nEnabling 4 bit mode\n");
		mmcsd_regs->mmc_ctl = mmcsd_regs->mmc_ctl | (1 << 2);
	} else {
		dev_dbg(&mmc_dev, "Disabling 4 bit mode\n");
		mmcsd_regs->mmc_ctl = mmcsd_regs->mmc_ctl & ~(1 << 2);
	}

	if (ios->bus_mode == MMC_BUSMODE_OPENDRAIN) {
		open_drain_freq = ((unsigned int)cpu_arm_clk
				/ (2 * MMCSD_INIT_CLOCK)) - 1;
		mmcsd_regs->mmc_clk = (mmcsd_regs->mmc_clk & ~(0xFF))
				| open_drain_freq;
	} else {
		mmc_push_pull_freq = calculate_freq_for_card(ios->clock);
		mmcsd_regs->mmc_clk &= ~(0x100);
		udelay(10);
		mmcsd_regs->mmc_clk = (mmcsd_regs->mmc_clk & ~(0xFF))
				| mmc_push_pull_freq;
		mmcsd_regs->mmc_clk |= 0x100;
		udelay(10);
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

	if (host->do_dma) {
		davinci_abort_dma(host);

		dma_unmap_sg(mmc_dev(host->mmc), data->sg, host->sg_len,
			     (data->
			      flags & MMC_DATA_WRITE) ? DMA_TO_DEVICE :
			     DMA_FROM_DEVICE);
	}

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

	if (!cmd) {
		dev_warn(&mmc_dev, "%s(): No cmd ptr\n", __FUNCTION__);
		return;
	}

	if (cmd->flags & MMC_RSP_PRESENT) {
		if (cmd->flags & MMC_RSP_136) {
			/* response type 2 */
			cmd->resp[3] = mmcsd_regs->mmc_rsp01;
			cmd->resp[2] = mmcsd_regs->mmc_rsp23;
			cmd->resp[1] = mmcsd_regs->mmc_rsp45;
			cmd->resp[0] = mmcsd_regs->mmc_rsp67;
		} else {
			/* response types 1, 1b, 3, 4, 5, 6 */
			cmd->resp[0] = mmcsd_regs->mmc_rsp67;
		}
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
					if (host->bytes_left > 0)
						davinci_fifo_data_trans(host);
				}
			}

			if (host->data_dir == DAVINCI_MMC_DATADIR_READ) {
				if (status & MMCSD_EVENT_READ) {
					/* Buffer almost empty */
					if (host->bytes_left > 0)
						davinci_fifo_data_trans(host);
				}
			}

			if (status & MMCSD_EVENT_BLOCK_XFERRED) {
				/* Block sent/received */
				if (host->data != NULL) {
					if (host->do_dma == 1) {
						end_transfer = 1;
					} else {
						/* if datasize<32 no RX ints are generated */
						if (host->bytes_left > 0) {
							davinci_fifo_data_trans
							    (host);
						}
						end_transfer = 1;
					}
				} else {
					dev_warn(&mmc_dev,
						 "TC:host->data is NULL\n");
				}
			}

			if (status & MMCSD_EVENT_ERROR_DATATIMEOUT) {
				/* Data timeout */
				if ((host->data) && (new_card_state != 0)) {
					host->data->error |= MMC_ERR_TIMEOUT;
					spin_lock_irqsave(&mmc_lock, flags);
					is_card_removed = 1;
					new_card_state = 0;
					is_card_initialized = 0;
					spin_unlock_irqrestore(&mmc_lock,
						flags);
					dev_dbg(&mmc_dev,
						"MMCSD: Data timeout, CMD%d and status is %x\r\n",
						host->cmd->opcode, status);

					if (host->cmd) {
						host->cmd->error |=
						    MMC_ERR_TIMEOUT;
					}
					end_transfer = 1;
				}
			}

			if (status & MMCSD_EVENT_ERROR_DATACRC) {
				/* DAT line portion is diabled and in reset state */
				mmcsd_regs->mmc_ctl =
				    mmcsd_regs->mmc_ctl | (1 << 1);
				udelay(10);
				mmcsd_regs->mmc_ctl =
				    mmcsd_regs->mmc_ctl & ~(1 << 1);

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
				if (host->do_dma)
					/* abort DMA transfer */
					davinci_abort_dma(host);

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
					/* Ignore CMD CRC errors during high speed operation */
					if (host->mmc->ios.clock <= 25000000) {
						host->cmd->error |=
						    MMC_ERR_BADCRC;
					}
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
				is_card_removed = 1;
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

		if (host->cmd_code == 1 || host->cmd_code == 55) {
			if (status & MMCSD_EVENT_EOFCMD) {
				spin_lock_irqsave(&mmc_lock, flags);
				is_card_removed = 0;
				new_card_state = 1;
				is_card_initialized = 0;
				spin_unlock_irqrestore(&mmc_lock, flags);
			} else {

				spin_lock_irqsave(&mmc_lock, flags);
				is_card_removed = 1;
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
				static int flag_sd_mmc;
				host->is_core_command = 0;

				if (flag_sd_mmc) {
					flag_sd_mmc = 0;
					host->cmd_code = 1;
					/* Issue cmd1 */
					mmcsd_regs->mmc_arghl = 0x80300000;
					mmcsd_regs->mmc_cmd = 0x00000601;
				} else {
					flag_sd_mmc = 1;
					host->cmd_code = 55;
					/* Issue cmd55 */
					mmcsd_regs->mmc_arghl = 0x0;
					mmcsd_regs->mmc_cmd =
					    ((0x0 | (1 << 9) | 55));
				}

				dev_dbg(&mmc_dev,
					"MMC-Probing mmc with cmd%d\n",
					host->cmd_code);
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
	.get_ro = mmc_davinci_get_ro
};

static int mmc_davinci_get_ro(struct mmc_host *mmc)
{
	return 0;
}

static void mmc_check_card(unsigned long data)
{
	struct mmc_davinci_host *host = (struct mmc_davinci_host *)data;
	unsigned long flags;
	struct mmc_card *card = NULL;

	if (host->mmc && host->mmc->card)
		card = host->mmc->card;

	if ((!is_card_detect_progress) || (!is_init_progress)) {
		if (is_card_initialized) {
			host->is_core_command = 0;
			host->cmd_code = 13;
			spin_lock_irqsave(&mmc_lock, flags);
			is_card_detect_progress = 1;
			spin_unlock_irqrestore(&mmc_lock, flags);
			/* Issue cmd13 */
			mmcsd_regs->mmc_arghl = (card && mmc_card_sd(card))
				? (card->rca << 16) : 0x10000;
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
			davinci_reinit_chan();
			init_mmcsd_host();
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
	udelay(10);

	mmcsd_regs->mmc_clk = 0x0;
	mmcsd_regs->mmc_clk = mmcsd_regs->mmc_clk | (1 << 8);

	mmcsd_regs->mmc_tor = 0xFFFF;
	mmcsd_regs->mmc_tod = 0xFFFF;

	mmcsd_regs->mmc_ctl = mmcsd_regs->mmc_ctl & ~(0x1);
	mmcsd_regs->mmc_ctl = mmcsd_regs->mmc_ctl & ~(1 << 1);
	udelay(10);
}

static int davinci_mmcsd_probe(struct platform_device *pdev)
{
	struct mmc_davinci_host *host;
	struct mmc_host *mmc;
	int ret = 0;

	mmc = mmc_alloc_host(sizeof(struct mmc_davinci_host), &pdev->dev);
	if (!mmc) {
		ret = -ENOMEM;
		goto out;
	}

	mmcsd_regs =
		(volatile mmcsd_regs_base *)IO_ADDRESS(MMCSD_REGS_BASE_ADDR);

	init_mmcsd_host();

	if (mmcsd_cfg.use_4bit_mode) {
		dev_warn(&mmc_dev, "Supporting 4-bit mode\n");
		mmc->caps |= MMC_CAP_4_BIT_DATA;
	} else
		dev_warn(&mmc_dev, "Not Supporting 4-bit mode\n");

	mmc->ops = &mmc_davinci_ops;
	mmc->f_min = 312500;
#ifdef CONFIG_MMC_HIGHSPEED
	mmc->f_max = 50000000;
#else
	mmc->f_max = 25000000;
#endif
	mmc->ocr_avail = MMC_VDD_32_33;

	mmc->max_phys_segs = 2;
	mmc->max_hw_segs   = 2;
	mmc->max_blk_size  = 4095;  /* BLEN is 11 bits */
	mmc->max_blk_count = 65535; /* NBLK is 16 bits */
	mmc->max_req_size  = mmc->max_blk_size * mmc->max_blk_count;
	mmc->max_seg_size  = mmc->max_req_size;

	dev_dbg(&mmc_dev, "max_phys_segs=%d\n", mmc->max_phys_segs);
	dev_dbg(&mmc_dev, "max_hw_segs=%d\n", mmc->max_hw_segs);
	dev_dbg(&mmc_dev, "max_blk_size=%d\n", mmc->max_blk_size);
	dev_dbg(&mmc_dev, "max_req_size=%d\n", mmc->max_req_size);
	dev_dbg(&mmc_dev, "max_seg_size=%d\n", mmc->max_seg_size);

	host = mmc_priv(mmc);
	host->mmc = mmc;	/* Important */

	if (mmcsd_cfg.use_dma) {
		dev_dbg(&mmc_dev, "Using DMA mode\n");
		if (davinci_acquire_dma_channels(host) != 0)
			goto out;
	} else
		dev_dbg(&mmc_dev, "Not Using DMA mode\n");

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
	unsigned long flags;

	platform_set_drvdata(pdev, NULL);
	if (host) {
		mmc_remove_host(host->mmc);
		free_irq(host->irq, host);

		spin_lock_irqsave(&mmc_lock, flags);
		del_timer(&host->timer);
		spin_unlock_irqrestore(&mmc_lock, flags);

		davinci_release_dma_channels(host);
	}

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

	if (clkp) {
		mmc_clkp = NULL;
		clk_put(clkp);
	}

	return -ENODEV;
}

static void __exit davinci_mmcsd_exit(void)
{
	struct clk *clkp;

	platform_driver_unregister(&davinci_mmcsd_driver);
	platform_device_unregister(&mmc_davinci_device);

	clkp = mmc_clkp;
	mmc_clkp = NULL;

	clk_disable(clkp);
	clk_put(clkp);
}

module_init(davinci_mmcsd_init);
module_exit(davinci_mmcsd_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("MMCSD driver for Davinci MMC controller");
