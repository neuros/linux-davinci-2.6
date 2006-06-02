/*
 * arch/arm/mach-davinci/mcbsp.c
 *
 * Copyright (C) 2004 Nokia Corporation
 * Author: Samuel Ortiz <samuel.ortiz@nokia.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * Multichannel mode not supported.
 *
 * 2005-10-01   Rishi Bhattacharya / Sharath Kumar - Modified to support TI
 *		Davinci DM644x processor
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/device.h>
#include <linux/wait.h>
#include <linux/completion.h>
#include <linux/interrupt.h>
#include <linux/err.h>
#include <linux/clk.h>

#include <asm/delay.h>
#include <asm/io.h>
#include <asm/irq.h>

#include <asm/arch/memory.h>
#include <asm/arch/edma.h>
#include <asm/arch/irqs.h>
#include <asm/arch/mcbsp.h>

struct clk *mbspclk = NULL;

/* #define CONFIG_MCBSP_DEBUG */

#ifdef CONFIG_MCBSP_DEBUG
#define DBG(x...)       printk(KERN_INFO x)
#else
#define DBG(x...)       do { } while (0)
#endif

struct davinci_mcbsp {
	u32 io_base;
	u8 id;
	u8 free;
	davinci_mcbsp_word_length rx_word_length;
	davinci_mcbsp_word_length tx_word_length;

	/* IRQ based TX/RX */
	int rx_irq;
	int tx_irq;

	/* DMA stuff */
	u8 dma_rx_sync;
	short dma_rx_lch;
	u8 dma_tx_sync;
	short dma_tx_lch;

	/* Completion queues */
	struct completion tx_irq_completion;
	struct completion rx_irq_completion;
	struct completion tx_dma_completion;
	struct completion rx_dma_completion;

	spinlock_t lock;
};

static struct davinci_mcbsp mcbsp[DAVINCI_MAX_MCBSP_COUNT];

unsigned short DAVINCI_MCBSP_READ(int base, int reg)
{
	unsigned long data, temp, *p = (unsigned long *)(base + reg);

	temp = (unsigned long)p;

	if (temp & 0x2) {
		/*non word offset */
		temp &= 0xfffffffc;
		p = (unsigned long *)temp;
		data = inl(p);
		return (unsigned short)(data >> 16);
	} else {
		/*word offset */
		return ((unsigned short)inl(p));
	}
}

void DAVINCI_MCBSP_WRITE(int base, int reg, unsigned short val)
{
	unsigned long data, temp, *p = (unsigned long *)(base + reg);

	temp = (unsigned long)p;

	if (temp & 0x2) {
		/*non word offset */
		temp &= 0xfffffffc;
		p = (unsigned long *)temp;
		data = inl(p);
		data &= 0x0000ffff;
		data |= ((unsigned long)val << 16);
		outl(data, p);
	} else {
		/*word offset */
		data = inl(p);
		data &= 0xffff0000;
		data |= val;
		outl(data, p);
	}
}
#if 0
static void davinci_mcbsp_dump_reg(u8 id)
{
	DBG("**** MCBSP%d regs ****\n", mcbsp[id].id);

	DBG("SPCR2: 0x%04x\n", DAVINCI_MCBSP_READ(mcbsp[id].io_base, SPCR2));
	DBG("SPCR1: 0x%04x\n",	DAVINCI_MCBSP_READ(mcbsp[id].io_base, SPCR1));
	DBG("RCR2:  0x%04x\n",	DAVINCI_MCBSP_READ(mcbsp[id].io_base, RCR2));
	DBG("RCR1:  0x%04x\n",	DAVINCI_MCBSP_READ(mcbsp[id].io_base, RCR1));
	DBG("XCR2:  0x%04x\n",	DAVINCI_MCBSP_READ(mcbsp[id].io_base, XCR2));
	DBG("XCR1:  0x%04x\n",	DAVINCI_MCBSP_READ(mcbsp[id].io_base, XCR1));
	DBG("SRGR2: 0x%04x\n",	DAVINCI_MCBSP_READ(mcbsp[id].io_base, SRGR2));
	DBG("SRGR1: 0x%04x\n",	DAVINCI_MCBSP_READ(mcbsp[id].io_base, SRGR1));
	DBG("PCR0:  0x%04x\n",	DAVINCI_MCBSP_READ(mcbsp[id].io_base, PCR0));
	DBG("***********************\n");

}
#endif

static void davinci_mcbsp_tx_dma_callback(int lch, u16 ch_status, void *data)
{
	struct davinci_mcbsp *mcbsp_dma_tx = (struct davinci_mcbsp *)(data);

	DBG("TX DMA callback : 0x%x\n",
	    DAVINCI_MCBSP_READ(mcbsp_dma_tx->io_base, SPCR2));

	/* We can free the channels */
	DBG("mcbsp_dma_tx->dma_tx_lch = %d\n",
	       mcbsp_dma_tx->dma_tx_lch);
	davinci_stop_dma(mcbsp_dma_tx->dma_tx_lch);
	davinci_free_dma(mcbsp_dma_tx->dma_tx_lch);
	mcbsp_dma_tx->dma_tx_lch = -1;
	complete(&mcbsp_dma_tx->tx_dma_completion);
}

static void davinci_mcbsp_rx_dma_callback(int lch, u16 ch_status, void *data)
{
	struct davinci_mcbsp *mcbsp_dma_rx = (struct davinci_mcbsp *)(data);

	DBG("RX DMA callback : 0x%x\n",
	    DAVINCI_MCBSP_READ(mcbsp_dma_rx->io_base, SPCR2));

	/* We can free the channels */
	davinci_free_dma(mcbsp_dma_rx->dma_rx_lch);
	mcbsp_dma_rx->dma_rx_lch = -1;

	complete(&mcbsp_dma_rx->rx_dma_completion);
}

/*
 * davinci_mcbsp_config simply write a config to the
 * appropriate McBSP.
 * You either call this function or set the McBSP registers
 * by yourself before calling davinci_mcbsp_start().
 */

void davinci_mcbsp_config(unsigned int id,
			  const struct davinci_mcbsp_reg_cfg *config)
{
	u32 io_base = mcbsp[id].io_base;

	DBG("davinci-McBSP: McBSP%d  io_base: 0x%8x\n", id + 1, io_base);

	DAVINCI_MCBSP_WRITE(io_base, PCR0, config->pcr0);

	DAVINCI_MCBSP_WRITE(io_base, RCR2, config->rcr2);
	DAVINCI_MCBSP_WRITE(io_base, RCR1, config->rcr1);

	DAVINCI_MCBSP_WRITE(io_base, XCR2, config->xcr2);
	DAVINCI_MCBSP_WRITE(io_base, XCR1, config->xcr1);

	DAVINCI_MCBSP_WRITE(io_base, SRGR2, config->srgr2);
	DAVINCI_MCBSP_WRITE(io_base, SRGR1, config->srgr1);

	/* We write the given config */
	DAVINCI_MCBSP_WRITE(io_base, SPCR2, config->spcr2);
	DAVINCI_MCBSP_WRITE(io_base, SPCR1, config->spcr1);

	return;
}

static int davinci_mcbsp_check(unsigned int id)
{
	if (id > DAVINCI_MAX_MCBSP_COUNT - 1) {
		DBG("DAVINCI-McBSP: McBSP%d doesn't exist\n", id + 1);
		return -1;
	}
	return 0;
}

int davinci_mcbsp_request(unsigned int id)
{
	if (davinci_mcbsp_check(id) < 0)
		return -EINVAL;

	spin_lock(&mcbsp[id].lock);
	if (!mcbsp[id].free) {
		DBG("DAVINCI-McBSP: McBSP%d is currently in use\n", id + 1);
		spin_unlock(&mcbsp[id].lock);
		return -1;
	}

	mcbsp[id].free = 0;
	spin_unlock(&mcbsp[id].lock);

	return 0;

}

void davinci_mcbsp_free(unsigned int id)
{
	if (davinci_mcbsp_check(id) < 0)
		return;

	spin_lock(&mcbsp[id].lock);
	if (mcbsp[id].free) {
		DBG("DAVINCI-McBSP: McBSP%d was not reserved\n", id + 1);
		spin_unlock(&mcbsp[id].lock);
		return;
	}

	mcbsp[id].free = 1;
	spin_unlock(&mcbsp[id].lock);

	return;
}

/*
 * Here we start the McBSP, by enabling the sample
 * generator, both transmitter and receivers,
 * and the frame sync.
 */
void davinci_mcbsp_start(unsigned int id)
{
	u32 io_base;
	u16 w;

	if (davinci_mcbsp_check(id) < 0)
		return;

	io_base = mcbsp[id].io_base;

	mcbsp[id].rx_word_length =
	    ((DAVINCI_MCBSP_READ(io_base, RCR1) >> 5) & 0x7);
	mcbsp[id].tx_word_length =
	    ((DAVINCI_MCBSP_READ(io_base, XCR1) >> 5) & 0x7);

	/* Start the sample generator */
	w = DAVINCI_MCBSP_READ(io_base, SPCR2);
	DAVINCI_MCBSP_WRITE(io_base, SPCR2, w | (1 << 6));

	/* Enable transmitter and receiver */
	w = DAVINCI_MCBSP_READ(io_base, SPCR2);
	DAVINCI_MCBSP_WRITE(io_base, SPCR2, w | 1);

	w = DAVINCI_MCBSP_READ(io_base, SPCR1);
	DAVINCI_MCBSP_WRITE(io_base, SPCR1, w | 1);

	udelay(100);

	/* Start frame sync */
	w = DAVINCI_MCBSP_READ(io_base, SPCR2);
	DAVINCI_MCBSP_WRITE(io_base, SPCR2, w | (1 << 7));

	return;
}

void davinci_mcbsp_stop(unsigned int id)
{
	u32 io_base;
	u16 w;

	if (davinci_mcbsp_check(id) < 0)
		return;

	io_base = mcbsp[id].io_base;

	/* Reset transmitter */
	w = DAVINCI_MCBSP_READ(io_base, SPCR2);
	DAVINCI_MCBSP_WRITE(io_base, SPCR2, w & ~(1));

	/* Reset receiver */
	w = DAVINCI_MCBSP_READ(io_base, SPCR1);
	DAVINCI_MCBSP_WRITE(io_base, SPCR1, w & ~(1));

	/* Reset the sample rate generator */
	w = DAVINCI_MCBSP_READ(io_base, SPCR2);
	DAVINCI_MCBSP_WRITE(io_base, SPCR2, w & ~(1 << 6));

	/* Reset the frame sync generator */
	w = DAVINCI_MCBSP_READ(io_base, SPCR2);
	DAVINCI_MCBSP_WRITE(io_base, SPCR2, w & ~(1 << 7));

	return;
}

/*
 * IRQ based word transmission.
 */
void davinci_mcbsp_xmit_word(unsigned int id, u32 word)
{

	u32 io_base;
	davinci_mcbsp_word_length word_length = mcbsp[id].tx_word_length;

	if (davinci_mcbsp_check(id) < 0)
		return;

	io_base = mcbsp[id].io_base;

	DBG(" io_base = 0x%x\n", io_base);
	if (word_length > DAVINCI_MCBSP_WORD_16) {
		DBG(" writing DXR2 register \n");
		DAVINCI_MCBSP_WRITE(io_base, DXR2, word >> 16);
	}
	DBG(" writing DXR1 register \n");
	DAVINCI_MCBSP_WRITE(io_base, DXR1, word & 0xffff);

}

u32 davinci_mcbsp_recv_word(unsigned int id)
{

	u32 io_base;
	u16 word_lsb, word_msb = 0;

	davinci_mcbsp_word_length word_length = mcbsp[id].rx_word_length;

	if (davinci_mcbsp_check(id) < 0)
		return -EINVAL;

	io_base = mcbsp[id].io_base;

	if (word_length > DAVINCI_MCBSP_WORD_16)
		word_msb = DAVINCI_MCBSP_READ(io_base, DRR2);
	word_lsb = DAVINCI_MCBSP_READ(io_base, DRR1);

	return (word_lsb | (word_msb << 16));

}

/*
 * Simple DMA based buffer rx/tx routines.
 * Nothing fancy, just a single buffer tx/rx through DMA.
 * The DMA resources are released once the transfer is done.
 * For anything fancier, you should use your own customized DMA
 * routines and callbacks.
 */
int davinci_mcbsp_xmit_buffer(unsigned int id, dma_addr_t buffer,
			      unsigned int length)
{
	int dma_tx_ch;
	int tcc;

	if (davinci_mcbsp_check(id) < 0)
		return -EINVAL;

	if (davinci_request_dma
	    (mcbsp[id].dma_tx_sync, "McBSP TX", davinci_mcbsp_tx_dma_callback,
	     &mcbsp[id], &dma_tx_ch, &tcc, EVENTQ_0)) {
		DBG("DAVINCI-McBSP: Unable to request DMA channel for McBSP%d TX. Trying IRQ based TX\n",
		     id + 1);
		return -EAGAIN;
	}

	mcbsp[id].dma_tx_lch = dma_tx_ch;

	DBG("TX DMA on channel %d\n", dma_tx_ch);

	init_completion(&(mcbsp[id].tx_dma_completion));

	davinci_set_dma_transfer_params(mcbsp[id].dma_tx_lch, 2, length / 2, 1,
					0, ASYNC);

	davinci_set_dma_dest_params(mcbsp[id].dma_tx_lch,
				    (unsigned long)0x01E02004, 0, 0);

	davinci_set_dma_src_params(mcbsp[id].dma_tx_lch, (buffer), 0, 0);
	davinci_set_dma_src_index(mcbsp[id].dma_tx_lch, 2, 0);
	davinci_set_dma_dest_index(mcbsp[id].dma_tx_lch, 0, 0);

	davinci_start_dma(mcbsp[id].dma_tx_lch);
	wait_for_completion(&(mcbsp[id].tx_dma_completion));
	return 0;
}

int davinci_mcbsp_recv_buffer(unsigned int id, dma_addr_t buffer,
			      unsigned int length)
{
	int dma_rx_ch;
	int tcc;

	if (davinci_mcbsp_check(id) < 0)
		return -EINVAL;

	if (davinci_request_dma
	    (mcbsp[id].dma_rx_sync, "McBSP RX", davinci_mcbsp_rx_dma_callback,
	     &mcbsp[id], &dma_rx_ch, &tcc, EVENTQ_0)) {
		DBG("Unable to request DMA channel for McBSP%d RX. Trying IRQ based RX\n",
		     id + 1);
		return -EAGAIN;
	}
	mcbsp[id].dma_rx_lch = dma_rx_ch;

	DBG("RX DMA on channel %d\n", dma_rx_ch);

	init_completion(&(mcbsp[id].rx_dma_completion));

	davinci_set_dma_transfer_params(mcbsp[id].dma_rx_lch, 2, length / 2, 1,
					0, ASYNC);

	davinci_set_dma_src_params(mcbsp[id].dma_rx_lch,
				   (unsigned long)0x01E02000, 0, 0);

	davinci_set_dma_dest_params(mcbsp[id].dma_rx_lch,
				    (unsigned long)virt_to_phys((void *)buffer),
				    0, 0);
	davinci_set_dma_src_index(mcbsp[id].dma_rx_lch, 0, 0);
	davinci_set_dma_dest_index(mcbsp[id].dma_rx_lch, 2, 0);

	davinci_start_dma(mcbsp[id].dma_rx_lch);

	wait_for_completion(&(mcbsp[id].rx_dma_completion));
	DBG(" davinci_mcbsp_recv_buffer: after wait_for_completion\n");
	return 0;
}

struct clk * davinci_mcbsp_get_clock(void)
{
	return mbspclk;
}

struct davinci_mcbsp_info {
	u32 virt_base;
	u8 dma_rx_sync, dma_tx_sync;
	u16 rx_irq, tx_irq;
};

static const struct davinci_mcbsp_info mcbsp_davinci[] = {
	[0] = {.virt_base = IO_ADDRESS(DAVINCI_MCBSP1_BASE),
	       .dma_rx_sync = DAVINCI_DMA_MCBSP1_RX,
	       .dma_tx_sync = DAVINCI_DMA_MCBSP1_TX,
	       .rx_irq = DAVINCI_McBSP1RX,
	       .tx_irq = DAVINCI_McBSP1TX},
};

static int __init davinci_mcbsp_init(void)
{
	int mcbsp_count = 0, i;
	static const struct davinci_mcbsp_info *mcbsp_info;
	struct clk *clkp;

	clkp = clk_get (NULL, "McBSPCLK");
	if (IS_ERR(clkp)) {
		return -1;
	}
	else
	{
		mbspclk = clkp;
		clk_enable (mbspclk);

		mcbsp_info = mcbsp_davinci;
		mcbsp_count = ARRAY_SIZE(mcbsp_davinci);

		for (i = 0; i < DAVINCI_MAX_MCBSP_COUNT; i++) {
			if (i >= mcbsp_count) {
				mcbsp[i].io_base = 0;
				mcbsp[i].free = 0;
				continue;
			}
			mcbsp[i].id = i + 1;
			mcbsp[i].free = 1;
			mcbsp[i].dma_tx_lch = -1;
			mcbsp[i].dma_rx_lch = -1;

			mcbsp[i].io_base = mcbsp_info[i].virt_base;
			mcbsp[i].tx_irq = mcbsp_info[i].tx_irq;
			mcbsp[i].rx_irq = mcbsp_info[i].rx_irq;
			mcbsp[i].dma_rx_sync = mcbsp_info[i].dma_rx_sync;
			mcbsp[i].dma_tx_sync = mcbsp_info[i].dma_tx_sync;
			spin_lock_init(&mcbsp[i].lock);
		}
	}
	return 0;
}

static void __exit davinci_mcbsp_exit(void)
{
	int i;

    for( i = 0; i < DAVINCI_MAX_MCBSP_COUNT; i++) {
	    mcbsp[i].free = 0;
		mcbsp[i].dma_tx_lch = -1;
		mcbsp[i].dma_rx_lch = -1;
    }

    clk_disable (mbspclk);

	return;
}

module_init(davinci_mcbsp_init);
module_exit(davinci_mcbsp_exit);

EXPORT_SYMBOL(davinci_mcbsp_config);
EXPORT_SYMBOL(davinci_mcbsp_request);
EXPORT_SYMBOL(davinci_mcbsp_free);
EXPORT_SYMBOL(davinci_mcbsp_start);
EXPORT_SYMBOL(davinci_mcbsp_stop);
EXPORT_SYMBOL(davinci_mcbsp_xmit_word);
EXPORT_SYMBOL(davinci_mcbsp_recv_word);
EXPORT_SYMBOL(davinci_mcbsp_xmit_buffer);
EXPORT_SYMBOL(davinci_mcbsp_recv_buffer);
EXPORT_SYMBOL(davinci_mcbsp_get_clock);

MODULE_AUTHOR("Texas Instruments");
MODULE_DESCRIPTION("McBSP driver for DaVinci.");
MODULE_LICENSE("GPL");
