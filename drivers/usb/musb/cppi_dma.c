/*
 * Copyright (C) 2005-2006 by Texas Instruments
 *
 * This file implements a DMA  interface using TI's CPPI DMA.
 * For now it's DaVinci-only, but CPPI isn't specific to DaVinci or USB.
 */

#include <linux/usb.h>

#include "musb_core.h"
#include "musb_host.h"
#include "cppi_dma.h"

/* CPPI DMA status 7-mar:
 *
 * - See musb_{host,gadget}.c for more info
 *
 * - Correct RX DMA generally forces the engine into irq-per-packet mode,
 *   which can easily saturate the CPU under non-mass-storage loads.
 */

/* REVISIT now we can avoid preallocating these descriptors; or
 * more simply, switch to a global freelist not per-channel ones.
 * Note: at full speed, 64 descriptors == 4K bulk data.
 */
#define NUM_TXCHAN_BD       64
#define NUM_RXCHAN_BD       64

static inline void cpu_drain_writebuffer(void)
{
	wmb();
#ifdef	CONFIG_CPU_ARM926T
	/* REVISIT this "should not be needed",
	 * but lack of it sure seemed to hurt ...
	 */
	asm("mcr p15, 0, r0, c7, c10, 4 @ drain write buffer\n");
#endif
}

static inline struct cppi_descriptor *cppi_bd_alloc(struct cppi_channel *c)
{
	struct cppi_descriptor	*bd = c->freelist;

	if (bd)
		c->freelist = bd->next;
	return bd;
}

static inline void
cppi_bd_free(struct cppi_channel *c, struct cppi_descriptor *bd)
{
	if (!bd)
		return;
	bd->next = c->freelist;
	c->freelist = bd;
}

/*
 *  Start DMA controller
 *
 *  Initialize the DMA controller as necessary.
 */

#define	CAST (void *__force __iomem)

/* zero out entire rx state RAM entry for the channel */
static void cppi_reset_rx(struct cppi_rx_stateram *__iomem rx)
{
	musb_writel(CAST & rx->rx_skipbytes, 0, 0);
	musb_writel(CAST & rx->rx_head, 0, 0);
	musb_writel(CAST & rx->rx_sop, 0, 0);
	musb_writel(CAST & rx->rx_current, 0, 0);
	musb_writel(CAST & rx->rx_buf_current, 0, 0);
	musb_writel(CAST & rx->rx_len_len, 0, 0);
	musb_writel(CAST & rx->rx_cnt_cnt, 0, 0);
}

static void __init cppi_pool_init(struct cppi *cppi, struct cppi_channel *c)
{
	int	j;

	/* initialize channel fields */
	c->head = NULL;
	c->tail = NULL;
	c->last_processed = NULL;
	c->channel.status = MUSB_DMA_STATUS_UNKNOWN;
	c->controller = cppi;
	c->is_rndis = 0;
	c->freelist = NULL;

	/* build the BD Free list for the channel */
	for (j = 0; j < NUM_TXCHAN_BD + 1; j++) {
		struct cppi_descriptor	*bd;
		dma_addr_t		dma;

		bd = dma_pool_alloc(cppi->pool, GFP_KERNEL, &dma);
		bd->dma = dma;
		cppi_bd_free(c, bd);
	}
}

static int cppi_channel_abort(struct dma_channel *);

static void cppi_pool_free(struct cppi_channel *c)
{
	struct cppi		*cppi = c->controller;
	struct cppi_descriptor	*bd;

	(void) cppi_channel_abort(&c->channel);
	c->channel.status = MUSB_DMA_STATUS_UNKNOWN;
	c->controller = NULL;

	/* free all its bds */
	bd = c->last_processed;
	do {
		if (bd)
			dma_pool_free(cppi->pool, bd, bd->dma);
		bd = cppi_bd_alloc(c);
	} while (bd);
	c->last_processed = NULL;
}

static int __init cppi_controller_start(struct dma_controller *c)
{
	struct cppi	*controller;
	void *__iomem	tibase;
	int		i;

	controller = container_of(c, struct cppi, controller);

	/* do whatever is necessary to start controller */
	for (i = 0; i < ARRAY_SIZE(controller->tx); i++) {
		controller->tx[i].transmit = true;
		controller->tx[i].index = i;
	}
	for (i = 0; i < ARRAY_SIZE(controller->rx); i++) {
		controller->rx[i].transmit = false;
		controller->rx[i].index = i;
	}

	/* setup BD list on a per channel basis */
	for (i = 0; i < ARRAY_SIZE(controller->tx); i++)
		cppi_pool_init(controller, controller->tx + i);
	for (i = 0; i < ARRAY_SIZE(controller->rx); i++)
		cppi_pool_init(controller, controller->rx + i);

	/* Do Necessary configuartion in H/w to get started */
	tibase = controller->mregs - DAVINCI_BASE_OFFSET;

	INIT_LIST_HEAD(&controller->tx_complete);

	/* initialise tx/rx channel head pointers to zero */
	for (i = 0; i < ARRAY_SIZE(controller->tx); i++) {
		struct cppi_channel	*tx_ch = controller->tx + i;
		struct cppi_tx_stateram *__iomem tx;

		INIT_LIST_HEAD(&tx_ch->tx_complete);

		tx = tibase + DAVINCI_TXCPPI_STATERAM_OFFSET(i);
		tx_ch->state_ram = tx;
		/* zero out entire state RAM entry for the channel */
		tx->tx_head = 0;
		tx->tx_buf = 0;
		tx->tx_current = 0;
		tx->tx_buf_current = 0;
		tx->tx_info = 0;
		tx->tx_rem_len = 0;
		/*txState->dummy = 0; */
		tx->tx_complete = 0;

	}
	for (i = 0; i < ARRAY_SIZE(controller->rx); i++) {
		struct cppi_channel	*rx_ch = controller->rx + i;
		struct cppi_rx_stateram *__iomem rx;

		INIT_LIST_HEAD(&rx_ch->tx_complete);

		rx = tibase + DAVINCI_RXCPPI_STATERAM_OFFSET(i);
		rx_ch->state_ram = rx;
		cppi_reset_rx(rx_ch->state_ram);
	}

	/* enable individual cppi channels */
	musb_writel(tibase, DAVINCI_TXCPPI_INTENAB_REG,
			DAVINCI_DMA_ALL_CHANNELS_ENABLE);
	musb_writel(tibase, DAVINCI_RXCPPI_INTENAB_REG,
			DAVINCI_DMA_ALL_CHANNELS_ENABLE);

	/* enable tx/rx CPPI control */
	musb_writel(tibase, DAVINCI_TXCPPI_CTRL_REG, DAVINCI_DMA_CTRL_ENABLE);
	musb_writel(tibase, DAVINCI_RXCPPI_CTRL_REG, DAVINCI_DMA_CTRL_ENABLE);

	/* disable RNDIS mode */
	musb_writel(tibase, DAVINCI_AUTOREQ_REG, 0);

	return true;
}

/*
 *  Stop DMA controller
 *
 *  De-Init the DMA controller as necessary.
 */

static int cppi_controller_stop(struct dma_controller *c)
{
	struct cppi		*controller;
	void __iomem		*tibase;
	int			i;

	controller = container_of(c, struct cppi, controller);
	tibase = controller->mregs - DAVINCI_BASE_OFFSET;
	/* DISABLE INDIVIDUAL CHANNEL Interrupts */
	musb_writel(tibase, DAVINCI_TXCPPI_INTCLR_REG,
			DAVINCI_DMA_ALL_CHANNELS_ENABLE);
	musb_writel(tibase, DAVINCI_RXCPPI_INTCLR_REG,
			DAVINCI_DMA_ALL_CHANNELS_ENABLE);

	DBG(1, "Tearing down RX and TX Channels\n");
	for (i = 0; i < ARRAY_SIZE(controller->tx); i++) {
		/* FIXME restructure of txdma to use bds like rxdma */
		controller->tx[i].last_processed = NULL;
		cppi_pool_free(controller->tx + i);
	}
	for (i = 0; i < ARRAY_SIZE(controller->rx); i++)
		cppi_pool_free(controller->rx + i);

	/* in Tx Case proper teardown is supported. We resort to disabling
	 * Tx/Rx CPPI after cleanup of Tx channels. Before TX teardown is
	 * complete TX CPPI cannot be disabled.
	 */
	/*disable tx/rx cppi */
	musb_writel(tibase, DAVINCI_TXCPPI_CTRL_REG, DAVINCI_DMA_CTRL_DISABLE);
	musb_writel(tibase, DAVINCI_RXCPPI_CTRL_REG, DAVINCI_DMA_CTRL_DISABLE);

	return 0;
}

/*
 * Allocate a CPPI Channel for DMA.  With CPPI, channels are bound to
 * each transfer direction of a non-control endpoint, so allocating
 * (and deallocating) is mostly a way to notice bad housekeeping on
 * the software side.  We assume the irqs are always active.
 */
static struct dma_channel *cppi_channel_allocate(struct dma_controller *c,
					 struct musb_hw_ep *ep, u8 transmit)
{
	struct cppi		 *controller;
	u8			 chnum;
	struct cppi_channel	 *cppi_ch;

	controller = container_of(c, struct cppi, controller);
	/* remember bLocalEnd: 1..Max_EndPt, and cppi ChNum:0..Max_EndPt-1 */
	chnum = ep->epnum  - 1;

	/* as of now, just return the corresponding CPPI Channel Handle */
	if (transmit) {
		if (chnum  > ARRAY_SIZE(controller->tx)) {
			DBG(1, "no %cX DMA channel for ep%d\n", 'T', chnum);
			return NULL;
		}
		cppi_ch = controller->tx + chnum;
	} else {
		if (chnum > ARRAY_SIZE(controller->rx)) {
			DBG(1, "no %cX DMA channel for ep%d\n", 'R', chnum);
			return NULL;
		}
		cppi_ch = controller->rx + chnum;
	}

	/* REVISIT make this an error later once the same driver code works
	 * with the Mentor DMA engine too
	 */
	if (cppi_ch->hw_ep)
		DBG(6, "re-allocating DMA%d %cX channel %p\n",
		    chnum, transmit ? 'T' : 'R', cppi_ch);
	cppi_ch->hw_ep = ep;
	cppi_ch->channel.status = MUSB_DMA_STATUS_FREE;

	DBG(4, "Allocate CPPI%d %cX\n", chnum, transmit ? 'T' : 'R');
	return &cppi_ch->channel;
}

/* Release a CPPI Channel.  */
static void cppi_channel_release(struct dma_channel *channel)
{
	struct cppi_channel *c;

	/* REVISIT:  for paranoia, check state and abort if needed... */

	c = container_of(channel, struct cppi_channel, channel);
	if (!c->hw_ep)
		DBG(1, "releasing idle DMA channel %p\n", c);

	/* but for now, not its IRQ */
	c->hw_ep = NULL;
	channel->status = MUSB_DMA_STATUS_UNKNOWN;
}

/* Context: controller irqlocked */
static void cppi_dump_rx(int level, struct cppi_channel *c, const char *tag)
{
	void *__iomem base = c->controller->mregs;

	musb_ep_select(base, c->index + 1);

	DBG(level, "RX DMA%d%s: %d left, csr %04x, "
		"%08x H%08x S%08x C%08x, " "B%08x L%08x %08x .. %08x"
		"\n",
		c->index, tag, musb_readl(base - DAVINCI_BASE_OFFSET,
		DAVINCI_RXCPPI_BUFCNT0_REG + 4 * c->index),
		musb_readw(c->hw_ep->regs, MUSB_RXCSR),
		musb_readl(c->state_ram, 0 * 4),	/* buf offset */
		musb_readl(c->state_ram, 1 * 4),	/* head ptr */
		musb_readl(c->state_ram, 2 * 4),	/* sop bd */
		musb_readl(c->state_ram, 3 * 4),	/* current bd */
		musb_readl(c->state_ram, 4 * 4),	/* current buf */
		musb_readl(c->state_ram, 5 * 4),	/* pkt len */
		musb_readl(c->state_ram, 6 * 4),	/* byte cnt */
		musb_readl(c->state_ram, 7 * 4)		/* completion */
	    );
}

/* Context: controller irqlocked */
static void cppi_dump_tx(int level, struct cppi_channel *c, const char *tag)
{
	void *__iomem base = c->controller->mregs;

	musb_ep_select(base, c->index + 1);

	DBG(level, "TX DMA%d%s: csr %04x, "
		"H%08x S%08x C%08x %08x, "
		"F%08x L%08x .. %08x" "\n",
		c->index, tag, musb_readw(c->hw_ep->regs, MUSB_TXCSR),
		musb_readl(c->state_ram, 0 * 4),	/* head ptr */
		musb_readl(c->state_ram, 1 * 4),	/* sop bd */
		musb_readl(c->state_ram, 2 * 4),	/* current bd */
		musb_readl(c->state_ram, 3 * 4),	/* buf offset */
		musb_readl(c->state_ram, 4 * 4),	/* flags */
		musb_readl(c->state_ram, 5 * 4),	/* len */
		/* dummy/unused word 6 */
		musb_readl(c->state_ram, 7 * 4)		/* completion */
	);
}

/* Context: controller irqlocked */
static inline void
cppi_rndis_update(struct cppi_channel *c, int is_rx,
		  void *__iomem tibase, int is_rndis)
{
	/* we may need to change the rndis flag for this cppi channel */
	if (c->is_rndis != is_rndis) {
		u32 regval = musb_readl(tibase, DAVINCI_RNDIS_REG);
		u32 temp = 1 << (c->index);

		if (is_rx)
			temp <<= 16;
		if (is_rndis)
			regval |= temp;
		else
			regval &= ~temp;
		musb_writel(tibase, DAVINCI_RNDIS_REG, regval);
		c->is_rndis = is_rndis;
	}
}

#if MUSB_DEBUG > 0
static void cppi_dump_rxbd(const char *tag, struct cppi_descriptor *bd)
{
	pr_debug("RXBD/%s %08x: "
		 "nxt %08x buf %08x off.blen %08x opt.plen %08x\n",
		 tag, bd->dma,
		 bd->hw_next, bd->hw_bufp, bd->hw_off_len, bd->hw_options);
}
#endif

static void cppi_dump_rxq(int level, const char *tag, struct cppi_channel *rx)
{
#if MUSB_DEBUG > 0
	struct cppi_descriptor	*bd;

	if (!_dbg_level(level))
		return;
	cppi_dump_rx(level, rx, tag);
	if (rx->last_processed)
		cppi_dump_rxbd("last", rx->last_processed);
	for (bd = rx->head; bd; bd = bd->next)
		cppi_dump_rxbd("active", bd);
#endif
}

static inline int cppi_autoreq_update(struct cppi_channel *rx,
				      void *__iomem tibase, int shortpkt,
				      u8 rndis, signed n_bds, u8 startreq,
				      u8 endreq)
{
	u32 tmp, val;

	/* assert(is_host_active(musb)) */

	/* start from "AutoReq never" */
	tmp = musb_readl(tibase, DAVINCI_AUTOREQ_REG);
	val = tmp & ~((0x3) << (rx->index * 2));

	/* HCD arranged reqpkt for packet #1.  we arrange int
	 * for all but the last one, maybe in two segments.
	 */

	if (shortpkt && rndis) {
		val = (val | ((0x01) << (rx->index * 2)));
		rx->autoreq = 0x01;
	} else if (shortpkt && !rndis) {
		rx->autoreq = 0x00;
	} else if ((!shortpkt) && (n_bds > 2)) {
	/* there might be shortpacket not request we might convert in to
	 * RNDIS mode
	 */
		val = (val | ((0x03) << (rx->index * 2)));
		rx->autoreq = 0x03;
		if (endreq)
			n_bds -= 2;
	} else {
		rx->autoreq = 0;
	}

	if (val != tmp) {
		musb_writel(tibase, DAVINCI_AUTOREQ_REG, val);
	}

	return n_bds;
}


/* Buffer enqueuing Logic:
 *
 *  - RX builds new queues each time, to help handle routine "early
 *    termination" cases (faults, including errors and short reads)
 *    more correctly.
 *
 *  - for now, TX reuses the same queue of BDs every time
 *
 * REVISIT long term, we want a normal dynamic model.
 * ... the goal will be to append to the
 * existing queue, processing completed "dma buffers" (segments) on the fly.
 *
 * Otherwise we force an IRQ latency between requests, which slows us a lot
 * (especially in "transparent" dma).  Unfortunately that model seems to be
 * inherent in the DMA model from the Mentor code, except in the rare case
 * of transfers big enough (~128+ KB) that we could append "middle" segments
 * in the TX paths.  (RX can't do this, see below.)
 *
 * That's true even in the CPPI- friendly iso case, where most urbs have
 * several small segments provided in a group and where the "packet at a time"
 * "transparent" DMA model is always correct, even on the RX side.
 */

/*
 * CPPI TX:
 * ========
 * TX is a lot more reasonable than RX; it doesn't need to run in
 * irq-per-packet mode very often.  RNDIS mode seems to behave too
 * (other how it handles the exactly-N-packets case).  Building a
 * txdma queue with multiple requests (urb or usb_request) looks
 * like it would work ... but fault handling still needs testing.
 */
static void
cppi_next_tx_segment(struct musb *musb, struct cppi_channel *tx, int rndis)
{
	unsigned maxpacket = tx->maxpacket;
	size_t length = tx->buf_len - tx->offset;
	struct cppi_descriptor *bd;
	unsigned n_bds;
	unsigned i, iso = (tx->rxmode == 2)? 1 : 0;
	int	iso_desc = tx->iso_desc;
	struct cppi_tx_stateram *txstate = tx->state_ram;

	/* TX can use the CPPI "rndis" mode, where we can probably fit this
	 * transfer in one BD and one IRQ; though some common cases (like
	 * packet length not being n*64 bytes) can't work that way.
	 *
	 * To cppi hardware (but not the RNDIS protocol!) RNDIS is mostly a
	 * "short packet termination" mode.  So the only time we would NOT
	 * want to use it is to avoid sending spurious zero length packets,
	 * or when hardware constraints prevent it.
	 */

#if 0				/* Disable RNDIS on both Tx & Rx*/
	if (!rndis && (length % maxpacket) != 0)
		rndis = 1;
	if (rndis && (length > 0xffff || (maxpacket & 0x3f) != 0
		      /* "undocumented" rndis mode constraint on txlen */
		      || (length & 0x3f) != 0))
#endif
		rndis = 0;

	if (iso) {
		n_bds = (tx->hw_ep->num_iso_desc - iso_desc)
				>= NUM_TXCHAN_BD ?  NUM_TXCHAN_BD :
				tx->hw_ep->num_iso_desc - iso_desc;
		tx->offset = tx->hw_ep->iso_desc[iso_desc].offset;
	} else if (!length) {
		rndis = 0;
		n_bds = 1;
	} else if (rndis) {
		maxpacket = length;
		n_bds = 1;
	} else {
		n_bds = length / maxpacket;
		if ((length % maxpacket) || (tx->rxmode == 1))
			n_bds++;

		n_bds = min(n_bds, (unsigned)NUM_TXCHAN_BD);
		length = min(n_bds * maxpacket, length);
	}

	DBG(4, "TX DMA%d, pktsz %d %s bds %d dma 0x%x len %u\n",
	    tx->index,
	    maxpacket,
	    rndis ? "rndis" : "transparent",
	    n_bds, tx->buf_dma + tx->offset, length);

	cppi_rndis_update(tx, 0, musb->ctrl_base, rndis);

	/* assuming here that DmaProgramChannel is called during
	 * transfer initiation ... current code maintains state
	 * for one outstanding request only (no queues, not even
	 * the implicit ones of an iso urb).
	 */

	bd = tx->freelist;
	tx->head = tx->freelist;
	tx->last_processed = NULL;

	/* Prepare queue of BDs first, then hand it to hardware.
	 * All BDs except maybe the last should be of full packet
	 * size; for RNDIS there _is_ only that last packet.
	 */
	for (i = 0; i < n_bds;) {
		if (++i < n_bds && bd->next)
			bd->hw_next = bd->next->dma;
		else
			bd->hw_next = 0;

		bd->hw_bufp = tx->buf_dma + tx->offset;

		if (iso) {
			bd->hw_off_len = tx->hw_ep->iso_desc[iso_desc].length;
			bd->hw_options = CPPI_SOP_SET|CPPI_EOP_SET|CPPI_OWN_SET;
			if (tx->hw_ep->iso_desc[iso_desc].length == 0)
				bd->hw_options |= CPPI_ZERO_SET|1;
			else
				bd->hw_options |= tx->hw_ep->
						iso_desc[iso_desc].length;

			tx->offset =  tx->hw_ep->
						iso_desc[++iso_desc].offset;
		} else if ((tx->offset + maxpacket)
		    <= tx->buf_len) {
			tx->offset += maxpacket;
			bd->hw_off_len = maxpacket;
			bd->hw_options = CPPI_SOP_SET | CPPI_EOP_SET
			    | CPPI_OWN_SET | maxpacket;
		} else {
			/* only this one may be a partial USB Packet */
			u32 buffsz;

			buffsz = tx->buf_len - tx->offset;
			tx->offset = tx->buf_len;
			bd->hw_off_len = (buffsz) ? buffsz : 1;

			bd->hw_options = CPPI_SOP_SET | CPPI_EOP_SET
			    | CPPI_OWN_SET | (buffsz ? buffsz : 1);
			if (buffsz == 0) {
				tx->hw_ep->zero = 1;
				bd->hw_options |= CPPI_ZERO_SET;
			}
		}

		DBG(5, "TXBD %p: nxt %08x buf %08x len %04x opt %08x\n",
			bd, bd->hw_next, bd->hw_bufp, bd->hw_off_len,
			bd->hw_options);

		/* update the last BD enqueued to the list */
		tx->tail = bd;
		bd = bd->next;
	}

	/* BDs live in DMA-coherent memory, but writes might be pending */
	/*cpu_drain_writebuffer();*/

	/* Write to the HeadPtr in StateRam to trigger */
	txstate->tx_head = (u32) tx->freelist->dma;

	cppi_dump_tx(5, tx, "/S");
}

/*
 * CPPI RX:
 * ========
 * Consider a 1KB bulk RX buffer in two scenarios:  (a) it's fed two 300 byte
 * packets back-to-back, and (b) it's fed two 512 byte packets back-to-back.
 * (Full speed transfers have similar scenarios.)
 *
 * The correct behavior for Linux is that (a) fills the buffer with 300 bytes,
 * and the next packet goes into a buffer that's queued later; while (b) fills
 * the buffer with 1024 bytes.  How to do that with CPPI?
 *
 * - CPPI RX queues in "rndis" mode -- one single BD -- handle (a) correctly,
 *   but (b) loses _badly_ because nothing (!) happens when that second packet
 *   fills the buffer, much less when a third one arrives.  (Which makes this
 *   not a "true" RNDIS mode.  In the RNDIS protocol short-packet termination
 *   is optional, and it's fine if senders pad messages out to end-of-buffer.)
 *
 * - CPPI RX queues in "transparent" mode -- two BDs with 512 bytes each -- have
 *   converse problems:  (b) is handled correctly, but (a) loses badly.  CPPI RX
 *   ignores SOP/EOP markings and processes both of those BDs; so both packets
 *   are loaded into the buffer (with a 212 byte gap between them), and the next
 *   buffer queued will NOT get its 300 bytes of data. (It seems like SOP/EOP
 *   are intended as outputs for RX queues, not inputs...)
 *
 * - A variant of "transparent" mode -- one BD at a time -- is the only way to
 *   reliably make both cases work, with software handling both cases correctly
 *   and at the significant penalty of needing an IRQ per packet.  (The lack of
 *   I/O overlap can be slightly ameliorated by enabling double buffering.)
 *
 * So how to get rid of IRQ-per-packet?  The transparent multi-BD case could
 * be used in special cases like mass storage, which sets URB_SHORT_NOT_OK
 * (or maybe its peripheral side counterpart) to flag (a) scenarios as errors
 * with guaranteed driver level fault recovery and scrubbing out what's left
 * of that garbaged datastream.
 *
 * But there seems to be no way to identify the cases where CPPI RNDIS mode
 * is appropriate -- which do NOT include the RNDIS driver, but do include
 * the CDC Ethernet driver! -- and the documentation is incomplete/wrong.
 * So we can't _ever_ use RX RNDIS mode.
 *
 * Leaving only "transparent" mode; we avoid multi-bd modes in almost all
 * cases other than mass storage class.  Otherwise e're correct but slow,
 * since CPPI penalizes our need for a "true RNDIS" default mode.
 */

/**
 * cppi_next_rx_segment - dma read for the next chunk of a buffer
 * @musb: the controller
 * @rx: dma channel
 * @onepacket: true unless caller treats short reads as errors, and
 *	 performs fault recovery above usbcore.
 * Context: controller irqlocked
 *
 * See above notes about why we can't use multi-BD RX queues except in
 * rare cases (mass storage class), and can never use the hardware "rndis"
 * mode (since it's not a "true" RNDIS mode).
 *
 * It's ESSENTIAL that callers specify "onepacket" mode unless they kick in
 * code to recover from corrupted datastreams after each short transfer.
 */
static void
cppi_next_rx_segment(struct musb *musb, struct cppi_channel *rx, int shortpkt)
{
	unsigned maxpacket = rx->maxpacket;
	dma_addr_t addr = rx->buf_dma + rx->offset;
	size_t length = rx->buf_len - rx->offset;
	struct cppi_descriptor *bd, *tail;
	signed n_bds;
	signed i;
	void *__iomem tibase = musb->ctrl_base;
	u8 rndis = 0;
	int max_bd = 0;
	unsigned iso = (rx->rxmode == 2)? 1 : 0;
	int     iso_desc = rx->iso_desc;

	if (((rx->rxmode == 1) && ((maxpacket & 0x3f) == 0)
		/*REVISIT MAXPACKET CHECK!!!!*/
	     && ((length & 0x3f) == 0))) {
		rndis = 1;
		max_bd = 65536;	/* MAX buffer aize per RNDIS BD is 64 K */
		} else {
		rndis = 0;
		max_bd = maxpacket;
	}
#ifdef CONFIG_ARCH_DAVINCI
	/* Do not use RNDIS dma for DaVinci */
	rndis = 0;
	max_bd = maxpacket;
#endif
	if (iso)
		max_bd = rx->hw_ep->iso_desc[iso_desc].length;

	n_bds = length / max_bd;
	if (length % max_bd)
		n_bds++;

	n_bds = min(n_bds, (signed)NUM_RXCHAN_BD);
	if (n_bds == NUM_RXCHAN_BD)
		length = min(length, (size_t) (n_bds * max_bd));

	cppi_rndis_update(rx, 1, musb->ctrl_base, rndis);

	/* In host mode, autorequest logic can generate some IN tokens; it's
	 * tricky since we can't leave REQPKT set in RXCSR after the transfer
	 * finishes. So:  multipacket transfers involve two or more segments.
	 * And always at least two IRQs.
	 */
	if (is_host_active(musb))
		n_bds = cppi_autoreq_update(rx, tibase, shortpkt, rndis, n_bds,
					    (u8) !rx->offset,
					    (u8) (rx->buf_len <=
						  (rx->offset + length)
					    ));

	DBG(4, "RX DMA%d seg, maxp %d %spacket bds %d (cnt %d) "
	    "dma 0x%x len %u/%u/%u\n",
	    rx->index, max_bd,
	    shortpkt ? "one" : "multi",
	    n_bds,
	    musb_readl(tibase, DAVINCI_RXCPPI_BUFCNT0_REG + (rx->index * 4))
	    & 0xffff, addr, length, rx->actuallen, rx->buf_len);

	/* only queue one segment at a time, since the hardware prevents
	 * correct queue shutdown after unexpected short packets
	 */
	bd = cppi_bd_alloc(rx);
	rx->head = bd;

	/* Build BDs for all packets in this segment */
	for (i = 0, tail = NULL; bd && i < n_bds; i++, tail = bd) {
		u32 buffsz;

		if (i) {
			bd = cppi_bd_alloc(rx);
			if (!bd)
				break;
			tail->next = bd;
			tail->hw_next = bd->dma;
		}
		bd->hw_next = 0;

		/* all but the last packet will be maxpacket size */
		if (max_bd < length)
			buffsz = max_bd;
		else
			buffsz = length;

		bd->hw_bufp = addr;
		addr += buffsz;
		rx->offset += buffsz;

		bd->hw_off_len = (0 /*offset */  << 16) + buffsz;
		bd->buflen = buffsz;

		bd->hw_options = CPPI_OWN_SET | (i == 0 ? length : 0);
		length -= buffsz;
	}

	/* we always expect at least one reusable BD! */
	if (!tail) {
		WARN("rx dma%d -- no BDs? need %d\n", rx->index, n_bds);
		return;
	} else if (i < n_bds)
		WARN("rx dma%d -- only %d of %d BDs\n", rx->index, i, n_bds);

	tail->next = NULL;
	tail->hw_next = 0;

	bd = rx->head;
	rx->tail = tail;
#if 0
	/* short reads and other faults should terminate this entire
	 * dma segment.  we want one "dma packet" per dma segment, not
	 * one per USB packet, terminating the whole queue at once...
	 * NOTE that current hardware seems to ignore SOP and EOP.
	 */
	if (MGC_DebugLevel >= 5) {
		struct cppi_descriptor *d;

		for (d = rx->head; d; d = d->next)
			cppi_dump_rxbd("S", d);
	}
#endif
	rx->last_processed = NULL;
	/* BDs live in DMA-coherent memory, but writes might be pending */
	/*cpu_drain_writebuffer();*/

	/* REVISIT specs say to write this AFTER the BUFCNT register
	 * below ... but that loses badly.
	 */
	musb_writel(rx->state_ram, 4, bd->dma);

	/* bufferCount must be at least 3, and zeroes on completion
	 * unless it underflows below zero, or stops at two, or keeps
	 * growing ... grr.
	 */
	i = musb_readl(tibase, DAVINCI_RXCPPI_BUFCNT0_REG + (rx->index * 4))
	    & 0xffff;

	if (n_bds > (i - 2)) {
		musb_writel(tibase,
			    DAVINCI_RXCPPI_BUFCNT0_REG + (rx->index * 4),
			    n_bds - i + 2);
	}

	cppi_dump_rx(4, rx, "/S");
}

/**
 * cppi_channel_program - program channel for data transfer
 * @pChannel: the channel
 * @wPacketsz: max packet size
 * @mode: For RX, 1 unless the usb protocol driver promised to treat
 *	 all short reads as errors and kick in high level fault recovery.
 *	 For TX, 0 unless the protocol driver _requires_ short-packet
 *	 termination mode.
 * @dma_addr: dma address of buffer
 * @dwLength: length of buffer
 * Context: controller irqlocked
 */
static int  cppi_channel_program(struct dma_channel *ch,
			       u16 maxpacket, u8 mode,
			       dma_addr_t dma_addr, u32 len)
{
	struct cppi_channel *cppi_ch = container_of(ch, struct cppi_channel,
						channel);
	struct cppi *controller = cppi_ch->controller;
	struct musb *musb = controller->musb;

	switch (ch->status) {
	case MUSB_DMA_STATUS_BUS_ABORT:
	case MUSB_DMA_STATUS_CORE_ABORT:
		/* fault irq handler should have handled cleanup */
		WARN("%cX DMA%d not cleaned up after abort!\n",
		     cppi_ch->transmit ? 'T' : 'R', cppi_ch->index);
		/*WARN_ON(1);*/
		break;
	case MUSB_DMA_STATUS_BUSY:
		WARN("program active channel?  %cX DMA%d\n",
		     cppi_ch->transmit ? 'T' : 'R', cppi_ch->index);
		/*WARN_ON(1);*/
		break;
	case MUSB_DMA_STATUS_UNKNOWN:
		DBG(1, "%cX DMA%d not allocated!\n",
		    cppi_ch->transmit ? 'T' : 'R', cppi_ch->index);
		/* FALLTHROUGH */
	case MUSB_DMA_STATUS_FREE:
		break;
	}

	ch->status = MUSB_DMA_STATUS_BUSY;

	/* set transfer parameters, then queue up its first segment */
	cppi_ch->buf_dma = dma_addr;
	cppi_ch->offset = 0;
	cppi_ch->maxpacket = maxpacket;
	cppi_ch->actuallen = 0;
	cppi_ch->buf_len = len;
	cppi_ch->rxmode = mode;
	cppi_ch->reqcomplete = 0;
	cppi_ch->autoreq = 0;
	cppi_ch->iso_desc = 0;

	/* TX channel? or RX? */
	if (cppi_ch->transmit)
		cppi_next_tx_segment(musb, cppi_ch, mode);
	else
		cppi_next_rx_segment(musb, cppi_ch , mode);

	return true;
}

static int cppi_rx_scan(struct cppi *cppi, unsigned ch, u8 abort)
{
	struct cppi_channel *rx = &cppi->rx[ch];
	struct cppi_rx_stateram *state = rx->state_ram;
	struct cppi_descriptor *bd;
	struct cppi_descriptor *last = rx->last_processed;
	int completed = 0;
	dma_addr_t safe2ack;
	u32 csr;

	cppi_dump_rx(6, rx, "/K");

	if (abort) {
		safe2ack = musb_readl(CAST & state->rx_complete, 0);

		if ((last == NULL) || (safe2ack == last->dma)) {
			if (last == NULL)
				last = rx->head;
			goto free;
		}
	}

	bd = last ? last->next : rx->head;

	do {
		safe2ack = musb_readl(CAST & state->rx_complete, 0);
		if (completed)
			break;
		do {
			u16 len;
			/*rmb();*/
			DBG(5, "C/RXBD %08x: nxt %08x buf %08x "
			    "off.len %08x opt.len %08x (%d)\n",
			    bd->dma, bd->hw_next, bd->hw_bufp,
			    bd->hw_off_len, bd->hw_options, rx->actuallen);

			/* actual packet received length */
			len = bd->hw_off_len & CPPI_RECV_PKTLEN_MASK;
			if (bd->hw_options & CPPI_ZERO_SET)
				len = 0;

			if (bd->hw_next == 0)
				completed = 1;

			if ((len < bd->buflen) && (rx->rxmode != 2)) {
				rx->reqcomplete = 1;
				completed = 1;
				DBG(3, "rx short %d/%d (%d)\n",
				    len, bd->buflen, rx->actuallen);
			}

			if (rx->rxmode == 2) {
				rx->hw_ep->iso_desc[rx->iso_desc].length = len;
				if (completed) {
					rx->reqcomplete = 1;
					rx->iso_desc = 0;
				} else
					rx->iso_desc++;
			}

			rx->actuallen += len;
			cppi_bd_free(rx, last);
			last = bd;
			if (safe2ack == bd->dma) {
				bd = bd->next;
				break;
			}
			bd = bd->next;
		} while (!completed);
	} while (musb_readl(CAST & state->rx_complete, 0) != safe2ack);

	if (is_host_active(rx->controller->musb) && (!abort) &&
		(rx->autoreq == 0) &&
		((!completed) || (completed && (rx->reqcomplete == 0) &&
		(rx->actuallen != rx->buf_len)))) {
		csr = musb_readw(rx->hw_ep->regs, MUSB_RXCSR);
		csr |= MUSB_RXCSR_H_REQPKT;
		musb_writew(rx->hw_ep->regs, MUSB_RXCSR, csr);
	}

	rx->last_processed = last;
	musb_writel(CAST & state->rx_complete, 0, safe2ack);
free:
	if (completed || abort) {

		/* Flush BD's not consumed */
		while (last != NULL) {
			bd = last;
			last = last->next;
			cppi_bd_free(rx, bd);
		}
		if (abort) {
			safe2ack = musb_readl(CAST & state->rx_complete, 0);
			musb_writel(CAST & state->rx_complete, 0, safe2ack);
		}
		rx->last_processed = NULL;
		rx->head = NULL;
		rx->tail = NULL;
	}

	cppi_dump_rx(6, rx, completed ? "/completed" : "/cleaned");
	return completed;
}

void cppi_completion(struct musb *musb, u32 rx, u32 tx)
{
	void *__iomem tibase;
	int i, channum;
	u8 reqcomplete;
	struct cppi *cppi;
	struct cppi_descriptor *bdptr;

	cppi = container_of(musb->dma_controller, struct cppi, controller);
	tibase = cppi->mregs - DAVINCI_BASE_OFFSET;

	/* process TX channels */
	for (channum = 0; tx; tx = tx >> 1, channum++) {
		if (tx & 1) {
			struct cppi_channel *txchannel;
			struct cppi_tx_stateram *txstate;
			u32 safe2ack = 0;

			txchannel = cppi->tx + channum;
			txstate = txchannel->state_ram;

			/* FIXME  need a cppi_tx_scan() routine, which
			 * can also be called from abort code
					 */
			cppi_dump_tx(5, txchannel, "/E");

			bdptr = txchannel->head;
			i = 0;
			reqcomplete = 0;

			do {
				safe2ack = txstate->tx_complete;
				do {
					/*rmb();*/
					DBG(5, "C/TXBD %p n %x b %x off %x\
						opt %x\n",
						bdptr, bdptr->hw_next,
						bdptr->hw_bufp,
						bdptr->hw_off_len,
						bdptr->hw_options);

					txchannel->actuallen += (u16) (bdptr->
							hw_off_len &
							CPPI_BUFFER_LEN_MASK);
					if (txchannel->rxmode == 2) {
						txchannel->hw_ep->
						iso_desc[txchannel->iso_desc].
						status = 0;
						txchannel->iso_desc++;
					}
					if (bdptr->dma == safe2ack) {
						if (bdptr->hw_options &
							CPPI_ZERO_SET)
							txchannel->actuallen -=
							1;
						if (bdptr->hw_next == 0)
							reqcomplete = 1;
						txchannel->last_processed =
						    bdptr;
						bdptr = bdptr->next;
						break;
					}
					bdptr = bdptr->next;
				} while (1);
			} while (txstate->tx_complete != safe2ack);

			txstate->tx_complete = txchannel->last_processed->
						dma;

			/* on end of segment, maybe go to next one */
			if (reqcomplete) {
				cppi_dump_tx(4, txchannel, "/complete");

				/* transfer more, or report completion */
				if ((txchannel->actuallen
				    >= txchannel->buf_len)) {
					if ((txchannel->rxmode == 1) &&
						(!(txchannel->actuallen %
						txchannel->maxpacket)) &&
						(!txchannel->hw_ep->zero)) {
						cppi_next_tx_segment(musb,
							txchannel,
							txchannel->rxmode);
					} else {
						txchannel->hw_ep->zero = 0;
						txchannel->head =
							NULL;
						txchannel->tail =
							NULL;
						txchannel->channel.status =
						    MUSB_DMA_STATUS_FREE;
						txchannel->channel.
							actual_len =
						    txchannel->actuallen;
						musb_dma_completion(musb,
							channum + 1, 1);
					}

				} else {
					/* Bigger transfer than we could fit in
					 * that first batch of descriptors...
				 */
					cppi_next_tx_segment(musb,
							txchannel,
							txchannel->rxmode);
			}
			} else
				txchannel->head = bdptr;
		}
	}

	/* Start processing the RX block */
	for (channum = 0; rx; rx = rx >> 1, channum++) {

		if (rx & 1) {
			struct cppi_channel *rxchannel;

			rxchannel = cppi->rx + channum;
			/* There is a race condition between abort channel and
			 * on going traffic.  Ensure that pending interrupts
			 * (that were raised during clean up but was held due
			 * to interrupt block) are acknowledged correctly by
			 * adressing those interrupts as abort (from interrupt
			 * context, as we already handled the user initiated
			 * abort which actually lead to this race). This occurs
			 * possible due to the fact that we do not support
			 * clean RX teardown at this point (May not require this
			 * code once RX teardown is correctly supported in
			 * hardware).
			 */
			if (rxchannel->head)
				reqcomplete = cppi_rx_scan(cppi, channum, 0);
			else {
				cppi_rx_scan(cppi, channum, 1);
				continue;
			}

			/* let incomplete dma segments finish */
			if (!reqcomplete)
				continue;

			/* start another dma segment if needed */
			if ((rxchannel->actuallen != rxchannel->buf_len)
			    && !(rxchannel->reqcomplete)) {
				cppi_next_rx_segment(musb, rxchannel,
						     rxchannel->rxmode);
				continue;
			}

			/* all segments completed! */
			rxchannel->channel.status = MUSB_DMA_STATUS_FREE;
			rxchannel->channel.actual_len =
			    rxchannel->actuallen;
			musb_dma_completion(musb, channum + 1, 0);
		}
	}

	/* write to CPPI EOI register to re-enable interrupts */
	musb_writel(tibase, DAVINCI_CPPI_EOI_REG, 0);
}

/* Instantiate a software object representing a DMA controller. */
struct dma_controller *__init dma_controller_create(struct musb *musb,
						void __iomem *mregs)
{
	struct cppi		*controller;

	controller = kzalloc(sizeof *controller, GFP_KERNEL);
	if (!controller)
		return NULL;

	controller->mregs = mregs;
	controller->tibase = mregs - DAVINCI_BASE_OFFSET;

	controller->musb = musb;
	controller->controller.start = cppi_controller_start;
	controller->controller.stop = cppi_controller_stop;
	controller->controller.channel_alloc = cppi_channel_allocate;
	controller->controller.channel_release = cppi_channel_release;
	controller->controller.channel_program = cppi_channel_program;
	controller->controller.channel_abort = cppi_channel_abort;

	/* NOTE: allocating from on-chip SRAM would give the least
	 * contention for memory access, if that ever matters here.
	 */

	/* setup BufferPool */
	controller->pool = dma_pool_create("cppi",
					    controller->musb->controller,
					    sizeof(struct cppi_descriptor),
					    CPPI_DESCRIPTOR_ALIGN, 0);
	if (!controller->pool) {
		kfree(controller);
		return NULL;
	}

	return &controller->controller;
}

/*
 *  Destroy a previously-instantiated DMA controller.
 */
void dma_controller_destroy(struct dma_controller *c)
{
	struct cppi *cpcontroller = container_of(c, struct cppi, controller);;

	/* assert:  caller stopped the controller first */
	dma_pool_destroy(cpcontroller->pool);

	kfree(cpcontroller);

}

/*
 * Context: controller irqlocked, endpoint selected
 */
static int cppi_channel_abort(struct dma_channel *channel)
{
	struct cppi_channel *cppi_ch = container_of(channel,
						struct cppi_channel, channel);
	struct cppi *controller = cppi_ch->controller;
	int chnum = cppi_ch->index;
	void *__iomem mbase;
	void *__iomem tibase;
	u32 regval;

	switch (channel->status) {
	case MUSB_DMA_STATUS_BUS_ABORT:
	case MUSB_DMA_STATUS_CORE_ABORT:
		/* from RX or TX fault irq handler */
	case MUSB_DMA_STATUS_BUSY:
		/* the hardware needs shutting down */
		break;
	case MUSB_DMA_STATUS_UNKNOWN:
		DBG(8, "%cX DMA%d not allocated\n",
		    cppi_ch->transmit ? 'T' : 'R', cppi_ch->index);
		/* FALLTHROUGH */
	case MUSB_DMA_STATUS_FREE:
		return 0;
	}

	if (chnum & ~CPPI_CHNUM_BITS_MASK)
		return -EINVAL;

	if (!cppi_ch->transmit && cppi_ch->head)
		cppi_dump_rxq(4, "/abort", cppi_ch);

	mbase = controller->mregs;
	tibase = mbase - DAVINCI_BASE_OFFSET;

	/* REVISIT should rely on caller having done this,
	 * and caller should rely on us not changing it.
	 * peripheral code is safe ... check host too.
	 */
	musb_ep_select(mbase, chnum + 1);

	if (cppi_ch->transmit) {
		struct cppi_tx_stateram *__iomem txstate;
		int enabled;

		/* mask interrupts raised to signal teardown complete.  */
		enabled = musb_readl(tibase, DAVINCI_TXCPPI_INTENAB_REG)
		    & (1 << cppi_ch->index);
		if (enabled)
			musb_writel(tibase, DAVINCI_TXCPPI_INTCLR_REG,
				    (1 << cppi_ch->index));

		/* REVISIT put timeouts on these controller handshakes */

		cppi_dump_tx(6, cppi_ch, " (teardown)");

		/* teardown DMA engine then usb core */
		do {
			regval = musb_readl(tibase, DAVINCI_TXCPPI_TEAR_REG);
		} while (!(regval & CPPI_TEAR_READY));
		musb_writel(tibase, DAVINCI_TXCPPI_TEAR_REG, chnum);

		txstate = cppi_ch->state_ram;
		do {
			regval = txstate->tx_complete;
		} while (0xFFFFFFFC != regval);
		txstate->tx_complete = 0xFFFFFFFC;

		musb_writel(tibase, DAVINCI_CPPI_EOI_REG, 0);

		/* FIXME clean up the transfer state ... here?
		 * the completion routine should get called with
		 * an appropriate status code.
		 */

		regval = musb_readw(cppi_ch->hw_ep->regs, MUSB_TXCSR);
		regval |= MUSB_TXCSR_FLUSHFIFO;
		musb_writew(cppi_ch->hw_ep->regs, MUSB_TXCSR, regval);
		musb_writew(cppi_ch->hw_ep->regs, MUSB_TXCSR, regval);

		txstate->tx_head = 0;
		txstate->tx_buf = 0;
		txstate->tx_buf_current = 0;
		txstate->tx_current = 0;
		txstate->tx_info = 0;
		txstate->tx_rem_len = 0;

		/* Ensure that we clean up any Interrupt asserted
		 * 1. Write to completion Ptr value 0x1(bit 0 set)
		 *    (write back mode)
		 * 2. Write to completion Ptr value 0x0(bit 0 cleared)
		 *    (compare mode)
		 * Value written is compared(for bits 31:2) and being
		 * equal interrupt deasserted?
		 */

		/* write back mode, bit 0 set, hence completion Ptr
		 * must be updated
		 */
		txstate->tx_complete = 0x1;
		/* compare mode, write back zero now */
		txstate->tx_complete = 0;

		/* re-enable interrupt */
		if (enabled)
			musb_writel(tibase, DAVINCI_TXCPPI_INTENAB_REG,
				    (1 << cppi_ch->index));

		cppi_dump_tx(5, cppi_ch, " (done teardown)");

		/* REVISIT tx side _should_ clean up the same way
		 * as the RX side ... this does no cleanup at all!
		 */

	} else {		/* RX */

		u16 csr;

		/* NOTE: docs don't guarantee any of this works ...  we
		 * expect that if the usb core stops telling the cppi core
		 * to pull more data from it, then it'll be safe to flush
		 * current RX DMA state iff any pending fifo transfer is done.
		 */

		/* for host, ensure ReqPkt is never set again */
		if (is_host_active(cppi_ch->controller->musb)) {
			regval = musb_readl(tibase, DAVINCI_AUTOREQ_REG);
			regval &= ~((0x3) << (cppi_ch->index * 2));
			musb_writel(tibase, DAVINCI_AUTOREQ_REG, regval);
		}

		csr = musb_readw(cppi_ch->hw_ep->regs, MUSB_RXCSR);

		/* for host, clear (just) ReqPkt at end of current packet(s) */
		if (is_host_active(cppi_ch->controller->musb)) {
			csr |= MUSB_RXCSR_H_WZC_BITS;
			csr &= ~MUSB_RXCSR_H_REQPKT;
		} else
			csr |= MUSB_RXCSR_H_WZC_BITS;

		/* clear dma enable */
		csr &= ~(MUSB_RXCSR_DMAENAB);
		musb_writew(cppi_ch->hw_ep->regs, MUSB_RXCSR, csr);

		/* quiesce: wait for current dma to finish (if not cleanup)
		 * we can't use bit zero of stateram->sopDescPtr since that
		 * refers to an entire "DMA packet" not just emptying the
		 * current fifo, and most segements need multiple fifos.
		 */
		if (channel->status == MUSB_DMA_STATUS_BUSY)
			udelay(50);

		/* scan the current list, reporting any data that was
		 * transferred and acking any IRQ
		 */
		cppi_rx_scan(controller, chnum, 1);
		channel->actual_len += cppi_ch->actuallen;

		/* clobber the existing state once it's idle
		 *
		 * NOTE:  arguably, we should also wait for all the other
		 * RX channels to quiesce (how??) and then temporarily
		 * disable RXCPPI_CTRL_REG ... but it seems that we can
		 * rely on the controller restarting from state ram, with
		 * only RXCPPI_BUFCNT state being bogus.  BUFCNT will
		 * correct itself after the next DMA transfer though.
		 *
		 * REVISIT does using rndis mode change that?
		 */
		cppi_reset_rx(cppi_ch->state_ram);

		/* next DMA request _should_ load cppi head ptr */

		/* ... we don't "free" that list, only mutate it in place.  */
		cppi_dump_rx(5, cppi_ch, " (done abort)");
	}

	channel->status = MUSB_DMA_STATUS_FREE;
	cppi_ch->buf_dma = 0;
	cppi_ch->offset = 0;
	cppi_ch->buf_len = 0;
	cppi_ch->maxpacket = 0;
	return 0;
}

/* TBD Queries:
 *
 * Power Management ... probably turn off cppi during suspend, restart;
 * check state ram?  Clocking is presumably shared with usb core.
 */
