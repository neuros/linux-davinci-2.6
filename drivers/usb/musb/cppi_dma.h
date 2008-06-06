/* Copyright (C) 2005-2006 by Texas Instruments */

#ifndef _CPPI_DMA_H_
#define _CPPI_DMA_H_

#include <linux/slab.h>
#include <linux/list.h>
#include <linux/smp_lock.h>
#include <linux/errno.h>
#include <linux/dmapool.h>

#include "musb_dma.h"
#include "musb_core.h"
#include "davinci.h"

/* hOptions bit masks for CPPI BDs */
#define CPPI_SOP_SET	((u32)(1 << 31))
#define CPPI_EOP_SET	((u32)(1 << 30))
#define CPPI_OWN_SET	((u32)(1 << 29))	/* owned by cppi */
#define CPPI_EOQ_MASK	((u32)(1 << 28))
#define CPPI_ZERO_SET	((u32)(1 << 23))	/* rx saw zlp; tx issues one */
#define CPPI_RXABT_MASK	((u32)(1 << 19))	/* need more rx buffers */

#define CPPI_RECV_PKTLEN_MASK 0xFFFF
#define CPPI_BUFFER_LEN_MASK 0xFFFF

#define CPPI_TEAR_READY ((u32)(1 << 31))
#define CPPI_CHNUM_BITS_MASK  0x3

/* CPPI RX/TX state RAM */

struct cppi_tx_stateram {
	u32 tx_head;		    	/* "DMA packet" head descriptor */
	u32 tx_buf;
	u32 tx_current;		 	/* current descriptor */
	u32 tx_buf_current;
	u32 tx_info;		    	/* flags, remaining buflen */
	u32 tx_rem_len;
	u32 tx_dummy;		   	/* unused */
	u32 tx_complete;
};

struct cppi_rx_stateram {
	u32 rx_skipbytes;
	u32 rx_head;
	u32 rx_sop;		     	/* "DMA packet" head descriptor */
	u32 rx_current;		 	/* current descriptor */
	u32 rx_buf_current;
	u32 rx_len_len;
	u32 rx_cnt_cnt;
	u32 rx_complete;
};

/* CPPI data structure definitions */

/**
 *  CPPI  Buffer Descriptor
 *
 *  Buffer Descriptor structure for USB OTG Module CPPI.Using the same across
 *  Tx/Rx
 */

#define	CPPI_DESCRIPTOR_ALIGN	16	/* bytes; 5-dec docs say 4-byte align*/

struct cppi_descriptor {
	/* Hardware Overlay */
	u32 hw_next;     /**< Next(hardware) Buffer Descriptor Pointer */
	u32 hw_bufp;	   /**<Buffer Pointer (dma_addr_t) */
	u32 hw_off_len;	    /**<Buffer_offset16,buffer_length16 */
	u32 hw_options;	    /**<Option fields for SOP,EOP etc*/

	struct cppi_descriptor *next; /**<Next(software) Buffer Descriptor
					*pointer
					*/
	dma_addr_t dma;		/* address of this descriptor */
	/* for Rx Desc, keep track of enqueued Buffer len to detect
	 * short packets
	 */
	u32 buflen;
} __attribute__ ((aligned(CPPI_DESCRIPTOR_ALIGN)));

/* forward declaration for CppiDmaController structure */
struct cppi;

/**
 *  Channel Control Structure
 *
 * CPPI  Channel Control structure. Using he same for Tx/Rx. If need be
 * derive out of this later.
 */
struct cppi_channel {
	/* First field must be dma_channel for easy type casting
	 * FIXME just use container_of() and be typesafe instead!
	 */
	struct dma_channel channel;

	/* back pointer to the Dma Controller structure */
	struct cppi *controller;

	/* which direction of which endpoint? */
	struct musb_hw_ep *hw_ep;
	u8 transmit;
	u8 index;

	/* DMA modes:  RNDIS or "transparent" */
	u8 is_rndis;
	u8 autoreq;		/* For now keep this remove this
				 * one RNDIS + length <64 segmenstation
				 * will done
				 */
	/* Rx Requested mode */
	u8 rxmode;
	u8 reqcomplete;		/* zero packet handling*/
	/* book keeping for current transfer request */
	dma_addr_t buf_dma;
	u32 buf_len;
	u32 maxpacket;
	u32 offset;		/* requested segments */
	u32 actuallen;		/* completed (Channel.actual) */

	void __iomem *state_ram;	/* CPPI state */

	/* BD management fields */
	struct cppi_descriptor *freelist;	/* Free BD Pool head pointer */
	struct cppi_descriptor *head;
	struct cppi_descriptor *tail;
	struct cppi_descriptor *last_processed;

	/* use tx_complete in host role to track endpoints waiting for
	 * FIFONOTEMPTY to clear.
	 */
	struct list_head tx_complete;
	u32 iso_desc;
};

/**
 *  CPPI Dma Controller Object
 *
 *  CPPI Dma controller object.Encapsulates all bookeeping and Data
 *  structures pertaining to the CPPI Dma Controller.
 */
struct cppi {
	/* FIXME switchover to container_of() and remove the
	 * unsafe typecasts...
	 */
	struct dma_controller controller;
	struct musb *musb;
	void __iomem *mregs;
	void __iomem *tibase;


	struct cppi_channel tx[MUSB_C_NUM_EPT - 1];
	struct cppi_channel rx[MUSB_C_NUM_EPR - 1];

	struct dma_pool *pool;

	struct list_head tx_complete;
};

/* irq handling hook */
extern void cppi_completion(struct musb *, u32 rx, u32 tx);

#endif				/* end of ifndef _CPPI_DMA_H_ */
