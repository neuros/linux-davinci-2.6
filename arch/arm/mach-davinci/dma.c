/*
 * linux/arch/arm/mach-davinci/dma.c
 *
 * TI DaVinci DMA file
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
 *
 */
#include <linux/sched.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/spinlock.h>
#include <asm/io.h>
#include <asm/arch/memory.h>
#include <linux/kernel.h>
#include <asm/arch/hardware.h>
#include <asm/arch/irqs.h>

#include <asm/arch/edma.h>

static spinlock_t dma_chan_lock;
static struct device_driver edma_driver;
static struct platform_device edma_dev;

#define LOCK_INIT     spin_lock_init(&dma_chan_lock)
#define LOCK          spin_lock(&dma_chan_lock)
#define UNLOCK        spin_unlock(&dma_chan_lock)

typedef void (*intr_callback) (void);
static int register_dma_interrupts(intr_callback, intr_callback, intr_callback,
				   intr_callback);

#define DAVINCI_DMA_REGISTER_BASE DAVINCI_DMA_3PCC_BASE

static edmacc_regs *get_edma_base(void)
{
	return ((edmacc_regs *) IO_ADDRESS(DAVINCI_DMA_REGISTER_BASE));
}

static intr_callback cb[4];

/* Structure containing the dma channel parameters */
static struct davinci_dma_lch {
	int dev_id;
	int in_use;		/* 1-used 0-unused */
	int link_lch;
	int dma_running;
	int param_no;
	int tcc;
} dma_chan[DAVINCI_EDMA_NUM_PARAMENTRY];

static struct dma_interrupt_data {
	void (*callback) (int lch, unsigned short ch_status, void *data);
	void *data;
} intr_data[64];

/*
  Each bit field of the elements bellow indicate the corresponding EDMA channel
  availability  on arm side events
*/
static unsigned long edma_channels_arm[] = {
	0xffffffff,
	0xffffffff
};

/*
  Each bit field of the elements bellow indicate the corresponding QDMA channel
  availability  on arm side events
*/
static unsigned char qdma_channels_arm[] = {
	0xff
};

/*
   Each bit field of the elements bellow indicate corresponding PARAM entry
   availibility on arm side events
*/
static unsigned long param_entry_arm[] = {
	0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff,
	0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff,
	0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff,
	0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff
};

/*
   Each bit field of the elements bellow indicate whether a PARAM entry
   is free or in use
   1 - free
   0 - in use
*/
static unsigned long param_entry_use_status[] = {
	0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff,
	0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff,
	0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff,
	0xffffffff
};

/*
   Each bit field of the elements bellow indicate whether a intrerrupt
   is free or in use
   1 - free
   0 - in use
*/
static unsigned long dma_intr_use_status[] = {
	0xffffffff,
	0xffffffff
};

/*
    This lists the DMA channel numbers which does not have any events
    associated with it
*/
static int dma_chan_no_event[] = {
	0, 1, 12, 13, 14, 15, 25, 30, 31, 45, 46, 47, 55, 56, 57, 58, 59, 60,
	61, 62, 63, -1
};

static int channel_queue_mapping[][2] = {
/* {channel no, event queue no } */
	{0, 0}, {1, 1}, {2, 0}, {3, 1}, {4, 0}, {5, 1}, {6, 0}, {7, 1},
	{8, 0}, {9, 1}, {10, 0}, {11, 1}, {12, 0}, {13, 1}, {14, 0},
	{15, 1}, {16, 0}, {17, 1}, {18, 0}, {19, 1}, {20, 0}, {21, 1},
	{22, 0}, {23, 1}, {24, 0}, {25, 1}, {26, 0}, {27, 1}, {28, 0},
	{29, 1}, {30, 0}, {31, 1}, {32, 0}, {33, 1}, {34, 0}, {35, 1},
	{36, 0}, {37, 1}, {38, 0}, {39, 1}, {40, 0}, {41, 1}, {42, 0},
	{43, 1}, {44, 0}, {45, 1}, {46, 0}, {47, 1}, {48, 0}, {49, 1},
	{50, 0}, {51, 1}, {52, 0}, {53, 1}, {54, 0}, {55, 1}, {56, 0},
	{57, 1}, {58, 0}, {59, 1}, {60, 0}, {61, 1}, {62, 0}, {63, 1},
	{64, 0}, {65, 1}, {66, 0}, {67, 1}, {68, 0}, {69, 1}, {70, 0},
	{71, 1}, {-1, -1}
};

static int queue_tc_mapping[DAVINCI_EDMA_NUM_EVQUE + 1][2] = {
/* {event queue no, TC no} */
	{0, 0},
	{1, 1},
	{-1, -1}
};

static int queue_priority_mapping[DAVINCI_EDMA_NUM_EVQUE + 1][2] = {
	/* {event queue no, Priority} */
	{0, 0},
	{1, 1},
	{-1, -1}
};

static int qdam_to_param_mapping[8] = { 0 };

volatile edmacc_regs *ptr_edmacc_regs = NULL;

/*****************************************************************************/

static void map_dmach_queue(int ch_no, int queue_no)
{
	if (ch_no < DAVINCI_EDMA_NUM_DMACH) {
		int bit_start = (ch_no % 8) * 4;
		ptr_edmacc_regs->dmaqnum[ch_no >> 3] &= (~(0x7 << bit_start));
		ptr_edmacc_regs->dmaqnum[ch_no >> 3] |=
		    ((queue_no & 0x7) << bit_start);
	} else if (ch_no >= DAVINCI_EDMA_NUM_DMACH
		   &&
		   ch_no < (DAVINCI_EDMA_NUM_DMACH + DAVINCI_EDMA_NUM_QDMACH)) {
		int bit_start = (ch_no - DAVINCI_EDMA_NUM_DMACH) * 4;
		ptr_edmacc_regs->qdmaqnum &= (~(0x7 << bit_start));
		ptr_edmacc_regs->qdmaqnum |= ((queue_no & 0x7) << bit_start);
	}
}

/* For Davinci this Macro supports mapping only for QDMA channels and PaRam
   entry */
static void map_dmach_param(int ch_no, int param_no)
{
	if (ch_no >= DAVINCI_EDMA_NUM_DMACH
	    && ch_no < (DAVINCI_EDMA_NUM_DMACH + DAVINCI_EDMA_NUM_QDMACH)) {
		ptr_edmacc_regs->qchmap[ch_no - DAVINCI_EDMA_NUM_DMACH] &=
		    ~(PAENTRY | TRWORD);
		ptr_edmacc_regs->qchmap[ch_no - DAVINCI_EDMA_NUM_DMACH] |=
		    (((param_no & 0x1ff) << 5) | (QDMA_TRWORD << 2));
	}
}

static void map_queue_tc(int queue_no, int tc_no)
{
	int bit_start = queue_no * 4;
	ptr_edmacc_regs->quetcmap &= ~(0x7 << bit_start);
	ptr_edmacc_regs->quetcmap |= ((tc_no & 0x7) << bit_start);
}

static void assign_priority_to_queue(int queue_no, int priority)
{
	int bit_start = queue_no * 4;
	ptr_edmacc_regs->quepri &= ~(0x7 << bit_start);
	ptr_edmacc_regs->quepri |= ((priority & 0x7) << bit_start);
}

/******************************************************************************
 *
 * DMA Param entry requests: Requests for the param structure entry for the dma
 *                          channel passed
 * Arguments:
 *      lch  - logical channel for which param entry is being requested.
 *
 * Return: param number on success, or negative error number on failure
 *
 *****************************************************************************/
static int request_param(int lch, int dev_id)
{
	int i = 0, j = 0, is_break = 0;
	if (lch >= 0 && lch < DAVINCI_EDMA_NUM_DMACH) {
		/*
		   In davinci there is 1:1 mapping between edma channels
		   and param sets
		 */
		LOCK;
		/* It maintains param entry availability bitmap which
		   could be updated by several thread  same channel
		   and so requires protection
		 */
		param_entry_use_status[lch / 32] &= (~(1 << (lch % 32)));
		UNLOCK;
		return lch;
	} else {
		if (dev_id >= DAVINCI_DMA_QDMA0 &&
		    dev_id <= DAVINCI_DMA_QDMA7) {
			i = 0;
		} else if (dev_id == DAVINCI_EDMA_PARAM_ANY) {
			i = DAVINCI_EDMA_NUM_DMACH;
		}

		/* This allocation alogrithm requires complete lock because
		   availabilty of param entry is checked from structure
		   param_entry_use_status and same struct is updated back also
		   once allocated
		 */

		LOCK;
		while (i < DAVINCI_EDMA_NUM_PARAMENTRY) {
			j = 0, is_break = 1;
			if ((param_entry_arm[i / 32] & (1 << (i % 32))) &&
			    (param_entry_use_status[i / 32] & (1 << (i % 32))))
			{
				if (dev_id != DAVINCI_EDMA_PARAM_ANY) {
					while (dma_chan_no_event[j] != -1) {
						if (dma_chan_no_event[j] == i) {
							is_break = 0;
						}
						j++;
					}
					if (!is_break) {
						break;
					}
				} else {
					break;
				}
				i++;
			} else {
				i++;
			}
		}
		if (i < DAVINCI_EDMA_NUM_PARAMENTRY) {
			param_entry_use_status[i / 32] &= (~(1 << (i % 32)));
			UNLOCK;
			dev_dbg(&edma_dev.dev, "param no=%d\r\n", i);
			return i;
		} else {
			UNLOCK;
			return -1;	/* no free param */
		}
	}
}

/******************************************************************************
 *
 * Free dma param entry: Freethe param entry number passed
 * Arguments:
 *      param_no - Param entry to be released or freed out
 *
 * Return: N/A
 *
 *****************************************************************************/
static void free_param(int param_no)
{
	if (param_no >= 0 && param_no < DAVINCI_EDMA_NUM_PARAMENTRY) {
		LOCK;
		/* This is global data structure and could be accessed
		   by several thread
		 */
		param_entry_use_status[param_no / 32] |= (1 << (param_no % 32));
		UNLOCK;
	}
}

/******************************************************************************
 *
 * DMA interrupt requests: Requests for the interrupt on the free channel
 *
 * Arguments:
 *      lch - logical channel number for which the interrupt is to be requested
 *            for the free channel.
 *      callback - callback function registered for the requested interrupt
 *                 channel
 *      data - channel private data.
 *
 * Return: free interrupt channel number on success, or negative error number
 *              on failure
 *
 *****************************************************************************/
static int request_dma_interrupt(int lch,
				 void (*callback) (int lch,
						   unsigned short ch_status,
						   void *data),
				 void *data, int param_no, int requested_tcc)
{
	signed int free_intr_no = -1;
	int i = 0, j = 0, is_break = 0;
	/* edma channels */
	if (lch >= 0 && lch < DAVINCI_EDMA_NUM_DMACH) {
		/* Bitmap dma_intr_use_status is used to identify availabe tcc
		   for interrupt purpose. This could be modified by several
		   thread and same structure is checked availabilty as well as
		   updated once it's found that resource is avialable */
		LOCK;
		if (dma_intr_use_status[lch / 32] & (1 << (lch % 32))) {
			/* in use */
			dma_intr_use_status[lch / 32] &= (~(1 << (lch % 32)));
			UNLOCK;
			free_intr_no = lch;
			dev_dbg(&edma_dev.dev, "interrupt no=%d\r\n", free_intr_no);
		} else {
			UNLOCK;
			dev_dbg(&edma_dev.dev, "EDMA:Error\r\n");
			return -1;
		}
	}

	/* qdma channels */
	else if (lch >= DAVINCI_EDMA_NUM_DMACH
		 && lch < (DAVINCI_EDMA_NUM_DMACH + DAVINCI_EDMA_NUM_QDMACH)) {
		if (requested_tcc != TCC_ANY) {
			/* Complete allocation algo requires lock and as it's
			   shared resources could be invoked by several thread.
			   Structure dma_intr_use_status is used to check
			   whether resource is availabe or not and latter marked
			   as not available in the same structure */
			LOCK;
			if (dma_intr_use_status[requested_tcc / 32] &
			    (1 << (requested_tcc % 32))) {
				j = 0;
				is_break = 1;
				while (dma_chan_no_event[j] != -1) {
					if (dma_chan_no_event[j] ==
					    requested_tcc) {
						is_break = 0;
						break;
					}
					j++;
				}
				if (!is_break) {
					dma_intr_use_status[requested_tcc / 32]
					    &= (~(1 << (requested_tcc % 32)));
					free_intr_no = requested_tcc;
					dev_dbg(&edma_dev.dev,
						"interrupt no=%d\r\n",
						free_intr_no);
				} else {
					UNLOCK;
					dev_dbg(&edma_dev.dev,
						"Error - wrong tcc passed\r\n");
					return -1;
				}
				UNLOCK;
			} else {
				UNLOCK;
				dev_dbg(&edma_dev.dev,
					"Error - wrong tcc passed\r\n");
				return -1;
			}
		} else {
			i = 0;
			LOCK;
			while (i < DAVINCI_EDMA_NUM_DMACH) {
				j = 0;
				is_break = 1;
				if (dma_intr_use_status[i / 32] &
				    (1 << (i % 32))) {
					while (dma_chan_no_event[j] != -1) {
						if (dma_chan_no_event[j] == i) {
							is_break = 0;
							break;
						}
						j++;
					}
					if (!is_break) {
						dma_intr_use_status[i / 32] &=
						    (~(1 << (i % 32)));
						free_intr_no = i;

						dev_dbg(&edma_dev.dev,
							"interrupt no=%d\r\n",
							free_intr_no);
						break;
					}
					i++;
				} else {
					i++;
				}
			}
			UNLOCK;
		}
	} else {
		dev_dbg(&edma_dev.dev, "ERROR lch = %d\r\n", lch);
	}
	if (is_break) {
		dev_dbg(&edma_dev.dev, "While allocating EDMA channel for QDMA");
	}
	if (lch >= DAVINCI_EDMA_NUM_DMACH && lch <
	    (DAVINCI_EDMA_NUM_DMACH + DAVINCI_EDMA_NUM_QDMACH)) {
		if (free_intr_no < 32) {
			ptr_edmacc_regs->dra[0].drae =
			    ptr_edmacc_regs->dra[0].drae | (1 << free_intr_no);
		} else {
			ptr_edmacc_regs->dra[0].draeh =
			    ptr_edmacc_regs->dra[0].
			    draeh | (1 << (free_intr_no - 32));
		}
	}
	if (free_intr_no >= 0 && free_intr_no < 64) {
		(free_intr_no < 32) ?
		    (ptr_edmacc_regs->shadow[0].iesr |= (1UL << free_intr_no))
		    : (ptr_edmacc_regs->shadow[0].iesrh |=
		       (1UL << (free_intr_no - 32)));
		intr_data[free_intr_no].callback = callback;
		intr_data[free_intr_no].data = data;
	}
	return free_intr_no;
}

/******************************************************************************
 *
 * Free the dma interrupt: Releases the dma interrupt on the channel
 *
 * Arguments:
 *      intr_no - interrupt number on the channel to be released or freed out
 *
 * Return: N/A
 *
 *****************************************************************************/
static void free_dma_interrupt(int intr_no)
{
	if (intr_no >= 0 && intr_no < 64) {
		(intr_no < 32) ? (ptr_edmacc_regs->shadow[0].icr |=
				  (1UL << (intr_no))) : (ptr_edmacc_regs->
							 shadow[0].icrh |=
							 (1UL <<
							  (intr_no - 32)));
		LOCK;
		/* Global structure and could be modified by several task */
		dma_intr_use_status[intr_no / 32] |= (1 << (intr_no % 32));
		UNLOCK;
		intr_data[intr_no].callback = NULL;
		intr_data[intr_no].data = NULL;

	}
}

/******************************************************************************
 *
 * DMA interrupt handler
 *
 *****************************************************************************/
static void dma_irq_handler(void)
{
	int i;
	unsigned int cnt;
	cnt = 0;
	if ((ptr_edmacc_regs->shadow[0].ipr == 0)
	    && (ptr_edmacc_regs->shadow[0].iprh == 0))
		return;
	while (1) {
		if (ptr_edmacc_regs->shadow[0].ipr) {
			dev_dbg(&edma_dev.dev, "IPR =%d\r\n",
				ptr_edmacc_regs->shadow[0].ipr);
			for (i = 0; i < 32; i++) {
				if (ptr_edmacc_regs->shadow[0].ipr & (1 << i)) {
					/* Clear the corresponding IPR bits */
					ptr_edmacc_regs->shadow[0].icr |=
					    (1 << i);
					if (intr_data[i].callback) {
						intr_data[i].callback(i,
								      DMA_COMPLETE,
								      intr_data
								      [i].data);

					}
				}
			}
		} else if (ptr_edmacc_regs->shadow[0].iprh) {
			dev_dbg(&edma_dev.dev, "IPRH =%d\r\n",
				ptr_edmacc_regs->shadow[0].iprh);
			for (i = 0; i < 32; i++) {
				if (ptr_edmacc_regs->shadow[0].iprh & (1 << i)) {
					/* Clear the corresponding IPR bits */
					ptr_edmacc_regs->shadow[0].icrh |=
					    (1 << i);
					if (intr_data[32 + i].callback) {
						intr_data[32 + i].callback(32 +
									   i,
									   DMA_COMPLETE,
									   intr_data
									   [32 +
									    i].
									   data);
					}
				}
			}
		}
		if ((ptr_edmacc_regs->shadow[0].ipr == 0)
		    && (ptr_edmacc_regs->shadow[0].iprh == 0)) {
			break;
		}
		cnt++;
		if (cnt > 10) {
			break;
		}
	}
	ptr_edmacc_regs->shadow[0].ieval = 0x1;
}

/******************************************************************************
 *
 * DMA error interrupt handler
 *
 *****************************************************************************/
static void dma_ccerr_handler(void)
{
	int i;
	unsigned int cnt;
	cnt = 0;
	if ((ptr_edmacc_regs->emr == 0) && (ptr_edmacc_regs->emr == 0) &&
	    (ptr_edmacc_regs->qemr == 0) && (ptr_edmacc_regs->ccerr == 0))
		return;
	while (1) {
		if (ptr_edmacc_regs->emr) {
			dev_dbg(&edma_dev.dev, "EMR =%d\r\n", ptr_edmacc_regs->emr);
			for (i = 0; i < 32; i++) {
				if (ptr_edmacc_regs->emr & (1 << i)) {
					/* Clear the corresponding EMR bits */
					ptr_edmacc_regs->emcr |= (1 << i);
					/* Clear any SER */
					ptr_edmacc_regs->shadow[0].secr |=
					    (1 << i);
					if (intr_data[i].callback) {
						intr_data[i].callback(i,
								      DMA_CC_ERROR,
								      intr_data
								      [i].data);
					}
				}
			}
		} else if (ptr_edmacc_regs->emrh) {
			dev_dbg(&edma_dev.dev, "EMRH =%d\r\n",
				ptr_edmacc_regs->emrh);
			for (i = 0; i < 32; i++) {
				if (ptr_edmacc_regs->emrh & (1 << i)) {
					/* Clear the corresponding IPR bits */
					ptr_edmacc_regs->emcrh |= (1 << i);
					/* Clear any SER */
					ptr_edmacc_regs->shadow[0].secrh |=
					    (1 << i);
					if (intr_data[i].callback) {
						intr_data[i].callback(i,
								      DMA_CC_ERROR,
								      intr_data
								      [i].data);
					}
				}
			}
		} else if (ptr_edmacc_regs->qemr) {
			dev_dbg(&edma_dev.dev, "QEMR =%d\r\n",
				ptr_edmacc_regs->qemr);
			for (i = 0; i < 8; i++) {
				if (ptr_edmacc_regs->qemr & (1 << i)) {
					/* Clear the corresponding IPR bits */
					ptr_edmacc_regs->qemcr |= (1 << i);
					ptr_edmacc_regs->shadow[0].qsecr |=
					    (1 << i);
				}
			}
		} else if (ptr_edmacc_regs->ccerr) {
			dev_dbg(&edma_dev.dev, "CCERR =%d\r\n",
				ptr_edmacc_regs->ccerr);
			for (i = 0; i < 8; i++) {
				if (ptr_edmacc_regs->ccerr & (1 << i)) {
					/* Clear the corresponding IPR bits */
					ptr_edmacc_regs->ccerrclr |= (1 << i);
				}
			}
		}
		if ((ptr_edmacc_regs->emr == 0)
		    && (ptr_edmacc_regs->emrh == 0)
		    && (ptr_edmacc_regs->qemr == 0)
		    && (ptr_edmacc_regs->ccerr == 0)) {
			break;
		}
		cnt++;
		if (cnt > 10) {
			break;
		}
	}
	ptr_edmacc_regs->eeval = 0x1;
}

/******************************************************************************
 *
 * DMA error interrupt handler
 *
 *****************************************************************************/
static void dma_tc1err_handler(void)
{

}

/******************************************************************************
 *
 * DMA error interrupt handler
 *
 *****************************************************************************/
static void dma_tc2err_handler(void)
{

}

/******************************************************************************
 *
 * DMA initialisation on davinci
 *
 *****************************************************************************/
int __init arch_dma_init(void)
{
	int i;
	edma_driver.name = "edma";
	edma_dev.name = "dma";
	edma_dev.id = -1;
	edma_dev.dev.driver = &edma_driver;

	ptr_edmacc_regs = get_edma_base();
	dev_dbg(&edma_dev.dev, "DMA REG BASE ADDR=%x\n", ptr_edmacc_regs);
	memset(dma_chan, 0x00, sizeof(dma_chan));
	memset((void *)&(ptr_edmacc_regs->paramentry[0]), 0x00,
	       sizeof(ptr_edmacc_regs->paramentry));
	i = 0;
	/* Channel to queue mapping */
	while (channel_queue_mapping[i][0] != -1) {
		map_dmach_queue(channel_queue_mapping[i][0],
				channel_queue_mapping[i][1]);
		i++;
	}
	i = 0;
	/* Event queue to TC mapping */
	while (queue_tc_mapping[i][0] != -1) {
		map_queue_tc(queue_tc_mapping[i][0], queue_tc_mapping[i][1]);
		i++;
	}
	i = 0;
	/* Event queue priority mapping */
	while (queue_priority_mapping[i][0] != -1) {
		assign_priority_to_queue(queue_priority_mapping[i][0],
					 queue_priority_mapping[i][1]);
		i++;
	}
	for (i = 0; i < DAVINCI_EDMA_NUM_REGIONS; i++) {
		ptr_edmacc_regs->dra[i].drae = 0x0;
		ptr_edmacc_regs->dra[i].draeh = 0x0;
		ptr_edmacc_regs->qrae[i] = 0x0;
	}
	LOCK_INIT;
	return 0;
}

/******************************************************************************
 *
 * DMA channel requests: Requests for the dma device passed if it is free
 *
 * Arguments:
 *      dev_id     - request for the param entry device id
 *      dev_name   - device name
 *      callback   - pointer to the channel callback.
 *      Arguments:
 *          lch  - channel no, which is the IPR bit position,
 *		   indicating from which channel the interrupt arised.
 *          data - channel private data, which is received as one of the
 *		   arguments in davinci_request_dma.
 *      data - private data for the channel to be requested, which is used to
 *                   pass as a parameter in the callback function
 *		     in irq handler.
 *      lch - contains the device id allocated
 *  tcc        - Transfer Completion Code, used to set the IPR register bit
 *                   after transfer completion on that channel.
 *  eventq_no  - Event Queue no to which the channel will be associated with
 *               (valied only if you are requesting for a DMA MasterChannel)
 *               Values : 0 to 7
 *                       -1 for Default queue
 * INPUT:   dev_id
 * OUTPUT:  *dma_ch_out
 *
 * Return: zero on success, or corresponding error no on failure
 *
 *****************************************************************************/
int davinci_request_dma(int dev_id, const char *dev_name,
			void (*callback) (int lch, unsigned short ch_status,
					  void *data),
			void *data, int *lch,
			int *tcc, enum dma_event_q eventq_no)
{

	int ret_val = 0, i = 0;
	static int req_flag = 0;
	int temp_ch = 0;
	/* checking the ARM side events */
	if (dev_id >= 0 && (dev_id < DAVINCI_EDMA_NUM_DMACH)) {
		if (!(edma_channels_arm[dev_id / 32] & (0x1 << (dev_id % 32)))) {
			dev_dbg(&edma_dev.dev,
				"dev_id = %d not supported on ARM side\r\n",
				dev_id);
			return -EINVAL;
		}
	} else if (dev_id >= DAVINCI_EDMA_NUM_DMACH
		   && dev_id <=
		   (DAVINCI_EDMA_NUM_DMACH + DAVINCI_EDMA_NUM_QDMACH)) {
		if (!(qdma_channels_arm[0] &
		      (0x1 << (dev_id - DAVINCI_EDMA_NUM_DMACH)))) {

			dev_dbg(&edma_dev.dev,
				"dev_id = %d not supported on ARM side\r\n",
				dev_id);
			return -EINVAL;
		}
	}

	if ((dev_id != DAVINCI_DMA_CHANNEL_ANY)
	    && (dev_id != DAVINCI_EDMA_PARAM_ANY)) {
		if (dev_id >= DAVINCI_EDMA_NUM_DMACH
		    &&
		    dev_id < (DAVINCI_EDMA_NUM_DMACH + DAVINCI_EDMA_NUM_QDMACH)
		    ) {
			ptr_edmacc_regs->qrae[0] =
			    ptr_edmacc_regs->qrae[0] |
			    (1 << (dev_id - DAVINCI_EDMA_NUM_DMACH));
		} else {
			if (dev_id < 32) {
				ptr_edmacc_regs->dra[0].drae =
				    ptr_edmacc_regs->dra[0].drae |
				    (1 << dev_id);
			} else {
				ptr_edmacc_regs->dra[0].draeh =
				    ptr_edmacc_regs->dra[0].draeh |
				    (1 << (dev_id - 32));
			}
		}
	}

	if (!req_flag) {
		if (register_dma_interrupts
		    (dma_irq_handler, dma_ccerr_handler,
		     dma_tc1err_handler, dma_tc2err_handler)) {
			dev_dbg(&edma_dev.dev,
				"register_dma_interrupts failed\r\n");
			return -EINVAL;
		} else
			req_flag = 1;
	}

	if (dev_id >= 0 && dev_id < (DAVINCI_EDMA_NUM_DMACH)) {
		/* The 64 Channels are mapped to the first 64 PARAM entries */
		if (!dma_chan[dev_id].in_use) {
			*lch = dev_id;
			dma_chan[*lch].param_no = request_param(*lch, dev_id);
			if (dma_chan[*lch].param_no == -1) {
				return -EINVAL;
			} else
				dev_dbg(&edma_dev.dev, "param_no=%d\r\n",
					dma_chan[*lch].param_no);
			if (callback) {
				dma_chan[*lch].tcc =
				    request_dma_interrupt(*lch, callback, data,
							  dma_chan[*lch].
							  param_no, *tcc);
				if (dma_chan[*lch].tcc == -1) {
					return -EINVAL;
				} else {
					*tcc = dma_chan[*lch].tcc;
					dev_dbg(&edma_dev.dev, "tcc_no=%d\r\n",
						dma_chan[*lch].tcc);
				}
			} else
				dma_chan[*lch].tcc = -1;

			map_dmach_queue(dev_id, eventq_no);
			ret_val = 0;
		} else
			ret_val = -EINVAL;
	}

	else if (dev_id >= DAVINCI_EDMA_NUM_DMACH && dev_id <
		 (DAVINCI_EDMA_NUM_DMACH + DAVINCI_EDMA_NUM_QDMACH)) {
		if ((qdam_to_param_mapping[dev_id - DAVINCI_EDMA_NUM_DMACH] !=
		     -1)
		    &&
		    (dma_chan
		     [qdam_to_param_mapping[dev_id - DAVINCI_EDMA_NUM_DMACH]].
		     in_use)
		    ) {
			ret_val = -EINVAL;
		} else {
			*lch = dev_id;
			dma_chan[*lch].param_no = request_param(*lch, dev_id);
			if (dma_chan[*lch].param_no == -1) {
				dev_dbg(&edma_dev.dev, "request_param failed\r\n");
				return -EINVAL;
			} else {
				dev_dbg(&edma_dev.dev, "param_no=%d\r\n",
					dma_chan[*lch].param_no);
				map_dmach_param(*lch, dma_chan[*lch].param_no);
			}
			if (callback) {
				dma_chan[*lch].tcc =
				    request_dma_interrupt(*lch, callback, data,
							  dma_chan[*lch].
							  param_no, *tcc);
				if (dma_chan[*lch].tcc == -1) {
					return -EINVAL;
				} else {
					*tcc = dma_chan[*lch].tcc;
					dev_dbg(&edma_dev.dev, "tcc_no=%d\r\n",
						dma_chan[*lch].tcc);
				}
			} else
				dma_chan[*lch].tcc = -1;
			map_dmach_queue(dev_id, eventq_no);
			ret_val = 0;
		}
	} else if (dev_id == DAVINCI_DMA_CHANNEL_ANY) {
		i = 0;
		ret_val = 0;
		while (dma_chan_no_event[i] != -1) {
			if (!dma_chan[dma_chan_no_event[i]].in_use) {
				*lch = dma_chan_no_event[i];
				dma_chan[*lch].param_no =
				    request_param(*lch, dev_id);
				if (dma_chan[*lch].param_no == -1) {
					return -EINVAL;
				}
				dev_dbg(&edma_dev.dev, "param_no=%d\r\n",
					dma_chan[*lch].param_no);
				if (dma_chan[*lch].param_no >=
				    DAVINCI_EDMA_NUM_DMACH
				    &&
				    dma_chan[*lch].param_no <
				    (DAVINCI_EDMA_NUM_DMACH +
				     DAVINCI_EDMA_NUM_QDMACH)
				    ) {

					ptr_edmacc_regs->qrae[0] =
					    ptr_edmacc_regs->qrae[0] |
					    (1 << (dma_chan[*lch].param_no -
						   DAVINCI_EDMA_NUM_DMACH));

				} else {
					if (dma_chan[*lch].param_no < 32) {
						ptr_edmacc_regs->dra[0].drae =
						    ptr_edmacc_regs->dra[0].drae
						    |
						    (1 << dma_chan[*lch].
						     param_no);
					} else {
						ptr_edmacc_regs->dra[0].draeh =
						    ptr_edmacc_regs->dra[0].
						    draeh | (1 <<
							     (dma_chan[*lch].
							      param_no - 32));
					}
				}
				if (callback) {
					dma_chan[*lch].tcc =
					    request_dma_interrupt(*lch,
								  callback,
								  data,
								  dma_chan
								  [*lch].
								  param_no,
								  *tcc);
					if (dma_chan[*lch].tcc == -1) {
						return -EINVAL;
					} else {
						*tcc = dma_chan[*lch].tcc;
					}
				} else {
					dma_chan[*lch].tcc = -1;
				}
				map_dmach_queue(dev_id, eventq_no);
				ret_val = 0;
				break;
			}
			i++;
		}
	}

	else if (dev_id == DAVINCI_EDMA_PARAM_ANY) {
		ret_val = 0;
		for (i = (DAVINCI_EDMA_NUM_DMACH + DAVINCI_EDMA_NUM_QDMACH);
		     i < DAVINCI_EDMA_NUM_PARAMENTRY; i++) {
			if (!dma_chan[i].in_use) {
				dev_dbg(&edma_dev.dev, "any link = %d\r\n", i);
				*lch = i;
				dma_chan[*lch].param_no =
				    request_param(*lch, dev_id);
				if (dma_chan[*lch].param_no == -1) {
					dev_dbg(&edma_dev.dev,
						"request_param failed\r\n");
					return -EINVAL;
				} else {
					dev_dbg(&edma_dev.dev, "param_no=%d\r\n",
						dma_chan[*lch].param_no);
				}
				if (*tcc != -1)
					dma_chan[*lch].tcc = *tcc;
				else
					dma_chan[*lch].tcc = -1;
				ret_val = 0;
				break;
			}
		}
	} else {
		ret_val = -EINVAL;
	}
	if (!ret_val) {
		if (dev_id >= DAVINCI_EDMA_NUM_DMACH && dev_id <
		    (DAVINCI_EDMA_NUM_DMACH + DAVINCI_EDMA_NUM_QDMACH)) {
			/* Master Channel */
			qdam_to_param_mapping[dev_id -
					      DAVINCI_EDMA_NUM_DMACH] =
			    dma_chan[*lch].param_no;
			LOCK;
			/* It's used global data structure and used to find out
			   whether channel is available or not */
			dma_chan[qdam_to_param_mapping
				 [dev_id - DAVINCI_EDMA_NUM_DMACH]].in_use = 1;
			UNLOCK;
			dma_chan[qdam_to_param_mapping
				 [dev_id - DAVINCI_EDMA_NUM_DMACH]].dev_id =
			    *lch;
			dma_chan[qdam_to_param_mapping
				 [dev_id - DAVINCI_EDMA_NUM_DMACH]].tcc =
			    dma_chan[*lch].tcc;
			temp_ch =
			    qdam_to_param_mapping[dev_id -
						  DAVINCI_EDMA_NUM_DMACH];
			dma_chan[temp_ch].param_no = dma_chan[*lch].param_no;
			if (dma_chan[*lch].tcc != -1) {
				ptr_edmacc_regs->paramentry[dma_chan[temp_ch].
							    param_no].opt &=
				    (~TCC);
				ptr_edmacc_regs->paramentry[dma_chan[temp_ch].
							    param_no].opt |=
				    ((0x3f & dma_chan[*lch].tcc) << 12);
				/* set TCINTEN bit in PARAM entry */
				ptr_edmacc_regs->
				    paramentry[dma_chan[temp_ch].param_no].
				    opt |= TCINTEN;
			} else {
				ptr_edmacc_regs->paramentry[dma_chan[temp_ch].
							    param_no].opt &=
				    ~TCINTEN;
			}
			/* assign the link field to no link. i.e 0xffff */
			ptr_edmacc_regs->paramentry[dma_chan[temp_ch].
						    param_no].
			    link_bcntrld |= 0xffff;
		} else {
			/* Slave Channel */
			LOCK;
			/* Global structure to identify whether resoures is
			   available or not */
			dma_chan[*lch].in_use = 1;
			UNLOCK;
			dma_chan[*lch].dev_id = *lch;
			if (dma_chan[*lch].tcc != -1) {
				ptr_edmacc_regs->paramentry[dma_chan[*lch].
							    param_no].opt &=
				    (~TCC);
				ptr_edmacc_regs->paramentry[dma_chan[*lch].
							    param_no].opt |=
				    ((0x3f & dma_chan[*lch].tcc) << 12);
				/* set TCINTEN bit in PARAM entry */
				ptr_edmacc_regs->paramentry[dma_chan[*lch].
							    param_no].opt |=
				    TCINTEN;
			} else {
				ptr_edmacc_regs->paramentry[dma_chan[*lch].
							    param_no].opt &=
				    ~TCINTEN;
			}
			/* assign the link field to no link. i.e 0xffff */
			ptr_edmacc_regs->paramentry[dma_chan[*lch].
						    param_no].
			    link_bcntrld |= 0xffff;
		}
	}
	return ret_val;
}

/******************************************************************************
 *
 * DMA channel free: Free dma channle
 * Arguments:
 *      dev_id     - request for the param entry device id
 *
 * Return: zero on success, or corresponding error no on failure
 *
 *****************************************************************************/
void davinci_free_dma(int lch)
{
	int temp_ch = 0;
	if (lch >= DAVINCI_EDMA_NUM_DMACH && lch <
	    (DAVINCI_EDMA_NUM_DMACH + DAVINCI_EDMA_NUM_QDMACH)) {
		temp_ch = qdam_to_param_mapping[lch - DAVINCI_EDMA_NUM_DMACH];
		lch = temp_ch;
	}
	LOCK;
	dma_chan[lch].in_use = 0;
	UNLOCK;
	free_param(dma_chan[lch].param_no);

	if (lch >= 0
	    && lch < (DAVINCI_EDMA_NUM_DMACH + DAVINCI_EDMA_NUM_QDMACH)) {
		free_dma_interrupt(dma_chan[lch].tcc);
	}
}

/******************************************************************************
 *
 * DMA source parameters setup
 * ARGUMENTS:
 *      lch         - channel for which the source parameters to be configured
 *      src_port    - Source port address
 *      addressMode - indicates wether addressing mode is fifo.
 *
 *****************************************************************************/
void davinci_set_dma_src_params(int lch, unsigned long src_port,
				enum address_mode mode, enum fifo_width width)
{
	int temp_ch = 0;
	if (lch >= DAVINCI_EDMA_NUM_DMACH && lch <
	    (DAVINCI_EDMA_NUM_DMACH + DAVINCI_EDMA_NUM_QDMACH)) {
		temp_ch = qdam_to_param_mapping[lch - DAVINCI_EDMA_NUM_DMACH];
		lch = temp_ch;
	}
	if (lch >= 0 && lch < DAVINCI_EDMA_NUM_PARAMENTRY) {
		/* set the source port address
		   in source register of param structure */
		ptr_edmacc_regs->paramentry[dma_chan[lch].param_no].src =
		    src_port;
		/* set the fifo addressing mode */
		if (mode) {	/* reset SAM and FWID */
			ptr_edmacc_regs->paramentry[dma_chan[lch].param_no].opt
			    &= (~(SAM | EDMA_FWID));
			/* set SAM and program FWID */
			ptr_edmacc_regs->paramentry[dma_chan[lch].param_no].opt
			    |= (mode | ((width & 0x7) << 8));
		}
	}
}

/******************************************************************************
 *
 * DMA destination parameters setup
 * ARGUMENTS:
 *    lch - channel or param device for destination parameters to be configured
 *    dest_port    - Destination port address
 *    addressMode  - indicates wether addressing mode is fifo.
 *
 *****************************************************************************/
void davinci_set_dma_dest_params(int lch, unsigned long dest_port,
				 enum address_mode mode, enum fifo_width width)
{
	int temp_ch = 0;
	if (lch >= DAVINCI_EDMA_NUM_DMACH && lch <
	    (DAVINCI_EDMA_NUM_DMACH + DAVINCI_EDMA_NUM_QDMACH)) {
		temp_ch = qdam_to_param_mapping[lch - DAVINCI_EDMA_NUM_DMACH];
		lch = temp_ch;
	}
	if (lch >= 0 && lch < DAVINCI_EDMA_NUM_PARAMENTRY) {
		/* set the destination port address
		   in dest register of param structure */
		ptr_edmacc_regs->paramentry[dma_chan[lch].param_no].dst =
		    dest_port;
		/* set the fifo addressing mode */
		if (mode) {	/* reset DAM and FWID */

			ptr_edmacc_regs->paramentry[dma_chan[lch].param_no].opt
			    &= (~(DAM | EDMA_FWID));
			/* set DAM and program FWID */
			ptr_edmacc_regs->paramentry[dma_chan[lch].param_no].opt
			    |= ((mode << 1) | ((width & 0x7) << 8));
		}
	}
}

/******************************************************************************
 *
 * DMA source index setup
 * ARGUMENTS:
 *      lch     - channel or param device for configuration of source index
 *      srcbidx - source B-register index
 *      srccidx - source C-register index
 *
 *****************************************************************************/
void davinci_set_dma_src_index(int lch, short src_bidx, short src_cidx)
{
	int temp_ch = 0;
	if (lch >= DAVINCI_EDMA_NUM_DMACH && lch <
	    (DAVINCI_EDMA_NUM_DMACH + DAVINCI_EDMA_NUM_QDMACH)) {
		temp_ch = qdam_to_param_mapping[lch - DAVINCI_EDMA_NUM_DMACH];
		lch = temp_ch;
	}

	if (lch >= 0 && lch < DAVINCI_EDMA_NUM_PARAMENTRY) {
		ptr_edmacc_regs->paramentry[dma_chan[lch].param_no].src_dst_bidx
		    &= 0xffff0000;
		ptr_edmacc_regs->paramentry[dma_chan[lch].param_no].src_dst_bidx
		    |= src_bidx;
		ptr_edmacc_regs->paramentry[dma_chan[lch].param_no].src_dst_cidx
		    &= 0xffff0000;
		ptr_edmacc_regs->paramentry[dma_chan[lch].param_no].src_dst_cidx
		    |= src_cidx;
	}
}

/******************************************************************************
 *
 * DMA destination index setup
 * ARGUMENTS:
 *      lch    - channel or param device for configuration of destination index
 *      srcbidx - dest B-register index
 *      srccidx - dest C-register index
 *
 *****************************************************************************/
void davinci_set_dma_dest_index(int lch, short dest_bidx, short dest_cidx)
{
	int temp_ch = 0;
	if (lch >= DAVINCI_EDMA_NUM_DMACH && lch <
	    (DAVINCI_EDMA_NUM_DMACH + DAVINCI_EDMA_NUM_QDMACH)) {
		temp_ch = qdam_to_param_mapping[lch - DAVINCI_EDMA_NUM_DMACH];
		lch = temp_ch;
	}
	if (lch >= 0 && lch < DAVINCI_EDMA_NUM_PARAMENTRY) {
		ptr_edmacc_regs->paramentry[dma_chan[lch].param_no].src_dst_bidx
		    &= 0x0000ffff;
		ptr_edmacc_regs->paramentry[dma_chan[lch].param_no].src_dst_bidx
		    |= ((unsigned long)dest_bidx << 16);
		ptr_edmacc_regs->paramentry[dma_chan[lch].param_no].src_dst_cidx
		    &= 0x0000ffff;
		ptr_edmacc_regs->paramentry[dma_chan[lch].param_no].src_dst_cidx
		    |= ((unsigned long)dest_cidx << 16);
	}
}

/******************************************************************************
 *
 * DMA transfer parameters setup
 * ARGUMENTS:
 *      lch  - channel or param device for configuration of aCount, bCount and
 *         cCount regs.
 *      acnt - acnt register value to be configured
 *      bcnt - bcnt register value to be configured
 *      ccnt - ccnt register value to be configured
 *
 *****************************************************************************/
void davinci_set_dma_transfer_params(int lch, unsigned short acnt,
				     unsigned short bcnt, unsigned short ccnt,
				     unsigned short bcntrld,
				     enum sync_dimension sync_mode)
{
	int temp_ch = 0;
	if (lch >= DAVINCI_EDMA_NUM_DMACH && lch <
	    (DAVINCI_EDMA_NUM_DMACH + DAVINCI_EDMA_NUM_QDMACH)) {
		temp_ch = qdam_to_param_mapping[lch - DAVINCI_EDMA_NUM_DMACH];
		lch = temp_ch;
	}
	if (lch >= 0 && lch < DAVINCI_EDMA_NUM_PARAMENTRY) {
		ptr_edmacc_regs->paramentry[dma_chan[lch].param_no].link_bcntrld
		    &= 0x0000ffff;
		ptr_edmacc_regs->paramentry[dma_chan[lch].param_no].link_bcntrld
		    |= (bcntrld << 16);
		if (sync_mode == ASYNC)
			ptr_edmacc_regs->paramentry[dma_chan[lch].param_no].opt
			    &= (~SYNCDIM);
		else
			ptr_edmacc_regs->paramentry[dma_chan[lch].param_no].opt
			    |= SYNCDIM;
		/* Set the acount, bcount, ccount registers */
		ptr_edmacc_regs->paramentry[dma_chan[lch].param_no].a_b_cnt =
		    (bcnt << 16) | acnt;
		ptr_edmacc_regs->paramentry[dma_chan[lch].param_no].ccnt = ccnt;
	}
}

/******************************************************************************
 *
 * davinci_set_dma_params -
 * ARGUMENTS:
 *      lch - logical channel number
 *
 *****************************************************************************/
void davinci_set_dma_params(int lch, edmacc_paramentry_regs * temp)
{
	int temp_ch = 0;
	if (lch >= DAVINCI_EDMA_NUM_DMACH && lch <
	    (DAVINCI_EDMA_NUM_DMACH + DAVINCI_EDMA_NUM_QDMACH)) {
		temp_ch = qdam_to_param_mapping[lch - DAVINCI_EDMA_NUM_DMACH];
		lch = temp_ch;
	}
	if (lch >= 0 && lch < DAVINCI_EDMA_NUM_PARAMENTRY) {
		memcpy((void *)
		       &(ptr_edmacc_regs->
			 paramentry[dma_chan[lch].param_no].opt),
		       (void *)temp, sizeof(edmacc_paramentry_regs));
	}
}

/******************************************************************************
 *
 * davinci_get_dma_params -
 * ARGUMENTS:
 *      lch - logical channel number
 *
 *****************************************************************************/
void davinci_get_dma_params(int lch, edmacc_paramentry_regs * temp)
{
	int temp_ch = 0;
	if (lch >= DAVINCI_EDMA_NUM_DMACH && lch <
	    (DAVINCI_EDMA_NUM_DMACH + DAVINCI_EDMA_NUM_QDMACH)) {
		temp_ch = qdam_to_param_mapping[lch - DAVINCI_EDMA_NUM_DMACH];
		lch = temp_ch;
	}
	if (lch >= 0 && lch < DAVINCI_EDMA_NUM_PARAMENTRY) {
		memcpy((void *)temp,
		       (void *)&(ptr_edmacc_regs->
				 paramentry[dma_chan[lch].param_no].opt),
		       sizeof(edmacc_paramentry_regs));
	}
}

/******************************************************************************
 *
 * DMA Strat - Starts the dma on the channel passed
 * ARGUMENTS:
 *      lch - logical channel number
 *
 *****************************************************************************/
int davinci_start_dma(int lch)
{
	int ret_val;
	if (lch >= 0 && (lch < DAVINCI_EDMA_NUM_DMACH)) {
		int i = 0;
		int flag = 0;
		/* If the dma start request is for the unused events */
		while (dma_chan_no_event[i] != -1) {
			if (dma_chan_no_event[i] == lch) {
				/* EDMA channels without event association */
				dev_dbg(&edma_dev.dev, "ESR=%x\r\n",
					ptr_edmacc_regs->shadow[0].esr);

				(lch < 32) ?
				    (ptr_edmacc_regs->shadow[0].esr |=
				     (1UL << lch)) : (ptr_edmacc_regs->
						      shadow[0].esrh |=
						      (1UL << (lch - 32)));
				flag = 1;
				ret_val = 0;
				break;
			}
			i++;
		}
		if (!flag) {
			/* EDMA channel with event association */
			dev_dbg(&edma_dev.dev, "ER=%d\r\n",
				ptr_edmacc_regs->shadow[0].er);
			/* Clear any pedning error */
			(lch < 32) ?
			    (ptr_edmacc_regs->emcr |=
			     (1UL << lch)) :
			    (ptr_edmacc_regs->emcrh |= (1UL << (lch - 32)));
			/* Clear any SER */
			(lch < 32) ?
			    (ptr_edmacc_regs->shadow[0].secr |=
			     (1UL << lch)) :
			    (ptr_edmacc_regs->shadow[0].secrh |=
			     (1UL << (lch - 32)));

			(lch < 32) ?
			    (ptr_edmacc_regs->shadow[0].eesr |=
			     (1UL << lch)) :
			    (ptr_edmacc_regs->shadow[0].eesrh |=
			     (1UL << (lch - 32)));

			dev_dbg(&edma_dev.dev, "EER=%d\r\n",
				ptr_edmacc_regs->shadow[0].eer);
			ret_val = 0;
		}
	} else if ((lch >= DAVINCI_EDMA_NUM_DMACH)
		   && (lch <
		       (DAVINCI_EDMA_NUM_DMACH + DAVINCI_EDMA_NUM_QDMACH))) {
		ptr_edmacc_regs->shadow[0].qeesr |=
		    (1 << (lch - DAVINCI_EDMA_NUM_DMACH));
		ret_val = 0;
	} else {		/* for slaveChannels */
		ret_val = EINVAL;
	}
	return ret_val;
}

/******************************************************************************
 *
 * DMA Stop - Stops the dma on the channel passed
 * ARGUMENTS:
 *      lch - logical channel number
 *
 *****************************************************************************/
void davinci_stop_dma(int lch)
{
	if (lch < DAVINCI_EDMA_NUM_DMACH) {
		int flag = 0;
		int i = 0;
		/* If the dma stop request is for the unused events */
		while (dma_chan_no_event[i] != -1) {
			if (dma_chan_no_event[i] == lch) {
				/* EDMA channels without event association */
				/* if the requested channel is one of the
				   unused channels then reset the coresponding
				   bit of ESR-Event Set Register */
				flag = 1;
				break;
			}
			i++;
		}
		if (!flag) {
			/* EDMA channel with event association */
			(lch < 32) ? (ptr_edmacc_regs->shadow[0].eecr |=
				      (1UL << lch)) :
			    (ptr_edmacc_regs->shadow[0].eecrh |=
			     (1UL << (lch - 32)));
			if (lch < 32) {
				if (ptr_edmacc_regs->shadow[0].er & (1 << lch)) {
					dev_dbg(&edma_dev.dev, "ER=%x\n",
						ptr_edmacc_regs->shadow[0].er);
					ptr_edmacc_regs->shadow[0].ecr |=
					    (1 << lch);
				}
			} else {
				if (ptr_edmacc_regs->shadow[0].erh
				    & (1 << (lch - 32))) {
					dev_dbg(&edma_dev.dev, "ERH=%x\n",
						ptr_edmacc_regs->shadow[0].erh);
					ptr_edmacc_regs->shadow[0].ecrh |=
					    (1 << (lch - 32));
				}
			}
			if (lch < 32) {
				if (ptr_edmacc_regs->shadow[0].ser & (1 << lch)) {
					dev_dbg(&edma_dev.dev, "SER=%x\n",
						ptr_edmacc_regs->shadow[0].ser);
					ptr_edmacc_regs->shadow[0].secr |=
					    (1 << lch);
				} else {
				}
			} else {
				if (ptr_edmacc_regs->
				    shadow[0].serh & (1 << (lch - 32))) {
					dev_dbg(&edma_dev.dev, "SERH=%x\n",
						ptr_edmacc_regs->shadow[0].
						serh);
					ptr_edmacc_regs->shadow[0].secrh |=
					    (1 << (lch - 32));
				}
			}
			if (lch < 32) {
				if (ptr_edmacc_regs->emr & (1 << lch)) {
					dev_dbg(&edma_dev.dev, "EMR=%x\n",
						ptr_edmacc_regs->emr);
					ptr_edmacc_regs->emcr |= (1 << lch);
				}
			} else {
				if (ptr_edmacc_regs->emrh & (1 << (lch - 32))) {
					dev_dbg(&edma_dev.dev, "EMRH=%x\n",
						ptr_edmacc_regs->emrh);
					ptr_edmacc_regs->emcrh |=
					    (1 << (lch - 32));
				}
			}
			dev_dbg(&edma_dev.dev, "EER=%d\r\n",
				ptr_edmacc_regs->shadow[0].eer);
			/* if the requested channel is one of the event channels
			   then just set the link field of the corresponding
			   param entry to 0xffff */
		}
	} else if ((lch >= DAVINCI_EDMA_NUM_DMACH)
		   &&
		   (lch < (DAVINCI_EDMA_NUM_DMACH + DAVINCI_EDMA_NUM_QDMACH))) {
		/* for QDMA channels */
		ptr_edmacc_regs->qeecr |= (1 << (lch - DAVINCI_EDMA_NUM_DMACH));
		dev_dbg(&edma_dev.dev, "QER=%d\r\n", ptr_edmacc_regs->qer);
		dev_dbg(&edma_dev.dev, "QEER=%d\r\n", ptr_edmacc_regs->qeer);
	} else if ((lch >= (DAVINCI_EDMA_NUM_DMACH + DAVINCI_EDMA_NUM_QDMACH))
		   && lch < DAVINCI_EDMA_NUM_PARAMENTRY) {
		/* for slaveChannels */
		ptr_edmacc_regs->paramentry[lch].link_bcntrld &= 0xffff0000;
		ptr_edmacc_regs->paramentry[lch].link_bcntrld |= 0xffff;
	} else {
	}
}

/******************************************************************************
 *
 * DMA channel link - link the two logical channels passed through by linking
 *                    the link field of head to the param pointed by the lch_queue.
 * ARGUMENTS:
 *      lch_head  - logical channel number, in which the link field is linked
 *                  to the param pointed to by lch_queue
 * lch_queue - logical channel number or the param entry number, which is to be
 *                  linked to the lch_head
 *
 *****************************************************************************/
void davinci_dma_link_lch(int lch_head, int lch_queue)
{
	unsigned long link;
	int temp_ch = 0;
	if (lch_head >=
	    DAVINCI_EDMA_NUM_DMACH
	    && lch_head < (DAVINCI_EDMA_NUM_DMACH + DAVINCI_EDMA_NUM_QDMACH)) {
		temp_ch =
		    qdam_to_param_mapping[lch_head - DAVINCI_EDMA_NUM_DMACH];
		lch_head = temp_ch;
	}
	if (lch_queue >=
	    DAVINCI_EDMA_NUM_DMACH
	    && lch_queue < (DAVINCI_EDMA_NUM_DMACH + DAVINCI_EDMA_NUM_QDMACH)) {
		temp_ch =
		    qdam_to_param_mapping[lch_queue - DAVINCI_EDMA_NUM_DMACH];
		lch_queue = temp_ch;
	}
	if ((lch_head >= 0 && lch_head < DAVINCI_EDMA_NUM_PARAMENTRY)
	    && (lch_queue >= 0 && lch_queue < DAVINCI_EDMA_NUM_PARAMENTRY)) {
		/* program LINK */
		link =
		    (unsigned
		     long)(&
			   (ptr_edmacc_regs->
			    paramentry[dma_chan[lch_queue].param_no].opt));
		ptr_edmacc_regs->
		    paramentry[dma_chan
			       [lch_head].param_no].link_bcntrld &= 0xffff0000;
		ptr_edmacc_regs->
		    paramentry[dma_chan
			       [lch_head].
			       param_no].link_bcntrld |= ((unsigned short)
							  link);
		dma_chan[lch_head].link_lch = lch_queue;
	}
}

/******************************************************************************
 *
 * DMA channel unlink - unlink the two logical channels passed through by
 *                   setting the link field of head to 0xffff.
 * ARGUMENTS:
 * lch_head - logical channel number, from which the link field is to be removed
 * lch_queue - logical channel number or the param entry number, which is to be
 *             unlinked from lch_head
 *
 *****************************************************************************/
void davinci_dma_unlink_lch(int lch_head, int lch_queue)
{
	int temp_ch = 0;
	if (lch_head >=
	    DAVINCI_EDMA_NUM_DMACH
	    && lch_head < (DAVINCI_EDMA_NUM_DMACH + DAVINCI_EDMA_NUM_QDMACH)) {
		temp_ch =
		    qdam_to_param_mapping[lch_head - DAVINCI_EDMA_NUM_DMACH];
		lch_head = temp_ch;
	}
	if (lch_queue >=
	    DAVINCI_EDMA_NUM_DMACH
	    && lch_queue < (DAVINCI_EDMA_NUM_DMACH + DAVINCI_EDMA_NUM_QDMACH)) {
		temp_ch =
		    qdam_to_param_mapping[lch_queue - DAVINCI_EDMA_NUM_DMACH];
		lch_queue = temp_ch;
	}
	if ((lch_head >= 0 && lch_head < DAVINCI_EDMA_NUM_PARAMENTRY)
	    && (lch_queue >= 0 && lch_queue < DAVINCI_EDMA_NUM_PARAMENTRY)) {
		ptr_edmacc_regs->
		    paramentry[dma_chan
			       [lch_head].param_no].link_bcntrld |= 0xffff;
		dma_chan[lch_head].link_lch = -1;
	}
}

/******************************************************************************
 *
 * DMA channel chain - chains the two logical channels passed through by
 * ARGUMENTS:
 * lch_head - logical channel number, from which the link field is to be removed
 * lch_queue - logical channel number or the param entry number, which is to be
 *             unlinked from lch_head
 *
 *****************************************************************************/
void davinci_dma_chain_lch(int lch_head, int lch_queue)
{
	int temp_ch = 0;
	if (lch_head >=
	    DAVINCI_EDMA_NUM_DMACH
	    && lch_head < (DAVINCI_EDMA_NUM_DMACH + DAVINCI_EDMA_NUM_QDMACH)) {
		temp_ch =
		    qdam_to_param_mapping[lch_head - DAVINCI_EDMA_NUM_DMACH];
		lch_head = temp_ch;
	}
	if (lch_queue >=
	    DAVINCI_EDMA_NUM_DMACH
	    && lch_queue < (DAVINCI_EDMA_NUM_DMACH + DAVINCI_EDMA_NUM_QDMACH)) {
		temp_ch =
		    qdam_to_param_mapping[lch_queue - DAVINCI_EDMA_NUM_DMACH];
		lch_queue = temp_ch;
	}
	if ((lch_head >= 0
	     && lch_head < (DAVINCI_EDMA_NUM_DMACH + DAVINCI_EDMA_NUM_QDMACH))
	    &&
	    (lch_queue >= 0
	     && lch_queue < (DAVINCI_EDMA_NUM_DMACH + DAVINCI_EDMA_NUM_QDMACH))
	    ) {			/* set TCCHEN */
		/* set TCCHEN */
		ptr_edmacc_regs->paramentry[lch_head].opt |= TCCHEN;
		/* program tcc */
		ptr_edmacc_regs->paramentry[lch_head].opt &= (~TCC);
		ptr_edmacc_regs->
		    paramentry[lch_head].opt |= (lch_queue & 0x3f) << 12;
	}
}

/******************************************************************************
 *
 * DMA channel unchain - unchain the two logical channels passed through by
 * ARGUMENTS:
 * lch_head - logical channel number, from which the link field is to be removed
 * lch_queue - logical channel number or the param entry number, which is to be
 *             unlinked from lch_head
 *
 *****************************************************************************/
void davinci_dma_unchain_lch(int lch_head, int lch_queue)
{
	int temp_ch = 0;
	if (lch_head >=
	    DAVINCI_EDMA_NUM_DMACH
	    && lch_head < (DAVINCI_EDMA_NUM_DMACH + DAVINCI_EDMA_NUM_QDMACH)) {
		temp_ch =
		    qdam_to_param_mapping[lch_head - DAVINCI_EDMA_NUM_DMACH];
		lch_head = temp_ch;
	}
	if (lch_queue >=
	    DAVINCI_EDMA_NUM_DMACH
	    && lch_queue < (DAVINCI_EDMA_NUM_DMACH + DAVINCI_EDMA_NUM_QDMACH)) {
		temp_ch =
		    qdam_to_param_mapping[lch_queue - DAVINCI_EDMA_NUM_DMACH];
		lch_queue = temp_ch;
	}
	if ((lch_head >= 0
	     && lch_head < (DAVINCI_EDMA_NUM_DMACH + DAVINCI_EDMA_NUM_QDMACH))
	    && (lch_queue >= 0
		&& lch_queue <
		(DAVINCI_EDMA_NUM_DMACH + DAVINCI_EDMA_NUM_QDMACH))) {
		/* reset TCCHEN */
		ptr_edmacc_regs->paramentry[lch_head].opt &= ~TCCHEN;
	}
}

/******************************************************************************
 *
 * It cleans ParamEntry qand bring back EDMA to initial state if media has
 * been removed before EDMA has finished.It is usedful for removable media.
 * Arguments:
 *      ch_no     - channel no
 *
 * Return: zero on success, or corresponding error no on failure
 *
 *****************************************************************************/

void davinci_clean_channel(int ch_no)
{
	int i;
	dev_dbg(&edma_dev.dev, "EMR =%d\r\n", ptr_edmacc_regs->emr);
	if (ch_no < 32) {
		for (i = 0; i < 32; i++) {
			if (ch_no == i) {
				ptr_edmacc_regs->shadow[0].ecr |= (1 << i);
				/* Clear the corresponding EMR bits */
				ptr_edmacc_regs->emcr |= (1 << i);
				/* Clear any SER */
				ptr_edmacc_regs->shadow[0].secr |= (1 << i);
				ptr_edmacc_regs->ccerrclr |= ((1 << 16) | 0x3);
			}
		}
	}

	if (ch_no > 32) {
		dev_dbg(&edma_dev.dev, "EMRH =%d\r\n", ptr_edmacc_regs->emrh);
		for (i = 0; i < 32; i++) {
			if (ch_no == (i + 32)) {
				ptr_edmacc_regs->shadow[0].ecrh |= (1 << i);
				/* Clear the corresponding IPR bits */
				ptr_edmacc_regs->emcrh |= (1 << i);
				/* Clear any SER */
				ptr_edmacc_regs->shadow[0].secrh |= (1 << i);
				ptr_edmacc_regs->ccerrclr |= ((1 << 16) | 0x3);
			}
		}
	}
}

/******************************************************************************
 *
 * DMA interrupt handlers
 *
 *****************************************************************************/
static int dma_irq_handler_l(int sound_curr_lch, void
			     *ch_status, struct
			     pt_regs
			     *data)
{
	dev_dbg(&edma_dev.dev, "dma_irq_handler\n");
	(*cb[0]) ();
	return IRQ_HANDLED;
}

static int
    dma_ccerr_handler_l
    (int sound_curr_lch, void *ch_status, struct pt_regs *data) {
	dev_dbg(&edma_dev.dev, "dma_ccerr_handler\n");
	(*cb[1]) ();
	return IRQ_HANDLED;
}

static int
    dma_tc1err_handler_l
    (int sound_curr_lch, void *ch_status, struct pt_regs *data) {
	dev_dbg(&edma_dev.dev, "dma_tc1err_handler\n");
	(*cb[2]) ();
	return IRQ_HANDLED;
}

static int
    dma_tc2err_handler_l
    (int sound_curr_lch, void *ch_status, struct pt_regs *data) {
	dev_dbg(&edma_dev.dev, "dma_tc2err_handler\n");
	(*cb[3]) ();
	return IRQ_HANDLED;
}

int register_dma_interrupts
    (intr_callback cb1,
     intr_callback cb2, intr_callback cb3, intr_callback cb4) {
	cb[0] = cb1;
	cb[1] = cb2;
	cb[2] = cb3;
	cb[3] = cb4;
	if (!cb1 || !cb2 || !cb3 || !cb4) {
		dev_dbg(&edma_dev.dev, "NULL callback\n");
		return -1;
	}

	if (request_irq(IRQ_CCINT0, dma_irq_handler_l, 0, "EDMA", NULL)) {
		dev_dbg(&edma_dev.dev, "request_irq failed\n");
		return -1;
	}
	if (request_irq
	    (IRQ_CCERRINT, dma_ccerr_handler_l, 0, "EDMA CC Err", NULL)) {
		dev_dbg(&edma_dev.dev, "request_irq failed\n");
		return -1;
	}
	if (request_irq
	    (IRQ_TCERRINT0, dma_tc1err_handler_l, 0, "EDMA TC1 Err", NULL)) {
		dev_dbg(&edma_dev.dev, "request_irq failed\n");
		return -1;
	}
	if (request_irq
	    (IRQ_TCERRINT, dma_tc2err_handler_l, 0, "EDMA TC2 Err", NULL)) {
		dev_dbg(&edma_dev.dev, "request_irq failed\n");
		return -1;
	}
	return 0;
}

arch_initcall(arch_dma_init);
EXPORT_SYMBOL(davinci_start_dma);
EXPORT_SYMBOL(davinci_dma_link_lch);
EXPORT_SYMBOL(davinci_set_dma_params);
EXPORT_SYMBOL(davinci_get_dma_params);
EXPORT_SYMBOL(davinci_set_dma_transfer_params);
EXPORT_SYMBOL(davinci_set_dma_dest_index);
EXPORT_SYMBOL(davinci_set_dma_src_index);
EXPORT_SYMBOL(davinci_set_dma_dest_params);
EXPORT_SYMBOL(davinci_set_dma_src_params);
EXPORT_SYMBOL(davinci_request_dma);
EXPORT_SYMBOL(davinci_stop_dma);
EXPORT_SYMBOL(davinci_clean_channel);
EXPORT_SYMBOL(davinci_free_dma);
EXPORT_SYMBOL(davinci_dma_chain_lch);
EXPORT_SYMBOL(davinci_dma_unchain_lch);
EXPORT_SYMBOL(davinci_dma_unlink_lch);
