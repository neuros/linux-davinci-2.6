/*
 * linux/drivers/net/davinci_emac_phy.c
 *
 * EMAC MII-MDIO Module - Polling State Machine.
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
 *  HISTORY:
 *  Date      	Modifier         		Notes
 *  2001/02 	Denis, Bill, Michael		Original
 *  14Feb2006	Anant Gole 			Re-written for linux
 *  07Dec2006	Paul Bartholomew		Fix half-duplex,
 *           					use PHY_DUPLEX_* constants
 */
#include <linux/kernel.h>

#include "davinci_emac_phy.h"

#define EMAC_PHY_DEBUG

#ifdef EMAC_PHY_DEBUG
/* note: prints function name for you */
#define DPRINTK(fmt, args...) if (emac_phy->debug_mode) printk(KERN_ERR "\n%s: " fmt, __FUNCTION__ , ## args)
#else
#define DPRINTK(fmt, args...)
#endif

/* Phy Registers */
#define PHY_CONTROL_REG       	0
#define MII_PHY_RESET           (1<<15)
#define MII_PHY_LOOP            (1<<14)
#define MII_PHY_100             (1<<13)
#define MII_AUTO_NEGOTIATE_EN   (1<<12)
#define MII_PHY_PDOWN           (1<<11)
#define MII_PHY_ISOLATE         (1<<10)
#define MII_RENEGOTIATE         (1<<9)
#define MII_PHY_FD              (1<<8)

#define PHY_STATUS_REG        	1
#define MII_NWAY_COMPLETE       (1<<5)
#define MII_NWAY_CAPABLE        (1<<3)
#define MII_PHY_LINKED          (1<<2)

#define NWAY_ADVERTIZE_REG    	4
#define NWAY_REMADVERTISE_REG 	5
#define MII_NWAY_FD100          (1<<8)
#define MII_NWAY_HD100          (1<<7)
#define MII_NWAY_FD10           (1<<6)
#define MII_NWAY_HD10           (1<<5)
#define MII_NWAY_SEL            (1<<0)

/* Timeout values - since timer tikc is expected to be 10 mSecs fixed these
 * values are in (value * 10 mSecs) */
#define PHY_FIND_TIMEOUT (2)
#define PHY_RECK_TIMEOUT (200)
#define PHY_LINK_TIMEOUT (500)
#define PHY_NWST_TIMEOUT (500)
#define PHY_NWDN_TIMEOUT (800)
#define PHY_MDIX_TIMEOUT (274)	/* 2.74 Seconds <--Spec and empirical */

/* Mask & Control defines */
#define MDIO_CONTROL_CLKDIV           	(0xFF)
#define MDIO_CONTROL_ENABLE           	(1 << 30)
#define MDIO_USERACCESS_GO     			(1 << 31)
#define MDIO_USERACCESS_WRITE  			(1 << 30)
#define MDIO_USERACCESS_READ   			(0 << 30)
#define MDIO_USERACCESS_WRITE  			(1 << 30)
#define MDIO_USERACCESS_REGADR 			(0x1F << 21)
#define MDIO_USERACCESS_PHYADR 			(0x1F << 16)
#define MDIO_USERACCESS_DATA   			(0xFFFF)
#define MDIO_USERPHYSEL_LINKSEL         (1 << 7)
#define MDIO_VER_MODID         			(0xFFFF << 16)
#define MDIO_VER_REVMAJ        			(0xFF   << 8)
#define MDIO_VER_REVMIN        			(0xFF)

/* PHY Registers */
#define MDIO_VER						(0x00)
#define MDIO_CONTROL					(0x04)
#define MDIO_ALIVE						(0x08)
#define MDIO_LINK						(0x0C)
#define MDIO_LINKINTRAW					(0x10)
#define MDIO_LINKINTMASKED				(0x14)
#define MDIO_USERINTRAW					(0x20)
#define MDIO_USERINTMASKED				(0x24)
#define MDIO_USERINTMASKED_SET			(0x28)
#define MDIO_USERINTMASKED_CLR			(0x2C)
#define MDIO_USERACCESS(inst)			(0x80+(inst*8))
#define MDIO_USERPHYSEL(inst) 			(0x84+(inst*8))

#define MDIO_REG(reg)					(*((volatile unsigned int *)(emac_phy->base + (reg))))

#define MDIO_REG_VER					MDIO_REG(MDIO_VER)
#define MDIO_REG_CONTROL				MDIO_REG(MDIO_CONTROL)
#define MDIO_REG_ALIVE					MDIO_REG(MDIO_ALIVE)
#define MDIO_REG_LINK					MDIO_REG(MDIO_LINK)
#define MDIO_REG_LINKINTRAW				MDIO_REG(MDIO_LINKINTRAW)
#define MDIO_REG_LINKINTMASKED			MDIO_REG(MDIO_LINKINTMASKED)
#define MDIO_REG_USERINTMASKED			MDIO_REG(MDIO_USERINTMASKED)
#define MDIO_REG_USERINTMASKED_SET		MDIO_REG(MDIO_USERINTMASKED_SET)
#define MDIO_REG_USERINTMASKED_CLR		MDIO_REG(MDIO_USERINTMASKED_CLR)
#define MDIO_REG_USERACCESS				MDIO_REG(MDIO_USERACCESS(emac_phy->inst))
#define MDIO_REG_USERPHYSEL				MDIO_REG(MDIO_USERPHYSEL(emac_phy->inst))

/* Phy State */
#define PHY_NULL       (0)
#define PHY_INIT       (1)
#define PHY_FINDING    (2)
#define PHY_FOUND      (3)
#define PHY_NWAY_START (4)
#define PHY_NWAY_WAIT  (5)
#define PHY_LINK_WAIT  (6)
#define PHY_LINKED     (7)
#define PHY_LOOPBACK   (8)

static char *phy_state_str[] = {
	"NULL", "INIT", "FINDING", "FOUND", "NWAY_START", "NWAY_WAIT",
	"LINK_WAIT", "LINKED", "LOOPBACK"
};

#define PHY_NOT_FOUND  0xFFFF	/*  Used in Phy Detection */

struct phy_info {
	int inst;		/* Instance of PHY - for user sel register */
	unsigned int base;	/* Base address of mdio module */
	int state;		/* state of phy */
	int state_change;	/* phy state change ? */
	unsigned int timeout;	/* Timeout counter */
	unsigned int phy_mode;	/* requested phy mode */
	unsigned int speed;	/* current Speed - 10 / 100 */
	unsigned int duplex;	/* 0=Auto Negotiate, Full=3; Half=2, Unknown=1 */
	unsigned int phy_addr;	/* phy address */
	unsigned int phy_mask;	/* phy mask */
	unsigned int mlink_mask;/* mlink mask */
	int debug_mode;		/* debug mode */
};

/* Global phy structure instance */
struct phy_info emac_phy_info;
struct phy_info *emac_phy = &emac_phy_info;

void emac_mdio_get_ver(unsigned int mdio_base, unsigned int *module_id,
		       unsigned int *rev_major, unsigned int *rev_minor)
{
	unsigned int ver;

	emac_phy->base = mdio_base;
	ver = MDIO_REG_VER;

	*module_id = (ver & MDIO_VER_MODID) >> 16;
	*rev_major = (ver & MDIO_VER_REVMAJ) >> 8;
	*rev_minor = (ver & MDIO_VER_REVMIN);
}

/* Initialize mdio module */
int emac_mdio_init(unsigned int mdio_base,
		   unsigned int inst,
		   unsigned int phy_mask,
		   unsigned int mlink_mask,
		   unsigned int mdio_bus_freq,
		   unsigned int mdio_clock_freq, unsigned int verbose)
{
	unsigned int clk_div;

	/* Set base addr and init phy state */
	emac_phy->inst = inst;
	emac_phy->base = mdio_base;
	emac_phy->phy_mask = phy_mask;
	emac_phy->mlink_mask = mlink_mask;
	emac_phy->state = PHY_INIT;
	emac_phy->debug_mode = verbose;
	emac_phy->speed = 10;
	emac_phy->duplex = PHY_DUPLEX_HALF;	/* Half duplex */

	if (mdio_clock_freq & mdio_bus_freq) {
		clk_div = ((mdio_bus_freq / mdio_clock_freq) - 1);
	} else {
		clk_div = 0xFF;
	}
	clk_div &= MDIO_CONTROL_CLKDIV;

	/* Set enable and clock divider in MDIOControl */
	MDIO_REG_CONTROL = (clk_div | MDIO_CONTROL_ENABLE);

	return (0);
}

/* Set PHY mode - autonegotiation or any other */
void emac_mdio_set_phy_mode(unsigned int phy_mode)
{
	emac_phy->phy_mode = phy_mode;

	if ((emac_phy->state == PHY_NWAY_START) ||
	    (emac_phy->state == PHY_NWAY_WAIT) ||
	    (emac_phy->state == PHY_LINK_WAIT) ||
	    (emac_phy->state == PHY_LINKED) ||
	    (emac_phy->state == PHY_LOOPBACK)) {
		emac_phy->state = PHY_FOUND;
		emac_phy->state_change = 1;
	}

	DPRINTK("PhyMode:%08X Auto:%d, FD10:%d, HD10:%d, FD100:%d, HD100:%d\n",
		phy_mode,
		phy_mode & NWAY_AUTO, phy_mode & MII_NWAY_FD10,
		phy_mode & MII_NWAY_HD10, phy_mode & MII_NWAY_FD100,
		phy_mode & MII_NWAY_HD100);
}

/* Get linked status - check if link is on - 1=link on, 0=link off */
int emac_mdio_is_linked(void)
{
	return ((emac_phy->state == PHY_LINKED) ? 1 : 0);
}

/* Get speed - 10 / 100 Mbps */
int emac_mdio_get_speed(void)
{
	return (emac_phy->speed);
}

/* Get duplex - 0=Auto Negotiate, Full Duplex = 3; Half Duplex = 2 Unknown = 1 */
int emac_mdio_get_duplex(void)
{
	return (emac_phy->duplex);
}

/* Get Phy number/address */
int emac_mdio_get_phy_num(void)
{
	return (emac_phy->phy_addr);
}

/* Check if loopback enabled on phy */
int emac_mdio_is_loopback(void)
{
	return ((emac_phy->state == PHY_LOOPBACK) ? 1 : 0);
}

/* Wait until mdio is ready for next command */
#define MDIO_WAIT_FOR_USER_ACCESS while ((MDIO_REG_USERACCESS & MDIO_USERACCESS_GO) != 0) {}

/* Read from a phy register via mdio interface */
unsigned int emac_mdio_read(unsigned int phy_addr, unsigned int phy_reg)
{
	unsigned int phy_data = 0;
	unsigned int phy_control;

	/* Wait until mdio is ready for next command */
	MDIO_WAIT_FOR_USER_ACCESS;

	phy_control = (MDIO_USERACCESS_GO |
		       MDIO_USERACCESS_READ |
		       ((phy_reg << 21) & MDIO_USERACCESS_REGADR) |
		       ((phy_addr << 16) & MDIO_USERACCESS_PHYADR) |
		       (phy_data & MDIO_USERACCESS_DATA));
	MDIO_REG_USERACCESS = phy_control;

	/* Wait until mdio is ready for next command */
	MDIO_WAIT_FOR_USER_ACCESS;

	return (MDIO_REG_USERACCESS & MDIO_USERACCESS_DATA);
}

/* Write to a phy register via mdio interface */
void emac_mdio_write(unsigned int phy_addr, unsigned int phy_reg,
		     unsigned int phy_data)
{
	unsigned int control;

	/* Wait until mdio is ready for next command */
	MDIO_WAIT_FOR_USER_ACCESS;

	control = (MDIO_USERACCESS_GO |
		   MDIO_USERACCESS_WRITE |
		   ((phy_reg << 21) & MDIO_USERACCESS_REGADR) |
		   ((phy_addr << 16) & MDIO_USERACCESS_PHYADR) |
		   (phy_data & MDIO_USERACCESS_DATA));
	MDIO_REG_USERACCESS = control;

}

/* Reset the selected phy */
void emac_mdio_phy_reset(unsigned int phy_addr)
{
	unsigned int control;

	emac_mdio_write(phy_addr, PHY_CONTROL_REG, MII_PHY_RESET);

	do {
		control = emac_mdio_read(phy_addr, PHY_CONTROL_REG);
	} while (control & MII_PHY_RESET);

	/* CRITICAL: Fix for increasing PHY signal drive strength for
	 * TX lockup issue. On DaVinci EVM, the Intel LXT971 PHY
  	 * signal strength was low causing  TX to fail randomly. The
	 * fix is to Set bit 11 (Increased MII drive strength) of PHY
         * register 26 (Digital Config register) on this phy. */
	control = emac_mdio_read(phy_addr, 26);
	emac_mdio_write(phy_addr, 26, (control | 0x800));
	control = emac_mdio_read(phy_addr, 26);
}

/* Timeout condition handler in PHY state machine */
void emac_mdio_phy_timeout(void)
{
	emac_phy->state = PHY_FOUND;
	emac_phy->state_change = 1;

	/* If MDI/MDIX is supported then switch MDIX state */
}

/* PHY state machine : Init state handler */
void emac_mdio_init_state(void)
{
	emac_phy->state = PHY_FINDING;
	emac_phy->state_change = 1;
	emac_phy->timeout = PHY_FIND_TIMEOUT;
}

/* PHY state machine : Finding state handler */
void emac_mdio_finding_state(void)
{
	unsigned int phy_alive_status;
	int i, j;

	emac_phy->phy_addr = PHY_NOT_FOUND;

	/* Find if timeout complete */
	if (emac_phy->timeout) {
		/* Allow some time for phy to show up in alive register */
		--emac_phy->timeout;
	} else {

		phy_alive_status = MDIO_REG_LINK;
		/* Check phys based upon user mask */
		phy_alive_status &= emac_phy->phy_mask;

		/* Find the first interesting alive phy */
		for (i = 0, j = 1;
		     (i < 32) && ((j & phy_alive_status) == 0); i++, j <<= 1) ;

		if ((phy_alive_status) && (i < 32)) {
			emac_phy->phy_addr = i;
		}

		if (emac_phy->phy_addr != PHY_NOT_FOUND) {
			DPRINTK("PHY Found. Phy Number=%d\n",
				emac_phy->phy_addr);
			emac_phy->state = PHY_FOUND;
			emac_phy->state_change = 1;
		} else {
			/* Set Timer for finding timeout */
			DPRINTK("PHY NOT Found. Starting timeout\n");
			emac_phy->timeout = PHY_RECK_TIMEOUT;
		}
	}
}

/* PHY state machine : Found state handler */
void emac_mdio_found_state(void)
{
	unsigned int phy_status;
	unsigned int phy_num;
	unsigned int cnt;
	unsigned int nway_advertise;

	/* Check if there is any phy mode requested by the user */
	if (emac_phy->phy_mode == 0) {
		return;
	}

	/* Check alive phy's */
	phy_status = MDIO_REG_LINK;
	phy_status &= emac_phy->phy_mask;	/* Check phys based upon user mask */

	/* we will now isolate all our phys, except the one we have decided to use */
	for (phy_num = 0, cnt = 1; phy_num < 32; phy_num++, cnt <<= 1) {
		if (phy_status & cnt) {
			if (phy_num != emac_phy->phy_addr) {
				/* Disable a phy that we are not using */
				/* CRITICAL: Note that this code assums that there is only 1 phy connected
				 * if this is not the case then the next statement should be commented
				 */
				emac_mdio_write(emac_phy->phy_addr,
						PHY_CONTROL_REG,
						(MII_PHY_ISOLATE |
						 MII_PHY_PDOWN));
			}
		}
	}

	/*  Reset the Phy and proceed with auto-negotiation */
	emac_mdio_phy_reset(emac_phy->phy_addr);

	/*  Set the way Link will be Monitored, Check the Link Selection Method */
	if ((1 << emac_phy->phy_addr) & emac_phy->mlink_mask) {
		MDIO_REG_USERPHYSEL =
		    (emac_phy->phy_addr | MDIO_USERPHYSEL_LINKSEL);
	}

	/* For Phy Internal loopback , need to wait until Phy found */
	if (emac_phy->phy_mode & NWAY_LPBK) {
		/* Set Phy in Loopback and read mdio to confirm */
		emac_mdio_write(emac_phy->phy_addr, PHY_CONTROL_REG,
				(MII_PHY_LOOP | MII_PHY_FD));
		emac_mdio_read(emac_phy->phy_addr, PHY_STATUS_REG);
		emac_phy->state = PHY_LOOPBACK;
		emac_phy->state_change = 1;
		return;
	}

	/* Start negotiation */
	nway_advertise = MII_NWAY_SEL;
	if (emac_phy->phy_mode & NWAY_FD100)
		nway_advertise |= MII_NWAY_FD100;
	if (emac_phy->phy_mode & NWAY_HD100)
		nway_advertise |= MII_NWAY_HD100;
	if (emac_phy->phy_mode & NWAY_FD10)
		nway_advertise |= MII_NWAY_FD10;
	if (emac_phy->phy_mode & NWAY_HD10)
		nway_advertise |= MII_NWAY_HD10;

	phy_status = emac_mdio_read(emac_phy->phy_addr, PHY_STATUS_REG);

	if ((phy_status & MII_NWAY_CAPABLE) && (emac_phy->phy_mode & NWAY_AUTO)) {

		/* NWAY Phy Detected - following procedure for NWAY compliant Phys */
		emac_mdio_write(emac_phy->phy_addr, NWAY_ADVERTIZE_REG,
				nway_advertise);
		if (emac_phy->debug_mode) {
			DPRINTK("NWAY Advertising: ");
			if (nway_advertise & MII_NWAY_FD100)
				DPRINTK("100 Mbps FullDuplex");
			if (nway_advertise & MII_NWAY_HD100)
				DPRINTK("100 Mbps HalfDuplex");
			if (nway_advertise & MII_NWAY_FD10)
				DPRINTK("10 Mbps FullDuplex");
			if (nway_advertise & MII_NWAY_HD10)
				DPRINTK("10 Mbps HalfDuplex");
			DPRINTK("\n");
		}

		/* Start/Restart autonegotiation */
		emac_mdio_write(emac_phy->phy_addr, PHY_CONTROL_REG,
				MII_AUTO_NEGOTIATE_EN);
		emac_mdio_write(emac_phy->phy_addr, PHY_CONTROL_REG,
				(MII_AUTO_NEGOTIATE_EN | MII_RENEGOTIATE));
		emac_phy->state = PHY_NWAY_START;
		emac_phy->state_change = 1;
		emac_phy->timeout = PHY_NWST_TIMEOUT;

	} else {
		/* Phy cannot do auto negotiation */
		emac_phy->phy_mode &= ~NWAY_AUTO;
		nway_advertise &= ~MII_NWAY_SEL;
		phy_status = 0;

		if (nway_advertise & (MII_NWAY_FD100 | MII_NWAY_HD100)) {
			phy_status = MII_PHY_100;	/* Set 100 Mbps if requested */
			nway_advertise &= (MII_NWAY_FD100 | MII_NWAY_HD100);
		} else {
			nway_advertise &= (MII_NWAY_FD10 | MII_NWAY_HD10);
		}

		if (nway_advertise & (MII_NWAY_FD100 | MII_NWAY_FD10)) {
			phy_status |= MII_PHY_FD;	/* Set Full duplex if requested */
		}

		/* Set requested speed and duplex mode on phy */
		emac_mdio_write(emac_phy->phy_addr, PHY_CONTROL_REG,
				phy_status);

		/* Set the phy speed and duplex mode */
		emac_phy->speed = (phy_status & MII_PHY_100) ? 100 : 10;
		emac_phy->duplex = (phy_status & MII_PHY_FD) ? 3 : 2;

		emac_phy->state = PHY_LINK_WAIT;
		emac_phy->state_change = 1;
		emac_phy->timeout = PHY_LINK_TIMEOUT;
	}

	/* TODO: When Auto MDIX is supported, add delay here
	   emac_mdio_mdix_delay();
	 */
}

/* PHY state machine : NWAY Start state handler */
void emac_mdio_nwaystart_state(void)
{
	unsigned int status;

	status = emac_mdio_read(emac_phy->phy_addr, PHY_CONTROL_REG);
	if ((status & MII_RENEGOTIATE) == 0) {
		/* Flush pending latched bits */
		status = emac_mdio_read(emac_phy->phy_addr, PHY_STATUS_REG);
		emac_phy->state = PHY_NWAY_WAIT;
		emac_phy->state_change = 1;
		emac_phy->timeout = PHY_NWDN_TIMEOUT;
	} else {
		if (emac_phy->timeout) {
			--emac_phy->timeout;
		} else {
			/* Timed Out for NWAY to start - very unlikely condition, back to Found */
			emac_mdio_phy_timeout();
		}
	}
}

/* PHY state machine : NWAY Wait state handler */
void emac_mdio_nwaywait_state(void)
{
	unsigned int status;
	unsigned int my_cap, partner_cap, neg_mode;

	/* Check if nway negotiation complete */
	status = emac_mdio_read(emac_phy->phy_addr, PHY_STATUS_REG);

	if (status & MII_NWAY_COMPLETE) {
		/* negotiation complete, check for partner capabilities */
		emac_phy->state_change = 1;
		my_cap = emac_mdio_read(emac_phy->phy_addr, NWAY_ADVERTIZE_REG);
		partner_cap =
		    emac_mdio_read(emac_phy->phy_addr, NWAY_REMADVERTISE_REG);

		/* Negotiated mode is what we and partnet have in common */
		neg_mode = my_cap & partner_cap;
		if (emac_phy->debug_mode) {
			DPRINTK
			    ("Phy %d, neg_mode %04X, my_cap %04X, partner_cap %04X\n",
			     emac_phy->phy_addr, neg_mode, my_cap, partner_cap);
		}

		/* Limit negotiation to fields below */
		neg_mode &=
		    (MII_NWAY_FD100 | MII_NWAY_HD100 | MII_NWAY_FD10 |
		     MII_NWAY_HD10);
		if (neg_mode == 0)
			DPRINTK
			    ("WARNING: Negotiation complete but NO agreement, default is 10HD\n");

		if (neg_mode & MII_NWAY_FD100)
			DPRINTK("100 Mbps FullDuplex");
		if (neg_mode & MII_NWAY_HD100)
			DPRINTK("100 Mbps HalfDuplex");
		if (neg_mode & MII_NWAY_FD10)
			DPRINTK("10 Mbps FullDuplex");
		if (neg_mode & MII_NWAY_HD10)
			DPRINTK("10 Mbps HalfDuplex");
		DPRINTK("\n");

		if (neg_mode != 0) {
			if (status & MII_PHY_LINKED) {
				emac_phy->state = PHY_LINKED;
			} else {
				emac_phy->state = PHY_LINK_WAIT;
			}
		}

		/* Set the phy speed and duplex mode */
		emac_phy->speed =
		    (neg_mode & (MII_NWAY_FD100 | MII_NWAY_HD100)) ? 100 : 10;
		emac_phy->duplex =
		    (neg_mode & (MII_NWAY_FD100 | MII_NWAY_FD10)) ?
				PHY_DUPLEX_FULL : PHY_DUPLEX_HALF;

	} else {

		if (emac_phy->timeout) {
			--emac_phy->timeout;
		} else {
			/* Timed Out for NWAY to start - very unlikely condition, back to Found */
			emac_mdio_phy_timeout();
		}
	}
}

/* PHY state machine : Link Wait state handler */
void emac_mdio_linkwait_state(void)
{
	unsigned int status;

	/* Check if nway negotiation complete */
	status = emac_mdio_read(emac_phy->phy_addr, PHY_STATUS_REG);
	if (status & MII_PHY_LINKED) {
		emac_phy->state = PHY_LINKED;
		emac_phy->state_change = 1;
	} else {
		if (emac_phy->timeout) {
			--emac_phy->timeout;
		} else {
			/* Timed Out for link - very unlikely condition, back to Found */
			emac_mdio_phy_timeout();
		}
	}
}

/* PHY state machine : Linked handler */
void emac_mdio_linked_state(void)
{
	if (MDIO_REG_LINK & (1 << emac_phy->phy_addr)) {
		return;		/* do nothing if already linked */
	}

	/* If not linked, move mode to nway down or waiting for link */
	emac_phy->state_change = 1;
	if (emac_phy->phy_mode & NWAY_AUTO) {
		emac_phy->state = PHY_NWAY_WAIT;
		emac_phy->timeout = PHY_NWDN_TIMEOUT;
	} else {
		emac_phy->state = PHY_LINK_WAIT;
		emac_phy->timeout = PHY_LINK_TIMEOUT;
	}

	/* TODO: When Auto MDIX is supported, add delay here
	   emac_mdio_mdix_delay();
	 */
}

/* PHY state machine : Loopback handler */
void emac_mdio_loopback_state(void)
{
	return;
}

/* PHY state machine : Default handler */
void emac_mdio_default_state(void)
{
	/* Awaiting a init call */
	emac_phy->state_change = 1;
}

/* Detailed PHY dump for debug */
void emac_mdio_phy_dump(void)
{
	unsigned int status;

	DPRINTK("\n");
	DPRINTK("PHY Addr/Num=%d, PHY State=%s, Speed=%d, Duplex=%d\n",
		emac_phy->phy_addr, phy_state_str[emac_phy->state],
		emac_phy->speed, emac_phy->duplex);

	/* 0: Control register */
	status = emac_mdio_read(emac_phy->phy_addr, PHY_CONTROL_REG);
	DPRINTK("PhyControl: %04X, Loopback=%s, Speed=%s, Duplex=%s\n",
		status,
		status & MII_PHY_LOOP ? "On" : "Off",
		status & MII_PHY_100 ? "100" : "10",
		status & MII_PHY_FD ? "Full" : "Half");

	/* 1: Status register */
	status = emac_mdio_read(emac_phy->phy_addr, PHY_STATUS_REG);
	DPRINTK("PhyStatus: %04X, AutoNeg=%s, Link=%s\n",
		status,
		status & MII_NWAY_COMPLETE ? "Complete" : "NotComplete",
		status & MII_PHY_LINKED ? "Up" : "Down");

	/* 4: Auto Negotiation Advertisement register */
	status = emac_mdio_read(emac_phy->phy_addr, NWAY_ADVERTIZE_REG);
	DPRINTK("PhyMyCapability: %04X, 100FD=%s, 100HD=%s, 10FD=%s, 10HD=%s\n",
		status,
		status & MII_NWAY_FD100 ? "Yes" : "No",
		status & MII_NWAY_HD100 ? "Yes" : "No",
		status & MII_NWAY_FD10 ? "Yes" : "No",
		status & MII_NWAY_HD10 ? "Yes" : "No");

	/* 5: Auto Negotiation Advertisement register */
	status = emac_mdio_read(emac_phy->phy_addr, NWAY_REMADVERTISE_REG);
	DPRINTK
	    ("PhyPartnerCapability: %04X, 100FD=%s, 100HD=%s, 10FD=%s, 10HD=%s\n",
	     status, status & MII_NWAY_FD100 ? "Yes" : "No",
	     status & MII_NWAY_HD100 ? "Yes" : "No",
	     status & MII_NWAY_FD10 ? "Yes" : "No",
	     status & MII_NWAY_HD10 ? "Yes" : "No");
}

/* emac_mdio_tick is called every 10 mili seconds to process Phy states */
int emac_mdio_tick(void)
{

	switch (emac_phy->state) {

	case PHY_INIT:
		emac_mdio_init_state();
		break;

	case PHY_FINDING:
		emac_mdio_finding_state();
		break;

	case PHY_FOUND:
		emac_mdio_found_state();
		break;

	case PHY_NWAY_START:
		emac_mdio_nwaystart_state();
		break;

	case PHY_NWAY_WAIT:
		emac_mdio_nwaywait_state();
		break;

	case PHY_LINK_WAIT:
		emac_mdio_linkwait_state();
		break;

	case PHY_LINKED:
		emac_mdio_linked_state();
		break;

	case PHY_LOOPBACK:
		emac_mdio_loopback_state();
		break;

	default:
		emac_mdio_default_state();
		break;
	}

	/* Check is MDI/MDIX mode switch is needed - not supported in DaVinci hardware */

	/* Return state change to user */
	if (emac_phy->state_change) {
		emac_mdio_phy_dump();
		emac_phy->state_change = 0;
		return (1);
	} else {
		return (0);
	}
}
