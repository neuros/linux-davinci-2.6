/*
 * linux/drivers/net/davinci_emac_phy.h
 *
 * MDIO Polling State Machine API. Functions will enable mii-Phy
 * negotiation.
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
 * Modifications:
 *  HISTORY:
 *  Date      Modifier         Ver    Notes
 *  01Jan01 Denis, Bill             Original
 *  27Mar02 Michael Hanrahan Original (modified from emacmdio.h)
 *  04Apr02 Michael Hanrahan Added Interrupt Support
 *  07Dec06 Paul Bartholomew Added PHY_DUPLEX_* defines
 */
#ifndef _DAVINCI_EMAC_PHY_H_
#define _DAVINCI_EMAC_PHY_H_

/* phy mode values  */
#define NWAY_AUTOMDIX       (1<<16)
#define NWAY_FD1000         (1<<13)
#define NWAY_HD1000         (1<<12)
#define NWAY_NOPHY          (1<<10)
#define NWAY_LPBK           (1<<9)
#define NWAY_FD100          (1<<8)
#define NWAY_HD100          (1<<7)
#define NWAY_FD10           (1<<6)
#define NWAY_HD10           (1<<5)
#define NWAY_AUTO           (1<<0)

/* phy duplex values */
#define	PHY_DUPLEX_AUTO		0	/* Auto Negotiate */
#define	PHY_DUPLEX_UNKNOWN	1	/* Unknown */
#define	PHY_DUPLEX_HALF		2	/* Half Duplex */
#define	PHY_DUPLEX_FULL		3	/* Full Duplex */

/*
 *    Tic() return values
 */

#define _MIIMDIO_MDIXFLIP (1<<28)

#define _AUTOMDIX_DELAY_MIN  80	/* milli-seconds */
#define _AUTOMDIX_DELAY_MAX 200	/* milli-seconds */

/* Get module version */
void emac_mdio_get_ver(unsigned int mdio_base, unsigned int *module_id,
		       unsigned int *rev_major, unsigned int *rev_minor);

/* Initialize mdio module */
int emac_mdio_init(unsigned int mdio_base,
		   unsigned int inst,
		   unsigned int phy_mask,
		   unsigned int mlink_mask,
		   unsigned int mdio_bus_freq,
		   unsigned int mdio_clock_freq, unsigned int verbose);

/* Set PHY mode - autonegotiation or any other */
void emac_mdio_set_phy_mode(unsigned int phy_mode);

/* Get linked status - check if link is on - 1=link on, 0=link off */
int emac_mdio_is_linked(void);

/* Get speed - 10 / 100 Mbps */
int emac_mdio_get_speed(void);

/* Get duplex - 2=full duplex, 1=half duplex */
int emac_mdio_get_duplex(void);

/* Get Phy number/address */
int emac_mdio_get_phy_num(void);

/* Check if loopback enabled on phy */
int emac_mdio_is_loopback(void);

/* Read from a phy register via mdio interface */
unsigned int emac_mdio_read(unsigned int phy_addr, unsigned int phy_reg);

/* Write to a phy register via mdio interface */
void emac_mdio_write(unsigned int phy_addr, unsigned int phy_reg,
		     unsigned int phy_data);

/* MDIO tick function - to be called every 10 mSecs */
int emac_mdio_tick(void);

#endif				/* _DAVINIC_EMAC_PHY_H_ */
