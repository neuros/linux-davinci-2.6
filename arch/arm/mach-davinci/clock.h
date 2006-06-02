/*
 *  linux/arch/arm/mach-davinci/clock.h
 *
 *  BRIEF MODULE DESCRIPTION
 *      TI DAVINCI clock definitions
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


#ifndef __ARCH_ARM_DAVINCI_CLOCK_H
#define __ARCH_ARM_DAVINCI_CLOCK_H

struct module;

struct clk {
	struct list_head	node;
	struct module		*owner;
	const char		*name;
	unsigned int		*rate;
	__s8			usecount;
	__u8			flags;
	__u8			lpsc;
};


/* Clock flags */
#define RATE_CKCTL		1
#define RATE_FIXED		2
#define RATE_PROPAGATES		4
#define VIRTUAL_CLOCK		8
#define ALWAYS_ENABLED		16
#define ENABLE_REG_32BIT	32

int clk_register(struct clk *clk);
void clk_unregister(struct clk *clk);
int clk_init(void);
void board_setup_psc(unsigned int domain, unsigned int id, char enable);
#endif
