/*
 *  linux/include/asm-arm/arch-davinci/memory.h
 *
 *  BRIEF MODULE DESCRIPTION
 *      DAVINCI Virtual memory definitions
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

#ifndef __ASM_ARCH_MEMORY_H
#define __ASM_ARCH_MEMORY_H

/**************************************************************************
 * Included Files
 **************************************************************************/

#include <linux/config.h>
#include <asm/page.h>
#include <asm/sizes.h>
#include <asm/arch/hardware.h>

/**************************************************************************
 * Definitions
 **************************************************************************/
#define DAVINCI_DDR_BASE    0x80000000

#define DAVINCI_IRAM_BASE   0x00008000 /* ARM Internal RAM (Data) */
#define DAVINCI_IRAM_VIRT   0xe1400000 /* after 4M of IO space (io.h) */

/* Linux Memory Pool */

/* The start of physical memory available to the kernel. This value
 * is used in the arch-specific bootup code like setup_arch &
 * bootmem_init.  This may or may not be the start of physical memory;
 * There may be memory reserved "in front" of the kernel for other
 * purposes.
 */

#define PHYS_OFFSET DAVINCI_DDR_BASE

/*
 * Increase size of DMA-consistent memory region
 */
#define CONSISTENT_DMA_SIZE (14<<20)

#ifndef __ASSEMBLY__
/*
 * Restrict DMA-able region to workaround silicon bug.  The bug
 * restricts buffers available for DMA to video hardware to be
 * below 128M
 */
static inline void
__arch_adjust_zones(int node, unsigned long *size, unsigned long *holes)
{
	unsigned int sz = (128<<20) >> PAGE_SHIFT;

	if (node != 0)
		sz = 0;

	size[1] = size[0] - sz;
	size[0] = sz;
}

#define arch_adjust_zones(node, zone_size, holes) \
        if ((meminfo.bank[0].size >> 20) > 128) __arch_adjust_zones(node, zone_size, holes)

#define ISA_DMA_THRESHOLD	(PHYS_OFFSET + (128<<20) - 1)

#endif

/*
 * Bus address is physical address
 */
#define __virt_to_bus(x)	__virt_to_phys(x)
#define __bus_to_virt(x)	__phys_to_virt(x)

#endif /* __ASM_ARCH_MEMORY_H */
