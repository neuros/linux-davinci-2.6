/*
 *  linux/include/asm-arm/arch-davinci/hardware.h
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
#ifndef __ASM_ARCH_HARDWARE_H
#define __ASM_ARCH_HARDWARE_H

/**************************************************************************
 * Included Files
 **************************************************************************/

#include <linux/config.h>
#include <asm/sizes.h>
#include <asm/memory.h>
#include <asm/arch/io.h>

/*
 * Base register addresses
 */
#define DAVINCI_DMA_3PCC_BASE              (0x01C00000)
#define DAVINCI_DMA_3PTC0_BASE             (0x01C10000)
#define DAVINCI_DMA_3PTC1_BASE             (0x01C10400)
#define DAVINCI_UART0_BASE                 (0x01C20000)
#define DAVINCI_UART1_BASE                 (0x01C20400)
#define DAVINCI_UART2_BASE                 (0x01C20800)
#define DAVINCI_I2C_BASE                   (0x01C21000)
#define DAVINCI_TIMER0_BASE                (0x01C21400)
#define DAVINCI_TIMER1_BASE                (0x01C21800)
#define DAVINCI_WDOG_BASE                  (0x01C21C00)
#define DAVINCI_PWM0_BASE                  (0x01C22000)
#define DAVINCI_PWM1_BASE                  (0x01C22400)
#define DAVINCI_PWM2_BASE                  (0x01C22800)
#define DAVINCI_SYSTEM_MODULE_BASE         (0x01C40000)
#define DAVINCI_PLL_CNTRL0_BASE            (0x01C40800)
#define DAVINCI_PLL_CNTRL1_BASE            (0x01C40C00)
#define DAVINCI_PWR_SLEEP_CNTRL_BASE       (0x01C41000)
#define DAVINCI_SYSTEM_DFT_BASE            (0x01C42000)
#define DAVINCI_ARM_INTC_BASE              (0x01C48000)
#define DAVINCI_IEEE1394_BASE              (0x01C60000)
#define DAVINCI_USB_OTG_BASE               (0x01C64000)
#define DAVINCI_CFC_ATA_BASE               (0x01C66000)
#define DAVINCI_SPI_BASE                   (0x01C66800)
#define DAVINCI_GPIO_BASE                  (0x01C67000)
#define DAVINCI_UHPI_BASE                  (0x01C67800)
#define DAVINCI_VPSS_REGS_BASE             (0x01C70000)
#define DAVINCI_EMAC_CNTRL_REGS_BASE       (0x01C80000)
#define DAVINCI_EMAC_WRAPPER_CNTRL_REGS_BASE   (0x01C81000)
#define DAVINCI_EMAC_WRAPPER_RAM_BASE      (0x01C82000)
#define DAVINCI_MDIO_CNTRL_REGS_BASE       (0x01C84000)
#define DAVINCI_IMCOP_BASE                 (0x01CC0000)
#define DAVINCI_ASYNC_EMIF_CNTRL_BASE      (0x01E00000)
#define DAVINCI_VLYNQ_BASE                 (0x01E01000)
#define DAVINCI_MCBSP_BASE                 (0x01E02000)
#define DAVINCI_MMC_SD_BASE                (0x01E10000)
#define DAVINCI_MS_BASE                    (0x01E20000)
#define DAVINCI_ASYNC_EMIF_DATA_CE0_BASE   (0x02000000)
#define DAVINCI_ASYNC_EMIF_DATA_CE1_BASE   (0x04000000)
#define DAVINCI_ASYNC_EMIF_DATA_CE2_BASE   (0x06000000)
#define DAVINCI_ASYNC_EMIF_DATA_CE3_BASE   (0x08000000)
#define DAVINCI_VLYNQ_REMOTE_BASE          (0x0C000000)

/* Power and Sleep Controller (PSC) Domains */
#define DAVINCI_GPSC_ARMDOMAIN      0
#define DAVINCI_GPSC_DSPDOMAIN      1

#define DAVINCI_LPSC_VPSSMSTR       0       // VPSS Master LPSC
#define DAVINCI_LPSC_VPSSSLV        1       // VPSS Slave LPSC
#define DAVINCI_LPSC_TPCC           2       // TPCC LPSC
#define DAVINCI_LPSC_TPTC0          3       // TPTC0 LPSC
#define DAVINCI_LPSC_TPTC1          4       // TPTC1 LPSC
#define DAVINCI_LPSC_EMAC           5       // EMAC LPSC
#define DAVINCI_LPSC_EMAC_WRAPPER   6       // EMAC WRAPPER LPSC
#define DAVINCI_LPSC_MDIO           7       // MDIO LPSC
#define DAVINCI_LPSC_IEEE1394       8       // IEEE1394 LPSC
#define DAVINCI_LPSC_USB            9       // USB LPSC
#define DAVINCI_LPSC_ATA            10      // ATA LPSC
#define DAVINCI_LPSC_VLYNQ          11      // VLYNQ LPSC
#define DAVINCI_LPSC_UHPI           12      // UHPI LPSC
#define DAVINCI_LPSC_DDR_EMIF       13      // DDR_EMIF LPSC
#define DAVINCI_LPSC_AEMIF          14      // AEMIF LPSC
#define DAVINCI_LPSC_MMC_SD         15      // MMC_SD LPSC
#define DAVINCI_LPSC_MEMSTICK       16      // MEMSTICK LPSC
#define DAVINCI_LPSC_McBSP          17      // McBSP LPSC
#define DAVINCI_LPSC_I2C            18      // I2C LPSC
#define DAVINCI_LPSC_UART0          19      // UART0 LPSC
#define DAVINCI_LPSC_UART1          20      // UART1 LPSC
#define DAVINCI_LPSC_UART2          21      // UART2 LPSC
#define DAVINCI_LPSC_SPI            22      // SPI LPSC
#define DAVINCI_LPSC_PWM0           23      // PWM0 LPSC
#define DAVINCI_LPSC_PWM1           24      // PWM1 LPSC
#define DAVINCI_LPSC_PWM2           25      // PWM2 LPSC
#define DAVINCI_LPSC_GPIO           26      // GPIO LPSC
#define DAVINCI_LPSC_TIMER0         27      // TIMER0 LPSC
#define DAVINCI_LPSC_TIMER1         28      // TIMER1 LPSC
#define DAVINCI_LPSC_TIMER2         29      // TIMER2 LPSC
#define DAVINCI_LPSC_SYSTEM_SUBSYS  30      // SYSTEM SUBSYSTEM LPSC
#define DAVINCI_LPSC_ARM            31      // ARM LPSC
#define DAVINCI_LPSC_SCR2           32      // SCR2 LPSC
#define DAVINCI_LPSC_SCR3           33      // SCR3 LPSC
#define DAVINCI_LPSC_SCR4           34      // SCR4 LPSC
#define DAVINCI_LPSC_CROSSBAR       35      // CROSSBAR LPSC
#define DAVINCI_LPSC_CFG27          36      // CFG27 LPSC
#define DAVINCI_LPSC_CFG3           37      // CFG3 LPSC
#define DAVINCI_LPSC_CFG5           38      // CFG5 LPSC
#define DAVINCI_LPSC_GEM            39      // GEM LPSC
#define DAVINCI_LPSC_IMCOP          40      // IMCOP LPSC

#endif /* __ASM_ARCH_HARDWARE_H */
