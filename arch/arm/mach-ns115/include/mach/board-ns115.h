/*
 * arch/arm/mach-ns115/include/mach/board-ns115.h
 *
 * Copyright (C) 2010 NUFRONT Limited
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 */

#ifndef __ASM_ARCH_BOARD_NS115_H
#define __ASM_ARCH_BOARD_NS115_H

#include <mach/platform.h>

/*
 * Internal Memory
 */
#define NS115_BOOTROM_BASE              0x00000000 /* BootROM 32K */
#define NS115_ROMCTRL_BASE              0x03000000 /* ROM CTRL 64K */

/*
 * External Memory
 */
#define NS115_DDR_BASE                  0x10000000 /* LPDDR2/DDR3 4G */

/*
 * Debug APB
 */
#define NS115_DEBUG_ROM                 0x04000000
#define NS115_ETB                       0x04001000
#define NS115_CTI                       0x04002000
#define NS115_IPIU                      0x04003000
#define NS115_FUNNEL                    0x04004000
#define NS115_ITM                       0x04005000
#define NS115_SWO                       0x04006000
#define NS115_A9_DEBUG_ROM              0x04100000
#define NS115_COREDBG0                  0x04110000
#define NS115_PMU0                      0x04111000
#define NS115_COREDBG1                  0x04112000
#define NS115_PMU1                      0x04113000
#define NS115_CTI0                      0x04118000
#define NS115_CTI1                      0x04119000
#define NS115_PTM0                      0x0411C000
#define NS115_PTM1                      0x0411D000

/*
 * ARM MPU
 */
#define NS115_SCU_BASE                  0x05040000 /* MPCore-A9 8K */
#define NS115_GIC_CPU_BASE              (NS115_SCU_BASE + 0x100)
#define NS115_GTIMER_BASE               (NS115_SCU_BASE + 0x200)
#define NS115_TWD_BASE                  (NS115_SCU_BASE + 0x600)
#define NS115_TWD_SIZE                  0x00000100
#define NS115_TWD_PERCPU_BASE           (NS115_SCU_BASE + 0x700)
#define NS115_GIC_DIST_BASE             (NS115_SCU_BASE + 0x1000)      
#define NS115_L220_BASE                 0x05042000

/*
 * L4_CFG(0x05000000 ~ 0x05800000)
 */
#define NS115_DISPCTR0_BASE             0x05010000
#define NS115_DISPCTR1_BASE             0x05011000
#define NS115_DMA_330_BASE              0x05020000
#define NS115_DMA_330S_BASE             0x05021000
#define NS115_MALI_BASE                 0x05030000 /* MALI 64K */
#define NS115_SDMMC_BASE                0x05070000 /* SD/MMC 64K */
#define NS115_USB_OHCI_BASE             0x05080000 /* USB OHCI 64K */
#define NS115_USB_EHCI_BASE             0x05090000 /* USB EHCI 64K */
#define NS115_USB_OTG_BASE              0x050A0000 /* USB OTG 64K */
#define NS115_ON2_DEC_BASE              0x050C0000 /* ON2 Decoder 64K */
#define NS115_ON2_ENC_BASE              0x050D0000 /* ON2 Encoder 64K */
#define NS115_DRAMCTRL_BASE             0x050E0000 /* DRAM controller 4K */
#define NS115_HDMI_BASE                 0x05110000 /* HDMI 64K */
#define NS115_GC300_BASE                0x05140000 /* GC300 256K */
#define NS115_CAMERIC_BASE              0x05180000 /* CamerIC 128K */

/*
 * L4_CFG_WAKEUP(0x05800000 ~ 0x05A00000)
 */
#define NS115_SARRAM_BASE               0x05800000 /* SARRAM 4K */
#define NS115_SARRAM_CTRL               0x05820000 /* SARRAM controller 4K */
#define NS115_PRCM_BASE                 0x05821000 /* PRCM 4K */
#define NS115_SCM_BASE                  0x05822000 /* SCM 4K */
#define NS115_GPIO0_BASE                0x05823000 /* GPIO 0 4K */
#define NS115_TIMER0_BASE               0x05824000 /* Timer 0 4K*/

/*
 * L4_PER(0x06000000 ~ 0x07000000)
 */
#define NS115_I2S0_BASE                 0x06060000 /* I2S 0 4K */
#define NS115_IISFIFO                   0x0800
#define NS115_I2S1_BASE                 0x06061000 /* I2S 1 4K */
#define NS115_UART0_BASE                0x06090000 /* UART 0 64K */
#define NS115_UART1_BASE                0x060A0000 /* UART 1 64K */
#define NS115_UART2_BASE                0x060B0000 /* UART 2 64K */
#define NS115_UART3_BASE                0x060C0000 /* UART 3 64K */
#define NS115_SPI0_BASE                 0x060D0000 /* SPI 0 64K */
#define NS115_SPI1_BASE                 0x060E0000 /* SPI 1 64K */
#define NS115_SPI2_BASE                 0x060F0000 /* SPI 2 64K */
#define NS115_I2C0_BASE                 0x06100000 /* I2c 0 64K */
#define NS115_I2C1_BASE                 0x06110000 /* I2c 1 64K */
#define NS115_I2C2_BASE                 0x06120000 /* I2c 2 64K */
#define NS115_I2C3_BASE                 0x06130000 /* I2c 3 64K */
#define NS115_GPIO1_BASE                0x06140000 /* GPIO 1 64K */
#define NS115_TIMER1_BASE               0x06150000 /* Timer 1 64K */
#define NS115_AUX_BASE                  0x06160000 /* AUX 4K */
#define NS115_SPI3_BASE                 0x06170000 /* SPI 3 64K */

/*
 * L3(0x07000000 ~ 0x08000000)
 */
#define NS115_ZSPMEM_BASE               0x07000000 /* ZSPMEM 512K */
#define NS115_ISS_CFG_BASE              0x07100000 /* L3 ISS config 64K */
#define NS115_L3_ISS_TA_BASE		0x07205C00 /* L3 ta_l3_iss.ta 1K */
#define NS115_CAMERIC_TA_BASE		0x05407C00 /* L4_CFG cameric.ta 1K */

/*
 * L3M(0x08000000 ~ 0x09000000)
 */
#define NS115_MEM_MAX0_BASE             0x08008000 /* MemMax0 1K */

#define CONTEXT_SIZE_WORDS              1024
#endif /*__ASM_ARCH_BOARD_NS115_H*/
