/*
 * arch/arm/mach-ns2816/include/mach/board-ns2816.h
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

#ifndef __ASM_ARCH_BOARD_NS2816_H
#define __ASM_ARCH_BOARD_NS2816_H

#include <mach/platform.h>

/*
 * On Chip AHB Accessd IPs-Debug APB
 */
#define NS2816_DEBUG_ROM		0x04000000
#define NS2816_ETB			0x04001000
#define NS2816_CTI			0x04002000
#define NS2816_IPIU			0x04003000
#define NS2816_FUNNEL			0x04004000
#define NS2816_ITM			0x04005000
#define NS2816_SWO			0x04006000
#define NS2816_A9_DEBUG_ROM		0x04100000
#define NS2816_COREDBG0			0x04110000
#define NS2816_PMU0			0x04111000
#define NS2816_COREDBG1			0x04112000
#define NS2816_PMU1			0x04113000
#define NS2816_CTI0			0x04118000
#define NS2816_CTI1			0x04119000
#define NS2816_PTM0			0x0411c000
#define NS2816_PTM1			0x0411d000

/*
 * Peripheral addresses 
 */
/* On Chip AXI Accessed IPs */
#define NS2816_AXI_MAIN_BASE		0x05000000	/* AXI MAIN	*/
#define NS2816_AXI_SUB_BASE		0x05010000	/* AXI SUB	*/
#define NS2816_DMA_330_BASE		0x05020000	/* DMA 330	*/	
#define NS2816_DMA_330S_BASE		0x05021000	/* DMA 330 secure */
/* TZASC -- TrustZone Address Space Controller */
#define NS2816_TZASC0_BASE		0x05022000	/* TZASC 0	*/
#define NS2816_TZASC1_BASE		0x05023000	/* TZASC 1	*/
#define NS2816_TZASC2_BASE		0x05024000	/* TZASC 2	*/
#define NS2816_TZASC3_BASE		0x05025000	/* TZASC 3	*/
#define NS2816_TZASC4_BASE		0x05026000	/* TZASC 4	*/
#define NS2816_MALI400_BASE		0x05030000	/* MALI 400	*/


/* On Chip AHB Main Accessed IPs */
#define NS2816_MEMCTL0_BASE		0x05050000	/* memory controller 0 */
#define NS2816_MEMCTL1_BASE		0x05060000	/* memory controller 1 */
#define NS2816_SDMMC_BASE		0x05070000	/* SD/MMC controller */
#define NS2816_USB_OHCI_BASE		0x05080000	/* USB OHCI */
#define NS2816_USB_EHCI_BASE		0x05090000	/* USB EHCI */
#define NS2816_ETH_BASE			0x050a0000	/* Ethernet */
#define NS2816_SATA_BASE		0x050b0000	/* SATA  controller*/
#define NS2816_VPU_BASE			0x050c0000	/* VPU */
#define NS2816_LCDC_BASE		0x050d0000	/* LCDC */
#define NS2816_DDR_CFG_BASE		0x050e0000	/* DDR CFG */

/* On Chip APB Accessed IPs */
#define NS2816_TIMER_BASE		0x05100000	/* TIMER */
#define NS2816_SSI0_BASE		0x05110000	/* SSI 0 */
#define NS2816_I2C0_BASE		0x05120000	/* I2C 0 */
#define NS2816_GPIO_BASE		0x05130000	/* GPIO port */
#define NS2816_I2S_BASE			0x05140000	/* I2S */
#define NS2816_UART0_BASE		0x05150000	/* UART 0 */
#define NS2816_UART1_BASE		0x05160000	/* UART 1 */
#define NS2816_SSI1_BASE		0x05170000	/* SSI 1 */
#define NS2816_I2C2_BASE		0x05180000	/* I2C 2 */
#define NS2816_I2C1_BASE		0x05190000	/* I2C 1 */
#define NS2816_LPC_BASE			0x051a0000	/* LPC */
#define NS2816_PRCM_BASE		0x051b0000	/* Power, Reset and Clock Management */
#define NS2816_SCM_BASE			0x051c0000	/* System Control Management */
#define NS2816_RTC_BASE			0x051d0000	/* Real Time Clock */
#define NS2816_RAP_BASE			0x051e0000	/* RAP */

/* On Chip AHB_Sub Accessed IPs */
#define NS2816_AHBARBITER_BASE		0x05200000	/* AHB Arbiter */
#define NS2816_DMAC_BASE		0x05210000	/* Direct Memory Access (DMA) Controller */

#define NS2816_LT_BASE			0xC0000000	/* Logic Tile expansion */
#define NS2816_SDRAM_BASE		0x80000000	/* SDRAM bank 6 256MB */

/* Osprey */

#define NS2816_SCU_BASE		0x05040000	/* SCU registers */
//#define NS2816_L220_BASE		0x05042000 	/* L220 registers */
//#define NS2816_SCU_BASE			0x1F000000	/* SCU registers */
 /* Private Generic interrupt controller CPU interface */
#define NS2816_GIC_CPU_BASE		(NS2816_SCU_BASE + 0x100)
#define NS2816_GTIMER_BASE		(NS2816_SCU_BASE + 0x200)	/* Global timer */ 
#define NS2816_TWD_BASE			(NS2816_SCU_BASE + 0x600)	/* Private timers and watchdogs */
#define NS2816_TWD_PERCPU_BASE		(NS2816_SCU_BASE + 0x700)
#define NS2816_TWD_SIZE			0x00000100
/* Private Generic interrupt controller distributor */
#define NS2816_GIC_DIST_BASE		(NS2816_SCU_BASE + 0x1000)      
#define NS2816_L220_BASE		(NS2816_SCU_BASE + 0x2000)	/* L220 registers , PL310 base*/

#define NS2816_IRAM_BASE                (0x00000000)
#define NS2816_IRAM_CODE_AREA           (NS2816_IRAM_BASE + SZ_4K)
 
#define CONTEXT_SIZE_WORDS                                      1024
/*
 * Core tile identification (REALVIEW_SYS_PROCID)
 */
#define NS2816_PROC_MASK          0xFF000000
#define NS2816_PROC_MPCOREA9      0x00000000

#endif	/* __ASM_ARCH_BOARD_PBX_H */
