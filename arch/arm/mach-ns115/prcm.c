/*
 *  linux/arch/arm/mach-ns115/prcm.c
 *
 *  Copyright (C) 2010 NUFRONT Limited.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/list.h>
#include <linux/errno.h>
#include <linux/err.h>
#include <linux/string.h>
#include <linux/cnt32_to_63.h>
#include <linux/clk.h>
#include <linux/mutex.h>
#include <linux/clkdev.h>
#include <asm/delay.h>
#include <linux/io.h>

#include <mach/hardware.h>
#include <mach/board-ns115.h>
#include <mach/platform.h>

#include "clock.h"
#include "prcm.h"


#define CLK_NORMAL	1000000000
#define CLK_HIGH	1200000000
#define CLK_ULTRA	1500000000

/* SECTION 1: Common Tasks Definition */
void reg_clr_bits(void __iomem *addr, unsigned int clr)
{
	unsigned int reg_data = __raw_readl(addr);
	reg_data &= ~(clr);
	__raw_writel(reg_data, addr);
}

void reg_set_bits(void __iomem *addr, unsigned int set)
{
	unsigned reg_data = __raw_readl(addr);
	reg_data |= (set);
	__raw_writel(reg_data, addr);
}

void sys_pwr_ctrl_set_bits(unsigned int set, unsigned int status) 
{
	void __iomem *sys_pwr_ctrl = __io_address(PRCM_SYS_PWR_CTRL);
	void __iomem *sys_pwr_status = __io_address(PRCM_SYS_PWR_STATUS);
	unsigned int sys_pwr_status_reg = 0;

	// Set System Power Control Bits
	reg_set_bits(sys_pwr_ctrl, set);

	// Wait until Power Status is ALIVE
	while ((sys_pwr_status_reg & status) == 0) {
		udelay(100);
		sys_pwr_status_reg = __raw_readl(sys_pwr_status);
	}
}

void sys_pwr_ctrl_clr_bits(unsigned int clr, unsigned int status) 
{
	void __iomem *sys_pwr_ctrl = __io_address(PRCM_SYS_PWR_CTRL);
	void __iomem *sys_pwr_status = __io_address(PRCM_SYS_PWR_STATUS);
	unsigned int sys_pwr_status_reg = 0;

	// Clear System Power Control Bits
	reg_clr_bits(sys_pwr_ctrl, clr);

	// Wait until Power Status is NOT ALIVE
	sys_pwr_status_reg = __raw_readl(sys_pwr_status);
	while ((sys_pwr_status_reg & status) != 0) {
		udelay(100);
		sys_pwr_status_reg = __raw_readl(sys_pwr_status);
	}
}

int get_offset_pos(unsigned int mask) 
{
	int offset;
	for(offset=0; offset<32; offset++) {
		if(((0x01 << offset) & mask) != 0) break;
	}

	return offset;
}

void clk_set_rate_bits(void __iomem *addr, unsigned int set_en, 
		unsigned int set_mask, unsigned int set_bits)
{
	int set_offset_pos;

	// Clear Enable Bits
	reg_clr_bits(addr, set_en);

	// Set New Bits
	set_offset_pos = get_offset_pos(set_mask);
	set_bits  <<= set_offset_pos;
	reg_clr_bits(addr, set_mask);
	reg_set_bits(addr, set_bits);

	// Set Enable Bits
	reg_set_bits(addr, set_en);
}



/* SECTION 2: Clock Structure Definition */
static struct clk pll0_clk = {
	.enabled  = 0,
	.rate     = 800000000,
	.set_rate = ns115_pll0_clk_set_rate,
	.get_rate = clk_get_rate, 
};

static struct clk pll1_clk = {
	.enabled  = 0,
	.rate     = 666000000,
	.set_rate = ns115_pll1_clk_set_rate,
	.get_rate = clk_get_rate, 
};

static struct clk pll2_clk = {
	.enabled  = 0,
	.rate     = 800000000,
	.set_rate = ns115_pll2_clk_set_rate,
	.get_rate = clk_get_rate, 
};

static struct clk pll3_clk = {
	.enabled  = 0,
	.rate     = 297000000,
	.set_rate = ns115_pll3_clk_set_rate,
	.get_rate = clk_get_rate, 
};

static struct clk pll4_clk = {
	.enabled  = 0,
	.rate     = 297000000,
	.set_rate = ns115_pll4_clk_set_rate,
	.get_rate = clk_get_rate, 
};

static struct clk pll5_clk = {
	.enabled  = 0,
	.rate     = 984000000,
	.set_rate = ns115_pll5_clk_set_rate,
	.get_rate = clk_get_rate, 
};

static struct clk pll6_clk = {
	.enabled  = 0,
	.rate     = 480000000,
	.set_rate = NULL,
	.get_rate = clk_get_rate, 
};

static struct clk coresight_clk = {
	.enabled  = 0,
	.rate	  = 200000000,
	.enable   = ns115_coresight_clk_enable,
	.disable  = ns115_coresight_clk_disable,
	.set_rate = ns115_coresight_clk_set_rate,
	.get_rate = clk_get_rate, 
};

static struct clk ddr_clk = {
	.enabled  = 0,
	.rate	  = 333333333,
	.enable   = ns115_ddr_clk_enable,
	.disable  = ns115_ddr_clk_disable,
	.set_rate = ns115_ddr_clk_set_rate,
	.get_rate = clk_get_rate, 
};

static struct clk pl330_dma_clk = {
	.enabled  = 0,
	.rate	  = 83000000,
	.enable   = ns115_pl330dma_clk_enable,
	.disable  = ns115_pl330dma_clk_disable,
	.set_rate = NULL,
	.get_rate = clk_get_rate, 
};

static struct clk mali_clk = {
	.enabled  = 0,
	.rate	  = 240000000,
	.enable   = ns115_mali_clk_enable,
	.disable  = ns115_mali_clk_disable,
	.set_rate = ns115_mali_clk_set_rate,
	.get_rate = clk_get_rate, 
};

static struct clk gc300_clk = {
	.enabled  = 0,
	.rate	  = 400000000,
	.enable   = ns115_gc300_clk_enable,
	.disable  = ns115_gc300_clk_disable,
	.set_rate = ns115_gc300_clk_set_rate,
	.get_rate = clk_get_rate, 
};

static struct clk vpu_g1_clk = {
	.enabled  = 0,
	.rate	  = 200000000,
	.enable   = ns115_vpu_g1_clk_enable,
	.disable  = ns115_vpu_g1_clk_disable,
	.set_rate = ns115_vpu_g1_clk_set_rate,
	.get_rate = clk_get_rate, 
};

static struct clk vpu_h1_clk = {
	.enabled  = 0,
	.rate	  = 240000000,
	.enable   = ns115_vpu_h1_clk_enable,
	.disable  = ns115_vpu_h1_clk_disable,
	.set_rate = ns115_vpu_h1_clk_set_rate,
	.get_rate = clk_get_rate, 
};

static struct clk isp_jpeg_clk = {
	.enabled  = 0,
	.rate	  = 200000000,
	.enable   = ns115_isp_jpeg_clk_enable,
	.disable  = ns115_isp_jpeg_clk_disable,
	.set_rate = ns115_isp_jpeg_clk_set_rate,
	.get_rate = clk_get_rate, 
};

static struct clk isp_clk = {
	.enabled  = 0,
	.rate	  = 166500000,
	.enable   = ns115_isp_clk_enable,
	.disable  = ns115_isp_clk_disable,
	.set_rate = NULL,
	.get_rate = clk_get_rate, 
};

static struct clk sd_clk = {
	.enabled  = 0,
	.rate	  = 200000000,
	.enable   = ns115_sd_clk_enable,
	.disable  = ns115_sd_clk_disable,
	.set_rate = ns115_sd_clk_set_rate,
	.get_rate = clk_get_rate, 
};

static struct clk zsp_clk = {
	.enabled  = 0,
	.rate	  = 166500000,
	.enable   = ns115_zsp_clk_enable,
	.disable  = ns115_zsp_clk_disable,
	.set_rate = NULL,
	.get_rate = clk_get_rate, 
};

static struct clk spi0_clk = {
	.enabled  = 0,
	.rate	  = 50000000,
	.enable   = ns115_spi0_clk_enable,
	.disable  = ns115_spi0_clk_disable,
	.set_rate = ns115_spi01_clk_set_rate,
	.get_rate = clk_get_rate, 
};

static struct clk spi1_clk = {
	.enabled  = 0,
	.rate	  = 50000000,
	.enable   = ns115_spi1_clk_enable,
	.disable  = ns115_spi1_clk_disable,
	.set_rate = ns115_spi01_clk_set_rate,
	.get_rate = clk_get_rate, 
};

static struct clk spi2_clk = {
	.enabled  = 0,
	.rate	  = 400000000,
	.enable   = ns115_spi2_clk_enable,
	.disable  = ns115_spi2_clk_disable,
	.set_rate = ns115_spi23_clk_set_rate,
	.get_rate = clk_get_rate, 
};

static struct clk spi3_clk = {
	.enabled  = 0,
	.rate	  = 400000000,
	.enable   = ns115_spi3_clk_enable,
	.disable  = ns115_spi3_clk_disable,
	.set_rate = ns115_spi23_clk_set_rate,
	.get_rate = clk_get_rate, 
};

static struct clk i2c0_clk = {
	.enabled  = 0,
	.rate	  = 100000000,
	.enable   = ns115_i2c0_clk_enable,
	.disable  = ns115_i2c0_clk_disable,
	.set_rate = ns115_i2c_fs_clk_set_rate,
	.get_rate = clk_get_rate, 
	.reset	  = ns115_i2c0_clk_reset,
};

static struct clk i2c1_clk = {
	.enabled  = 0,
	.rate	  = 100000000,
	.enable   = ns115_i2c1_clk_enable,
	.disable  = ns115_i2c1_clk_disable,
	.set_rate = ns115_i2c_fs_clk_set_rate,
	.get_rate = clk_get_rate, 
	.reset    = ns115_i2c1_clk_reset,
};

static struct clk i2c2_clk = {
	.enabled  = 0,
	.rate	  = 100000000,
	.enable   = ns115_i2c2_clk_enable,
	.disable  = ns115_i2c2_clk_disable,
	.set_rate = ns115_i2c_fs_clk_set_rate,
	.get_rate = clk_get_rate, 
	.reset	  = ns115_i2c2_clk_reset,
};

static struct clk i2c3_clk = {
	.enabled  = 0,
	.rate	  = 133333333,
	.enable   = ns115_i2c3_clk_enable,
	.disable  = ns115_i2c3_clk_disable,
	.set_rate = ns115_i2c_hs_clk_set_rate,
	.get_rate = clk_get_rate,
	.reset	  = ns115_i2c3_clk_reset,
};

static struct clk uart0_clk = {
	.enabled  = 0,
	.rate	  = 66666666,
	.enable   = ns115_uart0_clk_enable,
	.disable  = ns115_uart0_clk_disable,
	.set_rate = ns115_uart01_clk_set_rate,
	.get_rate = clk_get_rate, 
};

static struct clk uart1_clk = {
	.enabled  = 0,
	.rate	  = 66666666,
	.enable   = ns115_uart1_clk_enable,
	.disable  = ns115_uart1_clk_disable,
	.set_rate = ns115_uart01_clk_set_rate,
	.get_rate = clk_get_rate, 
};

static struct clk uart2_clk = {
	.enabled  = 0,
	.rate	  = 66666666,
	.enable   = ns115_uart2_clk_enable,
	.disable  = ns115_uart2_clk_disable,
	.set_rate = ns115_uart2_clk_set_rate,
	.get_rate = clk_get_rate, 
};

static struct clk uart3_clk = {
	.enabled  = 0,
	.rate	  = 66666666,
	.enable   = ns115_uart3_clk_enable,
	.disable  = ns115_uart3_clk_disable,
	.set_rate = ns115_uart3_clk_set_rate,
	.get_rate = clk_get_rate, 
};

static struct clk rgbo_clk = {
	.enabled  = 0,
	.rate	  = 148500000,
	.enable   = ns115_rgbo_clk_enable,
	.disable  = ns115_rgbo_clk_disable,
	.set_rate = ns115_pxl0_clk_set_rate,
	.get_rate = clk_get_rate, 
};

static struct clk lcdc0_clk = {
	.enabled  = 0,
	.rate	  = 148500000,
	.enable   = ns115_lcdc0_clk_enable,
	.disable  = ns115_lcdc0_clk_disable,
	.set_rate = ns115_pxl0_clk_set_rate,
	.get_rate = clk_get_rate, 
};

static struct clk lcdc0_bus_clk = {
	.enabled  = 0,
	.enable   = ns115_lcdc0_bus_clk_enable,
	.disable  = ns115_lcdc0_bus_clk_disable,
};


static struct clk lcdc1_clk = {
	.enabled  = 0,
	.rate	  = 148500000,
	.enable   = ns115_lcdc1_clk_enable,
	.disable  = ns115_lcdc1_clk_disable,
	.set_rate = ns115_pxl1_clk_set_rate,
	.get_rate = clk_get_rate, 
};

static struct clk lcdc1_bus_clk = {
	.enabled  = 0,
	.enable   = ns115_lcdc1_bus_clk_enable,
	.disable  = ns115_lcdc1_bus_clk_disable,
};

static struct clk hdmi_pxl_clk = {
	.enabled  = 0,
	.rate	  = 148500000,
	.enable   = ns115_hdmi_pxl_clk_enable,
	.disable  = ns115_hdmi_pxl_clk_disable,
	.set_rate = ns115_pxl1_clk_set_rate,
	.get_rate = clk_get_rate, 
};

static struct clk i2s0_clk = {
	.enabled  = 0,
	.rate	  = 98304000,
	.delay	  = 0,
	.enable   = ns115_i2s0_clk_enable,
	.disable  = ns115_i2s0_clk_disable,
	.set_rate = ns115_i2s_clk_set_rate,
	.get_rate = clk_get_rate, 
};

static struct clk mclk_clk = {
	.enabled  = 0,
	.enable   = ns115_mclk_clk_enable,
	.disable  = ns115_mclk_clk_disable,
	.get_rate = clk_get_rate,
};

static struct clk i2s1_clk = {
	.enabled  = 0,
	.rate	  = 98304000,
	.delay	  = 0,
	.enable   = ns115_i2s1_clk_enable,
	.disable  = ns115_i2s1_clk_disable,
	.set_rate = ns115_i2s_clk_set_rate,
	.get_rate = clk_get_rate, 
};

static struct clk usb_otg_clk = {
	.enabled  = 0,
	.rate	  = 12000000,
	.enable   = ns115_usb_otg_clk_enable,
	.disable  = ns115_usb_otg_clk_disable,
	.set_rate = NULL,
	.get_rate = clk_get_rate,
};

static struct clk usb_utmi_clk = {
	.enabled  = 0,
	.rate	  = 60000000,
	.enable   = ns115_usb_utmi_clk_enable,
	.disable  = ns115_usb_utmi_clk_disable,
	.set_rate = NULL,
	.get_rate = clk_get_rate,
};

static struct clk usb_ohci_clk = {
	.enabled  = 0,
	.rate	  = 48000000,
	.enable   = ns115_usb_ohci_clk_enable,
	.disable  = ns115_usb_ohci_clk_disable,
	.set_rate = NULL,
	.get_rate = clk_get_rate,
};

static struct clk usb_hsic_clk = {
	.enabled  = 0,
	.rate	  = 480000000,
	.enable   = ns115_usb_hsic_clk_enable,
	.disable  = ns115_usb_hsic_clk_disable,
	.set_rate = NULL,
	.get_rate = clk_get_rate,
};

static struct clk core_mem_clk = {
	.enabled  = 0,
	.rate	  = 83000000,
	.enable   = ns115_core_mem_clk_enable,
	.disable  = ns115_core_mem_clk_disable,
	.set_rate = NULL,
	.get_rate = clk_get_rate,
};

static struct clk gpio_core_clk = {
	.enabled  = 0,
	.rate	  = 41625000,
	.enable   = ns115_gpio_core_clk_enable,
	.disable  = ns115_gpio_core_clk_disable,
	.set_rate = NULL,
	.get_rate = clk_get_rate,
};

static struct clk timer0_core_clk = {
	.enabled  = 0,
	.rate	  = 12000000,
	.enable   = ns115_timer0_core_clk_enable,
	.disable  = ns115_timer0_core_clk_disable,
	.set_rate = ns115_timer0_core_clk_set_rate,
	.get_rate = clk_get_rate,
};

static struct clk timer1_core_clk = {
	.enabled  = 0,
	.rate	  = 12000000,
	.enable   = ns115_timer1_core_clk_enable,
	.disable  = ns115_timer1_core_clk_disable,
	.set_rate = ns115_timer1_core_clk_set_rate,
	.get_rate = clk_get_rate,
};

static struct clk timer2_core_clk = {
	.enabled  = 0,
	.rate	  = 12000000,
	.enable   = ns115_timer2_core_clk_enable,
	.disable  = ns115_timer2_core_clk_disable,
	.set_rate = ns115_timer2_core_clk_set_rate,
	.get_rate = clk_get_rate,
};

static struct clk timer3_core_clk = {
	.enabled  = 0,
	.rate	  = 12000000,
	.enable   = ns115_timer3_core_clk_enable,
	.disable  = ns115_timer3_core_clk_disable,
	.set_rate = ns115_timer3_core_clk_set_rate,
	.get_rate = clk_get_rate,
};

static struct clk timer4_core_clk = {
	.enabled  = 0,
	.rate	  = 12000000,
	.enable   = ns115_timer4_core_clk_enable,
	.disable  = ns115_timer4_core_clk_disable,
	.set_rate = ns115_timer4_core_clk_set_rate,
	.get_rate = clk_get_rate,
};

static struct clk timer5_core_clk = {
	.enabled  = 0,
	.rate	  = 12000000,
	.enable   = ns115_timer5_core_clk_enable,
	.disable  = ns115_timer5_core_clk_disable,
	.set_rate = ns115_timer5_core_clk_set_rate,
	.get_rate = clk_get_rate,
};

static struct clk timer6_core_clk = {
	.enabled  = 0,
	.rate	  = 12000000,
	.enable   = ns115_timer6_core_clk_enable,
	.disable  = ns115_timer6_core_clk_disable,
	.set_rate = ns115_timer6_core_clk_set_rate,
	.get_rate = clk_get_rate,
};

static struct clk timer7_core_clk = {
	.enabled  = 0,
	.rate	  = 12000000,
	.enable   = ns115_timer7_core_clk_enable,
	.disable  = ns115_timer7_core_clk_disable,
	.set_rate = ns115_timer7_core_clk_set_rate,
	.get_rate = clk_get_rate,
};

static struct clk wkup_mem_clk = {
	.enabled  = 0,
	.rate	  = 41625000,
	.enable   = ns115_wkup_mem_clk_enable,
	.disable  = ns115_wkup_mem_clk_disable,
	.set_rate = NULL,
	.get_rate = clk_get_rate,
};

static struct clk gpio_wkup_clk = {
	.enabled  = 0,
	.rate	  = 3200,
	.enable   = ns115_gpio_wkup_clk_enable,
	.disable  = ns115_gpio_wkup_clk_disable,
	.set_rate = ns115_gpio_wkup_clk_set_rate,
	.get_rate = clk_get_rate,
};

static struct clk timer0_wkup_clk = {
	.enabled  = 0,
	.rate	  = 12000000,
	.enable   = ns115_timer0_wkup_clk_enable,
	.disable  = ns115_timer0_wkup_clk_disable,
	.set_rate = ns115_timer0_wkup_clk_set_rate,
	.get_rate = clk_get_rate,
};

static struct clk timer1_wkup_clk = {
	.enabled  = 0,
	.rate	  = 12000000,
	.enable   = ns115_timer1_wkup_clk_enable,
	.disable  = ns115_timer1_wkup_clk_disable,
	.set_rate = ns115_timer1_wkup_clk_set_rate,
	.get_rate = clk_get_rate,
};

static struct clk alt0_clk = {
	.enabled  = 0,
	.rate	  = 12000000,
	.enable   = ns115_alt0_clk_enable,
	.disable  = ns115_alt0_clk_disable,
	.set_rate = ns115_alt0_clk_set_rate,
	.get_rate = clk_get_rate,
};

static struct clk alt1_clk = {
	.enabled  = 0,
	.rate	  = 12000000,
	.enable   = ns115_alt1_clk_enable,
	.disable  = ns115_alt1_clk_disable,
	.set_rate = ns115_alt1_clk_set_rate,
	.get_rate = clk_get_rate,
};

static struct clk aux_clk = {
	.enabled  = 0,
	.rate	  = 41625000,
	.enable   = ns115_aux_clk_enable,
	.disable  = ns115_aux_clk_disable,
	.set_rate = NULL,
	.get_rate = clk_get_rate,
};

static struct clk efuse_clk = {
	.enabled  = 0,
	.rate	  = 12000000,
	.enable   = ns115_efuse_clk_enable,
	.disable  = ns115_efuse_clk_disable,
	.set_rate = NULL,
	.get_rate = clk_get_rate,
};

static struct clk pwm_clk = {
	.enabled  = 0,
	.rate	  = 12000000,
	.enable   = ns115_pwm_clk_enable,
	.disable  = ns115_pwm_clk_disable,
	.set_rate = NULL,
	.get_rate = clk_get_rate,
};

static struct clk cpu_clk = {
	.enabled  = 1,
	.rate	  = 800000000,
	.set_rate = ns115_cpu_clk_set_rate,
	.get_rate = clk_get_rate,
};


int pll_calc_fout(int fout, int *pnf, int *pnr, int *pod, int *pnb);

/* SECTION 3: Clock Control Function/Tasks Definition */

// PLL configuration function/tasks
void pll_config_reg(void __iomem *base, unsigned int pll_nr, unsigned int pll_nf, unsigned int pll_od, unsigned int pll_nb) 
{
	unsigned int reg_value;

	// PLLn_CFG0
	reg_value = (((pll_nf-1)&0x1fff) << 8) | ((pll_nr-1)&0x3f);
	__raw_writel(reg_value, base);

	// PLLn_CFG1
	reg_value = (0x1<<18) | (((pll_nb-1)&0xfff)<<4) | ((pll_od -1)&0xf);
	__raw_writel(reg_value, (base+0x4));
}

int ns115_pll_get_param(unsigned long rate, unsigned int * pll_nr, unsigned int * pll_nf, unsigned int * pll_od, unsigned int * pll_nb) 
{
	switch (rate) {
		case 50000000:
			*pll_nr = PRCM_PLL_NR_50MHZ;
			*pll_nf = PRCM_PLL_NF_50MHZ;
			*pll_od = PRCM_PLL_OD_50MHZ;
			*pll_nb = PRCM_PLL_NB_50MHZ;
			break;
		case 200000000:
			*pll_nr = PRCM_PLL_NR_200MHZ;
			*pll_nf = PRCM_PLL_NF_200MHZ;
			*pll_od = PRCM_PLL_OD_200MHZ;
			*pll_nb = PRCM_PLL_NB_200MHZ;
			break;
		case 250000000:
			*pll_nr = PRCM_PLL_NR_250MHZ;
			*pll_nf = PRCM_PLL_NF_250MHZ;
			*pll_od = PRCM_PLL_OD_250MHZ;
			*pll_nb = PRCM_PLL_NB_250MHZ;
			break;
		case 297000000:
			*pll_nr = PRCM_PLL_NR_297MHZ;
			*pll_nf = PRCM_PLL_NF_297MHZ;
			*pll_od = PRCM_PLL_OD_297MHZ;
			*pll_nb = PRCM_PLL_NB_297MHZ;
			break;
		case 300000000:
			*pll_nr = PRCM_PLL_NR_300MHZ;
			*pll_nf = PRCM_PLL_NF_300MHZ;
			*pll_od = PRCM_PLL_OD_300MHZ;
			*pll_nb = PRCM_PLL_NB_300MHZ;
			break;
		case 350000000:
			*pll_nr = PRCM_PLL_NR_350MHZ;
			*pll_nf = PRCM_PLL_NF_350MHZ;
			*pll_od = PRCM_PLL_OD_350MHZ;
			*pll_nb = PRCM_PLL_NB_350MHZ;
			break;
		case 400000000:
			*pll_nr = PRCM_PLL_NR_400MHZ;
			*pll_nf = PRCM_PLL_NF_400MHZ;
			*pll_od = PRCM_PLL_OD_400MHZ;
			*pll_nb = PRCM_PLL_NB_400MHZ;
			break;
		case 450000000:
			*pll_nr = PRCM_PLL_NR_450MHZ;
			*pll_nf = PRCM_PLL_NF_450MHZ;
			*pll_od = PRCM_PLL_OD_450MHZ;
			*pll_nb = PRCM_PLL_NB_450MHZ;
			break;
		case 480000000:
			*pll_nr = PRCM_PLL_NR_480MHZ;
			*pll_nf = PRCM_PLL_NF_480MHZ;
			*pll_od = PRCM_PLL_OD_480MHZ;
			*pll_nb = PRCM_PLL_NB_480MHZ;
			break;
		case 500000000:
			*pll_nr = PRCM_PLL_NR_500MHZ;
			*pll_nf = PRCM_PLL_NF_500MHZ;
			*pll_od = PRCM_PLL_OD_500MHZ;
			*pll_nb = PRCM_PLL_NB_500MHZ;
			break;
		case 533000000:
			*pll_nr = PRCM_PLL_NR_533MHZ;
			*pll_nf = PRCM_PLL_NF_533MHZ;
			*pll_od = PRCM_PLL_OD_533MHZ;
			*pll_nb = PRCM_PLL_NB_533MHZ;
			break;
		case 550000000:
			*pll_nr = PRCM_PLL_NR_550MHZ;
			*pll_nf = PRCM_PLL_NF_550MHZ;
			*pll_od = PRCM_PLL_OD_550MHZ;
			*pll_nb = PRCM_PLL_NB_550MHZ;
			break;
		case 600000000:
			*pll_nr = PRCM_PLL_NR_600MHZ;
			*pll_nf = PRCM_PLL_NF_600MHZ;
			*pll_od = PRCM_PLL_OD_600MHZ;
			*pll_nb = PRCM_PLL_NB_600MHZ;
			break;
		case 650000000:
			*pll_nr = PRCM_PLL_NR_650MHZ;
			*pll_nf = PRCM_PLL_NF_650MHZ;
			*pll_od = PRCM_PLL_OD_650MHZ;
			*pll_nb = PRCM_PLL_NB_650MHZ;
			break;
		case 666000000:
			*pll_nr = PRCM_PLL_NR_666MHZ;
			*pll_nf = PRCM_PLL_NF_666MHZ;
			*pll_od = PRCM_PLL_OD_666MHZ;
			*pll_nb = PRCM_PLL_NB_666MHZ;
			break;
		case 700000000:
			*pll_nr = PRCM_PLL_NR_700MHZ;
			*pll_nf = PRCM_PLL_NF_700MHZ;
			*pll_od = PRCM_PLL_OD_700MHZ;
			*pll_nb = PRCM_PLL_NB_700MHZ;
			break;
		case 720000000:
			*pll_nr = PRCM_PLL_NR_720MHZ;
			*pll_nf = PRCM_PLL_NF_720MHZ;
			*pll_od = PRCM_PLL_OD_720MHZ;
			*pll_nb = PRCM_PLL_NB_720MHZ;
			break;
		case 750000000:
			*pll_nr = PRCM_PLL_NR_750MHZ;
			*pll_nf = PRCM_PLL_NF_750MHZ;
			*pll_od = PRCM_PLL_OD_750MHZ;
			*pll_nb = PRCM_PLL_NB_750MHZ;
			break;
		case 800000000:
			*pll_nr = PRCM_PLL_NR_800MHZ;
			*pll_nf = PRCM_PLL_NF_800MHZ;
			*pll_od = PRCM_PLL_OD_800MHZ;
			*pll_nb = PRCM_PLL_NB_800MHZ;
			break;
		case 850000000: 
			*pll_nr = PRCM_PLL_NR_850MHZ; 
			*pll_nf = PRCM_PLL_NF_850MHZ; 
			*pll_od = PRCM_PLL_OD_850MHZ; 
			*pll_nb = PRCM_PLL_NB_850MHZ; 
			break; 
		case 891000000:
			*pll_nr = PRCM_PLL_NR_891MHZ;
			*pll_nf = PRCM_PLL_NF_891MHZ;
			*pll_od = PRCM_PLL_OD_891MHZ;
			*pll_nb = PRCM_PLL_NB_891MHZ;
			break;
		case 900000000: 
			*pll_nr = PRCM_PLL_NR_900MHZ; 
			*pll_nf = PRCM_PLL_NF_900MHZ; 
			*pll_od = PRCM_PLL_OD_900MHZ; 
			*pll_nb = PRCM_PLL_NB_900MHZ; 
			break; 
		case 950000000: 
			*pll_nr = PRCM_PLL_NR_950MHZ; 
			*pll_nf = PRCM_PLL_NF_950MHZ; 
			*pll_od = PRCM_PLL_OD_950MHZ; 
			*pll_nb = PRCM_PLL_NB_950MHZ; 
			break; 
		case 983040000:
			*pll_nr = PRCM_PLL_NR_983M04HZ;
			*pll_nf = PRCM_PLL_NF_983M04HZ;
			*pll_od = PRCM_PLL_OD_983M04HZ;
			*pll_nb = PRCM_PLL_NB_983M04HZ;
			break;
		case 1000000000: 
			*pll_nr = PRCM_PLL_NR_1000MHZ; 
			*pll_nf = PRCM_PLL_NF_1000MHZ; 
			*pll_od = PRCM_PLL_OD_1000MHZ; 
			*pll_nb = PRCM_PLL_NB_1000MHZ; 
			break; 
		case 1050000000:
			*pll_nr = PRCM_PLL_NR_1050MHZ;
			*pll_nf = PRCM_PLL_NF_1050MHZ;
			*pll_od = PRCM_PLL_OD_1050MHZ;
			*pll_nb = PRCM_PLL_NB_1050MHZ;
			break;
		case 1100000000:
			*pll_nr = PRCM_PLL_NR_1100MHZ;
			*pll_nf = PRCM_PLL_NF_1100MHZ;
			*pll_od = PRCM_PLL_OD_1100MHZ;
			*pll_nb = PRCM_PLL_NB_1100MHZ;
			break;
		case 1150000000:
			*pll_nr = PRCM_PLL_NR_1150MHZ;
			*pll_nf = PRCM_PLL_NF_1150MHZ;
			*pll_od = PRCM_PLL_OD_1150MHZ;
			*pll_nb = PRCM_PLL_NB_1150MHZ;
			break;
		case 1200000000:
			*pll_nr = PRCM_PLL_NR_1200MHZ;
			*pll_nf = PRCM_PLL_NF_1200MHZ;
			*pll_od = PRCM_PLL_OD_1200MHZ;
			*pll_nb = PRCM_PLL_NB_1200MHZ;
			break;
		case 1250000000:
			*pll_nr = PRCM_PLL_NR_1250MHZ;
			*pll_nf = PRCM_PLL_NF_1250MHZ;
			*pll_od = PRCM_PLL_OD_1250MHZ;
			*pll_nb = PRCM_PLL_NB_1250MHZ;
			break;
		case 1300000000:
			*pll_nr = PRCM_PLL_NR_1300MHZ;
			*pll_nf = PRCM_PLL_NF_1300MHZ;
			*pll_od = PRCM_PLL_OD_1300MHZ;
			*pll_nb = PRCM_PLL_NB_1300MHZ;
			break;
		case 1350000000:
			*pll_nr = PRCM_PLL_NR_1350MHZ;
			*pll_nf = PRCM_PLL_NF_1350MHZ;
			*pll_od = PRCM_PLL_OD_1350MHZ;
			*pll_nb = PRCM_PLL_NB_1350MHZ;
			break;
		case 1400000000:
			*pll_nr = PRCM_PLL_NR_1400MHZ;
			*pll_nf = PRCM_PLL_NF_1400MHZ;
			*pll_od = PRCM_PLL_OD_1400MHZ;
			*pll_nb = PRCM_PLL_NB_1400MHZ;
			break;
		case 1450000000:
			*pll_nr = PRCM_PLL_NR_1450MHZ;
			*pll_nf = PRCM_PLL_NF_1450MHZ;
			*pll_od = PRCM_PLL_OD_1450MHZ;
			*pll_nb = PRCM_PLL_NB_1450MHZ;
			break;
		case 1500000000:
			*pll_nr = PRCM_PLL_NR_1500MHZ;
			*pll_nf = PRCM_PLL_NF_1500MHZ;
			*pll_od = PRCM_PLL_OD_1500MHZ;
			*pll_nb = PRCM_PLL_NB_1500MHZ;
			break;

			// lcdc only
		case 50350000: /* 640*480*/
			*pll_nr = PRCM_PLL_NR_50M35HZ;
			*pll_nf = PRCM_PLL_NF_50M35HZ;
			*pll_od = PRCM_PLL_OD_50M35HZ;
			*pll_nb = PRCM_PLL_NB_50M35HZ;
			break;
		case 56000000: /* 800*480*/
        		*pll_nr = PRCM_PLL_NR_56MHZ;
        		*pll_nf = PRCM_PLL_NF_56MHZ;
        		*pll_od = PRCM_PLL_OD_56MHZ;
        		*pll_nb = PRCM_PLL_NB_56MHZ;
        		break;
		case 58464000: /* 800*480*/
			*pll_nr = PRCM_PLL_NR_58M464HZ;
			*pll_nf = PRCM_PLL_NF_58M464HZ;
			*pll_od = PRCM_PLL_OD_58M464HZ;
			*pll_nb = PRCM_PLL_NB_58M464HZ;
			break;
		case 66528000: /* 800*480*/
			*pll_nr = PRCM_PLL_NR_66M528HZ;
			*pll_nf = PRCM_PLL_NF_66M528HZ;
			*pll_od = PRCM_PLL_OD_66M528HZ;
			*pll_nb = PRCM_PLL_NB_66M528HZ;
			break;
		case 80000000: /* 800*600*/
			*pll_nr = PRCM_PLL_NR_80MHZ;
			*pll_nf = PRCM_PLL_NF_80MHZ;
			*pll_od = PRCM_PLL_OD_80MHZ;
			*pll_nb = PRCM_PLL_NB_80MHZ;
			break;
		case 87500000:/*1024*600*/
			*pll_nr = PRCM_PLL_NR_87M5HZ;
			*pll_nf = PRCM_PLL_NF_87M5HZ;
			*pll_od = PRCM_PLL_OD_87M5HZ;
			*pll_nb = PRCM_PLL_NB_87M5HZ;
			break;
		case 100800000:/*1024*600*/
			*pll_nr = PRCM_PLL_NR_100M8HZ;
			*pll_nf = PRCM_PLL_NF_100M8HZ;
			*pll_od = PRCM_PLL_OD_100M8HZ;
			*pll_nb = PRCM_PLL_NB_100M8HZ;
			break;
		case 102400000:/*1024*600*/
			*pll_nr = PRCM_PLL_NR_102M4HZ;
			*pll_nf = PRCM_PLL_NF_102M4HZ;
			*pll_od = PRCM_PLL_OD_102M4HZ;
			*pll_nb = PRCM_PLL_NB_102M4HZ;
			break;
		case 102412800:/*1024*600*/
			*pll_nr = PRCM_PLL_NR_102M4128HZ;
			*pll_nf = PRCM_PLL_NF_102M4128HZ;
			*pll_od = PRCM_PLL_OD_102M4128HZ;
			*pll_nb = PRCM_PLL_NB_102M4128HZ;
			break;
		case 130000000: /* 1024*768*/
			*pll_nr = PRCM_PLL_NR_130MHZ;
			*pll_nf = PRCM_PLL_NF_130MHZ;
			*pll_od = PRCM_PLL_OD_130MHZ;
			*pll_nb = PRCM_PLL_NB_130MHZ;
			break;
		case 130004000: /* 1024*768 calc*/
			*pll_nr = PRCM_PLL_NR_130MHZ;
			*pll_nf = PRCM_PLL_NF_130MHZ;
			*pll_od = PRCM_PLL_OD_130MHZ;
			*pll_nb = PRCM_PLL_NB_130MHZ;
			break;
		case 142000000: /* 1280*800 (RB)*/
			*pll_nr = PRCM_PLL_NR_142MHZ;
			*pll_nf = PRCM_PLL_NF_142MHZ;
			*pll_od = PRCM_PLL_OD_142MHZ;
			*pll_nb = PRCM_PLL_NB_142MHZ;
			break;
		case 148500000: /* 1280*720*/
			*pll_nr = PRCM_PLL_NR_148M5HZ;
			*pll_nf = PRCM_PLL_NF_148M5HZ;
			*pll_od = PRCM_PLL_OD_148M5HZ;
			*pll_nb = PRCM_PLL_NB_148M5HZ;
			break;
		case 167000000: /* 1280*800 */
			*pll_nr = PRCM_PLL_NR_167MHZ;
			*pll_nf = PRCM_PLL_NF_167MHZ;
			*pll_od = PRCM_PLL_OD_167MHZ;
			*pll_nb = PRCM_PLL_NB_167MHZ;
			break;
		case 166002000: /* 1280*800 calc*/
			*pll_nr = PRCM_PLL_NR_167MHZ;
			*pll_nf = PRCM_PLL_NF_167MHZ;
			*pll_od = PRCM_PLL_OD_167MHZ;
			*pll_nb = PRCM_PLL_NB_167MHZ;
			break;
		case 144864000: /* 1366*768 calc*/
			*pll_nr = PRCM_PLL_NR_144MHZ;
			*pll_nf = PRCM_PLL_NF_144MHZ;
			*pll_od = PRCM_PLL_OD_144MHZ;
			*pll_nb = PRCM_PLL_NB_144MHZ;
			break;
		case 171012000: /* 1360*768 calc*/
			*pll_nr = PRCM_PLL_NR_171MHZ;
			*pll_nf = PRCM_PLL_NF_171MHZ;
			*pll_od = PRCM_PLL_OD_171MHZ;
			*pll_nb = PRCM_PLL_NB_171MHZ;
			break;
		case 171000000: /* 1360*768*/
			*pll_nr = PRCM_PLL_NR_171MHZ;
			*pll_nf = PRCM_PLL_NF_171MHZ;
			*pll_od = PRCM_PLL_OD_171MHZ;
			*pll_nb = PRCM_PLL_NB_171MHZ;
			break;
		case 177500000: /* 1440*900 RB*/
			*pll_nr = PRCM_PLL_NR_177M5HZ;
			*pll_nf = PRCM_PLL_NF_177M5HZ;
			*pll_od = PRCM_PLL_OD_177M5HZ;
			*pll_nb = PRCM_PLL_NB_177M5HZ;
			break;
		case 213014000: /* 1440*900 calc*/
			*pll_nr = PRCM_PLL_NR_213MHZ;
			*pll_nf = PRCM_PLL_NF_213MHZ;
			*pll_od = PRCM_PLL_OD_213MHZ;
			*pll_nb = PRCM_PLL_NB_213MHZ;
			break;
		case 238000000: /* 1680*1050 RB*/
			*pll_nr = PRCM_PLL_NR_238MHZ;
			*pll_nf = PRCM_PLL_NF_238MHZ;
			*pll_od = PRCM_PLL_OD_238MHZ;
			*pll_nb = PRCM_PLL_NB_238MHZ;
			break;
		case 292056000: /* 1680*1050 calc*/
			*pll_nr = PRCM_PLL_NR_292M5HZ;
			*pll_nf = PRCM_PLL_NF_292M5HZ;
			*pll_od = PRCM_PLL_OD_292M5HZ;
			*pll_nb = PRCM_PLL_NB_292M5HZ;
			break;
			//case 297000000: /* 1920*1080*/
			//	*pll_nr = PRCM_PLL_NR_297MHZ;
			//	*pll_nf = PRCM_PLL_NF_297MHZ;
			//	*pll_od = PRCM_PLL_OD_297MHZ;
			//	*pll_nb = PRCM_PLL_NB_297MHZ;
			//	break;
		case 324000000: /* 1600*1200 */
			*pll_nr = PRCM_PLL_NR_324MHZ;
			*pll_nf = PRCM_PLL_NF_324MHZ;
			*pll_od = PRCM_PLL_OD_324MHZ;
			*pll_nb = PRCM_PLL_NB_324MHZ;
			break;
		case 324044000: /* 1600*1200 calc*/
			*pll_nr = PRCM_PLL_NR_324MHZ;
			*pll_nf = PRCM_PLL_NF_324MHZ;
			*pll_od = PRCM_PLL_OD_324MHZ;
			*pll_nb = PRCM_PLL_NB_324MHZ;
			break;
		default:
			if(pll_calc_fout(rate, pll_nf, pll_nr, pll_od, pll_nb)) {
				printk(KERN_NOTICE "Cannot Generate parameters for this rate %ld.\n", rate);
				return -EINVAL;
			}
			break;
	}

	return 0;
}

#ifdef CONFIG_CLASS_NORMAL
#define ADAPT_CLK(x) min(x,CLK_NORMAL)
#elif defined CONFIG_CLASS_HIGH
#define ADAPT_CLK(x) min(x,CLK_HIGH)
#elif defined CONFIG_CLASS_ULTRA
#define ADAPT_CLK(x) min(x,CLK_ULTRA)
#else
#define ADAPT_CLK(x) (x)
#endif

int ns115_pll0_config(unsigned long rate)
{
	unsigned int pll_nr; 
	unsigned int pll_nf; 
	unsigned int pll_od; 
	unsigned int pll_nb;

	void __iomem * pll0_cfg_base = __io_address(PRCM_PLL0_CFG0);
	void __iomem * sys_pll_ctrl   = __io_address(PRCM_SYS_PLL_CTRL);
	void __iomem * sys_pll_status = __io_address(PRCM_SYS_PLL_STATUS);

	unsigned int sys_pll_status_reg = 0;

	// Enable SW PLL Config (only needed for PLL 0/1/2)
	reg_set_bits(sys_pll_ctrl, PRCM_SYS_PLL_CTRL_PLL0_SW_CFG_EN);

	// Configure New PLL Parameter
	if (ns115_pll_get_param(ADAPT_CLK(rate), &pll_nr, &pll_nf, &pll_od, &pll_nb) == -EINVAL)  {
		return -EINVAL;
	}
	pll_config_reg(pll0_cfg_base, pll_nr, pll_nf, pll_od, pll_nb);

	//Disable PLL Clock (not needed for PLL 0/1/2)
	//reg_clr_bits(sys_pll_ctrl, PRCM_SYS_PLL_CTRL_PLL0_CLKEN);

	// Reset PLL
	reg_set_bits(sys_pll_ctrl, PRCM_SYS_PLL_CTRL_PLL0_RST);
	udelay(100);

	// Wait until PLL Stable
	while ((sys_pll_status_reg & PRCM_SYS_PLL_STATUS_PLL0_STABLE) == 0) {
		sys_pll_status_reg = __raw_readl(sys_pll_status);
	}

	// Clear Reset and Enable PLL
	reg_clr_bits(sys_pll_ctrl, PRCM_SYS_PLL_CTRL_PLL0_RST);
	//reg_set_bits(sys_pll_ctrl, PRCM_SYS_PLL_CTRL_PLL0_CLKEN); //(not needed for PLL 0/1/2)

	return 0;
}

int ns115_pll0_clk_set_rate(struct clk *c, unsigned long rate, unsigned int nouse)
{
	if (ns115_pll0_config(rate) == -EINVAL) {
		printk(KERN_NOTICE "Failed to set rate %ld to pll0.\n", rate);
		return -EINVAL;
	}
	c->rate = rate;

	printk(KERN_NOTICE "pll0 clock rate set to %ld.\n", rate);
	return 0;
}

int ns115_pll1_config(unsigned long rate)
{
	unsigned int pll_nr; 
	unsigned int pll_nf; 
	unsigned int pll_od; 
	unsigned int pll_nb;

	void __iomem * pll1_cfg_base = __io_address(PRCM_PLL1_CFG0);
	void __iomem * sys_pll_ctrl   = __io_address(PRCM_SYS_PLL_CTRL);
	void __iomem * sys_pll_status = __io_address(PRCM_SYS_PLL_STATUS);

	unsigned int sys_pll_status_reg = 0;

	// Enable SW PLL Config (only needed for PLL 0/1/2)
	reg_set_bits(sys_pll_ctrl, PRCM_SYS_PLL_CTRL_PLL1_SW_CFG_EN);

	// Configure New PLL Parameter
	if (ns115_pll_get_param(rate, &pll_nr, &pll_nf, &pll_od, &pll_nb) == -EINVAL)  {
		return -EINVAL;
	}
	pll_config_reg(pll1_cfg_base, pll_nr, pll_nf, pll_od, pll_nb);

	//Disable PLL Clock (not needed for PLL 0/1/2)
	//reg_clr_bits(sys_pll_ctrl, PRCM_SYS_PLL_CTRL_PLL1_CLKEN);

	// Reset PLL
	reg_set_bits(sys_pll_ctrl, PRCM_SYS_PLL_CTRL_PLL1_RST);
	udelay(100);

	// Wait until PLL Stable
	while ((sys_pll_status_reg & PRCM_SYS_PLL_STATUS_PLL1_STABLE) == 0) {
		sys_pll_status_reg = __raw_readl(sys_pll_status);
	}

	// Clear Reset and Enable PLL
	reg_clr_bits(sys_pll_ctrl, PRCM_SYS_PLL_CTRL_PLL1_RST);
	//reg_set_bits(sys_pll_ctrl, PRCM_SYS_PLL_CTRL_PLL1_CLKEN); //(not needed for PLL 0/1/2)

	return 0;
}

int ns115_pll1_clk_set_rate(struct clk *c, unsigned long rate, unsigned int nouse)
{
	if (ns115_pll1_config(rate) == -EINVAL) {
		printk(KERN_NOTICE "Failed to set rate %ld to pll1.\n", rate);
		return -EINVAL;
	}
	c->rate = rate;

	printk(KERN_NOTICE "pll1 clock rate set to %ld.\n", rate);
	return 0;
}

int ns115_pll2_config(unsigned long rate)
{
	unsigned int pll_nr; 
	unsigned int pll_nf; 
	unsigned int pll_od; 
	unsigned int pll_nb;

	void __iomem * pll2_cfg_base = __io_address(PRCM_PLL2_CFG0);
	void __iomem * sys_pll_ctrl   = __io_address(PRCM_SYS_PLL_CTRL);
	void __iomem * sys_pll_status = __io_address(PRCM_SYS_PLL_STATUS);

	unsigned int sys_pll_status_reg = 0;

	// Enable SW PLL Config (only needed for PLL 0/1/2)
	reg_set_bits(sys_pll_ctrl, PRCM_SYS_PLL_CTRL_PLL2_SW_CFG_EN);

	// Configure New PLL Parameter
	if (ns115_pll_get_param(rate, &pll_nr, &pll_nf, &pll_od, &pll_nb) == -EINVAL)  {
		return -EINVAL;
	}
	pll_config_reg(pll2_cfg_base, pll_nr, pll_nf, pll_od, pll_nb);

	//Disable PLL Clock (not needed for PLL 0/1/2)
	//reg_clr_bits(sys_pll_ctrl, PRCM_SYS_PLL_CTRL_PLL2_CLKEN);

	// Reset PLL
	reg_set_bits(sys_pll_ctrl, PRCM_SYS_PLL_CTRL_PLL2_RST);
	udelay(100);

	// Wait until PLL Stable
	while ((sys_pll_status_reg & PRCM_SYS_PLL_STATUS_PLL2_STABLE) == 0) {
		sys_pll_status_reg = __raw_readl(sys_pll_status);
	}

	// Clear Reset and Enable PLL
	reg_clr_bits(sys_pll_ctrl, PRCM_SYS_PLL_CTRL_PLL2_RST);
	//reg_set_bits(sys_pll_ctrl, PRCM_SYS_PLL_CTRL_PLL2_CLKEN); //(not needed for PLL 0/1/2)

	return 0;
}

int ns115_pll2_clk_set_rate(struct clk *c, unsigned long rate, unsigned int nouse)
{
	if (ns115_pll2_config(rate) == -EINVAL) {
		printk(KERN_NOTICE "Failed to set rate %ld to pll2.\n", rate);
		return -EINVAL;
	}
	c->rate = rate;

	printk(KERN_NOTICE "pll2 clock rate set to %ld.\n", rate);
	return 0;
}


int ns115_pll3_config(unsigned long rate)
{
	unsigned int pll_nr; 
	unsigned int pll_nf; 
	unsigned int pll_od; 
	unsigned int pll_nb;

	void __iomem * pll3_cfg_base = __io_address(PRCM_PLL3_CFG0);
	void __iomem * sys_pll_ctrl   = __io_address(PRCM_SYS_PLL_CTRL);
	void __iomem * sys_pll_status = __io_address(PRCM_SYS_PLL_STATUS);

	unsigned int sys_pll_status_reg = 0;

	// Enable SW PLL Config (only needed for PLL 0/1/2)
	//reg_set_bits(sys_pll_ctrl, PRCM_SYS_PLL_CTRL_PLL3_SW_CFG_EN);

	// Configure New PLL Parameter
	if (ns115_pll_get_param(rate, &pll_nr, &pll_nf, &pll_od, &pll_nb) == -EINVAL)  {
		return -EINVAL;
	}
	pll_config_reg(pll3_cfg_base, pll_nr, pll_nf, pll_od, pll_nb);

	//Disable PLL Clock (not needed for PLL 0/1/2)
	reg_clr_bits(sys_pll_ctrl, PRCM_SYS_PLL_CTRL_PLL3_CLKEN);

	// Reset PLL
	reg_set_bits(sys_pll_ctrl, PRCM_SYS_PLL_CTRL_PLL3_RST);
	udelay(100);

	// Wait until PLL Stable
	while ((sys_pll_status_reg & PRCM_SYS_PLL_STATUS_PLL3_STABLE) == 0) {
		sys_pll_status_reg = __raw_readl(sys_pll_status);
	}

	// Clear Reset and Enable PLL
	reg_clr_bits(sys_pll_ctrl, PRCM_SYS_PLL_CTRL_PLL3_RST);
	reg_set_bits(sys_pll_ctrl, PRCM_SYS_PLL_CTRL_PLL3_CLKEN); //(not needed for PLL 0/1/2)

	return 0;
}

int ns115_pll3_clk_set_rate(struct clk *c, unsigned long rate, unsigned int nouse)
{
	if (ns115_pll3_config(rate) == -EINVAL) {
		printk(KERN_NOTICE "Failed to set rate %ld to pll3.\n", rate);
		return -EINVAL;
	}
	c->rate = rate;

	printk(KERN_NOTICE "pll3 clock rate set to %ld.\n", rate);
	return 0;
}

int ns115_pll4_config(unsigned long rate)
{
	unsigned int pll_nr; 
	unsigned int pll_nf; 
	unsigned int pll_od; 
	unsigned int pll_nb;

	void __iomem * pll4_cfg_base = __io_address(PRCM_PLL4_CFG0);
	void __iomem * sys_pll_ctrl   = __io_address(PRCM_SYS_PLL_CTRL);
	void __iomem * sys_pll_status = __io_address(PRCM_SYS_PLL_STATUS);

	unsigned int sys_pll_status_reg = 0;

	// Enable SW PLL Config (only needed for PLL 0/1/2)
	//reg_set_bits(sys_pll_ctrl, PRCM_SYS_PLL_CTRL_PLL4_SW_CFG_EN);

	// Configure New PLL Parameter
	if (ns115_pll_get_param(rate, &pll_nr, &pll_nf, &pll_od, &pll_nb) == -EINVAL)  {
		return -EINVAL;
	}
	pll_config_reg(pll4_cfg_base, pll_nr, pll_nf, pll_od, pll_nb);

	//Disable PLL Clock (not needed for PLL 0/1/2)
	reg_clr_bits(sys_pll_ctrl, PRCM_SYS_PLL_CTRL_PLL4_CLKEN);

	// Reset PLL
	reg_set_bits(sys_pll_ctrl, PRCM_SYS_PLL_CTRL_PLL4_RST);
	udelay(100);

	// Wait until PLL Stable
	while ((sys_pll_status_reg & PRCM_SYS_PLL_STATUS_PLL4_STABLE) == 0) {
		sys_pll_status_reg = __raw_readl(sys_pll_status);
	}

	// Clear Reset and Enable PLL
	reg_clr_bits(sys_pll_ctrl, PRCM_SYS_PLL_CTRL_PLL4_RST);
	reg_set_bits(sys_pll_ctrl, PRCM_SYS_PLL_CTRL_PLL4_CLKEN); //(not needed for PLL 0/1/2)

	return 0;
}

int ns115_pll4_clk_set_rate(struct clk *c, unsigned long rate, unsigned int nouse)
{
	if (ns115_pll4_config(rate) == -EINVAL) {
		printk(KERN_NOTICE "Failed to set rate %ld to pll4.\n", rate);
		return -EINVAL;
	}
	c->rate = rate;

	printk(KERN_NOTICE "pll4 clock rate set to %ld.\n", rate);
	return 0;
}

int ns115_pll5_config(unsigned long rate)
{
	unsigned int pll_nr; 
	unsigned int pll_nf; 
	unsigned int pll_od; 
	unsigned int pll_nb;

	void __iomem * pll5_cfg_base = __io_address(PRCM_PLL5_CFG0);
	void __iomem * sys_pll_ctrl   = __io_address(PRCM_SYS_PLL_CTRL);
	void __iomem * sys_pll_status = __io_address(PRCM_SYS_PLL_STATUS);

	unsigned int sys_pll_status_reg = 0;

	// Enable SW PLL Config (only needed for PLL 0/1/2)
	//reg_set_bits(sys_pll_ctrl, PRCM_SYS_PLL_CTRL_PLL5_SW_CFG_EN);

	// Configure New PLL Parameter
	if (ns115_pll_get_param(rate, &pll_nr, &pll_nf, &pll_od, &pll_nb) == -EINVAL)  {
		return -EINVAL;
	}
	pll_config_reg(pll5_cfg_base, pll_nr, pll_nf, pll_od, pll_nb);

	//Disable PLL Clock (not needed for PLL 0/1/2)
	reg_clr_bits(sys_pll_ctrl, PRCM_SYS_PLL_CTRL_PLL5_CLKEN);

	// Reset PLL
	reg_set_bits(sys_pll_ctrl, PRCM_SYS_PLL_CTRL_PLL5_RST);
	udelay(100);

	// Wait until PLL Stable
	while ((sys_pll_status_reg & PRCM_SYS_PLL_STATUS_PLL5_STABLE) == 0) {
		sys_pll_status_reg = __raw_readl(sys_pll_status);
	}

	// Clear Reset and Enable PLL
	reg_clr_bits(sys_pll_ctrl, PRCM_SYS_PLL_CTRL_PLL5_RST);
	reg_set_bits(sys_pll_ctrl, PRCM_SYS_PLL_CTRL_PLL5_CLKEN); //(not needed for PLL 0/1/2)

	return 0;
}

int ns115_pll5_clk_set_rate(struct clk *c, unsigned long rate, unsigned int nouse)
{
	if (ns115_pll5_config(rate) == -EINVAL) {
		printk(KERN_NOTICE "Failed to set rate %ld to pll5.\n", rate);
		return -EINVAL;
	}
	c->rate = rate;

	printk(KERN_NOTICE "pll5 clock rate set to %ld.\n", rate);
	return 0;
}

int ns115_pll6_config(unsigned long rate)
{
	unsigned int pll_nr; 
	unsigned int pll_nf; 
	unsigned int pll_od; 
	unsigned int pll_nb;

	void __iomem * pll6_cfg_base = __io_address(PRCM_PLL6_CFG0);
	void __iomem * sys_pll_ctrl   = __io_address(PRCM_SYS_PLL_CTRL);
	void __iomem * sys_pll_status = __io_address(PRCM_SYS_PLL_STATUS);

	unsigned int sys_pll_status_reg = 0;

	// Enable SW PLL Config (only needed for PLL 0/1/2)
	//reg_set_bits(sys_pll_ctrl, PRCM_SYS_PLL_CTRL_PLL6_SW_CFG_EN);

	// Configure New PLL Parameter
	if (ns115_pll_get_param(rate, &pll_nr, &pll_nf, &pll_od, &pll_nb) == -EINVAL)  {
		return -EINVAL;
	}
	pll_config_reg(pll6_cfg_base, pll_nr, pll_nf, pll_od, pll_nb);

	//Disable PLL Clock (not needed for PLL 0/1/2)
	reg_clr_bits(sys_pll_ctrl, PRCM_SYS_PLL_CTRL_PLL6_CLKEN);

	// Reset PLL
	reg_set_bits(sys_pll_ctrl, PRCM_SYS_PLL_CTRL_PLL6_RST);
	udelay(100);

	// Wait until PLL Stable
	while ((sys_pll_status_reg & PRCM_SYS_PLL_STATUS_PLL6_STABLE) == 0) {
		sys_pll_status_reg = __raw_readl(sys_pll_status);
	}

	// Clear Reset and Enable PLL
	reg_clr_bits(sys_pll_ctrl, PRCM_SYS_PLL_CTRL_PLL6_RST);
	reg_set_bits(sys_pll_ctrl, PRCM_SYS_PLL_CTRL_PLL6_CLKEN); //(not needed for PLL 0/1/2)

	return 0;
}


// Coresight clock control function/tasks
int ns115_coresight_clk_enable(struct clk *clk)
{
	unsigned int reg_value;
	void __iomem *coresight_clk_ctrl = __io_address(PRCM_CORESIGHT_CLK_CTRL);

	reg_value = PRCM_CORESIGHT_CLK_CTRL_CS_ATCLKEN |
		PRCM_CORESIGHT_CLK_CTRL_CS_HCLKEN;
	reg_set_bits(coresight_clk_ctrl, reg_value);

	printk(KERN_NOTICE "coresight clock enabled.\n");
	return 0;
}

int ns115_coresight_clk_disable(struct clk *clk)
{
	unsigned int reg_value;
	void __iomem *coresight_clk_ctrl = __io_address(PRCM_CORESIGHT_CLK_CTRL);

	reg_value = PRCM_CORESIGHT_CLK_CTRL_CS_ATCLKEN |
		PRCM_CORESIGHT_CLK_CTRL_CS_HCLKEN;
	reg_clr_bits(coresight_clk_ctrl, reg_value);

	printk(KERN_NOTICE "coresight clock disabled.\n");
	return 0;
}

int ns115_coresight_clk_set_rate(struct clk *c, unsigned long rate, unsigned int nouse)
{
	void __iomem *coresight_clk_ctrl = __io_address(PRCM_CORESIGHT_CLK_CTRL);
	unsigned int coresight_clk_div = 800000000/rate;

	if (rate == 0) 
		return -EINVAL;

	if (coresight_clk_div >0x0F)
		return -EINVAL;

	clk_set_rate_bits(coresight_clk_ctrl, PRCM_CORESIGHT_CLK_CTRL_CS_AT_DIV_EN,
			PRCM_CORESIGHT_CLK_CTRL_CS_AT_DIV_MASK, coresight_clk_div);

	rate = coresight_clk_div*rate;
	c->rate = rate;

	printk(KERN_NOTICE "coresight clock rate set to %ld.\n", rate);
	return 0;
}

// DDR clock control function/tasks
int ns115_ddr_clk_enable(struct clk *clk)
{
	unsigned int reg_value;
	void __iomem *ddr_clk_ctrl = __io_address(PRCM_DDR_CLK_CTRL);

	reg_value = PRCM_DDR_CLK_CTRL_DDR_CLKEN |
		PRCM_DDR_CLK_CTRL_DDR_HCLKEN;
	reg_set_bits(ddr_clk_ctrl, reg_value);

	printk(KERN_NOTICE "ddr clock enabled.\n");
	return 0;
}

int ns115_ddr_clk_disable(struct clk *clk)
{
	unsigned int reg_value;
	void __iomem *ddr_clk_ctrl = __io_address(PRCM_DDR_CLK_CTRL);

	reg_value = PRCM_DDR_CLK_CTRL_DDR_CLKEN |
		PRCM_DDR_CLK_CTRL_DDR_HCLKEN;
	reg_clr_bits(ddr_clk_ctrl, reg_value);

	printk(KERN_NOTICE "ddr clock disabled.\n");
	return 0;
}

int ns115_ddr_clk_set_rate(struct clk *c, unsigned long rate, unsigned int nouse)
{
	void __iomem *ddr_clk_ctrl = __io_address(PRCM_DDR_CLK_CTRL);
	unsigned long pll1_rate;

	if (rate == 0)
		return -EINVAL;

	switch (rate) {
		case 400000000:
			pll1_rate = 800000000;
			break;
		case 333000000:
			pll1_rate = 666000000;
			break;
		case 266000000:
			pll1_rate = 533000000;
			break;
		case 200000000:
			pll1_rate = 400000000;
			break;
		default:
			return -EINVAL;
			break;
	}

	reg_clr_bits(ddr_clk_ctrl, PRCM_DDR_CLK_CTRL_DDR_CLKEN|PRCM_DDR_CLK_CTRL_DDR_HCLKEN);

	if(ns115_pll1_clk_set_rate(&pll1_clk, pll1_rate, 0) == -EINVAL) {
		printk(KERN_NOTICE "Failed to set ddr clock rate to %ld.\n", rate);
		return -EINVAL;
	}
	c->rate = rate;

	reg_set_bits(ddr_clk_ctrl, PRCM_DDR_CLK_CTRL_DDR_CLKEN|PRCM_DDR_CLK_CTRL_DDR_HCLKEN);

	printk(KERN_NOTICE "ddr clock rate set to %ld.\n", rate);
	return 0;
}

// PL330 DMA clock control function/tasks
int ns115_pl330dma_clk_enable(struct clk *clk)
{
	unsigned int reg_value;
	void __iomem *dma_clk_ctrl = __io_address(PRCM_DMA_CLK_CTRL);

	reg_value = PRCM_DMA_CLK_CTRL_DMA_ACLKEN;
	reg_set_bits(dma_clk_ctrl, reg_value);

	printk(KERN_NOTICE "pl330 dma clock enabled.\n");
	return 0;
}

int ns115_pl330dma_clk_disable(struct clk *clk)
{
	unsigned int reg_value;
	void __iomem *dma_clk_ctrl = __io_address(PRCM_DMA_CLK_CTRL);

	reg_value = PRCM_DMA_CLK_CTRL_DMA_ACLKEN;
	reg_clr_bits(dma_clk_ctrl, reg_value);

	printk(KERN_NOTICE "pl330 dma clock disabled.\n");
	return 0;
}

// MALI clock control function/tasks
int ns115_mali_clk_enable(struct clk *clk)
{
	unsigned int reg_value;
	void __iomem *mali_clk_ctrl = __io_address(PRCM_MALI_CLK_CTRL);

	// Power Up MALI
	sys_pwr_ctrl_set_bits(PRCM_SYS_PWR_CTRL_MALI_PWR_STATE_L2C | PRCM_SYS_PWR_CTRL_MALI_PWR_STATE_PP | PRCM_SYS_PWR_CTRL_MALI_PWR_STATE_GP,
			PRCM_SYS_PWR_STATUS_MALI_PWR_STATUS_TOTAL | PRCM_SYS_PWR_STATUS_MALI_PWR_STATUS_L2C |PRCM_SYS_PWR_STATUS_MALI_PWR_STATUS_PP |PRCM_SYS_PWR_STATUS_MALI_PWR_STATUS_GP);
	printk(KERN_NOTICE "mali power up.\n");

	// Enable MALI clocks
	reg_value = PRCM_MALI_CLK_CTRL_MALI_CLKEN;
	reg_set_bits(mali_clk_ctrl, reg_value);

	printk(KERN_NOTICE "mali clock enabled.\n");
	return 0;
}

int ns115_mali_clk_disable(struct clk *clk)
{
	unsigned int reg_value;
	void __iomem *mali_clk_ctrl = __io_address(PRCM_MALI_CLK_CTRL);

	// Disable MALI clocks
	reg_value = PRCM_MALI_CLK_CTRL_MALI_CLKEN;
	reg_clr_bits(mali_clk_ctrl, reg_value);

	printk(KERN_NOTICE "mali clock disabled.\n");

	// Power Down MALI
	sys_pwr_ctrl_clr_bits(PRCM_SYS_PWR_CTRL_MALI_PWR_STATE_L2C|PRCM_SYS_PWR_CTRL_MALI_PWR_STATE_PP|PRCM_SYS_PWR_CTRL_MALI_PWR_STATE_GP,
			PRCM_SYS_PWR_STATUS_MALI_PWR_STATUS_TOTAL|PRCM_SYS_PWR_STATUS_MALI_PWR_STATUS_L2C|PRCM_SYS_PWR_STATUS_MALI_PWR_STATUS_PP|PRCM_SYS_PWR_STATUS_MALI_PWR_STATUS_GP);
	printk(KERN_NOTICE "mali power down.\n");

	return 0;
}

int ns115_mali_pll2_set_rate(unsigned long rate)
{
	unsigned int mali_clk_sel = 0x0;
	void __iomem *mali_clk_ctrl = __io_address(PRCM_MALI_CLK_CTRL);

	// set mali clock source to PLL6
	clk_set_rate_bits(mali_clk_ctrl, PRCM_MALI_CLK_CTRL_MALI_CLKEN,
			PRCM_MALI_CLK_CTRL_MALI_CLK_SEL, mali_clk_sel);

	ns115_pll2_config(rate);

	return 0;
}

int ns115_mali_clk_set_rate(struct clk *c, unsigned long rate, unsigned int nouse)
{
	void __iomem *mali_clk_ctrl = __io_address(PRCM_MALI_CLK_CTRL);
	unsigned int mali_clk_sel, mali_clk_sel_cur;
	unsigned int mali_div0 = 0;
	unsigned int mali_div1 = 0;

	if (rate == 0)
		return -EINVAL;

	switch (rate) {
		case 160000000:
			mali_clk_sel = 0x0;
			mali_div0 = 0x3;
			break;
		case 500000000:
			ns115_mali_pll2_set_rate(1000000000);
			mali_clk_sel = 0x1;
			mali_div1 = 0x2;
			break;
		case 475000000:
			ns115_mali_pll2_set_rate(950000000);
			mali_clk_sel = 0x1;
			mali_div1 = 0x2;
			break;
		case 450000000:
			ns115_mali_pll2_set_rate(900000000);
			mali_clk_sel = 0x1;
			mali_div1 = 0x2;
			break;
		case 425000000:
			ns115_mali_pll2_set_rate(850000000);
			mali_clk_sel = 0x1;
			mali_div1 = 0x2;
			break;
		case 400000000:
			mali_clk_sel = 0x1;
			mali_div1 = 0x2;
			break;
		case 360000000:
			ns115_pll6_config(720000000);
			mali_clk_sel = 0x0;
			mali_div0 = 0x2;
		case 200000000:
			mali_clk_sel = 0x1;
			mali_div1 = 0x4;
			break;
		case 266000000:
			mali_clk_sel = 0x1;
			mali_div1 = 0x3;
			break;
		case 240000000:
			mali_clk_sel = 0x0;
			mali_div0 = 0x2;
			break;
		default:
			return -EINVAL;
			break;
	}

	if (mali_clk_sel == 0) {
		clk_set_rate_bits(mali_clk_ctrl, PRCM_MALI_CLK_CTRL_MALI_DIV0_EN, 
				PRCM_MALI_CLK_CTRL_MALI_DIV0_MASK, mali_div0);
	}
	else {
		clk_set_rate_bits(mali_clk_ctrl, PRCM_MALI_CLK_CTRL_MALI_DIV1_EN, 
				PRCM_MALI_CLK_CTRL_MALI_DIV1_MASK, mali_div1);
	}

	mali_clk_sel_cur = __raw_readl(mali_clk_ctrl) & PRCM_MALI_CLK_CTRL_MALI_CLK_SEL;
	if ((mali_clk_sel << get_offset_pos(PRCM_MALI_CLK_CTRL_MALI_CLK_SEL)) != mali_clk_sel_cur) {
		clk_set_rate_bits(mali_clk_ctrl, PRCM_MALI_CLK_CTRL_MALI_CLKEN, 
				PRCM_MALI_CLK_CTRL_MALI_CLK_SEL, mali_clk_sel);
	}

	c->rate = rate;

	printk(KERN_NOTICE "mali clock rate set to %ld.\n", rate);
	return 0;
}

// GC300 clock control function/tasks
int ns115_gc300_clk_enable(struct clk *clk)
{
	unsigned int reg_value;
	void __iomem *gc300_clk_ctrl = __io_address(PRCM_GC300_CLK_CTRL);

	// Power Up GC300
	sys_pwr_ctrl_set_bits(PRCM_SYS_PWR_CTRL_GC300_PWR_STATE, PRCM_SYS_PWR_STATUS_GC300_PWR_STATUS);
	printk(KERN_NOTICE "gc300 power up.\n");

	// Enable GC300 clocks
	reg_value  = PRCM_GC300_CLK_CTRL_GC300_CLKEN  |
		PRCM_GC300_CLK_CTRL_GC300_HCLKEN |
		PRCM_GC300_CLK_CTRL_GC300_ACLKEN;
	reg_set_bits(gc300_clk_ctrl, reg_value);

	printk(KERN_NOTICE "gc300 clock enabled.\n");
	return 0;
}

int ns115_gc300_clk_disable(struct clk *clk)
{
	unsigned int reg_value;
	void __iomem *gc300_clk_ctrl = __io_address(PRCM_GC300_CLK_CTRL);

	// Disable GC300 clocks
	reg_value  = PRCM_GC300_CLK_CTRL_GC300_CLKEN  |
		PRCM_GC300_CLK_CTRL_GC300_HCLKEN |
		PRCM_GC300_CLK_CTRL_GC300_ACLKEN;
	reg_clr_bits(gc300_clk_ctrl, reg_value);

	printk(KERN_NOTICE "gc300 clock disabled.\n");

	// Power Down GC300
	sys_pwr_ctrl_clr_bits(PRCM_SYS_PWR_CTRL_GC300_PWR_STATE, PRCM_SYS_PWR_STATUS_GC300_PWR_STATUS);

	printk(KERN_NOTICE "gc300 power down.\n");
	return 0;
}

int ns115_gc300_clk_set_rate(struct clk *c, unsigned long rate, unsigned int nouse)
{
	void __iomem *gc300_clk_ctrl = __io_address(PRCM_GC300_CLK_CTRL);
	unsigned int gc300_div = 0;
	unsigned int gc300_clk_sel, gc300_clk_sel_cur;

	if (rate == 0) 
		return -EINVAL;

	switch (rate) {
		case 480000000:
			gc300_clk_sel = 0x1;
			break;
		case 400000000: 
			gc300_clk_sel = 0x0;
			gc300_div = 0x2;
			break;
		default:
			gc300_clk_sel = 0x0;
			gc300_div = 800000000/rate;
			rate = 800000000/gc300_div;
			break;
	}

	if (gc300_div >0x0FF)
		return -EINVAL;

	if (gc300_clk_sel == 0) {
		clk_set_rate_bits(gc300_clk_ctrl, PRCM_GC300_CLK_CTRL_GC300_DIV_EN,
				PRCM_GC300_CLK_CTRL_GC300_DIV_MASK, gc300_div);
	}

	gc300_clk_sel_cur = __raw_readl(gc300_clk_ctrl) & PRCM_GC300_CLK_CTRL_GC300_CLK_SEL;

	if ((gc300_clk_sel << get_offset_pos(PRCM_GC300_CLK_CTRL_GC300_CLK_SEL)) != gc300_clk_sel_cur) {
		clk_set_rate_bits(gc300_clk_ctrl, PRCM_GC300_CLK_CTRL_GC300_CLKEN, 
				PRCM_GC300_CLK_CTRL_GC300_CLK_SEL, gc300_clk_sel);
	}

	c->rate = rate;

	printk(KERN_NOTICE "gc300 clock rate set to %ld.\n", rate);
	return 0;
}

// VPU_G1 clock control function/tasks
int ns115_vpu_g1_clk_enable(struct clk *clk)
{
	unsigned int reg_value;
	void __iomem *vpu_g1_clk_ctrl = __io_address(PRCM_VPU_G1_CLK_CTRL);

	// Power Up VPU_G1
	sys_pwr_ctrl_set_bits(PRCM_SYS_PWR_CTRL_VPU_G1_PWR_STATE, PRCM_SYS_PWR_STATUS_VPU_G1_STATUS);
	printk(KERN_NOTICE "vpu_g1 power up.\n");

	// Enable VPU_G1 clocks
	reg_value = PRCM_VPU_G1_CLK_CTRL_VPU_G1_CLKEN;
	reg_set_bits(vpu_g1_clk_ctrl, reg_value);

	printk(KERN_NOTICE "vpu_g1 clock enabled.\n");
	return 0;
}

int ns115_vpu_g1_clk_disable(struct clk *clk)
{
	unsigned int reg_value;
	void __iomem *vpu_g1_clk_ctrl = __io_address(PRCM_VPU_G1_CLK_CTRL);

	// Disable VPU_G1 clocks
	reg_value = PRCM_VPU_G1_CLK_CTRL_VPU_G1_CLKEN;
	reg_clr_bits(vpu_g1_clk_ctrl, reg_value);

	printk(KERN_NOTICE "vpu_g1 clock disabled.\n");

	// Power Down VPU_G1
	sys_pwr_ctrl_clr_bits(PRCM_SYS_PWR_CTRL_VPU_G1_PWR_STATE, PRCM_SYS_PWR_STATUS_VPU_G1_STATUS);

	printk(KERN_NOTICE "vpu_g1 power down.\n");
	return 0;
}

int ns115_vpu_g1_clk_set_rate(struct clk *c, unsigned long rate, unsigned int nouse)
{
	void __iomem *vpu_g1_clk_ctrl = __io_address(PRCM_VPU_G1_CLK_CTRL);
	unsigned int vpu_g1_div0 = 0;
	unsigned int vpu_g1_div1 = 0;
	unsigned int vpu_g1_clk_sel, vpu_g1_clk_sel_cur;
	unsigned long pll4_clk_rate = clk_get_rate(&pll4_clk);

	if (rate == 0) 
		return -EINVAL;

	switch (rate) {
		case 297000000:
			if (pll4_clk_rate == 297000000) {
				//vpu_g1_clk_sel  = 0x1;
				//vpu_g1_div1     = 0x1;
				printk(KERN_NOTICE "vpu_g1 clock rate cannot set to %ld when pll4_clk_rate is 297MHz.\n", rate);
				return -EINVAL;
			}
			else if (pll4_clk_rate == 891000000) {
				vpu_g1_clk_sel  = 0x1;
				vpu_g1_div1     = 0x3;
			}
			else {
				vpu_g1_clk_sel  = 0x0;
				vpu_g1_div0     = 800000000/rate;
			}
			break;
		case 400000000:
			vpu_g1_clk_sel  = 0x0;
			vpu_g1_div0     = 0x2;
			break;
		case 266000000:
			vpu_g1_clk_sel  = 0x0;
			vpu_g1_div0     = 0x3;
			break;
		case 200000000:
			vpu_g1_clk_sel  = 0x0;
			vpu_g1_div0     = 0x4;
			break;
		default:
			vpu_g1_clk_sel  = 0x0;
			vpu_g1_div0     = 800000000/rate;
			break;
	}

	if (vpu_g1_div0 >0x0FF)
		return -EINVAL;

	if (vpu_g1_clk_sel == 0) {
		clk_set_rate_bits(vpu_g1_clk_ctrl, PRCM_VPU_G1_CLK_CTRL_VPU_G1_DIV0_EN,
				PRCM_VPU_G1_CLK_CTRL_VPU_G1_DIV0_MASK, vpu_g1_div0);
	}
	else {
		clk_set_rate_bits(vpu_g1_clk_ctrl, PRCM_VPU_G1_CLK_CTRL_VPU_G1_DIV1_EN,
				PRCM_VPU_G1_CLK_CTRL_VPU_G1_DIV1_MASK, vpu_g1_div1);
	}

	vpu_g1_clk_sel_cur = __raw_readl(vpu_g1_clk_ctrl) & PRCM_VPU_G1_CLK_CTRL_VPU_G1_CLK_SEL;

	if ((vpu_g1_clk_sel << get_offset_pos(PRCM_VPU_G1_CLK_CTRL_VPU_G1_CLK_SEL)) != vpu_g1_clk_sel_cur) {
		clk_set_rate_bits(vpu_g1_clk_ctrl, PRCM_VPU_G1_CLK_CTRL_VPU_G1_CLKEN, 
				PRCM_VPU_G1_CLK_CTRL_VPU_G1_CLK_SEL, vpu_g1_clk_sel);
	}

	c->rate = rate;

	printk(KERN_NOTICE "vpu_g1 clock rate set to %ld.\n", rate);
	return 0;
}

// VPU_H1 clock control function/tasks
int ns115_vpu_h1_clk_enable(struct clk *clk)
{
	unsigned int reg_value;
	void __iomem *vpu_h1_clk_ctrl = __io_address(PRCM_VPU_H1_CLK_CTRL);

	// Power Up VPU_H1
	sys_pwr_ctrl_set_bits(PRCM_SYS_PWR_CTRL_VPU_H1_PWR_STATE, PRCM_SYS_PWR_STATUS_VPU_H1_STATUS);
	printk(KERN_NOTICE "vpu_h1 power up.\n");

	// Enable VPU_H1 clocks
	reg_value = PRCM_VPU_H1_CLK_CTRL_VPU_H1_CLKEN;
	reg_set_bits(vpu_h1_clk_ctrl, reg_value);

	printk(KERN_NOTICE "vpu_h1 clock enabled.\n");
	return 0;
}

int ns115_vpu_h1_clk_disable(struct clk *clk)
{
	unsigned int reg_value;
	void __iomem *vpu_h1_clk_ctrl = __io_address(PRCM_VPU_H1_CLK_CTRL);

	// Disable VPU_H1 clocks
	reg_value = PRCM_VPU_H1_CLK_CTRL_VPU_H1_CLKEN;
	reg_clr_bits(vpu_h1_clk_ctrl, reg_value);

	printk(KERN_NOTICE "vpu_h1 clock disabled.\n");

	// Power Down VPU_H1
	sys_pwr_ctrl_clr_bits(PRCM_SYS_PWR_CTRL_VPU_H1_PWR_STATE, PRCM_SYS_PWR_STATUS_VPU_H1_STATUS);

	printk(KERN_NOTICE "vpu_h1 power down.\n");
	return 0;
}

int ns115_vpu_h1_clk_set_rate(struct clk *c, unsigned long rate, unsigned int nouse)
{
	void __iomem *vpu_h1_clk_ctrl = __io_address(PRCM_VPU_H1_CLK_CTRL);
	unsigned int vpu_h1_div0 = 0;
	unsigned int vpu_h1_div1 = 0;
	unsigned int vpu_h1_clk_sel, vpu_h1_clk_sel_cur;

	if (rate == 0) 
		return -EINVAL;

	switch (rate) {
		case 160000000:
			vpu_h1_clk_sel  = 0x0;
			vpu_h1_div0     = 0x3;
			break;
		case 200000000:
			vpu_h1_clk_sel  = 0x1;
			vpu_h1_div1     = 0x4;
			break;
		case 240000000:
			vpu_h1_clk_sel  = 0x0;
			vpu_h1_div0     = 0x2;
			break;
		default:
			return -EINVAL;
			break;
	}


	if (vpu_h1_clk_sel == 0) {
		clk_set_rate_bits(vpu_h1_clk_ctrl, PRCM_VPU_H1_CLK_CTRL_VPU_H1_DIV0_EN,
				PRCM_VPU_H1_CLK_CTRL_VPU_H1_DIV0_MASK, vpu_h1_div0);
	}
	else {
		clk_set_rate_bits(vpu_h1_clk_ctrl, PRCM_VPU_H1_CLK_CTRL_VPU_H1_DIV1_EN,
				PRCM_VPU_H1_CLK_CTRL_VPU_H1_DIV1_MASK, vpu_h1_div1);
	}

	vpu_h1_clk_sel_cur = __raw_readl(vpu_h1_clk_ctrl) & PRCM_VPU_H1_CLK_CTRL_VPU_H1_CLK_SEL;

	if ((vpu_h1_clk_sel << get_offset_pos(PRCM_VPU_H1_CLK_CTRL_VPU_H1_CLK_SEL)) != vpu_h1_clk_sel_cur) {
		clk_set_rate_bits(vpu_h1_clk_ctrl, PRCM_VPU_H1_CLK_CTRL_VPU_H1_CLKEN, 
				PRCM_VPU_H1_CLK_CTRL_VPU_H1_CLK_SEL, vpu_h1_clk_sel);
	}

	c->rate = rate;

	printk(KERN_NOTICE "vpu_h1 clock rate set to %ld.\n", rate);
	return 0;
}

// ISP JPEG clock control function/tasks
int ns115_isp_jpeg_clk_enable(struct clk *clk)
{
	unsigned int reg_value;
	void __iomem *isp_jpeg_clk_ctrl = __io_address(PRCM_ISP_CLK_CTRL);

	// Power Up ISP 
	sys_pwr_ctrl_set_bits(PRCM_SYS_PWR_CTRL_ISP_PWR_STATE, PRCM_SYS_PWR_STATUS_ISP_STATUS);
	printk(KERN_NOTICE "isp power up.\n");

	// Enable ISP clocks
	reg_value = PRCM_ISP_CLK_CTRL_ISP_JPEGCLKEN |
		PRCM_ISP_CLK_CTRL_ISP_S_HCLKEN  |
		PRCM_ISP_CLK_CTRL_ISP_M_HCLKEN;
	reg_set_bits(isp_jpeg_clk_ctrl, reg_value);

	printk(KERN_NOTICE "isp_jpeg clock enabled.\n");
	return 0;
}

int ns115_isp_jpeg_clk_disable(struct clk *clk)
{
	unsigned int reg_value;
	void __iomem *isp_jpeg_clk_ctrl = __io_address(PRCM_ISP_CLK_CTRL);

	// Disable ISP clocks
	reg_value = PRCM_ISP_CLK_CTRL_ISP_JPEGCLKEN |
		PRCM_ISP_CLK_CTRL_ISP_S_HCLKEN  |
		PRCM_ISP_CLK_CTRL_ISP_M_HCLKEN;
	reg_clr_bits(isp_jpeg_clk_ctrl, reg_value);

	printk(KERN_NOTICE "isp_jpeg clock disabled.\n");

	// Power Down ISP 
	sys_pwr_ctrl_clr_bits(PRCM_SYS_PWR_CTRL_ISP_PWR_STATE, PRCM_SYS_PWR_STATUS_ISP_STATUS);

	printk(KERN_NOTICE "isp power down.\n");
	return 0;
}

int ns115_isp_jpeg_clk_set_rate(struct clk *c, unsigned long rate, unsigned int nouse)
{
	void __iomem *isp_clk_ctrl = __io_address(PRCM_ISP_CLK_CTRL);
	unsigned int isp_div = 800000000/rate;

	if (rate == 0) 
		return -EINVAL;

	if (isp_div > 0x0F)
		return -EINVAL;

	clk_set_rate_bits(isp_clk_ctrl, PRCM_ISP_CLK_CTRL_ISP_JPEG_DIV_EN,
			PRCM_ISP_CLK_CTRL_ISP_JPEG_DIV_MASK, isp_div);

	rate = 800000000/isp_div;
	c->rate = rate; 

	printk(KERN_NOTICE "isp_jpeg clock rate set to %ld.\n", rate);
	return 0;

}

// ISP clock control function/tasks
int ns115_isp_clk_enable(struct clk *clk)
{
	unsigned int reg_value;
	void __iomem *isp_clk_ctrl = __io_address(PRCM_ISP_CLK_CTRL);
	void __iomem *isp_master_rst = __io_address(NS115_L3_ISS_TA_BASE + 0x20);
	void __iomem *isp_slave_rst = __io_address(NS115_CAMERIC_TA_BASE + 0x20);

	// Enable ISP clocks
	reg_value = PRCM_ISP_CLK_CTRL_ISP_CLKEN |
		PRCM_ISP_CLK_CTRL_ISP_S_HCLKEN |
		PRCM_ISP_CLK_CTRL_ISP_M_HCLKEN;
	reg_set_bits(isp_clk_ctrl, reg_value);
	printk(KERN_NOTICE "isp clock enabled.\n");

	// Power Up ISP
	sys_pwr_ctrl_set_bits(PRCM_SYS_PWR_CTRL_ISP_PWR_STATE, PRCM_SYS_PWR_STATUS_ISP_STATUS);
	printk(KERN_NOTICE "isp power up.\n");

	//Reset ISP
	reg_value = PRCM_ISP_CLK_CTRL_ISP_RSTN;
	reg_clr_bits(isp_clk_ctrl, reg_value);
	reg_set_bits(isp_master_rst, 1);
	reg_set_bits(isp_slave_rst, 1);
	udelay(10);
	reg_set_bits(isp_clk_ctrl, reg_value);
	reg_clr_bits(isp_master_rst, 1);
	reg_clr_bits(isp_slave_rst, 1);
	udelay(10);

	return 0;
}

int ns115_isp_clk_disable(struct clk *clk)
{
	unsigned int reg_value;
	void __iomem *isp_clk_ctrl = __io_address(PRCM_ISP_CLK_CTRL);

	// Power Down ISP
	sys_pwr_ctrl_clr_bits(PRCM_SYS_PWR_CTRL_ISP_PWR_STATE, PRCM_SYS_PWR_STATUS_ISP_STATUS);
	printk(KERN_NOTICE "isp power down.\n");

	// Disable ISP clocks
	reg_value = PRCM_ISP_CLK_CTRL_ISP_CLKEN |
		PRCM_ISP_CLK_CTRL_ISP_S_HCLKEN |
		PRCM_ISP_CLK_CTRL_ISP_M_HCLKEN;
	reg_clr_bits(isp_clk_ctrl, reg_value);
	printk(KERN_NOTICE "isp clock disabled.\n");

	return 0;
}

// SD clock control function/tasks
int ns115_sd_clk_enable(struct clk *clk)
{
	unsigned int reg_value;
	void __iomem *sd_clk_ctrl = __io_address(PRCM_SD_CLK_CTRL);

	reg_value = PRCM_SD_CLK_CTRL_SD_CLKEN |
		PRCM_SD_CLK_CTRL_SD_HCLKEN;
	reg_set_bits(sd_clk_ctrl, reg_value);

	printk(KERN_NOTICE "sd clock enabled.\n");
	return 0;
}

int ns115_sd_clk_disable(struct clk *clk)
{
	unsigned int reg_value;
	void __iomem *sd_clk_ctrl = __io_address(PRCM_SD_CLK_CTRL);

	reg_value = PRCM_SD_CLK_CTRL_SD_CLKEN |
		PRCM_SD_CLK_CTRL_SD_HCLKEN;
	reg_clr_bits(sd_clk_ctrl, reg_value);

	printk(KERN_NOTICE "sd clock disabled.\n");
	return 0;
}

int ns115_sd_clk_set_rate(struct clk *c, unsigned long rate, unsigned int nouse)
{
	void __iomem *sd_clk_ctrl = __io_address(PRCM_SD_CLK_CTRL);
	unsigned int sd_div = 800000000/rate;

	if (rate == 0) 
		return -EINVAL;

	if (sd_div > 0x0F)
		return -EINVAL;

	clk_set_rate_bits(sd_clk_ctrl, PRCM_SD_CLK_CTRL_SD_DIV_EN,
			PRCM_SD_CLK_CTRL_SD_DIV_MASK, sd_div);

	rate = 800000000/sd_div;
	c->rate = rate; 

	printk(KERN_NOTICE "sd clock rate set to %ld.\n", rate);
	return 0;
}

// ZSP clock control function/tasks
int ns115_zsp_clk_enable(struct clk *clk)
{
	unsigned int reg_value;
	void __iomem *zsp_clk_ctrl = __io_address(PRCM_ZSP_CLK_CTRL);

	// Power Up ZSP
	sys_pwr_ctrl_set_bits(PRCM_SYS_PWR_CTRL_ZSP_PWR_STATE, PRCM_SYS_PWR_STATUS_ZSP_STATUS);
	printk(KERN_NOTICE "zsp power up.\n");

	// Enable ZSP clocks
	reg_value = PRCM_ZSP_CLK_CTRL_ZSP_CLKEN |
		PRCM_ZSP_CLK_CTRL_ZSP_ACLKEN;
	reg_set_bits(zsp_clk_ctrl, reg_value);

	printk(KERN_NOTICE "zsp clock enabled.\n");
	return 0;
}

int ns115_zsp_clk_disable(struct clk *clk)
{
	unsigned int reg_value;
	void __iomem *zsp_clk_ctrl = __io_address(PRCM_ZSP_CLK_CTRL);

	// Disable ZSP clocks
	reg_value = PRCM_ZSP_CLK_CTRL_ZSP_CLKEN |
		PRCM_ZSP_CLK_CTRL_ZSP_ACLKEN;
	reg_clr_bits(zsp_clk_ctrl, reg_value);

	printk(KERN_NOTICE "zsp clock disabled.\n");

	// Power Down ZSP
	sys_pwr_ctrl_clr_bits(PRCM_SYS_PWR_CTRL_ZSP_PWR_STATE, PRCM_SYS_PWR_STATUS_ZSP_STATUS);

	printk(KERN_NOTICE "zsp power down.\n");
	return 0;
}

// SPI0 clock control function/tasks
int ns115_spi0_clk_enable(struct clk *clk)
{
	unsigned int reg_value;
	void __iomem *spi_clk_ctrl = __io_address(PRCM_SPI_CLK_CTRL);

	reg_value = PRCM_SPI_CLK_CTRL_SPI0_CLKEN |
		PRCM_SPI_CLK_CTRL_SPI0_PCLKEN;
	reg_set_bits(spi_clk_ctrl, reg_value);

	printk(KERN_NOTICE "spi0 clock enabled.\n");
	return 0;
}

int ns115_spi0_clk_disable(struct clk *clk)
{
	unsigned int reg_value;
	void __iomem *spi_clk_ctrl = __io_address(PRCM_SPI_CLK_CTRL);

	reg_value = PRCM_SPI_CLK_CTRL_SPI0_CLKEN |
		PRCM_SPI_CLK_CTRL_SPI0_PCLKEN;
	reg_clr_bits(spi_clk_ctrl, reg_value);

	printk(KERN_NOTICE "spi0 clock disabled.\n");
	return 0;
}

int ns115_spi01_clk_set_rate(struct clk *c, unsigned long rate, unsigned int nouse)
{
	void __iomem *spi_clk_ctrl = __io_address(PRCM_SPI_CLK_CTRL);
	unsigned int spi01_div = 800000000/rate;

	if (rate == 0) 
		return -EINVAL;

	if (spi01_div > 0x3F)
		return -EINVAL;

	clk_set_rate_bits(spi_clk_ctrl, PRCM_SPI_CLK_CTRL_SPI01_DIV_EN,
			PRCM_SPI_CLK_CTRL_SPI01_DIV_MASK, spi01_div);

	rate = 800000000/spi01_div;
	c->rate = rate; 

	printk(KERN_NOTICE "spi01 clock rate set to %ld.\n", rate);
	return 0;
}

// SPI1 clock control function/tasks
int ns115_spi1_clk_enable(struct clk *clk)
{
	unsigned int reg_value;
	void __iomem *spi_clk_ctrl = __io_address(PRCM_SPI_CLK_CTRL);

	reg_value = PRCM_SPI_CLK_CTRL_SPI1_CLKEN |
		PRCM_SPI_CLK_CTRL_SPI1_PCLKEN;
	reg_set_bits(spi_clk_ctrl, reg_value);

	printk(KERN_NOTICE "spi1 clock enabled.\n");
	return 0;
}

int ns115_spi1_clk_disable(struct clk *clk)
{
	unsigned int reg_value;
	void __iomem *spi_clk_ctrl = __io_address(PRCM_SPI_CLK_CTRL);

	reg_value = PRCM_SPI_CLK_CTRL_SPI1_CLKEN |
		PRCM_SPI_CLK_CTRL_SPI1_PCLKEN;
	reg_clr_bits(spi_clk_ctrl, reg_value);

	printk(KERN_NOTICE "spi1 clock disabled.\n");
	return 0;
}

// SPI2 clock control function/tasks
int ns115_spi2_clk_enable(struct clk *clk)
{
	unsigned int reg_value;
	void __iomem *spi_clk_ctrl = __io_address(PRCM_SPI_CLK_CTRL);

	reg_value = PRCM_SPI_CLK_CTRL_SPI2_CLKEN |
		PRCM_SPI_CLK_CTRL_SPI2_PCLKEN;
	reg_set_bits(spi_clk_ctrl, reg_value);

	printk(KERN_NOTICE "spi2 clock enabled.\n");
	return 0;
}

int ns115_spi2_clk_disable(struct clk *clk)
{
	unsigned int reg_value;
	void __iomem *spi_clk_ctrl = __io_address(PRCM_SPI_CLK_CTRL);

	reg_value = PRCM_SPI_CLK_CTRL_SPI2_CLKEN |
		PRCM_SPI_CLK_CTRL_SPI2_PCLKEN;
	reg_clr_bits(spi_clk_ctrl, reg_value);

	printk(KERN_NOTICE "spi2 clock disabled.\n");
	return 0;
}

int ns115_spi23_clk_set_rate(struct clk *c, unsigned long rate, unsigned int nouse)
{
	void __iomem *spi_clk_ctrl = __io_address(PRCM_SPI_CLK_CTRL);
	unsigned int spi23_div = 800000000/rate;

	if (rate == 0) 
		return -EINVAL;

	if (spi23_div > 0x7)
		return -EINVAL;

	clk_set_rate_bits(spi_clk_ctrl, PRCM_SPI_CLK_CTRL_SPI2_DIV_EN,
			PRCM_SPI_CLK_CTRL_SPI2_DIV_MASK, spi23_div);

	rate = 800000000/spi23_div;
	c->rate = rate; 

	printk(KERN_NOTICE "spi23 clock rate set to %ld.\n", rate);
	return 0;
}

// SPI3 clock control function/tasks
int ns115_spi3_clk_enable(struct clk *clk)
{
	unsigned int reg_value;
	void __iomem *spi_clk_ctrl = __io_address(PRCM_SPI_CLK_CTRL);

	reg_value = PRCM_SPI_CLK_CTRL_SPI3_CLKEN |
		PRCM_SPI_CLK_CTRL_SPI3_PCLKEN;
	reg_set_bits(spi_clk_ctrl, reg_value);

	printk(KERN_NOTICE "spi3 clock enabled.\n");
	return 0;
}

int ns115_spi3_clk_disable(struct clk *clk)
{
	unsigned int reg_value;
	void __iomem *spi_clk_ctrl = __io_address(PRCM_SPI_CLK_CTRL);

	reg_value = PRCM_SPI_CLK_CTRL_SPI3_CLKEN |
		PRCM_SPI_CLK_CTRL_SPI3_PCLKEN;
	reg_clr_bits(spi_clk_ctrl, reg_value);

	printk(KERN_NOTICE "spi3 clock disabled.\n");
	return 0;
}

// I2C0 clock control function/tasks
int ns115_i2c0_clk_enable(struct clk *clk)
{
	unsigned int reg_value;
	void __iomem *i2c_clk_ctrl = __io_address(PRCM_I2C_CLK_CTRL);

	reg_value = PRCM_I2C_CLK_CTRL_I2C0_CLKEN |
		PRCM_I2C_CLK_CTRL_I2C0_PCLKEN;
	reg_set_bits(i2c_clk_ctrl, reg_value);

	printk(KERN_NOTICE "i2c0 clock enabled.\n");
	return 0;
}

int ns115_i2c0_clk_disable(struct clk *clk)
{
	unsigned int reg_value;
	void __iomem *i2c_clk_ctrl = __io_address(PRCM_I2C_CLK_CTRL);

	reg_value = PRCM_I2C_CLK_CTRL_I2C0_CLKEN |
		PRCM_I2C_CLK_CTRL_I2C0_PCLKEN;
	reg_clr_bits(i2c_clk_ctrl, reg_value);

	printk(KERN_NOTICE "i2c0 clock disabled.\n");
	return 0;
}

int ns115_i2c0_clk_reset(struct clk *clk, bool flag)
{
       unsigned int reg_value;
       void __iomem *i2c_clk_ctrl = __io_address(PRCM_I2C_CLK_CTRL);

       reg_value = PRCM_I2C_CLK_CTRL_I2C0_RSTN;

       if (true == flag) {
               reg_clr_bits(i2c_clk_ctrl, reg_value);//low active reset assert
       } else {

               reg_set_bits(i2c_clk_ctrl, reg_value);//high active reset desert
       }

       printk(KERN_ERR "i2c0 clock reset.\n");
       return 0;
}

int ns115_i2c_fs_clk_set_rate(struct clk *c, unsigned long rate, unsigned int nouse)
{
	void __iomem *i2c_clk_ctrl = __io_address(PRCM_I2C_CLK_CTRL);
	unsigned int i2c_fs_div = 800000000/rate;

	if (rate == 0) 
		return -EINVAL;

	if (i2c_fs_div > 0x3F)
		return -EINVAL;

	clk_set_rate_bits(i2c_clk_ctrl, PRCM_I2C_CLK_CTRL_I2C_FS_DIV_EN,
			PRCM_I2C_CLK_CTRL_I2C_FS_DIV_MASK, i2c_fs_div);

	rate = 800000000/i2c_fs_div;
	c->rate = rate; 

	printk(KERN_NOTICE "i2c_fs clock rate set to %ld.\n", rate);
	return 0;
}

// I2C1 clock control function/tasks
int ns115_i2c1_clk_enable(struct clk *clk)
{
	unsigned int reg_value;
	void __iomem *i2c_clk_ctrl = __io_address(PRCM_I2C_CLK_CTRL);

	reg_value = PRCM_I2C_CLK_CTRL_I2C1_CLKEN |
		PRCM_I2C_CLK_CTRL_I2C1_PCLKEN;
	reg_set_bits(i2c_clk_ctrl, reg_value);

	printk(KERN_NOTICE "i2c1 clock enabled.\n");
	return 0;
}

int ns115_i2c1_clk_disable(struct clk *clk)
{
	unsigned int reg_value;
	void __iomem *i2c_clk_ctrl = __io_address(PRCM_I2C_CLK_CTRL);

	reg_value = PRCM_I2C_CLK_CTRL_I2C1_CLKEN |
		PRCM_I2C_CLK_CTRL_I2C1_PCLKEN;
	reg_clr_bits(i2c_clk_ctrl, reg_value);

	printk(KERN_NOTICE "i2c1 clock disabled.\n");
	return 0;
}

int ns115_i2c1_clk_reset(struct clk *clk, bool flag)
{
	unsigned int reg_value;
	void __iomem *i2c_clk_ctrl = __io_address(PRCM_I2C_CLK_CTRL);

	reg_value = PRCM_I2C_CLK_CTRL_I2C1_RSTN;

	if (true == flag) {
		reg_clr_bits(i2c_clk_ctrl, reg_value);//low active reset assert
	} else {

	       reg_set_bits(i2c_clk_ctrl, reg_value);//high active reset desert
	}

	printk(KERN_ERR "i2c1 clock reset.\n");
	return 0;
}

// I2C2 clock control function/tasks
int ns115_i2c2_clk_enable(struct clk *clk)
{
	unsigned int reg_value;
	void __iomem *i2c_clk_ctrl = __io_address(PRCM_I2C_CLK_CTRL);

	reg_value = PRCM_I2C_CLK_CTRL_I2C2_CLKEN |
		PRCM_I2C_CLK_CTRL_I2C2_PCLKEN;
	reg_set_bits(i2c_clk_ctrl, reg_value);

	printk(KERN_NOTICE "i2c2 clock enabled.\n");
	return 0;
}

int ns115_i2c2_clk_disable(struct clk *clk)
{
	unsigned int reg_value;
	void __iomem *i2c_clk_ctrl = __io_address(PRCM_I2C_CLK_CTRL);

	reg_value = PRCM_I2C_CLK_CTRL_I2C2_CLKEN |
		PRCM_I2C_CLK_CTRL_I2C2_PCLKEN;
	reg_clr_bits(i2c_clk_ctrl, reg_value);

	printk(KERN_NOTICE "i2c2 clock disabled.\n");
	return 0;
}

int ns115_i2c2_clk_reset(struct clk *clk, bool flag)
{
	unsigned int reg_value;
	void __iomem *i2c_clk_ctrl = __io_address(PRCM_I2C_CLK_CTRL);

	reg_value = PRCM_I2C_CLK_CTRL_I2C2_RSTN;
	if (true == flag) {
		reg_clr_bits(i2c_clk_ctrl, reg_value);//low active reset assert
	} else {
		reg_set_bits(i2c_clk_ctrl, reg_value);//high active reset desert
	}
	printk(KERN_ERR "i2c2 clock reset.\n");
	return 0;
}

// I2C3 clock control function/tasks
int ns115_i2c3_clk_enable(struct clk *clk)
{
	unsigned int reg_value;
	void __iomem *i2c_clk_ctrl = __io_address(PRCM_I2C_CLK_CTRL);

	reg_value = PRCM_I2C_CLK_CTRL_I2C3_CLKEN |
		PRCM_I2C_CLK_CTRL_I2C3_PCLKEN;
	reg_set_bits(i2c_clk_ctrl, reg_value);

	printk(KERN_NOTICE "i2c3 clock enabled.\n");
	return 0;
}

int ns115_i2c3_clk_disable(struct clk *clk)
{
	unsigned int reg_value;
	void __iomem *i2c_clk_ctrl = __io_address(PRCM_I2C_CLK_CTRL);

	reg_value = PRCM_I2C_CLK_CTRL_I2C3_CLKEN |
		PRCM_I2C_CLK_CTRL_I2C3_PCLKEN;
	reg_clr_bits(i2c_clk_ctrl, reg_value);

	printk(KERN_NOTICE "i2c3 clock disabled.\n");
	return 0;
}

int ns115_i2c3_clk_reset(struct clk *clk, bool flag)
{
	unsigned int reg_value;
	void __iomem *i2c_clk_ctrl = __io_address(PRCM_I2C_CLK_CTRL);

	reg_value = PRCM_I2C_CLK_CTRL_I2C3_RSTN;
	if (true == flag) {
		reg_clr_bits(i2c_clk_ctrl, reg_value);//low active reset assert
	} else {
		reg_set_bits(i2c_clk_ctrl, reg_value);//high active reset desert
	}
	printk(KERN_ERR "i2c3 clock reset.\n");
	return 0;
}

int ns115_i2c3_clk_reset_desert(struct clk *clk)
{
	unsigned int reg_value;
	void __iomem *i2c_clk_ctrl = __io_address(PRCM_I2C_CLK_CTRL);

	reg_value = PRCM_I2C_CLK_CTRL_I2C3_RSTN;
	reg_set_bits(i2c_clk_ctrl, reg_value);//low active
	printk(KERN_ERR "i2c3 clock reset desert.\n");
	return 0;
}

int ns115_i2c_hs_clk_set_rate(struct clk *c, unsigned long rate, unsigned int nouse)
{
	void __iomem *i2c_clk_ctrl = __io_address(PRCM_I2C_CLK_CTRL);
	unsigned int i2c_hs_div = 800000000/rate;

	if (rate == 0) 
		return -EINVAL;

	if (i2c_hs_div > 0x0F)
		return -EINVAL;

	clk_set_rate_bits(i2c_clk_ctrl, PRCM_I2C_CLK_CTRL_I2C_HS_DIV_EN,
			PRCM_I2C_CLK_CTRL_I2C_HS_DIV_MASK, i2c_hs_div);

	rate = 800000000/i2c_hs_div;
	c->rate = rate; 

	printk(KERN_NOTICE "i2c_hs clock rate set to %ld.\n", rate);
	return 0;
}

// UART0 clock control function/tasks
int ns115_uart0_clk_enable(struct clk *clk)
{
	unsigned int reg_value;
	void __iomem *uart_clk_ctrl = __io_address(PRCM_UART_CLK_CTRL1);

	reg_value = PRCM_UART_CLK_CTRL1_UART0_CLKEN |
		PRCM_UART_CLK_CTRL1_UART0_PCLKEN;
	reg_set_bits(uart_clk_ctrl, reg_value);

	printk(KERN_NOTICE "uart0 clock enabled.\n");
	return 0;
}

int ns115_uart0_clk_disable(struct clk *clk)
{
	unsigned int reg_value;
	void __iomem *uart_clk_ctrl = __io_address(PRCM_UART_CLK_CTRL1);

	reg_value = PRCM_UART_CLK_CTRL1_UART0_CLKEN |
		PRCM_UART_CLK_CTRL1_UART0_PCLKEN;
	reg_clr_bits(uart_clk_ctrl, reg_value);

	printk(KERN_NOTICE "uart0 clock disabled.\n");
	return 0;
}

int ns115_uart01_clk_set_rate(struct clk *c, unsigned long rate, unsigned int nouse)
{
	void __iomem *uart_clk_ctrl0 = __io_address(PRCM_UART_CLK_CTRL0);
	unsigned int uart01_div = 800000000/rate;

	if (rate == 0) 
		return -EINVAL;

	if (uart01_div > 0x1F)
		return -EINVAL;

	clk_set_rate_bits(uart_clk_ctrl0, PRCM_UART_CLK_CTRL0_UART01_DIV_EN,
			PRCM_UART_CLK_CTRL0_UART01_DIV_MASK, uart01_div);

	rate = 800000000/uart01_div;
	c->rate = rate; 

	printk(KERN_NOTICE "uart01 clock rate set to %ld.\n", rate);
	return 0;
}

// UART1 clock control function/tasks
int ns115_uart1_clk_enable(struct clk *clk)
{
	unsigned int reg_value;
	void __iomem *uart_clk_ctrl = __io_address(PRCM_UART_CLK_CTRL1);

	reg_value = PRCM_UART_CLK_CTRL1_UART1_CLKEN |
		PRCM_UART_CLK_CTRL1_UART1_PCLKEN;
	reg_set_bits(uart_clk_ctrl, reg_value);

	printk(KERN_NOTICE "uart1 clock enabled.\n");
	return 0;
}

int ns115_uart1_clk_disable(struct clk *clk)
{
	unsigned int reg_value;
	void __iomem *uart_clk_ctrl = __io_address(PRCM_UART_CLK_CTRL1);

	reg_value = PRCM_UART_CLK_CTRL1_UART1_CLKEN |
		PRCM_UART_CLK_CTRL1_UART1_PCLKEN;
	reg_clr_bits(uart_clk_ctrl, reg_value);

	printk(KERN_NOTICE "uart1 clock disabled.\n");
	return 0;
}

// UART2 clock control function/tasks
int ns115_uart2_clk_enable(struct clk *clk)
{
	unsigned int reg_value;
	void __iomem *uart_clk_ctrl = __io_address(PRCM_UART_CLK_CTRL1);

	reg_value = PRCM_UART_CLK_CTRL1_UART2_CLKEN |
		PRCM_UART_CLK_CTRL1_UART2_PCLKEN;
	reg_set_bits(uart_clk_ctrl, reg_value);

	printk(KERN_NOTICE "uart2 clock enabled.\n");
	return 0;
}

int ns115_uart2_clk_disable(struct clk *clk)
{
	unsigned int reg_value;
	void __iomem *uart_clk_ctrl = __io_address(PRCM_UART_CLK_CTRL1);

	reg_value = PRCM_UART_CLK_CTRL1_UART2_CLKEN |
		PRCM_UART_CLK_CTRL1_UART2_PCLKEN;
	reg_clr_bits(uart_clk_ctrl, reg_value);

	printk(KERN_NOTICE "uart2 clock disabled.\n");
	return 0;
}

int ns115_uart2_clk_set_rate(struct clk *c, unsigned long rate, unsigned int nouse)
{
	void __iomem *uart_clk_ctrl0 = __io_address(PRCM_UART_CLK_CTRL0);
	unsigned int uart2_div = 800000000/rate;

	if (rate == 0) 
		return -EINVAL;

	if (uart2_div > 0x1F)
		return -EINVAL;

	clk_set_rate_bits(uart_clk_ctrl0, PRCM_UART_CLK_CTRL0_UART2_DIV_EN,
			PRCM_UART_CLK_CTRL0_UART2_DIV_MASK, uart2_div);

	rate = 800000000/uart2_div;
	c->rate = rate; 

	printk(KERN_NOTICE "uart2 clock rate set to %ld.\n", rate);
	return 0;
}

// UART3 clock control function/tasks
int ns115_uart3_clk_enable(struct clk *clk)
{
	unsigned int reg_value;
	void __iomem *uart_clk_ctrl = __io_address(PRCM_UART_CLK_CTRL1);

	reg_value = PRCM_UART_CLK_CTRL1_UART3_CLKEN |
		PRCM_UART_CLK_CTRL1_UART3_PCLKEN;
	reg_set_bits(uart_clk_ctrl, reg_value);

	printk(KERN_NOTICE "uart3 clock enabled.\n");
	return 0;
}

int ns115_uart3_clk_disable(struct clk *clk)
{
	unsigned int reg_value;
	void __iomem *uart_clk_ctrl = __io_address(PRCM_UART_CLK_CTRL1);

	reg_value = PRCM_UART_CLK_CTRL1_UART3_CLKEN |
		PRCM_UART_CLK_CTRL1_UART3_PCLKEN;
	reg_clr_bits(uart_clk_ctrl, reg_value);

	printk(KERN_NOTICE "uart3 clock disabled.\n");
	return 0;
}

int ns115_uart3_clk_set_rate(struct clk *c, unsigned long rate, unsigned int nouse)
{
	void __iomem *uart_clk_ctrl0 = __io_address(PRCM_UART_CLK_CTRL0);
	unsigned int uart3_div = 800000000/rate;

	if (rate == 0) 
		return -EINVAL;

	if (uart3_div > 0x1F)
		return -EINVAL;

	clk_set_rate_bits(uart_clk_ctrl0, PRCM_UART_CLK_CTRL0_UART3_DIV_EN,
			PRCM_UART_CLK_CTRL0_UART3_DIV_MASK, uart3_div);

	rate = 800000000/uart3_div;
	c->rate = rate; 

	printk(KERN_NOTICE "uart3 clock rate set to %ld.\n", rate);
	return 0;
}

// RGBO clock control function/tasks
int ns115_rgbo_clk_enable(struct clk *clk)
{
	unsigned int reg_value;
	void __iomem *dis0_clk_ctrl = __io_address(PRCM_DIS0_CLK_CTRL);

	reg_value = PRCM_DIS0_CLK_CTRL_RGBO_CLKEN;
	reg_set_bits(dis0_clk_ctrl, reg_value);

	printk(KERN_NOTICE "rgbo clock enabled.\n");
	return 0;
}

int ns115_rgbo_clk_disable(struct clk *clk)
{
	unsigned int reg_value;
	void __iomem *dis0_clk_ctrl = __io_address(PRCM_DIS0_CLK_CTRL);

	reg_value =PRCM_DIS0_CLK_CTRL_RGBO_CLKEN;
	reg_clr_bits(dis0_clk_ctrl, reg_value);

	printk(KERN_NOTICE "rgbo clock disabled.\n");
	return 0;
}

int ns115_pxl0_clk_set_rate(struct clk *c, unsigned long rate, unsigned int nouse)
{
	void __iomem *dis0_clk_ctrl = __io_address(PRCM_DIS0_CLK_CTRL);
	unsigned long pll3_clk_rate;
	unsigned int pxl0_div = 2;
	unsigned int rgbo_inv_clk_sel = 1;

	nouse = 0;

	if (rate == 0) 
		return -EINVAL;

	if (pxl0_div > 0x0F)
		return -EINVAL;

	// Configure PLL3 clock freq to rate
	if(ns115_pll3_clk_set_rate(&pll3_clk, rate*2, nouse) == -EINVAL) {
		printk(KERN_NOTICE "Failed to set pxl0 clock rate to %ld.\n", rate);
		return -EINVAL;
	}

	// Set pxl0_div to 2
	pxl0_div = 2;
	clk_set_rate_bits(dis0_clk_ctrl, PRCM_DIS0_CLK_CTRL_PXL0_DIV_EN,
			PRCM_DIS0_CLK_CTRL_PXL0_DIV_MASK, pxl0_div);

	clk_set_rate_bits(dis0_clk_ctrl, PRCM_DIS0_CLK_CTRL_RGBO_CLKEN,
			PRCM_DIS0_CLK_CTRL_RGBO_INV_CLK_SEL, rgbo_inv_clk_sel);

	pll3_clk_rate = clk_get_rate(&pll3_clk);
	rate = pll3_clk_rate/pxl0_div;
	c->rate = rate; 

	printk(KERN_NOTICE "pxl0 clock rate set to %ld.\n", rate);
	return 0;
}

// LCDC0 clock control function/tasks
int ns115_lcdc0_clk_enable(struct clk *clk)
{
	unsigned int reg_value;
	void __iomem *dis0_clk_ctrl = __io_address(PRCM_DIS0_CLK_CTRL);

	reg_value = PRCM_DIS0_CLK_CTRL_LCDC0_CLKEN;
	reg_set_bits(dis0_clk_ctrl, reg_value);

	printk(KERN_NOTICE "lcdc0 clock enabled.\n");
	return 0;
}

int ns115_lcdc0_clk_disable(struct clk *clk)
{
	unsigned int reg_value;
	void __iomem *dis0_clk_ctrl = __io_address(PRCM_DIS0_CLK_CTRL);

	reg_value = PRCM_DIS0_CLK_CTRL_LCDC0_CLKEN ;
	reg_clr_bits(dis0_clk_ctrl, reg_value);

	printk(KERN_NOTICE "lcdc0 clock disabled.\n");
	return 0;
}

int ns115_lcdc0_bus_clk_enable(struct clk *clk)
{
	unsigned int reg_value;
	void __iomem *dis0_clk_ctrl = __io_address(PRCM_DIS0_CLK_CTRL);

	reg_value = PRCM_DIS0_CLK_CTRL_LCDC0_HCLKEN |
		PRCM_DIS0_CLK_CTRL_LCDC0_ACLKEN;
	reg_set_bits(dis0_clk_ctrl, reg_value);

	printk(KERN_NOTICE "lcdc0 bus clock enabled.\n");
	return 0;
}

int ns115_lcdc0_bus_clk_disable(struct clk *clk)
{
	unsigned int reg_value;
	void __iomem *dis0_clk_ctrl = __io_address(PRCM_DIS0_CLK_CTRL);

	reg_value = PRCM_DIS0_CLK_CTRL_LCDC0_HCLKEN |
		PRCM_DIS0_CLK_CTRL_LCDC0_ACLKEN;
	reg_clr_bits(dis0_clk_ctrl, reg_value);

	printk(KERN_NOTICE "lcdc0 bus clock disabled.\n");
	return 0;
}


// LCDC1 clock control function/tasks
int ns115_lcdc1_clk_enable(struct clk *clk)
{
	unsigned int reg_value;
	void __iomem *dis1_clk_ctrl = __io_address(PRCM_DIS1_CLK_CTRL);

	reg_value = PRCM_DIS1_CLK_CTRL_LCDC1_CLKEN  ;
	reg_set_bits(dis1_clk_ctrl, reg_value);

	printk(KERN_NOTICE "lcdc1 clock enabled.\n");
	return 0;
}

int ns115_lcdc1_clk_disable(struct clk *clk)
{
	unsigned int reg_value;
	void __iomem *dis1_clk_ctrl = __io_address(PRCM_DIS1_CLK_CTRL);

	reg_value = PRCM_DIS1_CLK_CTRL_LCDC1_CLKEN  ;
	reg_clr_bits(dis1_clk_ctrl, reg_value);

	printk(KERN_NOTICE "lcdc1 clock disabled.\n");
	return 0;
}

int ns115_lcdc1_bus_clk_enable(struct clk *clk)
{
	unsigned int reg_value;
	void __iomem *dis1_clk_ctrl = __io_address(PRCM_DIS1_CLK_CTRL);

	reg_value = PRCM_DIS1_CLK_CTRL_LCDC1_HCLKEN  |
		PRCM_DIS1_CLK_CTRL_LCDC1_ACLKEN;
	reg_set_bits(dis1_clk_ctrl, reg_value);

	printk(KERN_NOTICE "lcdc1 bus clock enabled.\n");
	return 0;
}

int ns115_lcdc1_bus_clk_disable(struct clk *clk)
{
	unsigned int reg_value;
	void __iomem *dis1_clk_ctrl = __io_address(PRCM_DIS1_CLK_CTRL);

	reg_value = PRCM_DIS1_CLK_CTRL_LCDC1_HCLKEN  |
		PRCM_DIS1_CLK_CTRL_LCDC1_ACLKEN;
	reg_clr_bits(dis1_clk_ctrl, reg_value);

	printk(KERN_NOTICE "lcdc1 bus clock disabled.\n");
	return 0;
}

int ns115_pxl1_clk_set_rate(struct clk *c, unsigned long rate, unsigned int nouse)
{
	void __iomem *dis1_clk_ctrl = __io_address(PRCM_DIS1_CLK_CTRL);
	unsigned long pll4_clk_rate;
	unsigned int pxl1_div =2;

	if (rate == 0) 
		return -EINVAL;

	if (pxl1_div > 0x0F)
		return -EINVAL;

	// Configure PLL4 clock freq to rate
	if(ns115_pll4_clk_set_rate(&pll4_clk, rate*2, nouse) == -EINVAL) {
		printk(KERN_NOTICE "Failed to set pxl1 clock rate to %ld.\n", rate);
		return -EINVAL;
	}

	// Set pxl1_div to 2
	pxl1_div = 2;
	clk_set_rate_bits(dis1_clk_ctrl, PRCM_DIS1_CLK_CTRL_PXL1_DIV_EN,
			PRCM_DIS1_CLK_CTRL_PXL1_DIV_MASK, pxl1_div);

	pll4_clk_rate = clk_get_rate(&pll4_clk);
	rate = pll4_clk_rate/pxl1_div;
	c->rate = rate; 

	printk(KERN_NOTICE "pxl1 clock rate set to %ld.\n", rate);
	return 0;
}

// HDMI_PXL clock control function/tasks
int ns115_hdmi_pxl_clk_enable(struct clk *clk)
{
	unsigned int reg_value;
	void __iomem *dis1_clk_ctrl = __io_address(PRCM_DIS1_CLK_CTRL);

	reg_value = PRCM_DIS1_CLK_CTRL_HDMI_PXL_CLKEN ;
	// PRCM_DIS1_CLK_CTRL_HDMI_CEC_CLKEN |
	// PRCM_DIS1_CLK_CTRL_HDMI_BCLKEN;
	reg_set_bits(dis1_clk_ctrl, reg_value);

	printk(KERN_NOTICE "hdmi_pxl clock enabled.\n");
	return 0;
}

int ns115_hdmi_pxl_clk_disable(struct clk *clk)
{
	unsigned int reg_value;
	void __iomem *dis1_clk_ctrl = __io_address(PRCM_DIS1_CLK_CTRL);

	reg_value = PRCM_DIS1_CLK_CTRL_HDMI_PXL_CLKEN ;
	// PRCM_DIS1_CLK_CTRL_HDMI_CEC_CLKEN |
	// PRCM_DIS1_CLK_CTRL_HDMI_BCLKEN;
	reg_clr_bits(dis1_clk_ctrl, reg_value);

	printk(KERN_NOTICE "hdmi_pxl clock disabled.\n");
	return 0;
}

// I2S0 clock control function/tasks
int ns115_i2s0_clk_enable(struct clk *clk)
{
	unsigned int reg_value;
	void __iomem *i2s_clk_ctrl = __io_address(PRCM_I2S_CLK_CTRL);

	reg_value = PRCM_I2S_CLK_CTRL_I2S0_CLKEN |
		PRCM_I2S_CLK_CTRL_I2S0_PCLKEN;
	reg_set_bits(i2s_clk_ctrl, reg_value);

	printk(KERN_NOTICE "i2s0 clock enabled.\n");
	return 0;
}

int ns115_i2s0_clk_disable(struct clk *clk)
{
	unsigned int reg_value;
	void __iomem *i2s_clk_ctrl = __io_address(PRCM_I2S_CLK_CTRL);

	reg_value = PRCM_I2S_CLK_CTRL_I2S0_CLKEN |
		PRCM_I2S_CLK_CTRL_I2S0_PCLKEN;
	reg_clr_bits(i2s_clk_ctrl, reg_value);

	printk(KERN_NOTICE "i2s0 clock disabled.\n");
	return 0;
}

int ns115_i2s_clk_set_rate(struct clk *c, unsigned long rate, unsigned int nouse)
{
	void __iomem *i2s_clk_ctrl = __io_address(PRCM_I2S_CLK_CTRL);
	unsigned long pll5_clk_rate = clk_get_rate(&pll5_clk);
	unsigned int i2s_div = pll5_clk_rate/rate;

	if (rate == 0) 
		return -EINVAL;

	if (i2s_div > 0x0F)
		return -EINVAL;

	clk_set_rate_bits(i2s_clk_ctrl, PRCM_I2S_CLK_CTRL_I2S_DIV_EN,
			PRCM_I2S_CLK_CTRL_I2S_DIV_MASK, i2s_div);

	rate = pll5_clk_rate/i2s_div;
	c->rate = rate; 

	printk(KERN_NOTICE "i2s clock rate set to %ld.\n", rate);
	return 0;
}

// mclock control function/tasks
int ns115_mclk_clk_enable(struct clk *clk)
{
	unsigned int reg_value;
	void __iomem *i2s_clk_ctrl = __io_address(PRCM_I2S_CLK_CTRL);

	reg_value = PRCM_I2S_CLK_CTRL_MCLKEN;
	reg_set_bits(i2s_clk_ctrl, reg_value);

	printk(KERN_NOTICE "mclk clock enabled.\n");
	return 0;
}

int ns115_mclk_clk_disable(struct clk *clk)
{
	unsigned int reg_value;
	void __iomem *i2s_clk_ctrl = __io_address(PRCM_I2S_CLK_CTRL);

	reg_value = PRCM_I2S_CLK_CTRL_MCLKEN;
	reg_clr_bits(i2s_clk_ctrl, reg_value);

	printk(KERN_NOTICE "mclk clock disabled.\n");
	return 0;
}


// I2S1 clock control function/tasks
int ns115_i2s1_clk_enable(struct clk *clk)
{
	unsigned int reg_value;
	void __iomem *i2s_clk_ctrl = __io_address(PRCM_I2S_CLK_CTRL);

	reg_value = PRCM_I2S_CLK_CTRL_I2S1_CLKEN  |
		PRCM_I2S_CLK_CTRL_I2S1_PCLKEN ;
	reg_set_bits(i2s_clk_ctrl, reg_value);

	printk(KERN_NOTICE "i2s1 clock enabled.\n");
	return 0;
}

int ns115_i2s1_clk_disable(struct clk *clk)
{
	unsigned int reg_value;
	void __iomem *i2s_clk_ctrl = __io_address(PRCM_I2S_CLK_CTRL);

	reg_value = PRCM_I2S_CLK_CTRL_I2S1_CLKEN  |
		PRCM_I2S_CLK_CTRL_I2S1_PCLKEN ;
	reg_clr_bits(i2s_clk_ctrl, reg_value);

	printk(KERN_NOTICE "i2s1 clock disabled.\n");
	return 0;
}

// USB OTG clock control function/tasks
int ns115_usb_otg_clk_enable(struct clk *clk)
{
	unsigned int reg_value;
	void __iomem *usb_clk_ctrl = __io_address(PRCM_USB_CLK_CTRL);

	reg_value = PRCM_USB_CLK_CTRL_OTG_HCLKEN |
		PRCM_USB_CLK_CTRL_USB_REF_CLKEN;
	reg_set_bits(usb_clk_ctrl, reg_value);

	printk(KERN_NOTICE "usb otg clock enabled.\n");
	return 0;
}

int ns115_usb_otg_clk_disable(struct clk *clk)
{
	unsigned int reg_value;
	void __iomem *usb_clk_ctrl = __io_address(PRCM_USB_CLK_CTRL);

	reg_value = PRCM_USB_CLK_CTRL_OTG_HCLKEN |
		PRCM_USB_CLK_CTRL_USB_REF_CLKEN;
	reg_clr_bits(usb_clk_ctrl, reg_value);

	printk(KERN_NOTICE "usb otg clock disabled.\n");
	return 0;
}

// USB UTMI clock control function/tasks
int ns115_usb_utmi_clk_enable(struct clk *clk)
{
	unsigned int reg_value;
	void __iomem *usb_clk_ctrl = __io_address(PRCM_USB_CLK_CTRL);

	reg_value = PRCM_USB_CLK_CTRL_UTMI_PHY_CLKEN |
		PRCM_USB_CLK_CTRL_USB2_HCLKEN;
	reg_set_bits(usb_clk_ctrl, reg_value);

	printk(KERN_NOTICE "usb utmi clock enabled.\n");
	return 0;
}

int ns115_usb_utmi_clk_disable(struct clk *clk)
{
	unsigned int reg_value;
	void __iomem *usb_clk_ctrl = __io_address(PRCM_USB_CLK_CTRL);

	reg_value = PRCM_USB_CLK_CTRL_UTMI_PHY_CLKEN |
		PRCM_USB_CLK_CTRL_USB2_HCLKEN;
	reg_clr_bits(usb_clk_ctrl, reg_value);

	printk(KERN_NOTICE "usb utmi clock disabled.\n");
	return 0;
}

// USB OHCI clock control function/tasks
int ns115_usb_ohci_clk_enable(struct clk *clk)
{
	unsigned int reg_value;
	void __iomem *usb_clk_ctrl = __io_address(PRCM_USB_CLK_CTRL);

	reg_value = PRCM_USB_CLK_CTRL_OHCI_48M_CLKEN |
		PRCM_USB_CLK_CTRL_USB2_HCLKEN;
	reg_set_bits(usb_clk_ctrl, reg_value);

	printk(KERN_NOTICE "usb ohci clock enabled.\n");
	return 0;
}

int ns115_usb_ohci_clk_disable(struct clk *clk)
{
	unsigned int reg_value;
	void __iomem *usb_clk_ctrl = __io_address(PRCM_USB_CLK_CTRL);

	reg_value = PRCM_USB_CLK_CTRL_OHCI_48M_CLKEN |
		PRCM_USB_CLK_CTRL_USB2_HCLKEN;
	reg_clr_bits(usb_clk_ctrl, reg_value);

	printk(KERN_NOTICE "usb ohci clock disabled.\n");
	return 0;
}

// USB HSIC clock control function/tasks
int ns115_usb_hsic_clk_enable(struct clk *clk)
{
	unsigned int reg_value;
	void __iomem *usb_clk_ctrl = __io_address(PRCM_USB_CLK_CTRL);

	reg_value = PRCM_USB_CLK_CTRL_HSIC_480M_CLKEN |
		PRCM_USB_CLK_CTRL_USB2_HCLKEN;
	reg_set_bits(usb_clk_ctrl, reg_value);

	printk(KERN_NOTICE "usb hsic clock enabled.\n");
	return 0;
}

int ns115_usb_hsic_clk_disable(struct clk *clk)
{
	unsigned int reg_value;
	void __iomem *usb_clk_ctrl = __io_address(PRCM_USB_CLK_CTRL);

	reg_value = PRCM_USB_CLK_CTRL_HSIC_480M_CLKEN |
		PRCM_USB_CLK_CTRL_USB2_HCLKEN;
	reg_clr_bits(usb_clk_ctrl, reg_value);

	printk(KERN_NOTICE "usb hsic clock disabled.\n");
	return 0;
}

// CORE MEM clock control function/tasks
int ns115_core_mem_clk_enable(struct clk *clk)
{
	unsigned int reg_value;
	void __iomem *core_mem_clk_ctrl = __io_address(PRCM_CORE_MEM_CLK_CTRL);

	reg_value = PRCM_CORE_MEM_CLK_CTRL_ROM_HCLKEN;
	reg_set_bits(core_mem_clk_ctrl, reg_value);

	printk(KERN_NOTICE "core rom clock enabled.\n");
	return 0;
}

int ns115_core_mem_clk_disable(struct clk *clk)
{
	unsigned int reg_value;
	void __iomem *core_mem_clk_ctrl = __io_address(PRCM_CORE_MEM_CLK_CTRL);

	reg_value =PRCM_CORE_MEM_CLK_CTRL_ROM_HCLKEN;
	reg_clr_bits(core_mem_clk_ctrl, reg_value);

	printk(KERN_NOTICE "core rom clock disabled.\n");
	return 0;
}

// GPIO CORE clock control function/tasks
int ns115_gpio_core_clk_enable(struct clk *clk)
{
	unsigned int reg_value;
	void __iomem *gpio_core_clk_ctrl = __io_address(PRCM_GPIO_CORE_CLK_CTRL);

	reg_value = PRCM_GPIO_CORE_CLK_CTRL_GPIO_CORE_PCLKEN;
	reg_set_bits(gpio_core_clk_ctrl, reg_value);

	printk(KERN_NOTICE "gpio core clock enabled.\n");
	return 0;
}

int ns115_gpio_core_clk_disable(struct clk *clk)
{
	unsigned int reg_value;
	void __iomem *gpio_core_clk_ctrl = __io_address(PRCM_GPIO_CORE_CLK_CTRL);

	reg_value =PRCM_GPIO_CORE_CLK_CTRL_GPIO_CORE_PCLKEN;
	reg_clr_bits(gpio_core_clk_ctrl, reg_value);

	printk(KERN_NOTICE "gpio core clock disabled.\n");
	return 0;
}

// TIMER0 CORE clock control function/tasks
int ns115_timer0_core_clk_enable(struct clk *clk)
{
	unsigned int reg_value;
	void __iomem *timer_core_clk_ctrl = __io_address(PRCM_TIMER_CORE_CLK_CTRL);

	reg_value = PRCM_TIMER_CORE_CLK_CTRL_TIMER0_CORE_CLKEN |
		PRCM_TIMER_CORE_CLK_CTRL_TIMER_CORE_PCLKEN;
	reg_set_bits(timer_core_clk_ctrl, reg_value);

	printk(KERN_NOTICE "timer0 core clock enabled.\n");
	return 0;
}

int ns115_timer0_core_clk_disable(struct clk *clk)
{
	unsigned int reg_value;
	void __iomem *timer_core_clk_ctrl = __io_address(PRCM_TIMER_CORE_CLK_CTRL);

	reg_value = PRCM_TIMER_CORE_CLK_CTRL_TIMER0_CORE_CLKEN |
		PRCM_TIMER_CORE_CLK_CTRL_TIMER_CORE_PCLKEN;
	reg_clr_bits(timer_core_clk_ctrl, reg_value);

	printk(KERN_NOTICE "timer0 core clock disabled.\n");
	return 0;
}

int ns115_timer0_core_clk_set_rate(struct clk *c, unsigned long rate, unsigned int nouse)
{
	void __iomem *timer_core_clk_ctrl = __io_address(PRCM_TIMER_CORE_CLK_CTRL);
	unsigned long i2s0_clk_rate = clk_get_rate(&i2s0_clk);
	unsigned int timer0_core_clk_sel;

	if (rate == i2s0_clk_rate/8) {
		timer0_core_clk_sel = 0x1;
	}
	else if (rate == 12000000) {
		timer0_core_clk_sel = 0x0;
	}
	else {
		printk(KERN_NOTICE "timer0 clock rate cannot set to %ld.\n", rate);
		return -EINVAL;
	}

	clk_set_rate_bits(timer_core_clk_ctrl, PRCM_TIMER_CORE_CLK_CTRL_TIMER0_CORE_CLKEN,
			PRCM_TIMER_CORE_CLK_CTRL_TIMER0_CORE_CLK_SEL, timer0_core_clk_sel);
	c->rate = rate;

	printk(KERN_NOTICE "timer0 clock rate set to %ld.\n", rate);
	return 0;
}

// TIMER1 CORE clock control function/tasks
int ns115_timer1_core_clk_enable(struct clk *clk)
{
	unsigned int reg_value;
	void __iomem *timer_core_clk_ctrl = __io_address(PRCM_TIMER_CORE_CLK_CTRL);

	reg_value = PRCM_TIMER_CORE_CLK_CTRL_TIMER1_CORE_CLKEN |
		PRCM_TIMER_CORE_CLK_CTRL_TIMER_CORE_PCLKEN;
	reg_set_bits(timer_core_clk_ctrl, reg_value);

	printk(KERN_NOTICE "timer1 core clock enabled.\n");
	return 0;
}

int ns115_timer1_core_clk_disable(struct clk *clk)
{
	unsigned int reg_value;
	void __iomem *timer_core_clk_ctrl = __io_address(PRCM_TIMER_CORE_CLK_CTRL);

	reg_value = PRCM_TIMER_CORE_CLK_CTRL_TIMER1_CORE_CLKEN |
		PRCM_TIMER_CORE_CLK_CTRL_TIMER_CORE_PCLKEN;
	reg_clr_bits(timer_core_clk_ctrl, reg_value);

	printk(KERN_NOTICE "timer1 core clock disabled.\n");
	return 0;
}

int ns115_timer1_core_clk_set_rate(struct clk *c, unsigned long rate, unsigned int nouse)
{
	void __iomem *timer_core_clk_ctrl = __io_address(PRCM_TIMER_CORE_CLK_CTRL);
	unsigned long i2s0_clk_rate = clk_get_rate(&i2s0_clk);
	unsigned int timer1_core_clk_sel;

	if (rate == i2s0_clk_rate/8) {
		timer1_core_clk_sel = 0x1;
	}
	else if (rate == 12000000) {
		timer1_core_clk_sel = 0x0;
	}
	else {
		printk(KERN_NOTICE "timer1 clock rate cannot set to %ld.\n", rate);
		return -EINVAL;
	}

	clk_set_rate_bits(timer_core_clk_ctrl, PRCM_TIMER_CORE_CLK_CTRL_TIMER1_CORE_CLKEN,
			PRCM_TIMER_CORE_CLK_CTRL_TIMER1_CORE_CLK_SEL, timer1_core_clk_sel);
	c->rate = rate;

	printk(KERN_NOTICE "timer1 clock rate set to %ld.\n", rate);
	return 0;
}

// TIMER2 CORE clock control function/tasks
int ns115_timer2_core_clk_enable(struct clk *clk)
{
	unsigned int reg_value;
	void __iomem *timer_core_clk_ctrl = __io_address(PRCM_TIMER_CORE_CLK_CTRL);

	reg_value = PRCM_TIMER_CORE_CLK_CTRL_TIMER2_CORE_CLKEN |
		PRCM_TIMER_CORE_CLK_CTRL_TIMER_CORE_PCLKEN;
	reg_set_bits(timer_core_clk_ctrl, reg_value);

	printk(KERN_NOTICE "timer2 core clock enabled.\n");
	return 0;
}

int ns115_timer2_core_clk_disable(struct clk *clk)
{
	unsigned int reg_value;
	void __iomem *timer_core_clk_ctrl = __io_address(PRCM_TIMER_CORE_CLK_CTRL);

	reg_value = PRCM_TIMER_CORE_CLK_CTRL_TIMER2_CORE_CLKEN |
		PRCM_TIMER_CORE_CLK_CTRL_TIMER_CORE_PCLKEN;
	reg_clr_bits(timer_core_clk_ctrl, reg_value);

	printk(KERN_NOTICE "timer2 core clock disabled.\n");
	return 0;
}

int ns115_timer2_core_clk_set_rate(struct clk *c, unsigned long rate, unsigned int nouse)
{
	void __iomem *timer_core_clk_ctrl = __io_address(PRCM_TIMER_CORE_CLK_CTRL);
	unsigned long i2s0_clk_rate = clk_get_rate(&i2s0_clk);
	unsigned int timer2_core_clk_sel;

	if (rate == i2s0_clk_rate/8) {
		timer2_core_clk_sel = 0x1;
	}
	else if (rate == 12000000) {
		timer2_core_clk_sel = 0x0;
	}
	else {
		printk(KERN_NOTICE "timer2 clock rate cannot set to %ld.\n", rate);
		return -EINVAL;
	}

	clk_set_rate_bits(timer_core_clk_ctrl, PRCM_TIMER_CORE_CLK_CTRL_TIMER2_CORE_CLKEN,
			PRCM_TIMER_CORE_CLK_CTRL_TIMER2_CORE_CLK_SEL, timer2_core_clk_sel);
	c->rate = rate;

	printk(KERN_NOTICE "timer2 clock rate set to %ld.\n", rate);
	return 0;
}

// TIMER3 CORE clock control function/tasks
int ns115_timer3_core_clk_enable(struct clk *clk)
{
	unsigned int reg_value;
	void __iomem *timer_core_clk_ctrl = __io_address(PRCM_TIMER_CORE_CLK_CTRL);

	reg_value = PRCM_TIMER_CORE_CLK_CTRL_TIMER3_CORE_CLKEN |
		PRCM_TIMER_CORE_CLK_CTRL_TIMER_CORE_PCLKEN;
	reg_set_bits(timer_core_clk_ctrl, reg_value);

	printk(KERN_NOTICE "timer3 core clock enabled.\n");
	return 0;
}

int ns115_timer3_core_clk_disable(struct clk *clk)
{
	unsigned int reg_value;
	void __iomem *timer_core_clk_ctrl = __io_address(PRCM_TIMER_CORE_CLK_CTRL);

	reg_value = PRCM_TIMER_CORE_CLK_CTRL_TIMER3_CORE_CLKEN |
		PRCM_TIMER_CORE_CLK_CTRL_TIMER_CORE_PCLKEN;
	reg_clr_bits(timer_core_clk_ctrl, reg_value);

	printk(KERN_NOTICE "timer3 core clock disabled.\n");
	return 0;
}

int ns115_timer3_core_clk_set_rate(struct clk *c, unsigned long rate, unsigned int nouse)
{
	void __iomem *timer_core_clk_ctrl = __io_address(PRCM_TIMER_CORE_CLK_CTRL);
	unsigned long i2s0_clk_rate = clk_get_rate(&i2s0_clk);
	unsigned int timer3_core_clk_sel;

	if (rate == i2s0_clk_rate/8) {
		timer3_core_clk_sel = 0x1;
	}
	else if (rate == 12000000) {
		timer3_core_clk_sel = 0x0;
	}
	else {
		printk(KERN_NOTICE "timer3 clock rate cannot set to %ld.\n", rate);
		return -EINVAL;
	}

	clk_set_rate_bits(timer_core_clk_ctrl, PRCM_TIMER_CORE_CLK_CTRL_TIMER3_CORE_CLKEN,
			PRCM_TIMER_CORE_CLK_CTRL_TIMER3_CORE_CLK_SEL, timer3_core_clk_sel);
	c->rate = rate;

	printk(KERN_NOTICE "timer3 clock rate set to %ld.\n", rate);
	return 0;
}

// TIMER4 CORE clock control function/tasks
int ns115_timer4_core_clk_enable(struct clk *clk)
{
	unsigned int reg_value;
	void __iomem *timer_core_clk_ctrl = __io_address(PRCM_TIMER_CORE_CLK_CTRL);

	reg_value = PRCM_TIMER_CORE_CLK_CTRL_TIMER4_CORE_CLKEN |
		PRCM_TIMER_CORE_CLK_CTRL_TIMER_CORE_PCLKEN;
	reg_set_bits(timer_core_clk_ctrl, reg_value);

	printk(KERN_NOTICE "timer4 core clock enabled.\n");
	return 0;
}

int ns115_timer4_core_clk_disable(struct clk *clk)
{
	unsigned int reg_value;
	void __iomem *timer_core_clk_ctrl = __io_address(PRCM_TIMER_CORE_CLK_CTRL);

	reg_value = PRCM_TIMER_CORE_CLK_CTRL_TIMER4_CORE_CLKEN |
		PRCM_TIMER_CORE_CLK_CTRL_TIMER_CORE_PCLKEN;
	reg_clr_bits(timer_core_clk_ctrl, reg_value);

	printk(KERN_NOTICE "timer4 core clock disabled.\n");
	return 0;
}

int ns115_timer4_core_clk_set_rate(struct clk *c, unsigned long rate, unsigned int nouse)
{
	void __iomem *timer_core_clk_ctrl = __io_address(PRCM_TIMER_CORE_CLK_CTRL);
	unsigned long i2s0_clk_rate = clk_get_rate(&i2s0_clk);
	unsigned int timer4_core_clk_sel;

	if (rate == i2s0_clk_rate/8) {
		timer4_core_clk_sel = 0x1;
	}
	else if (rate == 12000000) {
		timer4_core_clk_sel = 0x0;
	}
	else {
		printk(KERN_NOTICE "timer4 clock rate cannot set to %ld.\n", rate);
		return -EINVAL;
	}

	clk_set_rate_bits(timer_core_clk_ctrl, PRCM_TIMER_CORE_CLK_CTRL_TIMER4_CORE_CLKEN,
			PRCM_TIMER_CORE_CLK_CTRL_TIMER4_CORE_CLK_SEL, timer4_core_clk_sel);
	c->rate = rate;

	printk(KERN_NOTICE "timer4 clock rate set to %ld.\n", rate);
	return 0;
}

// TIMER5 CORE clock control function/tasks
int ns115_timer5_core_clk_enable(struct clk *clk)
{
	unsigned int reg_value;
	void __iomem *timer_core_clk_ctrl = __io_address(PRCM_TIMER_CORE_CLK_CTRL);

	reg_value = PRCM_TIMER_CORE_CLK_CTRL_TIMER5_CORE_CLKEN |
		PRCM_TIMER_CORE_CLK_CTRL_TIMER_CORE_PCLKEN;
	reg_set_bits(timer_core_clk_ctrl, reg_value);

	printk(KERN_NOTICE "timer5 core clock enabled.\n");
	return 0;
}

int ns115_timer5_core_clk_disable(struct clk *clk)
{
	unsigned int reg_value;
	void __iomem *timer_core_clk_ctrl = __io_address(PRCM_TIMER_CORE_CLK_CTRL);

	reg_value = PRCM_TIMER_CORE_CLK_CTRL_TIMER5_CORE_CLKEN |
		PRCM_TIMER_CORE_CLK_CTRL_TIMER_CORE_PCLKEN;
	reg_clr_bits(timer_core_clk_ctrl, reg_value);

	printk(KERN_NOTICE "timer5 core clock disabled.\n");
	return 0;
}

int ns115_timer5_core_clk_set_rate(struct clk *c, unsigned long rate, unsigned int nouse)
{
	void __iomem *timer_core_clk_ctrl = __io_address(PRCM_TIMER_CORE_CLK_CTRL);
	unsigned long i2s0_clk_rate = clk_get_rate(&i2s0_clk);
	unsigned int timer5_core_clk_sel;

	if (rate == i2s0_clk_rate/8) {
		timer5_core_clk_sel = 0x1;
	}
	else if (rate == 12000000) {
		timer5_core_clk_sel = 0x0;
	}
	else {
		printk(KERN_NOTICE "timer5 clock rate cannot set to %ld.\n", rate);
		return -EINVAL;
	}

	clk_set_rate_bits(timer_core_clk_ctrl, PRCM_TIMER_CORE_CLK_CTRL_TIMER5_CORE_CLKEN,
			PRCM_TIMER_CORE_CLK_CTRL_TIMER5_CORE_CLK_SEL, timer5_core_clk_sel);
	c->rate = rate;

	printk(KERN_NOTICE "timer5 clock rate set to %ld.\n", rate);
	return 0;
}

// TIMER6 CORE clock control function/tasks
int ns115_timer6_core_clk_enable(struct clk *clk)
{
	unsigned int reg_value;
	void __iomem *timer_core_clk_ctrl = __io_address(PRCM_TIMER_CORE_CLK_CTRL);

	reg_value = PRCM_TIMER_CORE_CLK_CTRL_TIMER6_CORE_CLKEN |
		PRCM_TIMER_CORE_CLK_CTRL_TIMER_CORE_PCLKEN;
	reg_set_bits(timer_core_clk_ctrl, reg_value);

	printk(KERN_NOTICE "timer6 core clock enabled.\n");
	return 0;
}

int ns115_timer6_core_clk_disable(struct clk *clk)
{
	unsigned int reg_value;
	void __iomem *timer_core_clk_ctrl = __io_address(PRCM_TIMER_CORE_CLK_CTRL);

	reg_value = PRCM_TIMER_CORE_CLK_CTRL_TIMER6_CORE_CLKEN |
		PRCM_TIMER_CORE_CLK_CTRL_TIMER_CORE_PCLKEN;
	reg_clr_bits(timer_core_clk_ctrl, reg_value);

	printk(KERN_NOTICE "timer6 core clock disabled.\n");
	return 0;
}

int ns115_timer6_core_clk_set_rate(struct clk *c, unsigned long rate, unsigned int nouse)
{
	void __iomem *timer_core_clk_ctrl = __io_address(PRCM_TIMER_CORE_CLK_CTRL);
	unsigned long i2s0_clk_rate = clk_get_rate(&i2s0_clk);
	unsigned int timer6_core_clk_sel;

	if (rate == i2s0_clk_rate/8) {
		timer6_core_clk_sel = 0x1;
	}
	else if (rate == 12000000) {
		timer6_core_clk_sel = 0x0;
	}
	else {
		printk(KERN_NOTICE "timer6 clock rate cannot set to %ld.\n", rate);
		return -EINVAL;
	}

	clk_set_rate_bits(timer_core_clk_ctrl, PRCM_TIMER_CORE_CLK_CTRL_TIMER6_CORE_CLKEN,
			PRCM_TIMER_CORE_CLK_CTRL_TIMER6_CORE_CLK_SEL, timer6_core_clk_sel);
	c->rate = rate;

	printk(KERN_NOTICE "timer6 clock rate set to %ld.\n", rate);
	return 0;
}

// TIMER7 CORE clock control function/tasks
int ns115_timer7_core_clk_enable(struct clk *clk)
{
	unsigned int reg_value;
	void __iomem *timer_core_clk_ctrl = __io_address(PRCM_TIMER_CORE_CLK_CTRL);

	reg_value = PRCM_TIMER_CORE_CLK_CTRL_TIMER7_CORE_CLKEN |
		PRCM_TIMER_CORE_CLK_CTRL_TIMER_CORE_PCLKEN;
	reg_set_bits(timer_core_clk_ctrl, reg_value);

	printk(KERN_NOTICE "timer7 core clock enabled.\n");
	return 0;
}

int ns115_timer7_core_clk_disable(struct clk *clk)
{
	unsigned int reg_value;
	void __iomem *timer_core_clk_ctrl = __io_address(PRCM_TIMER_CORE_CLK_CTRL);

	reg_value = PRCM_TIMER_CORE_CLK_CTRL_TIMER7_CORE_CLKEN |
		PRCM_TIMER_CORE_CLK_CTRL_TIMER_CORE_PCLKEN;
	reg_clr_bits(timer_core_clk_ctrl, reg_value);

	printk(KERN_NOTICE "timer7 core clock disabled.\n");
	return 0;
}

int ns115_timer7_core_clk_set_rate(struct clk *c, unsigned long rate, unsigned int nouse)
{
	void __iomem *timer_core_clk_ctrl = __io_address(PRCM_TIMER_CORE_CLK_CTRL);
	unsigned long i2s0_clk_rate = clk_get_rate(&i2s0_clk);
	unsigned int timer7_core_clk_sel;

	if (rate == i2s0_clk_rate/8) {
		timer7_core_clk_sel = 0x1;
	}
	else if (rate == 12000000) {
		timer7_core_clk_sel = 0x0;
	}
	else {
		printk(KERN_NOTICE "timer7 clock rate cannot set to %ld.\n", rate);
		return -EINVAL;
	}


	clk_set_rate_bits(timer_core_clk_ctrl, PRCM_TIMER_CORE_CLK_CTRL_TIMER7_CORE_CLKEN,
			PRCM_TIMER_CORE_CLK_CTRL_TIMER7_CORE_CLK_SEL, timer7_core_clk_sel);
	c->rate = rate;

	printk(KERN_NOTICE "timer7 clock rate set to %ld.\n", rate);
	return 0;
}

// WKUP MEM clock control function/tasks
int ns115_wkup_mem_clk_enable(struct clk *clk)
{
	unsigned int reg_value;
	void __iomem *wkup_mem_clk_ctrl = __io_address(PRCM_WKUP_MEM_CLK_CTRL);

	reg_value = PRCM_WKUP_MEM_CLK_CTRL_RAM_WKUP_HCLKEN;
	reg_set_bits(wkup_mem_clk_ctrl, reg_value);

	printk(KERN_NOTICE "wkup_mem clock enabled.\n");
	return 0;
}

int ns115_wkup_mem_clk_disable(struct clk *clk)
{
	unsigned int reg_value;
	void __iomem *wkup_mem_clk_ctrl = __io_address(PRCM_WKUP_MEM_CLK_CTRL);

	reg_value =PRCM_WKUP_MEM_CLK_CTRL_RAM_WKUP_HCLKEN;
	reg_clr_bits(wkup_mem_clk_ctrl, reg_value);

	printk(KERN_NOTICE "wkup_mem clock disabled.\n");
	return 0;
}

// GPIO WKUP clock control function/tasks
int ns115_gpio_wkup_clk_enable(struct clk *clk)
{
	unsigned int reg_value;
	void __iomem *gpio_wkup_clk_ctrl = __io_address(PRCM_GPIO_WKUP_CLK_CTRL);

	reg_value = PRCM_GPIO_WKUP_CLK_CTRL_GPIO_DBCLKEN |
		PRCM_GPIO_WKUP_CLK_CTRL_GPIO_WKUP_PCLKEN;
	reg_set_bits(gpio_wkup_clk_ctrl, reg_value);

	printk(KERN_NOTICE "gpio_wkup clock enabled.\n");
	return 0;
}

int ns115_gpio_wkup_clk_disable(struct clk *clk)
{
	unsigned int reg_value;
	void __iomem *gpio_wkup_clk_ctrl = __io_address(PRCM_GPIO_WKUP_CLK_CTRL);

	reg_value = PRCM_GPIO_WKUP_CLK_CTRL_GPIO_DBCLKEN |
		PRCM_GPIO_WKUP_CLK_CTRL_GPIO_WKUP_PCLKEN;
	reg_clr_bits(gpio_wkup_clk_ctrl, reg_value);

	printk(KERN_NOTICE "gpio_wkup clock disabled.\n");
	return 0;
}

int ns115_gpio_wkup_clk_set_rate(struct clk *c, unsigned long rate, unsigned int nouse)
{
	void __iomem *gpio_wkup_clk_ctrl = __io_address(PRCM_GPIO_WKUP_CLK_CTRL);
	unsigned int gpio_wkup_clk_div = 32000/rate;

	if (rate == 0) 
		return -EINVAL;

	if (gpio_wkup_clk_div >0x0FF)
		return -EINVAL;

	clk_set_rate_bits(gpio_wkup_clk_ctrl, PRCM_GPIO_WKUP_CLK_CTRL_GPIO_DBCLK_32K_DIV_EN,
			PRCM_GPIO_WKUP_CLK_CTRL_GPIO_DBCLK_32K_DIV_MASK, gpio_wkup_clk_div);

	rate = 32000/gpio_wkup_clk_div;
	c->rate = rate; 

	printk(KERN_NOTICE "gpio_wkup clock rate set to %ld.\n", rate);
	return 0;
}

// TIMER0 WKUP clock control function/tasks
int ns115_timer0_wkup_clk_enable(struct clk *clk)
{
	unsigned int reg_value;
	void __iomem *timer_wkup_clk_ctrl = __io_address(PRCM_TIMER_WKUP_CLK_CTRL);

	reg_value = PRCM_TIMER_WKUP_CLK_CTRL_TIMER0_WKUP_CLKEN |
		PRCM_TIMER_WKUP_CLK_CTRL_TIMER_WKUP_PCLKEN;
	reg_set_bits(timer_wkup_clk_ctrl, reg_value);

	printk(KERN_NOTICE "timer0_wkup clock enabled.\n");
	return 0;
}

int ns115_timer0_wkup_clk_disable(struct clk *clk)
{
	unsigned int reg_value;
	void __iomem *timer_wkup_clk_ctrl = __io_address(PRCM_TIMER_WKUP_CLK_CTRL);

	reg_value = PRCM_TIMER_WKUP_CLK_CTRL_TIMER0_WKUP_CLKEN |
		PRCM_TIMER_WKUP_CLK_CTRL_TIMER_WKUP_PCLKEN;
	reg_clr_bits(timer_wkup_clk_ctrl, reg_value);

	printk(KERN_NOTICE "timer0_wkup clock disabled.\n");
	return 0;
}

int ns115_timer0_wkup_clk_set_rate(struct clk *c, unsigned long rate, unsigned int nouse)
{
	void __iomem *timer_wkup_clk_ctrl = __io_address(PRCM_TIMER_WKUP_CLK_CTRL);
	unsigned int timer0_wkup_clk_sel;

	if (rate == 32000) {
		timer0_wkup_clk_sel = 0x1;
	}
	else if (rate == 12000000) {
		timer0_wkup_clk_sel = 0x0;
	}
	else {
		printk(KERN_NOTICE "timer0_wkup clock rate cannot set to %ld.\n", rate);
		return -EINVAL;
	}

	clk_set_rate_bits(timer_wkup_clk_ctrl, PRCM_TIMER_WKUP_CLK_CTRL_TIMER0_WKUP_CLKEN,
			PRCM_TIMER_WKUP_CLK_CTRL_TIMER0_WKUP_CLK_SEL, timer0_wkup_clk_sel);
	c->rate = rate;

	printk(KERN_NOTICE "timer0 clock rate set to %ld.\n", rate);
	return 0;
}

// TIMER1 WKUP clock control function/tasks
int ns115_timer1_wkup_clk_enable(struct clk *clk)
{
	unsigned int reg_value;
	void __iomem *timer_wkup_clk_ctrl = __io_address(PRCM_TIMER_WKUP_CLK_CTRL);

	reg_value = PRCM_TIMER_WKUP_CLK_CTRL_TIMER1_WKUP_CLKEN |
		PRCM_TIMER_WKUP_CLK_CTRL_TIMER_WKUP_PCLKEN;
	reg_set_bits(timer_wkup_clk_ctrl, reg_value);

	printk(KERN_NOTICE "timer1_wkup clock enabled.\n");
	return 0;
}

int ns115_timer1_wkup_clk_disable(struct clk *clk)
{
	unsigned int reg_value;
	void __iomem *timer_wkup_clk_ctrl = __io_address(PRCM_TIMER_WKUP_CLK_CTRL);

	reg_value = PRCM_TIMER_WKUP_CLK_CTRL_TIMER1_WKUP_CLKEN |
		PRCM_TIMER_WKUP_CLK_CTRL_TIMER_WKUP_PCLKEN;
	reg_clr_bits(timer_wkup_clk_ctrl, reg_value);

	printk(KERN_NOTICE "timer1_wkup clock disabled.\n");
	return 0;
}

int ns115_timer1_wkup_clk_set_rate(struct clk *c, unsigned long rate, unsigned int nouse)
{
	void __iomem *timer_wkup_clk_ctrl = __io_address(PRCM_TIMER_WKUP_CLK_CTRL);
	unsigned int timer1_wkup_clk_sel;

	if (rate == 32000) {
		timer1_wkup_clk_sel = 0x1;
	}
	else if (rate == 12000000) {
		timer1_wkup_clk_sel = 0x0;
	}
	else {
		printk(KERN_NOTICE "timer1_wkup clock rate cannot set to %ld.\n", rate);
		return -EINVAL;
	}

	clk_set_rate_bits(timer_wkup_clk_ctrl, PRCM_TIMER_WKUP_CLK_CTRL_TIMER1_WKUP_CLKEN,
			PRCM_TIMER_WKUP_CLK_CTRL_TIMER1_WKUP_CLK_SEL, timer1_wkup_clk_sel);
	c->rate = rate;

	printk(KERN_NOTICE "timer1 clock rate set to %ld.\n", rate);
	return 0;
}

// ALT0 clock control function/tasks
int ns115_alt0_clk_enable(struct clk *clk)
{
	unsigned int reg_value;
	void __iomem *alt_clk_ctrl = __io_address(PRCM_ALT_CLK_CTRL);

	reg_value = PRCM_ALT_CLK_CTRL_ALT0_CLKEN;
	reg_set_bits(alt_clk_ctrl, reg_value);

	printk(KERN_NOTICE "alt0 clock enabled.\n");
	return 0;
}

int ns115_alt0_clk_disable(struct clk *clk)
{
	unsigned int reg_value;
	void __iomem *alt_clk_ctrl = __io_address(PRCM_ALT_CLK_CTRL);

	reg_value = PRCM_ALT_CLK_CTRL_ALT0_CLKEN;
	reg_clr_bits(alt_clk_ctrl, reg_value);

	printk(KERN_NOTICE "alt0 clock disabled.\n");
	return 0;
}

int ns115_alt0_clk_set_rate(struct clk *c, unsigned long rate, unsigned int nouse)
{
	void __iomem *alt_clk_ctrl = __io_address(PRCM_ALT_CLK_CTRL);
	unsigned long pll6_clk_rate = clk_get_rate(&pll6_clk);
	unsigned int alt0_div = pll6_clk_rate/rate;

	if (rate == 0) 
		return -EINVAL;

	if (alt0_div >0x7FF)
		return -EINVAL;

	clk_set_rate_bits(alt_clk_ctrl, PRCM_ALT_CLK_CTRL_ALT0_DIV_EN,
			PRCM_ALT_CLK_CTRL_ALT0_DIV_MASK, alt0_div);

	rate = pll6_clk_rate/alt0_div;
	c->rate = rate;

	printk(KERN_NOTICE "alt clock rate set to %ld.\n", rate);
	return 0;
}

// ALT1 clock control function/tasks
int ns115_alt1_clk_enable(struct clk *clk)
{
	unsigned int reg_value;
	void __iomem *alt_clk_ctrl = __io_address(PRCM_ALT_CLK_CTRL);

	reg_value = PRCM_ALT_CLK_CTRL_ALT1_CLKEN;
	reg_set_bits(alt_clk_ctrl, reg_value);

	printk(KERN_NOTICE "alt1 clock enabled.\n");
	return 0;
}

int ns115_alt1_clk_disable(struct clk *clk)
{
	unsigned int reg_value;
	void __iomem *alt_clk_ctrl = __io_address(PRCM_ALT_CLK_CTRL);

	reg_value = PRCM_ALT_CLK_CTRL_ALT1_CLKEN;
	reg_clr_bits(alt_clk_ctrl, reg_value);

	printk(KERN_NOTICE "alt1 clock disabled.\n");
	return 0;
}

int ns115_alt1_clk_set_rate(struct clk *c, unsigned long rate, unsigned int nouse)
{
	void __iomem *alt_clk_ctrl = __io_address(PRCM_ALT_CLK_CTRL);
	unsigned long pll6_clk_rate = clk_get_rate(&pll6_clk);
	unsigned int alt1_clk_sel, alt1_clk_sel_cur;
	unsigned int alt1_div = 0;

	if (rate == 0) 
		return -EINVAL;

	if (rate == 32000) {
		alt1_clk_sel = 0x1;
	}
	else {
		alt1_clk_sel = 0x0;
		alt1_div = pll6_clk_rate/rate;
		rate = pll6_clk_rate/alt1_div;
	}

	if (alt1_div >0x7FF)
		return -EINVAL;

	if (alt1_clk_sel == 0x0) {
		clk_set_rate_bits(alt_clk_ctrl, PRCM_ALT_CLK_CTRL_ALT1_DIV_EN,
				PRCM_ALT_CLK_CTRL_ALT1_DIV_MASK, alt1_div);
	}

	alt1_clk_sel_cur = __raw_readl(alt_clk_ctrl) & PRCM_ALT_CLK_CTRL_ALT1_32K_CLK_SEL;

	if ((alt1_clk_sel << get_offset_pos(PRCM_ALT_CLK_CTRL_ALT1_32K_CLK_SEL)) != alt1_clk_sel_cur) {
		clk_set_rate_bits(alt_clk_ctrl, PRCM_ALT_CLK_CTRL_ALT1_CLKEN, 
				PRCM_ALT_CLK_CTRL_ALT1_32K_CLK_SEL, alt1_clk_sel);
	}

	c->rate = rate;

	printk(KERN_NOTICE "alt1 clock rate set to %ld.\n", rate);
	return 0;
}

// AUX clock control function/tasks
int ns115_aux_clk_enable(struct clk *clk)
{
	unsigned int reg_value;
	void __iomem *aux_clk_ctrl = __io_address(PRCM_AUX_CLK_CTRL);

	reg_value = PRCM_AUX_CLK_CTRL_AUX_PCLKEN;
	reg_set_bits(aux_clk_ctrl, reg_value);

	printk(KERN_NOTICE "aux clock enabled.\n");
	return 0;
}

int ns115_aux_clk_disable(struct clk *clk)
{
	unsigned int reg_value;
	void __iomem *aux_clk_ctrl = __io_address(PRCM_AUX_CLK_CTRL);

	reg_value = PRCM_AUX_CLK_CTRL_AUX_PCLKEN;
	reg_clr_bits(aux_clk_ctrl, reg_value);

	printk(KERN_NOTICE "aux clock disabled.\n");
	return 0;
}

// EFUSE clock control function/tasks
int ns115_efuse_clk_enable(struct clk *clk)
{
	unsigned int reg_value;
	void __iomem *aux_clk_ctrl = __io_address(PRCM_AUX_CLK_CTRL);

	reg_value = PRCM_AUX_CLK_CTRL_EFUSE_CLKEN;
	reg_set_bits(aux_clk_ctrl, reg_value);

	printk(KERN_NOTICE "efuse clock enabled.\n");
	return 0;
}

int ns115_efuse_clk_disable(struct clk *clk)
{
	unsigned int reg_value;
	void __iomem *aux_clk_ctrl = __io_address(PRCM_AUX_CLK_CTRL);

	reg_value = PRCM_AUX_CLK_CTRL_EFUSE_CLKEN;
	reg_clr_bits(aux_clk_ctrl, reg_value);

	printk(KERN_NOTICE "efuse clock disabled.\n");
	return 0;
}

// PWM clock control function/tasks
int ns115_pwm_clk_enable(struct clk *clk)
{
	unsigned int reg_value;
	void __iomem *aux_clk_ctrl = __io_address(PRCM_AUX_CLK_CTRL);

	reg_value = PRCM_AUX_CLK_CTRL_PWM_CLKEN;
	reg_set_bits(aux_clk_ctrl, reg_value);

	printk(KERN_NOTICE "pwm clock enabled.\n");
	return 0;
}

int ns115_pwm_clk_disable(struct clk *clk)
{
	unsigned int reg_value;
	void __iomem *aux_clk_ctrl = __io_address(PRCM_AUX_CLK_CTRL);

	reg_value = PRCM_AUX_CLK_CTRL_PWM_CLKEN;
	reg_clr_bits(aux_clk_ctrl, reg_value);

	printk(KERN_NOTICE "pwm clock disabled.\n");
	return 0;
}

// CPU Clock Rate Setting 
int ns115_cpu_clk_set_rate(struct clk *c, unsigned long rate, unsigned int nouse)
{
	void __iomem *cpu_clk_switch = __io_address(PRCM_CPU_CLK_SWITCH);
	void __iomem *cpu_clk_div    = __io_address(PRCM_CPU_CLK_DIV);

	unsigned int cpu_clk_switch_reg;

	//printk(KERN_NOTICE "current frequence is %ld Hz.\n", get_current_freq()); 

	// set to manual mode 
	//reg_clr_bits(cpu_clk_switch, PRCM_CPU_CLK_SWITCH_CPU_MU0_SEL_EN|PRCM_CPU_CLK_SWITCH_CPU_MU1_SEL_EN); 
	//reg_clr_bits(cpu_clk_switch, PRCM_CPU_CLK_SWITCH_CPU_AUTO_SEL_MODE);

	// set to auto mode
	reg_clr_bits(cpu_clk_switch, PRCM_CPU_CLK_SWITCH_CPU_AUTO_SEL_EN);
	reg_set_bits(cpu_clk_switch, PRCM_CPU_CLK_SWITCH_CPU_AUTO_SEL_MODE);

	cpu_clk_switch_reg = *((unsigned int*) cpu_clk_switch);

	// Manual mode
	if (!(cpu_clk_switch_reg & PRCM_CPU_CLK_SWITCH_CPU_AUTO_SEL_MODE)) {
		printk(KERN_NOTICE "Change CPU clock at Manual mode\n");
		reg_clr_bits(cpu_clk_switch, PRCM_CPU_CLK_SWITCH_CPU_MU0_SEL_EN|PRCM_CPU_CLK_SWITCH_CPU_MU1_SEL_EN);
		if (!(cpu_clk_switch_reg & PRCM_CPU_CLK_SWITCH_CPU_MUX_SWITCH)) { // current SWITCH0
			// switch to PLL1_CLKOUT, use SWITCH1
			reg_clr_bits(cpu_clk_switch, PRCM_CPU_CLK_SWITCH_CPU_MUX1_SEL_MASK);
			reg_set_bits(cpu_clk_switch, PRCM_CPU_CLK_SWITCH_CPU_MUX1_SEL_3); // Use PLL1_CLKOUT
			reg_set_bits(cpu_clk_switch, PRCM_CPU_CLK_SWITCH_CPU_MU1_SEL_EN);
			reg_set_bits(cpu_clk_switch, PRCM_CPU_CLK_SWITCH_CPU_MUX_SWITCH);// switch to SWITCH1

			//printk(KERN_NOTICE "current frequence is %ld Hz.\n", get_current_freq()); 

			// set clock rate for PLL0
			ns115_pll0_clk_set_rate(&pll1_clk,rate, nouse);

			// Switch to PLL1_CLKOUT, use SWITCH0
			reg_clr_bits(cpu_clk_switch, PRCM_CPU_CLK_SWITCH_CPU_MUX0_SEL_MASK);
			reg_set_bits(cpu_clk_switch, PRCM_CPU_CLK_SWITCH_CPU_MUX0_SEL_1); // Use PLL0_CLKOUT
			reg_set_bits(cpu_clk_switch, PRCM_CPU_CLK_SWITCH_CPU_MU0_SEL_EN);
			reg_clr_bits(cpu_clk_switch, PRCM_CPU_CLK_SWITCH_CPU_MUX_SWITCH);// switch back to SWITCH0
		}
		else {	// current SWITCH1
			// Set divider1 clock to PLL1_CLKOUT/3
			reg_clr_bits(cpu_clk_div, PRCM_CPU_CLK_DIV_CPU_DIV1_MASK);
			reg_set_bits(cpu_clk_div, PRCM_CPU_CLK_DIV_CPU_DIV1_3);
			reg_set_bits(cpu_clk_div, PRCM_CPU_CLK_DIV_CPU_DIV1_EN);

			// switch to PLL1_CLKOUT, use SWITCH0
			reg_clr_bits(cpu_clk_switch, PRCM_CPU_CLK_SWITCH_CPU_MUX0_SEL_MASK);
			reg_set_bits(cpu_clk_switch, PRCM_CPU_CLK_SWITCH_CPU_MUX0_SEL_4); // Use DIVIDER1_CLKOUT
			reg_set_bits(cpu_clk_switch, PRCM_CPU_CLK_SWITCH_CPU_MU0_SEL_EN);
			reg_clr_bits(cpu_clk_switch, PRCM_CPU_CLK_SWITCH_CPU_MUX_SWITCH);// switch to SWITCH0

			//printk(KERN_NOTICE "current frequence is %ld Hz.\n", get_current_freq()); 

			// set clock rate for PLL0
			ns115_pll0_clk_set_rate(&pll1_clk, rate, nouse);

			// Switch to PLL1_CLKOUT, use SWITCH1
			reg_clr_bits(cpu_clk_switch, PRCM_CPU_CLK_SWITCH_CPU_MUX1_SEL_MASK);
			reg_set_bits(cpu_clk_switch, PRCM_CPU_CLK_SWITCH_CPU_MUX1_SEL_1); // Use PLL0_CLKOUT
			reg_set_bits(cpu_clk_switch, PRCM_CPU_CLK_SWITCH_CPU_MU1_SEL_EN);
			reg_set_bits(cpu_clk_switch, PRCM_CPU_CLK_SWITCH_CPU_MUX_SWITCH);// switch back to SWITCH1
		}

	}
	else { // Auto mode
		printk(KERN_NOTICE"Change CPU clock at Auto mode\n");

		// Switch to PLL1
		reg_clr_bits(cpu_clk_switch, PRCM_CPU_CLK_SWITCH_CPU_AUTO_SEL_EN);
		reg_clr_bits(cpu_clk_switch, PRCM_CPU_CLK_SWITCH_CPU_AUTO_SEL_MASK);
		reg_set_bits(cpu_clk_switch, PRCM_CPU_CLK_SWITCH_CPU_AUTO_SEL_3);
		reg_set_bits(cpu_clk_switch, PRCM_CPU_CLK_SWITCH_CPU_AUTO_SEL_EN);

		//printk(KERN_NOTICE "current frequence is %ld Hz.\n", get_current_freq()); 

		// set clock rate for PLL0
		ns115_pll0_clk_set_rate(&pll1_clk, rate, nouse);

		// set autosel target to PLL0
		reg_clr_bits(cpu_clk_switch, PRCM_CPU_CLK_SWITCH_CPU_AUTO_SEL_EN);
		reg_clr_bits(cpu_clk_switch, PRCM_CPU_CLK_SWITCH_CPU_AUTO_SEL_MASK);
		reg_set_bits(cpu_clk_switch, PRCM_CPU_CLK_SWITCH_CPU_AUTO_SEL_1);
		reg_set_bits(cpu_clk_switch, PRCM_CPU_CLK_SWITCH_CPU_AUTO_SEL_EN);
	}

	//printk(KERN_NOTICE "cpu clock rate should set to %ld Hz, and current frequence is %ld Hz.\n", rate, get_current_freq()); 

	c->rate = rate;

	return 0;
}



/* SECTION 4: Clock Lookup Table Definition */
static struct clk_lookup lookups[] = {
	{   /* CORESIGHT */
		.clk		= &coresight_clk,
		.con_id	    = "ns115_coresight"
	}, {	/* PLL5 */
		.clk        	= &pll5_clk,
		.con_id		= "ns115_pll5",
	}, {    /* DDR */
		.clk        	= &ddr_clk,
		.con_id		= "ns115_ddr",
	}, {    /* PL330_DMA */
		.dev_id	 	= "dma-pl330",
		.clk  	        = &pl330_dma_clk,
		.con_id		= "ns115_pl330_dma",
	}, {    /* MALI */
		.clk        	= &mali_clk,
		.con_id	 	= "ns115_mali",
	}, {    /* GC300 */
		.clk        	= &gc300_clk,
		.con_id	 	= "ns115_gc300",
	}, {    /* VPU_G1 */
		.clk        	= &vpu_g1_clk,
		.con_id	 	= "ns115_vpu_g1",
	}, {    /* VPU_H1 */
		.clk        	= &vpu_h1_clk,
			.con_id	 	= "ns115_vpu_h1",
	}, {    /* ISP_JPEG */
		.clk        	= &isp_jpeg_clk,
			.con_id	 	= "ns115_isp_jpeg_clk",
	}, {    /* ISP */
		.clk        	= &isp_clk,
			.con_id	 	= "ns115_isp_clk",
	}, {    /* SD */
		.clk        	= &sd_clk,
			.con_id	 	= "ns115_sd",
	}, {    /* ZSP */
		.clk        	= &zsp_clk,
			.con_id	 	= "ns115_zsp_clk",
	}, {    /* SPI0_CLK */
		.clk        	= &spi0_clk,
			.con_id	 	= "ns115_spi0_clk",
	}, {    /* SPI1_CLK */
		.clk        	= &spi1_clk,
			.con_id	 	= "ns115_spi1_clk",
	}, {    /* SPI2_CLK */
		.clk        	= &spi2_clk,
			.con_id	 	= "ns115_spi2_clk",
	}, {    /* SPI3_CLK */
		.clk        	= &spi3_clk,
			.con_id	 	= "ns115_spi3_clk",
	}, {    /* I2C0 */
		.clk        	= &i2c0_clk,
			.con_id	 	= "ns115_i2c0",
	}, {    /* I2C1 */
		.clk        	= &i2c1_clk,
			.con_id	 	= "ns115_i2c1",
	}, {    /* I2C2 */
		.clk        	= &i2c2_clk,
			.con_id	 	= "ns115_i2c2",
	}, {    /* I2C3 */
		.clk        	= &i2c3_clk,
			.con_id	 	= "ns115_i2c3",
	}, {    /* UART0 */
		.clk        	= &uart0_clk,
			.con_id	 	= "ns115_uart0",
	}, {    /* UART1 */
		.clk        	= &uart1_clk,
			.con_id	 	= "ns115_uart1",
	}, {    /* UART2 */
		.clk        	= &uart2_clk,
			.con_id	 	= "ns115_uart2",
	}, {    /* UART3 */
		.clk        	= &uart3_clk,
			.con_id	 	= "ns115_uart3",
	}, {    /* RGBO */
		.clk        	= &rgbo_clk,
			.con_id	 	= "ns115_rgbo",
	}, {    /* LCDC0 */
		.clk        	= &lcdc0_clk,
			.con_id	 	= "ns115_lcdc0",
	}, {    /* LCDC0_BUS */
		.clk        	= &lcdc0_bus_clk,
			.con_id	 	= "ns115_lcdc0_bus",
	}, {    /* LCDC1 */
		.clk        	= &lcdc1_clk,
			.con_id	 	= "ns115_lcdc1",
	}, {    /* LCDC1_BUS */
		.clk        	= &lcdc1_bus_clk,
			.con_id	 	= "ns115_lcdc1_bus",
	}, {    /* HDMA_PXL */
		.clk        	= &hdmi_pxl_clk,
			.con_id	 	= "ns115_hdmi_pxl",
	}, {    /* I2S0 */
		.clk        	= &i2s0_clk,
			.con_id	 	= "ns115_i2s0",
	}, {    /* I2S1 */
		.clk        	= &i2s1_clk,
			.con_id	 	= "ns115_i2s1",
	}, {    /* MCLK */
		.clk        	= &mclk_clk,
			.con_id	 	= "ns115_mclk",
	}, {    /* USB_OTG */
		.clk        	= &usb_otg_clk,
			.con_id	 	= "ns115_usb_otg",
	}, {    /* USB_UTMI */
		.clk        	= &usb_utmi_clk,
			.con_id	 	= "ns115_usb_utmi",
	}, {    /* USB_OHCI */
		.clk        	= &usb_ohci_clk,
			.con_id	 	= "ns115_ohci",
	}, {    /* USB_HSIC */
		.clk        	= &usb_hsic_clk,
			.con_id	 	= "ns115_usb_hsic",
	}, {    /* CORE_MEM */
		.clk        	= &core_mem_clk,
			.con_id	 	= "ns115_core_mem",
	}, {    /* GPIO_CORE */
		.clk        	= &gpio_core_clk,
			.con_id	 	= "ns115_gpio_core",
	}, {    /* TIMER0_CORE */
		.clk        	= &timer0_core_clk,
			.con_id	 	= "ns115_timer0_core",
	}, {    /* TIMER1_CORE */
		.clk        	= &timer1_core_clk,
			.con_id	 	= "ns115_timer1_core",
	}, {    /* TIMER2_CORE */
		.clk        	= &timer2_core_clk,
			.con_id	 	= "ns115_timer2_core",
	}, {    /* TIMER3_CORE */
		.clk        	= &timer3_core_clk,
			.con_id	 	= "ns115_timer3_core",
	}, {    /* TIMER4_CORE */
		.clk        	= &timer4_core_clk,
			.con_id	 	= "ns115_timer4_core",
	}, {    /* TIMER5_CORE */
		.clk        	= &timer5_core_clk,
			.con_id	 	= "ns115_timer5_core",
	}, {    /* TIMER6_CORE */
		.clk        	= &timer6_core_clk,
			.con_id	 	= "ns115_timer6_core",
	}, {    /* TIMER7_CORE */
		.clk        	= &timer7_core_clk,
			.con_id	 	= "ns115_timer7_core",
	}, {    /* WKUP_MEM */
		.clk        	= &wkup_mem_clk,
			.con_id	 	= "ns115_wkup_mem",
	}, {    /* GPIO_WKUP */
		.clk        	= &gpio_wkup_clk,
			.con_id	 	= "ns115_gpio_wkup",
	}, {    /* TIMER0_WKUP */
		.clk        	= &timer0_wkup_clk,
			.con_id	 	= "ns115_timer0_wkup",
	}, {    /* TIMER1_WKUP */
		.clk        	= &timer1_wkup_clk,
			.con_id	 	= "ns115_timer1_wkup",
	}, {    /* ALT0 */
		.clk        	= &alt0_clk,
			.con_id	 	= "ns115_alt0",
	}, {    /* ALT1 */
		.clk        	= &alt1_clk,
			.con_id	 	= "ns115_alt1",
	}, {    /* AUX */
		.clk        	= &aux_clk,
			.con_id	 	= "ns115_aux",
	}, {    /* EFUSE */
		.clk        	= &efuse_clk,
			.con_id	 	= "ns115_efuse",
	}, {    /* PWM */
		.clk        	= &pwm_clk,
			.con_id	 	= "ns115_pwm",
	}, {    /* CPU */
		.clk        	= &cpu_clk,
			.con_id	 	= "ns115_cpu",
	}
};




/* SECTION 5: Other Function/Tasks Definition */

static int __init clk_init(void)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(lookups); i++)
		clkdev_add(&lookups[i]);
	return 0;
}

arch_initcall(clk_init);

/*
 * Global software reset control
 * posedge trigger a global software reset
 * NOTICE: global reset can not reset prcm registers
 */
void prcm_glb_soft_reset(void)
{
	void __iomem *sys_rst_ctrl = __io_address(PRCM_SYS_RST_CTRL);
	void __iomem *sys_pwr_ctrl = __io_address(PRCM_SYS_PWR_CTRL);
	void __iomem *zsp_clk_ctrl = __io_address(PRCM_ZSP_CLK_CTRL);
	void __iomem *misc_reg0 = __io_address(PRCM_MISC_REG0);
	void __iomem *misc_reg1 = __io_address(PRCM_MISC_REG1);
	void __iomem *misc_reg2 = __io_address(PRCM_MISC_REG2);
	void __iomem *misc_reg3 = __io_address(PRCM_MISC_REG3);

	reg_clr_bits(misc_reg0, 0xFFFFFFFF); //0
	reg_clr_bits(misc_reg1, 0xFFFFFFFF); //0
	reg_clr_bits(misc_reg2, 0xFFFFFFFF); //0
	reg_clr_bits(misc_reg3, 0xFFFFFFFF); //0

	reg_set_bits(zsp_clk_ctrl, PRCM_ZSP_CLK_CTRL_ZSP_ACLKEN | PRCM_ZSP_CLK_CTRL_ZSP_CLKEN);//1
	reg_set_bits(sys_pwr_ctrl, PRCM_SYS_PWR_CTRL_ZSP_PWR_STATE);//1

	reg_clr_bits(sys_rst_ctrl, PRCM_SYS_RST_CTRL_GLB_SW_RST); //0
	reg_set_bits(sys_rst_ctrl, PRCM_SYS_RST_CTRL_GLB_SW_RST);//1
	//	printk(KERN_EMERG "globle soft reset over!\n");
}

/*
 * Global software power off control
 */
void prcm_glb_soft_power_off(void)
{
	void __iomem *sys_pwr_ctrl = __io_address(PRCM_SYS_PWR_CTRL);
	void __iomem *sys_pll_ctrl = __io_address(PRCM_SYS_PLL_CTRL);

	reg_clr_bits(sys_pwr_ctrl, 0x7FFF); //0
	reg_set_bits(sys_pll_ctrl, 0x7F); //1
	asm("wfi");
}


/*
 * CPU system reset control: Reset all CPU related units
 */
void cpu_reset_all(void)
{
	unsigned int reg_data;
	void __iomem *sys_rst_ctrl = __io_address(PRCM_SYS_RST_CTRL);

	/* cpu1 soft reset */
	reg_data = (PRCM_SYS_RST_CTRL_NUSTAR_RST_DEBUG1_RST     | PRCM_SYS_RST_CTRL_NUSTAR_RST_DEBUG0_RST |
			PRCM_SYS_RST_CTRL_NUSTAR_RST_WATCHDOG1_RST  | PRCM_SYS_RST_CTRL_NUSTAR_RST_WATCHDOG0_RST |
			PRCM_SYS_RST_CTRL_NUSTAR_RST_NEON1_RST      | PRCM_SYS_RST_CTRL_NUSTAR_RST_NEON0_RST |
			PRCM_SYS_RST_CTRL_NUSTAR_RST_CPU1_RST); 

	reg_clr_bits(sys_rst_ctrl, reg_data);
	reg_set_bits(sys_rst_ctrl, reg_data);

	/* cpu0 soft reset */
	reg_data = (PRCM_SYS_RST_CTRL_NUSTAR_RST_DEBUG1_RST     | PRCM_SYS_RST_CTRL_NUSTAR_RST_DEBUG0_RST |
			PRCM_SYS_RST_CTRL_NUSTAR_RST_WATCHDOG1_RST  | PRCM_SYS_RST_CTRL_NUSTAR_RST_WATCHDOG0_RST |
			PRCM_SYS_RST_CTRL_NUSTAR_RST_NEON1_RST      | PRCM_SYS_RST_CTRL_NUSTAR_RST_NEON0_RST |
			PRCM_SYS_RST_CTRL_NUSTAR_RST_CPU0_RST); 

	reg_clr_bits(sys_rst_ctrl, reg_data);
	reg_set_bits(sys_rst_ctrl, reg_data);

	printk(KERN_NOTICE "cpu reset over!\n");
}


/*
 *reset and enable watchdog
 */
void watchdog_reset_enable(void)
{
	void __iomem *sys_rst_ctrl = __io_address(PRCM_SYS_RST_CTRL);
	void __iomem *private_watchdog = __io_address(NS115_GTIMER_BASE);
	unsigned int value = 0;

	//enable watchdog reset
	reg_set_bits(sys_rst_ctrl, PRCM_SYS_RST_CTRL_WATCHDOG_RST_EN);

	//watchdog control register
	__raw_writel(0, private_watchdog + NS115_SYS_WATCHDOG_CONTROL);
	//watchdog load register
	__raw_writel(0x00200, private_watchdog + NS115_SYS_WATCHDOG_LOAD);

	value = (NS115_SYS_WATCHDOG_CONTROL_ENABLE | NS115_SYS_WATCHDOG_CONTROL_MODE);
	__raw_writel(value, private_watchdog + NS115_SYS_WATCHDOG_CONTROL);
}

//#define PLL_DEBUG
#define nfmax	(1<<12)
#define nrmax	(1<<6)
#define odmax	(1<<4)
#define IS_VALID(nf, nr)	((12*nf/nr >= 300) && (12*nf/nr <= 1500))
#ifdef PLL_DEBUG
#define pll_dbg(args...) pr_emerg(args)
#else
#define pll_dbg(...)
#endif

int pll_calc_fout(int fout, int *pnf, int *pnr, int *pod, int *pnb)
{
	int factor = (fout)/120;
	int nf,nr,od;
	int fdst = 0, bias = 4000, tbias;
	int temp;
	int count = 0;
	pll_dbg("fout %d factor = %d\n nfmax = %d;nrmax = %d odmax = %d\n",\
		fout, factor, nfmax, nrmax, odmax);
	for(od=odmax; od>=1; od=(od==2)?1:(od-2)) {
		for(nr=1; nr<=nrmax; nr++) {
			temp = (factor * od * nr);

			nf = (temp / 50000);
			nf = (nf >> 1) + (nf & 0x1);
			if(nf > nfmax)
				break;

			count++;
			fdst = (nf*100000)/nr/od;
			tbias = fdst - factor;
			tbias = tbias > 0?tbias:(-tbias);
			if(tbias < bias) {
				if(IS_VALID(nf, nr)) {
					bias = tbias;
					*pnf = nf, *pnr = nr, *pod = od, *pnb = (nf>>1);
					pll_dbg("(nf:%d,nr:%d,od:%d)new bias = %d, fdst = %d, target = %d\n",\
							nf, nr, od, bias, fdst, factor);
					if(bias == 0) {
						pll_dbg("total %d\n", count);
						return bias == 4000;
					}
				}
			}
		}
	}
	pll_dbg("total %d\n", count);
	return bias == 4000;
}

