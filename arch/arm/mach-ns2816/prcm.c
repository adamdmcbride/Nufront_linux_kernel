/*
 *  linux/arch/arm/mach-ns2816/prcm.c
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
#include <mach/board-ns2816.h>
#include <mach/platform.h>

#include "clock.h"
#include "prcm.h"


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

int ns2816_i2sclk_enable(struct clk *clk)
{
	unsigned int reg_value;
	void __iomem *clk1_ctrl = __io_address(PRCM_CLK1_CTRL);

	reg_value = __raw_readl(clk1_ctrl);
	reg_value |= PRCM_CLK1_EN_I2S_SCLKEN;
	__raw_writel(reg_value, clk1_ctrl);

	printk(KERN_NOTICE "I2S clock enabled.\n");	
	//return clk_enable(clk);
	return 0;
}

int ns2816_dmac0_clk_enable(struct clk *clk)
{
        unsigned int reg_value;
        void __iomem *clk1_ctrl = __io_address(PRCM_CLK1_CTRL);

        reg_value = __raw_readl(clk1_ctrl);
        reg_value |= PRCM_CLK1_EN_I2S_SCLKEN;
        __raw_writel(reg_value, clk1_ctrl);

        printk(KERN_NOTICE "I2S clock enabled.\n");
        //return clk_enable(clk);
        return 0;
}

int ns2816_i2sclk_disable(struct clk *clk)
{
	unsigned int reg_value;
	void __iomem *clk1_ctrl = __io_address(PRCM_CLK1_CTRL);
	
	reg_value = __raw_readl(clk1_ctrl);
	reg_value &= ~PRCM_CLK1_EN_I2S_SCLKEN;
	__raw_writel(reg_value, clk1_ctrl);
	
	printk(KERN_NOTICE "I2S clock disabled.\n");	

	//clk_disable(clk);
	return 0;
}

void ns2816_i2sclk_set_rate(unsigned int rate)
{
	void __iomem *sel_ctrl = __io_address(PRCM_CLK_SEL_CTRL);

	reg_clr_bits(sel_ctrl, PRCM_CLK_SEL_I2S_CLK_MASK);
	reg_set_bits(sel_ctrl, rate);

	printk(KERN_NOTICE "Already set I2S clock to %d .\n", rate);
}

unsigned long ns2816_i2s_get_rate(struct clk *c)
{

	void __iomem *sel_ctrl = __io_address(PRCM_CLK_SEL_CTRL);

	return (unsigned long)(1806720000/((unsigned int)(__raw_readl(sel_ctrl)) >> 16));

	 

}

int ns2816_i2s_set_rate(struct clk *c, unsigned long rate, unsigned int nouse)
{
	unsigned int i2s_clk_sel = rate << 16;
/*
	switch(rate) {
	case 512000:
		i2s_clk_sel = PRCM_CLK_SEL_I2S_512K;
		break;	
	case 768000:
		i2s_clk_sel = PRCM_CLK_SEL_I2S_768K;
		break;
	case 1024000:
		i2s_clk_sel = PRCM_CLK_SEL_I2S_1024K;
		break;
	case 1536000:
		i2s_clk_sel = PRCM_CLK_SEL_I2S_1536K;
		break;
	case 2048000:
		i2s_clk_sel = PRCM_CLK_SEL_I2S_2048K;
		break;
	case 3072000:
		i2s_clk_sel = PRCM_CLK_SEL_I2S_3072K;
		break;
	case 12288000:
		i2s_clk_sel = PRCM_CLK_SEL_I2S_12288K;
		break;
	case 6144000:
		i2s_clk_sel = PRCM_CLK_SEL_I2S_6144K;
		break;
	case 2822000:
		i2s_clk_sel = PRCM_CLK_SEL_I2S_2822K;
		break;
	default:
		return -EINVAL;
		break;
	}
*/
	printk(KERN_NOTICE "ns2816_i2s_set_rate begin set i2s clock to %lu .\n", rate);
	ns2816_i2sclk_set_rate(i2s_clk_sel);

	return 0;	
}

struct lcd_pll_set {
	unsigned int nr;
	unsigned int nf;
	unsigned int no;
	unsigned int rate;
};

static struct lcd_pll_set lcd_pll_sets[] = {

	{ 24, 1546,1, 193250000 }, /*1920*1200@60*/
	{ 24, 1546,2, 96625000 }, /*1920*1200@30*/
	{ 6,  314, 1, 157000000 },	
	{ 24, 1094,1, 136750000 },
	{ 4,  142, 1, 106500000 },/*1440x900@60*/	 
	{ 12, 710, 2, 88750000  },/*1440x900@60red.blanking*/
             
	{ 12, 718, 1, 179500000 },
	{ 1,  52,  1, 156000000 },	
	{ 24, 974, 1, 121750000 },/*1400x1050@60*/
	{ 3,  202, 2, 101000000 },/*1400x1050@60red.blanking*/

	{ 2,  114, 2, 85500000  },/*1360x768@60vesa*/	
	{ 4,  210, 1, 157500000 },	
	{ 2,  90,  1, 135000000 },	
	{ 1,  36,  1, 108000000 },/*1280x960@60vesa*//*1280x1024@60vesa*/
             
	{ 4,  198, 2, 74250000 },/*1280x720@60*/
	{ 4,  198, 1, 148500000 },/*1920x1080@60*/
	{ 12, 470, 1, 117500000 },
	{ 12, 818, 2, 102250000 },
             
	{ 2,  106, 2, 79500000  },/*1280x768@60*/	
	{ 4,  182, 2, 68250000  },/*1280x768@60red.blanking*/
	{ 2,  126, 2, 94500000  },
             
	{ 4,  210, 2, 78750000  },
	{ 1,  50,  2, 75000000  },
	{ 3,  130, 2, 65000000  },/*1024x768@60*/
	{ 15, 898, 4, 44900000  },/*1024x768@43*/
             
	{ 15,  542,  2, 54000000  },/*1024x600@60 netboot*/
	{ 5,  84,  1, 50400000  },/*1024x600@60 10"1 50.4MHz*/
	{ 1,  30,  2, 45000000  },/*1024x600@60*/
	{ 3,  94,  4, 25175000  },/*640x480@60*/
	{ 3,  80,  2, 40000000  },/*800x600@60*/
};

int ns2816_lcdclk_enable(struct clk *clk)
{
	printk(KERN_NOTICE "LCD clock enabled.\n");	
	return 0;
}

int ns2816_lcdclk_disable(struct clk *clk)
{
/*
	void __iomem *ctrl_virt = __io_address(PRCM_LCD_CLKCTRL);
	reg_clr_bits(ctrl_virt, PRCM_LCD_CLKCTRL_CLK_EN);
*/
	printk(KERN_NOTICE "LCD clock disabled.\n");	
	//clk_disable(clk);
	return 0;
}

#define	lcd_pll_sets_len sizeof(lcd_pll_sets)/ sizeof(struct lcd_pll_set)

void ns2816_lcd_pixelclk_set(unsigned nr, unsigned int nf, unsigned int no)
{
	unsigned int clkctrl;
	void __iomem *ctrl_virt = __io_address(PRCM_LCD_CLKCTRL);

	clkctrl = __raw_readl(ctrl_virt);
	clkctrl &= ~(PRCM_LCD_CLKCTRL_FREQ_REFRESH);
	clkctrl |= PRCM_LCD_CLKCTRL_CLK_EN;

	__raw_writel(clkctrl, ctrl_virt);
	clkctrl = __raw_readl(ctrl_virt);

	clkctrl &= ~(PRCM_LCD_CLKCTRL_PLL_NR_MASK | 
		PRCM_LCD_CLKCTRL_PLL_NF_MASK | 
		PRCM_LCD_CLKCTRL_PLL_OD_MASK);

	clkctrl |= ((nr -1) << 19) | ((nf - 1) << 6) | ((no - 1) << 2) | 3;

	__raw_writel(clkctrl, ctrl_virt);
}


int ns2816_lcdc_set_rate(struct clk *c, unsigned long rate, unsigned int nouse)
{
	int i = 0;
	struct lcd_pll_set * pll_set = lcd_pll_sets;
	struct lcd_pll_set * pll_near = NULL;
	int nearmin = rate / 20;
	int near = 0;
  	unsigned int count = 0; 

	if(rate == 0)
		return -EINVAL;

	printk(KERN_NOTICE "begin set lcdc clock: rate = %lu.\n", rate);

	for( i = 0; i< lcd_pll_sets_len; i++) {
		if(rate == pll_set->rate)
			break;
		else {
			near = rate - pll_set->rate;
			near = near > 0?near:(-near);
			if(nearmin > near) {
				pll_near = pll_set;
				nearmin = near;
			}
		}
		pll_set ++;
	}
	if(i == lcd_pll_sets_len) {
		if(NULL == pll_near)
			return -EINVAL;
		else
			pll_set = pll_near;
	}

	printk(KERN_NOTICE "begin set lcdc clock: rate = %d.\n", pll_set->rate);

	ns2816_lcd_pixelclk_set(pll_set->nr, pll_set->nf, pll_set->no);	

	printk(KERN_NOTICE "alread set lcdc clock, nr = %d, nf = %d, no = %d .\n"
		, pll_set->nr, pll_set->nf, pll_set->no);

	//count = (pll_set->nr * 11 + 5) * 10 + 40;   //400 = 50us
	count = (pll_set->nr * 11 + 5);   //us

	c->delay = count;

	//return clk_enable(c);
	return 0;
}

/* usb ref clk switch
 * set switch clk
 * reset usb
 */
void prcm_usb_sel_refclk(unsigned int refclk_sel, unsigned int refclk_div)
{
	unsigned int reg_value = 0;
	void __iomem *usb_refclk = __io_address(PRCM_RSTCTRL_USB_REFCLK);

	reg_value = __raw_readl(usb_refclk);

	if((refclk_sel == (reg_value & PRCM_RST_CTRL_USB_REFCLK_SEL_MASK) ) 
	    && (refclk_div == (reg_value & PRCM_RST_CTRL_USB_REFCLK_DIV_MASK)))
	{
		printk(KERN_NOTICE "usb reference clock is already set to what your want! return.\n");
		return;
	}

	reg_clr_bits(usb_refclk, PRCM_RST_CTRL_USB_REFCLK_SEL_MASK);
	reg_set_bits(usb_refclk, refclk_sel);
 
	reg_clr_bits(usb_refclk, PRCM_RST_CTRL_USB_REFCLK_DIV_MASK);
	reg_set_bits(usb_refclk, refclk_div); 

	printk(KERN_NOTICE "alread set usb clock, refcle sel is %d, refclk divider is %d .\n"
		, refclk_sel, refclk_div);	
}

void prcm_usb_reset(void)
{
	unsigned int reg_value = 0;
	void __iomem *rst_usb = __io_address(PRCM_RSTCTRL_USB);

	reg_clr_bits(rst_usb, PRCM_RSTCTRL_USB_RST);	//0
	udelay(4);	
	reg_set_bits(rst_usb, PRCM_RSTCTRL_USB_RST);	//1
	udelay(800); //keep high to usb work normally

	reg_value = PRCM_RSTCTRL_USB_RST_PORT0 | PRCM_RSTCTRL_USB_RST_PORT1 |
			PRCM_RSTCTRL_USB_RST_PORT2 | PRCM_RSTCTRL_USB_RST_PORT3 ;

	//port reset is level trigger and high trigger reset.
	reg_set_bits(rst_usb, reg_value); //1
	udelay(100); 
	reg_clr_bits(rst_usb, reg_value); //0

	printk(KERN_NOTICE "usb reset...\n");
}

enum {
	USB_12M = 12000000,
	USB_24M = 24000000,
	USB_48M = 48000000,
};
 
int ns2816_usb_set_rate(struct clk *usb_clk, unsigned long rate, unsigned int refclksel)
{
	unsigned int refclk_div;

	switch(rate)
	{
		case USB_12M :
			refclk_div = (0 << 1);
			break;
		case USB_24M :
			refclk_div = (1 << 1);
			break;
		case USB_48M :
			refclk_div = (2 << 1);
			break;
		default:
			refclk_div = (0 << 1);
			break;
	}	

	printk(KERN_NOTICE "begine set usb clock to %lu .\n", rate);
	prcm_usb_sel_refclk((refclksel << 3), refclk_div);

	usb_clk->rate = rate;
	
	return 0;
} 

int ns2816_sdmmcclk_enable(struct clk *clk)
{
	unsigned int reg_value;
	void __iomem *clkena2 = __io_address(PRCM_CLK2_CTRL);

	reg_value = __raw_readl(clkena2);
	reg_value |= PRCM_CLK2_CTRL_MMC_INPUTCLK_ENA;
	__raw_writel(reg_value, clkena2);

	printk(KERN_NOTICE "SD/MMC clock enabled.\n");	
	//return clk_enable(clk);
	return 0;
}

int ns2816_sdmmcclk_disable(struct clk *clk)
{
	unsigned int reg_value;
	void __iomem *clkena2 = __io_address(PRCM_CLK2_CTRL);

	reg_value = __raw_readl(clkena2);
	reg_value &= ~PRCM_CLK2_CTRL_MMC_INPUTCLK_ENA;
	__raw_writel(reg_value, clkena2);

	printk(KERN_NOTICE "SD/MMC clock disabled.\n");	
	//clk_disable(clk);
	return 0;
}

int ns2816_pl330dma_clk_enable(struct clk *clk)
{
        unsigned int reg_value;
        void __iomem *clkena2 = __io_address(PRCM_CLK2_CTRL);

        reg_value = __raw_readl(clkena2);
        reg_value |= PRCM_CLK2_EN_DMA1_HCLKEN;
        __raw_writel(reg_value, clkena2);

        printk(KERN_NOTICE "pl330 dma clock enabled.\n");
        //return clk_enable(clk);
        return 0;
}

int ns2816_pl330dma_clk_disable(struct clk *clk)
{
        unsigned int reg_value;
        void __iomem *clkena2 = __io_address(PRCM_CLK2_CTRL);

        reg_value = __raw_readl(clkena2);
        reg_value &= ~PRCM_CLK2_EN_DMA1_HCLKEN;
        __raw_writel(reg_value, clkena2);

        printk(KERN_NOTICE "pl330 clock disabled.\n");
        //clk_disable(clk);
        return 0;
}

int ns2816_dwdma_clk_enable(struct clk *clk)
{
        unsigned int reg_value;
        void __iomem *clkena1 = __io_address(PRCM_CLK1_CTRL);

        reg_value = __raw_readl(clkena1);
        reg_value |= PRCM_CLK1_EN_DMAC0_ACLKEN;
        __raw_writel(reg_value, clkena1);

        printk(KERN_NOTICE "dw dma clock enabled.\n");
        //return clk_enable(clk);
        return 0;
}

int ns2816_dwdma_clk_disable(struct clk *clk)
{
        unsigned int reg_value;
        void __iomem *clkena1 = __io_address(PRCM_CLK1_CTRL);

        reg_value = __raw_readl(clkena1);
        reg_value &= ~PRCM_CLK1_EN_DMAC0_ACLKEN;
        __raw_writel(reg_value, clkena1);
        
        printk(KERN_NOTICE "dw clock disabled.\n");
        //clk_disable(clk);
        return 0;
}

/*
 * These are fixed clocks.
 */
static struct clk ref66_clk = {
	.rate	= 66666666,
};

static struct clk ref400_clk = {
	.rate	= 400000000,
};

static struct clk ref200_clk = {
	.rate	= 200000000,
};

static struct clk oscvco_clk = {
	.rate	= 200000000,
};

static struct clk pl330_dma_clk = {
	.enabled  = 0,
	.rate	  = 400000000,
	.enable   = ns2816_pl330dma_clk_enable,
	.disable  = ns2816_pl330dma_clk_disable,
	.set_rate = NULL,
	.get_rate = clk_get_rate, 
};

static struct clk dw_dma_clk = {
        .enabled  = 0,
        .rate     = 200000000,
        .enable   = ns2816_dwdma_clk_enable,
        .disable  = ns2816_dwdma_clk_disable,
        .set_rate = NULL,
        .get_rate = clk_get_rate,
};

static struct clk sdmmc_clk = {
	.enabled  = 0,
	.rate	  = 50000000,
	.enable   = ns2816_sdmmcclk_enable,
	.disable  = ns2816_sdmmcclk_disable,
	.set_rate = NULL,
	.get_rate = clk_get_rate, 
};

static struct clk i2s_clk = {
	.enabled  = 0,
	.rate	  = 0,
	.delay	  = 0,
	.enable   = ns2816_i2sclk_enable,
	.disable  = ns2816_i2sclk_disable,
	.set_rate = ns2816_i2s_set_rate,	
	.get_rate = ns2816_i2s_get_rate,
};

static struct clk lcdc_clk = {
	.enabled  = 0,
	.rate	  = 195000000,
	//.rate	  = 97500000,
	.delay	  = 0,
	/* lcd clock will be enabled when call set_rate */
	.enable   = ns2816_lcdclk_enable,
	.disable  = ns2816_lcdclk_disable,
	.set_rate = ns2816_lcdc_set_rate,	
	.get_rate = clk_get_rate,
};

static struct clk usb_clk = {
	.enabled  = 0,
	.rate	  = 48000000,
	.enable   = NULL,
	.disable  = NULL,
	.set_rate = ns2816_usb_set_rate,	
	.get_rate = clk_get_rate,
};

static struct clk_lookup lookups[] = {
	{	/* UART0 */
		.dev_id		= "ns2816-uart0",
		.clk		= &ref66_clk,
	}, {	/* UART1 */
		.dev_id		= "ns2816-uart1",
		.clk		= &ref66_clk,
	}, {	/* GP TIME */
		.dev_id		= "ns2816-gptimer",
		.clk		= &ref66_clk,
	}, {	/* i2c1 */
		.dev_id		= "ns2816-i2c0",
		.clk		= &ref66_clk,
	}, {	/* i2c2 */
		.dev_id		= "ns2816-i2c1",
		.clk		= &ref66_clk,
	}, {	/* dma330 */
		.dev_id		= "pl330_dma",  
		.clk		= &pl330_dma_clk,
		.con_id		= "pl330_dma_clk",  
	}, {	/* dw dma */
		.dev_id		= "ns2816_dw_dma",
		.clk		= &dw_dma_clk,
		.con_id		= "dw_dma_clk", 
	}, {	/* clcd */
		.dev_id		= "ns2816-clcd",
		.clk		= &oscvco_clk,
	}, {	/* i2s */
		.clk		= &i2s_clk,
		.con_id		= "soc-audio",
	}, {	/* on2 */
		.dev_id		= "ns2816-on2",
		.clk		= &ref200_clk,
	}, {	/* mali400 */
		.dev_id		= "ns2816-mali",
		.clk		= &ref400_clk,
	}, {	/* sd/mmc */
		.dev_id		= "ns2816-sdmmc.0",
		.clk		= &sdmmc_clk,
		.con_id		= "ns2816-sdmmc",
	}, {	/* usb */
		.dev_id		= "ns2816-usb",
		.clk		= &usb_clk,
		.con_id		= "ns2816-usb",
	}, {	/* lcd */
		.dev_id		= "ns2816fb.0",
		.clk		= &lcdc_clk,
		.con_id		= "ns2816fb",
	}, {	/* lcd */
		.dev_id		= "ns2816plfb.0",
		.clk		= &lcdc_clk,
		.con_id		= "ns2816plfb",
	}
};


static int __init clk_init(void)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(lookups); i++)
		clkdev_add(&lookups[i]);
	return 0;
}

arch_initcall(clk_init);

/*clk gating ctl.
 *scm_pclk cannot close, if close, all clock can not run.
 */
void clk_gating(void)
{
	unsigned int i;
	void __iomem *clk1_ctrl = __io_address(PRCM_CLK1_CTRL);
	void __iomem *clk2_ctrl = __io_address(PRCM_CLK2_CTRL);

	udelay(2000);

	for(i = 0; i < 7; i++)
 	{
		reg_clr_bits(clk1_ctrl, 1 << i);
		udelay(10);	
		reg_set_bits(clk1_ctrl, 1 << i);
		udelay(10);
	}

	for(i = 0; i < 30; i++)
	{
		if(i >= 18 && i<= 20)
			continue;
	
		reg_clr_bits(clk2_ctrl, 1 << i);	
		udelay(10);
		reg_set_bits(clk2_ctrl, 1 << i);
		udelay(10);
	}	
	printk(KERN_NOTICE "clock gating over!\n");
	udelay(80);
}

/*
 * Global software reset control
 * posedge trigger a global software reset
 * NOTICE: global reset can not reset prcm registers
 */
void prcm_glb_soft_reset(void)
{
	void __iomem *ctrl = __io_address(PRCM_RSTCTRL);
	reg_clr_bits(ctrl, PRCM_RSTCTRL_GLB_SW_RST); //0
	reg_set_bits(ctrl, PRCM_RSTCTRL_GLB_SW_RST);//1
//	printk(KERN_EMERG "globle soft reset over!\n");
}

/*
 * CPU power management and memory contorller power down
 */
void cpu_reset_all(void)
{
	unsigned int reg_data;
	void __iomem *pwctrl = __io_address(PRCM_PWCTRL);
/*
	for(i=2; i<8; i++)
	{
		reg_clr_bits(pwctrl, 1 << i);
		udelay(10);
		reg_set_bits(pwctrl, 1 << i);
		udelay(10);
	}
*/
	/* cpu1 soft reset */
	reg_data = (PRCM_PWCTRL_DBG1_SW_RST | PRCM_PWCTRL_DBG0_SW_RST |
		PRCM_PWCTRL_WD1_SW_RST | PRCM_PWCTRL_WD0_SW_RST |
		PRCM_PWCTRL_NEON1_SW_RST | PRCM_PWCTRL_NEON0_SW_RST |
		PRCM_PWCTRL_CPU1_SW_RST); 

	reg_clr_bits(pwctrl, reg_data);
	reg_set_bits(pwctrl, reg_data);

	/* cpu0 soft reset */
	reg_data = (PRCM_PWCTRL_DBG1_SW_RST | PRCM_PWCTRL_DBG0_SW_RST |
		PRCM_PWCTRL_WD1_SW_RST | PRCM_PWCTRL_WD0_SW_RST |
		PRCM_PWCTRL_NEON1_SW_RST | PRCM_PWCTRL_NEON0_SW_RST |
		PRCM_PWCTRL_CPU0_SW_RST); 

	reg_clr_bits(pwctrl, reg_data);
	reg_set_bits(pwctrl, reg_data);

	printk(KERN_NOTICE "cpu reset over!\n");
}

/*
 *reset and enable watchdog
 */
void watchdog_reset_enable(void)
{
	void __iomem *ctrl = __io_address(PRCM_RSTCTRL);
	void __iomem *private_watchdog = __io_address(NS2816_GTIMER_BASE);
	unsigned int value = 0;

	//enable watchdog reset
	reg_set_bits(ctrl, PRCM_RSTCTRL_WDRESET_EN);

	//watchdog control register
	__raw_writel(0, private_watchdog + NS2816_SYS_WATCHDOG_CONTROL);
	//watchdog load register
	__raw_writel(0x00200, private_watchdog + NS2816_SYS_WATCHDOG_LOAD);

	value = (NS2816_SYS_WATCHDOG_CONTROL_ENABLE | NS2816_SYS_WATCHDOG_CONTROL_MODE);
	__raw_writel(value, private_watchdog + NS2816_SYS_WATCHDOG_CONTROL);
}

void prcm_dll_soft_reset(unsigned int i)
{
	void __iomem *dll_rst = __io_address(PRCM_RSTCTRL_DLL);

	reg_set_bits(dll_rst, (0x1 << i));
}


/*switch cpu freq: 800MHz, 1066MHz, 1200MHz, 1333MHz, 1600MHz, 2000MHz
 *software dynamic switch cpu feq: 
 *	must set system freq mode update register [6] = 1
 *cpumode: cpu/axi/ahb ddr pll4
 */
void set_cpu_mode(unsigned int cpu_mode)
{
	void __iomem *mode_ctrl = __io_address(PRCM_SYSFREQ_MODE_CTRL);

	//clr CPU_FREQ_FRESH = 0
	reg_clr_bits(mode_ctrl, (1 << 5));
	reg_clr_bits(mode_ctrl, PRCM_SYSF_MODE_CPU_MASK);
	//set cpu freq
	reg_set_bits(mode_ctrl, PRCM_SYSF_MODE_CPU_FREQ_SW_EN | cpu_mode);

	//set CPU_FREQ_FRESH = 1
	reg_set_bits(mode_ctrl, (1 << 5));

	//wait for 100us to stable		
	udelay(100);
}

