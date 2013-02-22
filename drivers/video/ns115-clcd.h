/*
 * linux/include/amba/nusmart_clcd.h -- Integrator LCD High Resolution panel.
 *
 * Zeyuan Xu
 *
 * Copyright (C) 2010 NUSMART Limited
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file COPYING in the main directory of this archive
 * for more details.
 */

#ifndef __NS115_CLCD_H__
#define __NS115_CLCD_H__

#include <linux/fb.h>
#include <linux/clk.h>
#include <mach/hardware.h>
#include <mach/board-ns115.h>

int lcd_init_hx8369(void);
/*
 * CLCD Controller Internal Register addresses
 */
#define         NUSMART_LCDC_CTRL       0x00

#define         NUSMART_LCDC_FRAME_RESET	(1<<31)
#define         NUSMART_LCDC_UNDERFLOW	(1<<18)
#define         NUSMART_LCDC_OVERFLOW	(1<<15)
#define         NUSMART_LCDC_INTF_V     (1<<17)
#define         NUSMART_LCDC_INTF_H     (1<<16)
#define         NUSMART_LCDC_PTRF       (1<<13)	/*primary=0,secondary=1 */
#define         NUSMART_LCDC_ENABLE     (1<<11)
#define         NUSMART_LCDC_PSF        (1<<10)
#define         NUSMART_LCDC_BLANK      (1<<9)
#define         NUSMART_LCDC_TEST       (1<<8)
#define         NUSMART_LCDC_RGBF       (1<<7)	/*RGB24 */
#define		NUSMART_LCDC_RGB565	(1<<6)
#define         NUSMART_LCDC_INTM_ERR   ((1<<2)|(1<<3))
#define         NUSMART_LCDC_INTM_V     (1<<1)
#define         NUSMART_LCDC_INTM_H     (1<<0)

#define         NUSMART_LCDC_RESERV     0x04

#define         NUSMART_LCDC_HINTV      0x08
#define         NUSMART_LCDC_VINTV      0x0c
#define         INTV(fp,bp,syn)         ((fp<<23)|(bp<<12)|(syn))

#define         NUSMART_LCDC_PRIPTR     0x10
#define         NUSMART_LCDC_SECPTR     0x14
#define         NUSMART_LCDC_MODULO     0x18
#define         NUSMART_LCDC_HVSIZE     0x1c

#define         NUSMART_LCDC_CFG_AXI    0x20
#define         NUSMART_LCDC_XFER_NUM    0x30
#define		NUSMART_LCDC_ARBLEN_1	(0)
#define		NUSMART_LCDC_ARBLEN_2	(1<<8)
#define		NUSMART_LCDC_ARBLEN_4	(3<<8)
#define		NUSMART_LCDC_ARBLEN_8	(7<<8)
#define		NUSMART_LCDC_ARBLEN_16	(0xf<<8)

#define         HV_SIZE(sh,sv)          ((sv<<16)|sh)
#define         NUSMART_LCDC_BASE       0x050d0000

#define		FBIOPUT_FBCONVFLAG	_IOW('F', 0x19, int)
#define		FBIOPUT_FBTUNE		_IOW('F', 0x1f, struct fb_var_screeninfo)

#ifdef  WAIT_FOR_VSYNC
#define 	FBIO_WAITFORVSYNC       _IOW('F', 0x20, __u32)
#endif

#ifdef	NUSMART_PW_CTRL
#define		FBIOPUT_PWSTAT		_IOW('F', 0x21, struct clk_status)

#define 	GET_FB_BUFFER_ADDR	_IOW('F', 0x22, unsigned long)
#define 	CHNAGE_FRAMEBUFFER_ADDR _IOW('F', 0x23, unsigned long)
#define 	SET_RESOLUTION_RATIO	_IOW('F', 0x24, struct fb_videomode)
#define 	LCD_ON		(8 +11)

struct clk_status {
	int clk_neon0;
	int clk_cpu1;
	int clk_lcd;
	int clk_on2;
	int clk_mali;
};
#endif

#define REG_NUM 	15
struct nusmart_par {
	//      struct device   *dev;
	struct fb_info *fb;
	void __iomem *mmio;
	u32 ctrl;
	int node;
	int fbconv;
	int mvflag;
	struct clk *clk;
	struct clk *rgb_clk;
	int irq;
	int irq_vert;
	unsigned int regsave[REG_NUM];
#ifdef  WAIT_FOR_VSYNC
	struct completion vsync;
#endif
};

struct clcd_regs {
	u32 ctrl;
	u32 reserv;
	u32 hintv;
	u32 vintv;
	u32 priptr;
	u32 secptr;
	u32 modulo;
	u32 hvsize;
	u32 cfg_axi;
};

static inline __u32 lcd_max(__u32 a, __u32 b, __u32 c)
{
	__u32 d = b > c ? b : c;

	return a > d ? a : d;
}

static inline __u32 lcd_min(__u32 a, __u32 b, __u32 c)
{
	__u32 d = b < c ? b : c;

	return a < d ? a : d;
}

static inline void check_timing(struct fb_var_screeninfo *var)
{
	u32 min = 0, move = 0;
	int flag = 0;

	if ((var->right_margin > 511) || (var->left_margin > 511)
	    || (var->hsync_len > 511)) {
		flag = 1;
		printk(KERN_WARNING
		       "LCD timing exceed 511 rm %d lm %d hsync %d",
		       var->right_margin, var->left_margin, var->hsync_len);

		min =
		    lcd_min(var->right_margin, var->left_margin,
			    var->hsync_len);
		if (var->right_margin > 511) {
			move += (var->right_margin - 511);
			var->right_margin = 511;
		}

		if (var->left_margin > 511) {
			move += (var->left_margin - 511);
			var->left_margin = 511;
		}

		if (var->hsync_len > 511) {
			move += (var->hsync_len - 511);
			var->hsync_len = 511;
		}

		if (var->right_margin == min) {
			var->right_margin += move;
		} else if (var->left_margin == min) {
			var->left_margin += move;
		} else if (var->hsync_len == min) {
			var->hsync_len += move;
		}
	}

	move = 0;
	if ((var->lower_margin > 511) || (var->upper_margin > 511)
	    || (var->vsync_len > 511)) {
		flag = 1;
		printk(KERN_WARNING
		       "LCD timing exceed 511 rm %d lm %d hsync %d",
		       var->lower_margin, var->upper_margin, var->vsync_len);

		min =
		    lcd_min(var->lower_margin, var->upper_margin,
			    var->vsync_len);
		if (var->lower_margin > 511) {
			move = (var->lower_margin - 511);
			var->lower_margin = 511;
		}

		if (var->upper_margin > 511) {
			move += (var->upper_margin - 511);
			var->upper_margin = 511;
		}

		if (var->vsync_len > 511) {
			move += (var->vsync_len - 511);
			var->vsync_len = 511;
		}

		if (var->lower_margin == min) {
			var->lower_margin += move;
		} else if (var->upper_margin == min) {
			var->upper_margin += move;
		} else if (var->vsync_len == min) {
			var->vsync_len += move;
		}
	}

	if (flag) {
		printk(KERN_INFO,
		       "x %d,y %d,pclk %d,left %d,rm %d,um %d,low %d,hsync %d,vsync %d",
		       var->xres, var->yres, var->pixclock, var->left_margin,
		       var->right_margin, var->upper_margin, var->lower_margin,
		       var->hsync_len, var->vsync_len);
	}
}

 /**/
    static inline void nusmart_clcd_var_decode(struct fb_var_screeninfo *var,
					       struct clcd_regs *regs)
{
	u32 val = NUSMART_LCDC_ENABLE | NUSMART_LCDC_INTM_ERR;

	check_timing(var);
	regs->hintv = INTV(var->right_margin, var->left_margin, var->hsync_len);

	regs->vintv =
	    INTV(var->lower_margin, var->upper_margin, var->vsync_len);
	regs->hvsize = HV_SIZE(var->xres, var->yres);

	regs->modulo = 0;

	switch (var->bits_per_pixel) {
	case 15:
		//val &= ~(NUSMART_LCDC_RGBF | NUSMART_LCDC_RGB565);
		//break;
	case 16:
		if (5 == var->green.length) {
			val &= ~(NUSMART_LCDC_RGBF | NUSMART_LCDC_RGB565);

		} else {
			val &= ~NUSMART_LCDC_RGBF;
			val |= NUSMART_LCDC_RGB565;
		}
		break;
	case 24:
		val &= ~NUSMART_LCDC_RGB565;
		val |= NUSMART_LCDC_RGBF;
		break;
	case 32:
		val &= ~NUSMART_LCDC_RGB565;
		val |= NUSMART_LCDC_RGBF;
		break;
	}

	regs->ctrl = val;

}

static inline void nusmart_clcd_decode(struct fb_info *fb,
				       struct clcd_regs *regs)
{
	u32 val;
	struct nusmart_par *par = fb->par;
	//void __iomem * polreg = __io_address(NS115_SCM_BASE+0x8);
	/*
	 * Program the CLCD controller registers and start the CLCD
	 */
	check_timing(&fb->var);
#ifdef CONFIG_MACH_NS115_PAD_PROTOTYPE
	regs->hintv =
	    INTV(fb->var.left_margin, fb->var.right_margin, fb->var.hsync_len);
	regs->vintv =
	    INTV(fb->var.upper_margin, fb->var.lower_margin, fb->var.vsync_len);
#else
	regs->hintv =
	    INTV(fb->var.right_margin, fb->var.left_margin, fb->var.hsync_len);
	regs->vintv =
	    INTV(fb->var.lower_margin, fb->var.upper_margin, fb->var.vsync_len);
#endif
	regs->hvsize = HV_SIZE(fb->var.xres, fb->var.yres);

	regs->modulo = 0;
	val = par->ctrl;

	switch (fb->var.bits_per_pixel) {
	case 15:
		//val &= ~(NUSMART_LCDC_RGBF | NUSMART_LCDC_RGB565);
		//fb->var.bits_per_pixel = 16;
		//break;
	case 16:
		if (5 == fb->var.green.length) {
			//dbg("fb:rgb555");
			val &= ~(NUSMART_LCDC_RGBF | NUSMART_LCDC_RGB565);

		} else {
			//dbg("fb:rgb565");
			val &= ~NUSMART_LCDC_RGBF;
			val |= NUSMART_LCDC_RGB565;
		}
		break;
	case 24:
		val &= ~NUSMART_LCDC_RGB565;
		val |= NUSMART_LCDC_RGBF;
		break;
	case 32:
		val &= ~NUSMART_LCDC_RGB565;
		val |= NUSMART_LCDC_RGBF;
		break;
	}

	regs->ctrl = val | NUSMART_LCDC_INTM_ERR;
#if 0
	val = 0;
	val = readl(polreg);
	val &= 0xffff;
	val |= (3 << 17);
	if (FB_SYNC_HOR_HIGH_ACT & fb->var.sync) {
		val |= (1 << 20);
	}

	if (FB_SYNC_VERT_HIGH_ACT & fb->var.sync) {
		val |= (1 << 21);
	}

	if (!(FB_SYNC_COMP_HIGH_ACT & fb->var.sync)) {
		val |= (7 << 23);
	}
	printk("polvalue %x", val);
	//writel(val, polreg);
#endif
}
#endif	/*__NS115_CLCD_H__*/
