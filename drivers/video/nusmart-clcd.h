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
#include <linux/fb.h>
#include <linux/clk.h>
#include <mach/hardware.h>
#include <mach/board-ns2816.h>

#ifdef CONFIG_DRM_MALI
#include <mach/ump_interface.h>
#endif

/*
 * CLCD Controller Internal Register addresses
 */
#define         NUSMART_LCDC_CTRL       0x00
 
#define         NUSMART_LCDC_ERRF       (1<<18)
#define         NUSMART_LCDC_INTF_V     (1<<17)
#define         NUSMART_LCDC_INTF_H     (1<<16)
#define         NUSMART_LCDC_PTRF       (1<<13) /*primary=0,secondary=1*/
#define         NUSMART_LCDC_ENABLE     (1<<11)
#define         NUSMART_LCDC_PSF        (1<<10)
#define         NUSMART_LCDC_BLANK      (1<<9)
#define         NUSMART_LCDC_TEST       (1<<8)
#define         NUSMART_LCDC_RGBF       (1<<7)  /*RGB24*/
#define		NUSMART_LCDC_RGB565	(1<<6)
#define         NUSMART_LCDC_INTM_ERR   (1<<2)
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
struct clk_status
{
	int clk_neon0;
	int clk_cpu1;
	int clk_lcd;
	int clk_on2;
	int clk_mali;
};
#endif

#ifdef	NUSMART_FB_ON2_SPEC

/*on2 specific ioctl*/
/*ctrl on2 mode 1:on	0:off*/
#define		FBIOPUT_FBON2MODE	_IOW('F', 0x1a, int)	
/*query status*/
#define		FBIOGET_FBON2MODE	_IOR('F', 0x1e, int)	
/*get buffer for on2*/
#define		FBIOGET_FBON2BUF	_IOR('F', 0x1b, struct on2_buf)	
/*set clcd resolution for on2*/
#define		FBIOPUT_FBON2RES	_IOW('F', 0x1c, struct on2_res)	
/*set on2 on screen buffer by index.0,first buffer.1,second buffer...more if needed.*/
#define		FBIOPUT_FBON2SHOW	_IOW('F', 0x1d, unsigned int)	

/*get osd picture physical addresses when on2 does alpha blending*/
#define         FBIOGET_FBON2ADDR       _IOR('F', 0x22, struct on2_alpha)
/*set osd picture physical addresses when on2 does alpha blending*/
#define         FBIOSET_FBON2ADDR       _IOW('F', 0x23, struct on2_alpha)
#define         FB_LOG_STAGEFRIGHT      _IOW('F', 0x24, int)


#define NUSMART_PL330

#ifdef NUSMART_PL330

#define        FB_PL330_MEMCPY         _IOW('F', 0x25, struct pl330_memcpy)
struct pl330_memcpy
{
       unsigned int dst;
       unsigned int src;
       unsigned int len;

};

#endif


struct on2_alpha
{
     unsigned int width;
     unsigned int height;
     unsigned int x;
     unsigned int y;
     unsigned int address;

};

#ifdef CONFIG_DRM_MALI
#define		GET_UMP_SECURE_ID       _IOWR('F', 0x26, unsigned int)
#endif

#define		ON2BUF_COUNT		(4)
#define 	ON2BUF_SIZE		SZ_4M
/* 
 * This structure defines the hardware state of the graphics card. Normally
 * you place this in a header file in linux/include/video. This file usually
 * also includes register information. That allows other driver subsystems
 * and userland applications the ability to use the same header file to 
 * avoid duplicate work and easy porting of software. 
 */
struct on2_buf_item
{
	unsigned char *	vptr;
	unsigned char *	pptr;
};
struct on2_buf
{
	struct on2_buf_item buf[ON2BUF_COUNT];
};
struct on2_res
{
	int x;
	int y;
};
#endif

#define REG_NUM 	15
struct nusmart_par
{
	struct fb_info 	*fb;
	void __iomem	*mmio;
	u32		ctrl;
	int		fbconv;
	int		mvflag;
	struct clk *	clk;
	int		irq;
	int		irq_vert;
	unsigned int	regsave[REG_NUM];
#ifdef  WAIT_FOR_VSYNC
	struct completion vsync;
#endif
#ifdef	 NUSMART_FB_ON2_SPEC
	int 		on2resindex;
	struct on2_buf	on2buf;
	struct on2_res	on2res;
	struct on2_alpha on2alpha;
	int 		on2flag;
#endif
#ifdef CONFIG_DRM_MALI
	ump_dd_handle ump_wrapped_buffer;
#endif
};


struct clcd_regs {
	u32			ctrl;
	u32			reserv;
	u32			hintv;
	u32			vintv;
	u32			priptr;
	u32			secptr;
	u32			modulo;
	u32			hvsize;
	u32			cfg_axi;
};
/**/
static inline void nusmart_clcd_var_decode(struct fb_var_screeninfo * var, struct clcd_regs * regs)
{
	u32 val=NUSMART_LCDC_ENABLE | NUSMART_LCDC_INTM_ERR;

	regs->hintv = INTV(var->right_margin, var->left_margin, var->hsync_len);

	regs->vintv = INTV(var->lower_margin, var->upper_margin, var->vsync_len);
	regs->hvsize = HV_SIZE(var->xres, var->yres);

	regs->modulo = 0;

	switch (var->bits_per_pixel) {
	case 15:
		//val &= ~(NUSMART_LCDC_RGBF | NUSMART_LCDC_RGB565);
		//break;
	case 16:
		if(5 == var->green.length)
		{
			val &= ~(NUSMART_LCDC_RGBF | NUSMART_LCDC_RGB565);
		
		}else{
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
static inline void nusmart_clcd_decode(struct fb_info *fb, struct clcd_regs *regs)
{
	u32 val;
	struct nusmart_par *par = fb->par;
	void __iomem * polreg = __io_address(NS2816_SCM_BASE+0x8); 	
	/*
	 * Program the CLCD controller registers and start the CLCD
	 */
	regs->hintv = INTV(fb->var.right_margin, fb->var.left_margin, fb->var.hsync_len);

	regs->vintv = INTV(fb->var.lower_margin, fb->var.upper_margin, fb->var.vsync_len);
	regs->hvsize = HV_SIZE(fb->var.xres, fb->var.yres);

	regs->modulo = 0;
	val = par->ctrl;

	switch (fb->var.bits_per_pixel) {
	case 15:
//		val &= ~(NUSMART_LCDC_RGBF | NUSMART_LCDC_RGB565);
//		fb->var.bits_per_pixel = 16;
//		break;
	case 16:
		if(5 == fb->var.green.length)
		{
			dbg("fb:rgb555");
			val &= ~(NUSMART_LCDC_RGBF | NUSMART_LCDC_RGB565);
		
		}else{
			dbg("fb:rgb565");
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
	val = 0;
	val = readl(polreg);
	val &= 0xffff;
	val |= (3<<17);
	if(FB_SYNC_HOR_HIGH_ACT & fb->var.sync) {
		val |= (1<<20);	
	}
	
	if(FB_SYNC_VERT_HIGH_ACT & fb->var.sync) {
		val |= (1<<21);	
	}

	if(!(FB_SYNC_COMP_HIGH_ACT & fb->var.sync)) {
		val |= (7<<23);
	}
	printk("polvalue %x",val);
	writel(val, polreg);
}

