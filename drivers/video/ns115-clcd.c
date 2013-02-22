/*
 * linux/drivers/video/nusmart_clcd.c -- nusmart clcd for a frame buffer device
 *
 *  Modified to new api Jan 2010 by zeyuan.xu (zeyuan.xu@nufrontsoft.com)
 *
 *  Created 28 Dec 1997 by Geert Uytterhoeven
 *
 *
 *  I have started rewriting this driver as a example of the upcoming new API
 *  The primary goal is to remove the console code from fbdev and place it
 *  into fbcon.c. This reduces the code and makes writing a new fbdev driver
 *  easy since the author doesn't need to worry about console internals. It
 *  also allows the ability to run fbdev without a console/tty system on top 
 *  of it. 
 *
 *  First the roles of struct fb_info and struct display have changed. Struct
 *  display will go away. The way the new framebuffer console code will
 *  work is that it will act to translate data about the tty/console in 
 *  struct vc_data to data in a device independent way in struct fb_info. Then
 *  various functions in struct fb_ops will be called to store the device 
 *  dependent state in the par field in struct fb_info and to change the 
 *  hardware to that state. This allows a very clean separation of the fbdev
 *  layer from the console layer. It also allows one to use fbdev on its own
 *  which is a bounus for embedded devices. The reason this approach works is  
 *  for each framebuffer device when used as a tty/console device is allocated
 *  a set of virtual terminals to it. Only one virtual terminal can be active 
 *  per framebuffer device. We already have all the data we need in struct 
 *  vc_data so why store a bunch of colormaps and other fbdev specific data
 *  per virtual terminal. 
 *
 *  As you can see doing this makes the con parameter pretty much useless
 *  for struct fb_ops functions, as it should be. Also having struct  
 *  fb_var_screeninfo and other data in fb_info pretty much eliminates the 
 *  need for get_fix and get_var. Once all drivers use the fix, var, and cmap
 *  fbcon can be written around these fields. This will also eliminate the
 *  need to regenerate struct fb_var_screeninfo, struct fb_fix_screeninfo
 *  struct fb_cmap every time get_var, get_fix, get_cmap functions are called
 *  as many drivers do now. 
 *
 *  This file is subject to the terms and conditions of the GNU General Public
 *  License. See the file COPYING in the main directory of this archive for
 *  more details.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/string.h>
#include <linux/mm.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/fb.h>
#include <linux/init.h>
#include <linux/dma-mapping.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/gpio.h>
#include <linux/earlysuspend.h>
#include <asm/uaccess.h>
#include <mach/memory.h>
#include <mach/get_bootargs.h>
#include <mach/lcdc.h>
#include <asm/irq.h>
#include <asm/mach-types.h>
/* NS115 do NOT need these any more */
#define NUSMART_PW_CTRL
/* NS115 do NOT need these any more */

#define WAIT_FOR_VSYNC

#ifdef WAIT_FOR_VSYNC
#include <linux/completion.h>
#endif

/*define frame buffer size in byte*/
#define VIDEO_MEM_SIZE		nusmart_lcd_len() 
/*define frame buffer alloc by reserve*/
#define FB_RESERV
/*!frame buffer size limited to 4MB if don't reserve*/
#ifdef FB_RESERV
#define VIDEO_MEM_BASE		nusmart_lcd_base()	
#endif


/*******************************************************************/
#define DUAL_LCD

#define	DISP0_NAME 	"nusmartfb"
#define	DISP1_NAME 	"nusmartfb1"
#define err(x,args...)  printk(KERN_ERR "%s:"#x"\n",__func__,##args)
/*******************************************************************/

#define DEBUG
#ifdef DEBUG
//#define ctrace printk("ctrace:%s\n",__func__)
void display_modelist(struct list_head *head);
#define ctrace 
#define dbg(x,args...) printk(KERN_DEBUG "%s:"#x"\n",__func__,##args)
#define dbg_var(var)	dbg("resolution:%dx%d-%d",var->xres,var->yres,var->bits_per_pixel)
#else
#define ctrace 
#define dbg(x,args...) 
#define dbg_var(var)	
#endif /*DEBUG*/
#include "ns115-clcd.h"
#include "edid.h"

#define DDC_ARGS	"auto"
#define DDC_ARGS_16	"auto-16"
#define DDC_ARGS_32	"auto-32"
/*
 * Driver data
 */
static char *mode_option_disp0 = NULL;
static char *mode_option_disp1 = NULL;
static int  get_lcd1_args = 0;
static int  set_resolution = 0;

static u32	pseudo_palette[16];
static u32	pseudo_palette1[16];

static int fb0_bits_per_pixel = 32;

extern int send_new_resolution_event(unsigned long val, void *v);
/*
 * Here we define the default structs fb_fix_screeninfo and fb_var_screeninfo
 * if we don't use modedb. If we do use modedb see nusmartfb_init how to use it
 * to get a fb_var_screeninfo. Otherwise define a default var as well. 
 */
static struct fb_fix_screeninfo nusmartfb_fix __devinitdata = {
	.id =		DISP0_NAME,
	.type =		FB_TYPE_PACKED_PIXELS,
	.visual =	/*FB_VISUAL_PSEUDOCOLOR |*/ FB_VISUAL_TRUECOLOR,
	.xpanstep =	0,
	.ypanstep =	1,
	.ywrapstep =	0, 
	.accel =	FB_ACCEL_NONE,
};

static struct fb_fix_screeninfo nusmartfb_fix1 __devinitdata = {
	.id =		DISP1_NAME,
	.type =		FB_TYPE_PACKED_PIXELS,
	.visual =	/*FB_VISUAL_PSEUDOCOLOR |*/ FB_VISUAL_TRUECOLOR,
	.xpanstep =	0,
	.ypanstep =	1,
	.ywrapstep =	0, 
	.accel =	FB_ACCEL_NONE,
};


static int nusmartfb_init(void);
static int nusmartfb_setup(char *options, int id);
int hdmi_edid_fetch(unsigned char *edid, struct fb_info *info);
void nusmartfb_reset(struct fb_info *fb);
static unsigned char *hdmi_edid = NULL;
static unsigned int hdmi_plugin = 0;
static unsigned int hdmi_state = 0;
static ssize_t show_modes(struct device *device, struct device_attribute *attr,
			  char *buf);


static void ns115_regs_print(struct fb_info *fb)
{
	struct nusmart_par *par = fb->par;

	printk(KERN_INFO "CLCD: Base 0x%08x Registers value\n"
			KERN_INFO "  0x%02x 0x%08x\n"
			KERN_INFO "  0x%02x 0x%08x\n"
			KERN_INFO "  0x%02x 0x%08x\n"
			KERN_INFO "  0x%02x 0x%08x\n"
			KERN_INFO "  0x%02x 0x%08x\n"
			KERN_INFO "  0x%02x 0x%08x\n"
			KERN_INFO "  0x%02x 0x%08x\n"
			KERN_INFO "  0x%02x 0x%08x\n"
			KERN_INFO "  0x%02x 0x%08x\n"
			KERN_INFO "  0x%02x 0x%08x\n"
			KERN_INFO "  0x%02x 0x%08x\n"
			KERN_INFO "  0x%02x 0x%08x\n",

			fb->fix.mmio_start,
			NUSMART_LCDC_CTRL, readl(par->mmio + NUSMART_LCDC_CTRL), 
			NUSMART_LCDC_RESERV, readl(par->mmio + NUSMART_LCDC_RESERV),
			NUSMART_LCDC_HINTV, readl(par->mmio + NUSMART_LCDC_HINTV), 
			NUSMART_LCDC_VINTV, readl(par->mmio + NUSMART_LCDC_VINTV),
			NUSMART_LCDC_PRIPTR, readl(par->mmio + NUSMART_LCDC_PRIPTR), 
			NUSMART_LCDC_SECPTR, readl(par->mmio + NUSMART_LCDC_SECPTR),
			NUSMART_LCDC_MODULO, readl(par->mmio + NUSMART_LCDC_MODULO), 
			NUSMART_LCDC_HVSIZE, readl(par->mmio + NUSMART_LCDC_HVSIZE),
			NUSMART_LCDC_CFG_AXI, readl(par->mmio + NUSMART_LCDC_CFG_AXI),
			0x24, readl(par->mmio + 0x24),
			0x28, readl(par->mmio + 0x28),
			NUSMART_LCDC_XFER_NUM, readl(par->mmio + NUSMART_LCDC_XFER_NUM));

} 

static int set_bitfield(struct fb_var_screeninfo *var, struct fb_info *info)
{
	int ret = 0;

	/*Ctrace;*/
	memset(&var->transp, 0, sizeof(var->transp));

	var->red.msb_right = 0;
	var->green.msb_right = 0;
	var->blue.msb_right = 0;

	switch (var->bits_per_pixel) {
		case 1:
		case 2:
		case 4:
		case 8:
			/*
			   var->red.length 	= var->bits_per_pixel;
			   var->red.offset		=0;
			   var->blue.length 	= var->bits_per_pixel;
			   var->blue.offset	=0;
			   var->green.length 	= var->bits_per_pixel;
			   var->green.offset	=0;
			 */
			return -EINVAL;
			break;

		case 16:
			/*
			   if(var->red.length == 5 && 
			   var->blue.length == 5 &&
			   var->green.length ==5)
			   return 0;
			 */
			var->red.length = 5;
			var->blue.length = 5;
			/*
			 * Green length can be 5 or 6 depending whether
			 * we're operating in RGB555 or RGB565 mode.
			 */
			//	if (var->green.length != 5 && var->green.length != 6)
			var->green.length = 6;

			//		var->transp.length	= 0;
			//		var->transp.offset 	= 0;
			dbg("var->transp.length %u, var->transp.offset %u, var->green.length %d",var->transp.length, var->transp.offset, var->green.length);

			break;
		case 15:
			var->red.length = 5;
			var->blue.length = 5;
			var->green.length = 5;
			var->bits_per_pixel = 15;

			dbg("var->transp.length %u, var->transp.offset %u, var->green.length %d",var->transp.length, var->transp.offset, var->green.length);

			break;
		case 32:
		case 24:
			//		var->transp.length	= 8;
			//		var->transp.offset	= 24;
			var->red.length         = 8;
			var->green.length       = 8;
			var->blue.length        = 8;

			var->bits_per_pixel	= 32;

			dbg("var->transp.length %u , var->transp.offset %u",var->transp.length, var->transp.offset);

			break;
		default:
			dbg("bits_per_pixel(%d) not supported!",var->bits_per_pixel);
			ret = -EINVAL;
			break;
	}

	/*
	 * >= 16bpp displays have separate colour component bitfields
	 * encoded in the pixel data.  Calculate their position from
	 * the bitfield length defined above.
	 */
	if (ret == 0 && var->bits_per_pixel >= 15) {
		if (1) {
			/*little endian*/
			var->blue.offset = 0;
			var->green.offset = var->blue.offset + var->blue.length;
			var->red.offset = var->green.offset + var->green.length;
		} else {
			var->red.offset = 0;
			var->green.offset = var->red.offset + var->red.length;
			var->blue.offset = var->green.offset + var->green.length;
		}
	}

	return ret;	
}
/**
 *      nusmartfb_check_var - Optional function. Validates a var passed in. 
 *      @var: frame buffer variable screen structure
 *      @info: frame buffer structure that represents a single frame buffer 
 *
 *	Checks to see if the hardware supports the state requested by
 *	var passed in. This function does not alter the hardware state!!! 
 *	This means the data stored in struct fb_info and struct nusmart_par do 
 *      not change. This includes the var inside of struct fb_info. 
 *	Do NOT change these. This function can be called on its own if we
 *	intent to only test a mode and not actually set it. The stuff in 
 *	modedb.c is a example of this. If the var passed in is slightly 
 *	off by what the hardware can support then we alter the var PASSED in
 *	to what we can do.
 *
 *      For values that are off, this function must round them _up_ to the
 *      next value that is supported by the hardware.  If the value is
 *      greater than the highest value supported by the hardware, then this
 *      function must return -EINVAL.
 *
 *      Exception to the above rule:  Some drivers have a fixed mode, ie,
 *      the hardware is already set at boot up, and cannot be changed.  In
 *      this case, it is more acceptable that this function just return
 *      a copy of the currently working var (info->var). Better is to not
 *      implement this function, as the upper layer will do the copying
 *      of the current var for you.
 *
 *      Note:  This is the only function where the contents of var can be
 *      freely adjusted after the driver has been registered. If you find
 *      that you have code outside of this function that alters the content
 *      of var, then you are doing something wrong.  Note also that the
 *      contents of info->var must be left untouched at all times after
 *      driver registration.
 *
 *	Returns negative errno on error, or zero on success.
 */
static int nusmartfb_check_var(struct fb_var_screeninfo *var, struct fb_info *info)
{
	/* ... */
	int ret=0;
	int smem_len = 0;
	struct nusmart_par * par = info->par;

	dbg_var(var);

	if((var->xres*var->yres*var->bits_per_pixel/8) > VIDEO_MEM_SIZE) /*check for bus width*/{
		err("exceed limit %dx%d-%d", var->xres, var->yres, var->bits_per_pixel);
		ret = -EINVAL;
	}

	ret = set_bitfield(var, info);

	if(ret < 0)
		return ret;
	/*calculate smem_len after check_bitfield*/
	var->yres_virtual =  var->yres * 2;
	smem_len = var->xres_virtual * var->bits_per_pixel / 8 * var->yres_virtual;
	if(smem_len  > info->fix.smem_len) {
		dbg("not enough video mem alloced!");
		return -EINVAL;
	}
	return 0;	   	
}

struct fb_var_screeninfo cached_fb_var = {0,};
struct fb_var_screeninfo cached_fb_var1 = {0,};

static int is_fb_changed(struct fb_info *fb)
{
	struct nusmart_par *par = fb->par;
	ctrace;

	if(par->node){
		if (fb->var.xres != cached_fb_var1.xres ||
				fb->var.yres != cached_fb_var1.yres ||
				fb->var.xres_virtual != cached_fb_var1.xres_virtual ||
				fb->var.yres_virtual != cached_fb_var1.yres_virtual ||
				fb->var.bits_per_pixel != cached_fb_var1.bits_per_pixel ||
				fb->var.grayscale != cached_fb_var1.grayscale ||
				fb->var.green.length != cached_fb_var1.green.length ||
				fb->var.left_margin != cached_fb_var1.left_margin ||
				fb->var.right_margin != cached_fb_var1.right_margin ||
				fb->var.upper_margin != cached_fb_var1.upper_margin ||
				fb->var.lower_margin != cached_fb_var1.lower_margin ||
				fb->var.hsync_len != cached_fb_var1.hsync_len ||
				fb->var.vsync_len != cached_fb_var1.vsync_len ||
				fb->var.sync != cached_fb_var1.sync ||
				fb->var.rotate != cached_fb_var1.rotate) {

			cached_fb_var1 = fb->var;
			printk(KERN_INFO "%s %d\n", __func__, __LINE__);
			return 1;
		}
		else
			return 0;
	}else{
		if (fb->var.xres != cached_fb_var.xres ||
				fb->var.yres != cached_fb_var.yres ||
				fb->var.xres_virtual != cached_fb_var.xres_virtual ||
				fb->var.yres_virtual != cached_fb_var.yres_virtual ||
				fb->var.bits_per_pixel != cached_fb_var.bits_per_pixel ||
				fb->var.grayscale != cached_fb_var.grayscale ||
				fb->var.green.length != cached_fb_var.green.length ||
				fb->var.left_margin != cached_fb_var.left_margin ||
				fb->var.right_margin != cached_fb_var.right_margin ||
				fb->var.upper_margin != cached_fb_var.upper_margin ||
				fb->var.lower_margin != cached_fb_var.lower_margin ||
				fb->var.hsync_len != cached_fb_var.hsync_len ||
				fb->var.vsync_len != cached_fb_var.vsync_len ||
				fb->var.sync != cached_fb_var.sync ||
				fb->var.rotate != cached_fb_var.rotate) {

			cached_fb_var = fb->var;

			printk(KERN_INFO "%s %d\n", __func__, __LINE__);
			return 1;
		}
		else
			return 0;
	}
}

static void nusmart_clcd_enable(struct fb_info *fb, u32 ctrl) 
{
	struct nusmart_par * par = fb->par;
	ctrace;
	ctrl |= NUSMART_LCDC_ENABLE;
	ctrl |= 0x07000000;

#ifdef  WAIT_FOR_VSYNC
	ctrl |= NUSMART_LCDC_INTM_V;	
#endif	
	ctrl |= NUSMART_LCDC_INTM_ERR;

	//If the bit is set, DMA will fetch data when the FIFO is not almost full
	if(fb->var.xres >=1920)
		ctrl |= (1<<21);

	if(1 == par->node)
	{
#ifdef DEBUG
		printk(KERN_INFO "ctrl 0x%08x sync %d ", ctrl, fb->mode->sync);
#endif
		//bit 24 of CTRL
		if(fb->mode->sync & FB_SYNC_HOR_HIGH_ACT){
			ctrl &= (0xfeffffff);
		}else{
			ctrl |= (0x01000000);

		}

		//bit 25 of CTRL
		if(fb->mode->sync & FB_SYNC_VERT_HIGH_ACT){
			ctrl &= (0xfdffffff);
		}else{
			ctrl |= (0x02000000);
		}
	}

	writel(ctrl, par->mmio + NUSMART_LCDC_CTRL);

	/*udelay(20);*/
	/*enable clk*/
	clk_enable(par->clk);
	/*
	 * and now apply power.
	 */
	ctrl &= ~NUSMART_LCDC_PSF;
	ctrl &= ~NUSMART_LCDC_BLANK;
	writel(ctrl, par->mmio + NUSMART_LCDC_CTRL);

#ifdef DEBUG
	printk(KERN_INFO "-------------%s--------------\n", __func__);
	ns115_regs_print(fb);
#endif
}

static void nusmart_clcd_disable(struct fb_info *fb)
{
	struct nusmart_par *par = fb->par;
	u32 val;

	ctrace;

	val = readl(par->mmio + NUSMART_LCDC_CTRL);
#ifdef  WAIT_FOR_VSYNC
	val &= ~NUSMART_LCDC_INTM_V;	
#endif	
	//disable interrupt
	val &= ~NUSMART_LCDC_INTM_ERR;
	if (val & NUSMART_LCDC_ENABLE) {
		val &= ~NUSMART_LCDC_ENABLE;
		writel(val, par->mmio + NUSMART_LCDC_CTRL);
	}
	if (!(val & NUSMART_LCDC_PSF)) {
		val |= NUSMART_LCDC_PSF;
		writel(val, par->mmio + NUSMART_LCDC_CTRL);
	}

#ifdef DEBUG
	printk(KERN_INFO "-------------%s--------------\n", __func__);
	ns115_regs_print(fb);
#endif
//	clk_disable(par->clk);
}

/*bpp 16bit & res 1024x768*/
static unsigned int DISP_BASE0 = 0;
static unsigned int DISP_BASE1 = 0;
#define DIRECT_SHOW	0
#define INDIRECT_SHOW	1
static void nusmart_clcd_set_hwfb(struct fb_info *fb, int flag)
{
	struct nusmart_par *par = fb->par;
	unsigned long ustart = fb->fix.smem_start;
	static unsigned int DISP_BASE = 0;

	if(0 == par->node)
	{
		DISP_BASE0 = fb->var.xres * fb->var.yres;	
		DISP_BASE = DISP_BASE0;
	}else{
		DISP_BASE1 = fb->var.xres * fb->var.yres;	
		DISP_BASE = DISP_BASE1;
	}

	ctrace;
	if(flag == INDIRECT_SHOW) {
		ustart += fb->var.yoffset * fb->fix.line_length;
		ustart += DISP_BASE * 4;
		writel(ustart, par->mmio + NUSMART_LCDC_PRIPTR);
		writel(ustart, par->mmio + NUSMART_LCDC_SECPTR);
	}
	else {
		/* Modify for LCDC test by weixing */
		//		ustart += fb->var.yoffset * fb->fix.line_length;
		writel(ustart, par->mmio + NUSMART_LCDC_PRIPTR);
		ustart += fb->var.yres * fb->fix.line_length;
		writel(ustart, par->mmio + NUSMART_LCDC_SECPTR);
	}
}

static void nusmart_fb_set_burstlen(struct fb_info *fb)
{
	struct nusmart_par *par = fb->par;
	struct clcd_regs regs;
	struct fb_var_screeninfo * var = &fb->var;
	regs.cfg_axi = readl(par->mmio + NUSMART_LCDC_CFG_AXI);

	ctrace;

	//QVGA-320X240 85HZ, BurstLen <= 8 
	//if((320 == var->xres) && (240 == var->yres) && ( 8500000 == var->pixclock))
	if((320 == var->xres) && (240 == var->yres))
	{
		regs.cfg_axi &= ~(0xf00);
		regs.cfg_axi |= (NUSMART_LCDC_ARBLEN_2);
		writel(regs.cfg_axi, par->mmio + NUSMART_LCDC_CFG_AXI);
		dbg("change burst len to 8!\n");

		//SVGA-800X600 85HZ, BurstLen <= 4
		//}else if((800 == var->xres) && (600 == var->yres) && ( 17761 == var->pixclock))
	}else if((800 == var->xres) && (600 == var->yres)){
		regs.cfg_axi &= ~(0xf00);
		regs.cfg_axi |= (NUSMART_LCDC_ARBLEN_2);
		writel(regs.cfg_axi, par->mmio + NUSMART_LCDC_CFG_AXI);
		dbg("change burst len to 4!\n");

		//UXGA-1600X1200 60HZ, BurstLen <= 8
		//}else if((1600 == var->xres) && (1200 == var->yres) && ( 6172 == var->pixclock))
	}else if((1600 == var->xres) && (1200 == var->yres)){
		regs.cfg_axi &= ~(0xf00);
		regs.cfg_axi |= (NUSMART_LCDC_ARBLEN_2);
		writel(regs.cfg_axi, par->mmio + NUSMART_LCDC_CFG_AXI);
		dbg("change burst len to 8!\n");

		//WSXGA+-1680X1050 60HZ, BurstLen <= 2
		//}else if((1680 == var->xres) && (1050 == var->yres) && ( 6848 == var->pixclock))
	}else if((1680 == var->xres) && (1050 == var->yres)){
		regs.cfg_axi &= ~(0xf00);
		regs.cfg_axi |= (NUSMART_LCDC_ARBLEN_2);
		writel(regs.cfg_axi, par->mmio + NUSMART_LCDC_CFG_AXI);
		dbg("change burst len to 2!\n");
	}else{
		regs.cfg_axi &= ~(0xf00);
#ifdef CONFIG_MACH_NS115_STICK
		regs.cfg_axi |= (NUSMART_LCDC_ARBLEN_2);
#else
		regs.cfg_axi |= (NUSMART_LCDC_ARBLEN_8);
#endif
		writel(regs.cfg_axi, par->mmio + NUSMART_LCDC_CFG_AXI);
		dbg("change burst len to 8!\n");
	}

#if 0
	//set outstanding to 32
	regs.cfg_axi = readl(par->mmio + NUSMART_LCDC_CFG_AXI);
	regs.cfg_axi |= (0x1f<<26);
	//writel(regs.cfg_axi, par->mmio + NUSMART_LCDC_CFG_AXI);
	writel(0x1C80C700, par->mmio + NUSMART_LCDC_CFG_AXI);

	writel(0x180, par->mmio + 0x24);
	writel(0x64, par->mmio + 0x28);
	//writel(0x170, par->mmio + 0x24);
	//writel(0xc8, par->mmio + 0x28);
	nusmartfb_reset(fb);

#ifdef DEBUG
	printk(KERN_INFO "-------------%s--------------\n", __func__);
	ns115_regs_print(fb);
#endif
#endif
}

void nusmartfb_reset(struct fb_info *fb)
{
	struct nusmart_par *par = fb->par;
	unsigned int ctrl;
	
	ctrl = readl(par->mmio + NUSMART_LCDC_CTRL);

	//rgb disable
	ctrl &= ~(NUSMART_LCDC_ENABLE);
	writel(ctrl, par->mmio + NUSMART_LCDC_CTRL);

	//rgb enable
	ctrl |= NUSMART_LCDC_ENABLE;
	writel(ctrl, par->mmio + NUSMART_LCDC_CTRL);

	printk(KERN_INFO "%s %d", __func__, fb->node);

}

/**
 *      nusmartfb_set_par - Optional function. Alters the hardware state.
 *      @info: frame buffer structure that represents a single frame buffer
 *
 *	Using the fb_var_screeninfo in fb_info we set the resolution of the
 *	this particular framebuffer. This function alters the par AND the
 *	fb_fix_screeninfo stored in fb_info. It doesn't not alter var in 
 *	fb_info since we are using that data. This means we depend on the
 *	data in var inside fb_info to be supported by the hardware. 
 *
 *      This function is also used to recover/restore the hardware to a
 *      known working state.
 *
 *	nusmartfb_check_var is always called before nusmartfb_set_par to ensure that
 *      the contents of var is always valid.
 *
 *	Again if you can't change the resolution you don't need this function.
 *
 *      However, even if your hardware does not support mode changing,
 *      a set_par might be needed to at least initialize the hardware to
 *      a known working state, especially if it came back from another
 *      process that also modifies the same hardware, such as X.
 *
 *      If this is the case, a combination such as the following should work:
 *
 *      static int nusmartfb_check_var(struct fb_var_screeninfo *var,
 *                                struct fb_info *info)
 *      {
 *              *var = info->var;
 *              return 0;
 *      }
 *
 *      static int nusmartfb_set_par(struct fb_info *info)
 *      {
 *              init your hardware here
 *      }
 *
 *	Returns negative errno on error, or zero on success.
 */
#ifdef CONFIG_MACH_NS115_STICK
static int first_flag = 2;
#endif
static int nusmartfb_set_par(struct fb_info *fb)
{
	struct nusmart_par *par = fb->par;
	/* ... */
	struct clcd_regs regs;
	struct fb_var_screeninfo * var = &fb->var;
	int clk=1000000000;
	int xfer_num, xres, yres, rgb32, burst_len;

	printk(KERN_EMERG "%s %d ", __func__, first_flag);
	ctrace;
	dbg_var(var);

	fb->fix.line_length = fb->var.xres_virtual *
		fb->var.bits_per_pixel / 8;
	if(fb->var.bits_per_pixel<15) {
		dbg("bit_per_pixel(%d) not supported",fb->var.bits_per_pixel);
	}	
	fb->fix.visual = FB_VISUAL_TRUECOLOR;

#ifdef CONFIG_MACH_NS115_STICK
	if(first_flag)
	{
		first_flag--;

		if((!first_flag) && (!hdmi_state))
			goto LCD_RESET;
		return 0;
	}

#endif

LCD_RESET:

#ifdef CONFIG_MACH_NS115_STICK
	if(is_fb_changed(fb) || hdmi_plugin || (!hdmi_state)) {
#else
	if(is_fb_changed(fb) || hdmi_plugin) {
#endif
		nusmart_clcd_decode(fb, &regs);
		nusmart_clcd_disable(fb);

		nusmart_fb_set_burstlen(fb);	
		writel(regs.hintv, par->mmio + NUSMART_LCDC_HINTV);
		writel(regs.vintv, par->mmio + NUSMART_LCDC_VINTV);
		writel(regs.modulo, par->mmio + NUSMART_LCDC_MODULO);
		writel(regs.hvsize, par->mmio + NUSMART_LCDC_HVSIZE);

		xres = fb->var.xres;
		yres = fb->var.yres;
		rgb32 = (fb->var.bits_per_pixel == 32) ? 1 : 0;
		burst_len = ( (readl(par->mmio + NUSMART_LCDC_CFG_AXI) & 0xF00)  >> 8) + 1;

		xfer_num =  (xres * yres / ((rgb32 ? 4: 8) * burst_len )) - 1;
		writel(xfer_num, par->mmio + NUSMART_LCDC_XFER_NUM);
		printk(KERN_ERR "xfer_num 0x%x, xres %d, yres %d, rgb32 %d, burst_len %d\n", xfer_num,
				xres, yres, rgb32, burst_len);

		nusmart_clcd_set_hwfb(fb, DIRECT_SHOW);	
		par->ctrl = regs.ctrl;
		clk /= fb->var.pixclock;

		printk(KERN_INFO "clk %d \n", clk);
		clk *= 1000;

		printk(KERN_EMERG "%s set  clock rate to %d, pixclock id %d\n", __func__, clk, fb->var.pixclock); 
#if 0		
		if(50400000 == clk){
			clk = 87500000/2;
		}
#endif
		if(clk_set_rate(par->clk, clk)<0)
			printk(KERN_EMERG "set clcd clock failed!%d\n",clk);

		if(0 == par->node)
		{
			DISP_BASE0 = fb->var.xres * fb->var.yres;	
		}else{
			DISP_BASE1 = fb->var.xres * fb->var.yres;	
		}

		nusmart_clcd_enable(fb, regs.ctrl);
	}
#ifdef DEBUG
	printk(KERN_INFO "-------------%s--------------\n", __func__);
	ns115_regs_print(fb);
#endif
	return 0;	
}

static int set_resolution_ratio(struct fb_info * fb,struct fb_videomode *find_mode)
{
	const struct fb_videomode * m = NULL;
	struct fb_var_screeninfo var = {0,};
	int retval;
	struct fb_var_screeninfo stdvar;
	struct fb_videomode mode;
	char mode_option[50] = {};

	fb_videomode_to_var(&var, find_mode);

#ifdef CONFIG_MACH_NS115_STICK
#ifdef CONFIG_USES_FAKE_1080P
		var.bits_per_pixel = 32;
#else
	if((var.xres * var.yres) <= (1280 * 720))
	{
		var.bits_per_pixel = 32;
	}else{
 		var.bits_per_pixel = 16;
	}
#endif
#else
	if(fb0_bits_per_pixel == 16 && (var.xres * var.yres) <= (1280 * 720))
	{
		var.bits_per_pixel = 32;
	}else{
 		var.bits_per_pixel = 16;
	}
#endif

	if(!find_mode->refresh)
		find_mode->refresh = 60;

	printk(KERN_INFO "set resolution to %ux%u-%u@%d\n", find_mode->xres, find_mode->yres, var.bits_per_pixel, find_mode->refresh);

#ifdef DEBUG
	display_modelist(&fb->modelist);
#endif
	m = fb_find_nearest_mode(find_mode, &fb->modelist);

	if ((m && (m->xres != var.xres || m->yres != var.yres))
            || (!m && ((var.xres == 1280 && var.yres == 720) || (var.xres == 1920 && var.yres == 1080)))) {
		sprintf(mode_option, "%ux%u-%u@%u", var.xres, var.yres, var.bits_per_pixel, find_mode->refresh);
		retval = fb_find_mode(&stdvar, fb, mode_option, NULL, 0,NULL, var.bits_per_pixel);
		if (retval > 0) {
			fb_var_to_videomode(&mode, &stdvar);
			fb_videomode_to_var(&fb->var, &mode);
			// fb->mode = &mode;
			memcpy(fb->mode, &mode, sizeof(struct fb_videomode));
			goto done;
		}
	}

	if (m) {
		printk(KERN_INFO "xres %d,yres %d,pixclk %d\n"
				"leftm %d,rm %d,um %d,lowm %d"
				"hsync %d,vsync %d, sync %d",
				m->xres,m->yres,m->pixclock,
				m->left_margin,m->right_margin,m->upper_margin,m->lower_margin,
				m->hsync_len,m->vsync_len, m->sync);

		fb_videomode_to_var(&fb->var, m);
		memcpy(fb->mode, m, sizeof(struct fb_videomode));
		//fb->mode = m;
	} else {
		printk(KERN_EMERG "fb%d: %s set resolution failed! %dx%d resolution not support!\n", 
				fb->node, fb->fix.id, var.xres, var.yres);
		retval = -2;

		return retval;
	}

done:
#ifdef CONFIG_MACH_NS115_STICK
#ifdef CONFIG_USES_FAKE_1080P
		fb->var.bits_per_pixel = 32;
#else
	if((fb->mode->xres * fb->mode->yres) <= (1280 * 720))
	{
		fb->var.bits_per_pixel = 32;
	}else{
		fb->var.bits_per_pixel = 16;
	}
#endif
#else
	if(fb0_bits_per_pixel == 16 && (fb->mode->xres * fb->mode->yres) <= (1280 * 720))
	{
		fb->var.bits_per_pixel = 32;
	}else{
		fb->var.bits_per_pixel = 16;
	}
#endif
	retval = nusmartfb_check_var(&fb->var, fb);
	memset(fb->screen_base, 0x0, fb->fix.smem_len);
	nusmartfb_set_par(fb);
#ifdef CONFIG_FB_HDMI_NS115
	send_new_resolution_event(1, fb->mode);
#endif

	return retval;
}

static int set_var(struct fb_info * info,struct fb_var_screeninfo * var)
{
	struct nusmart_par * par = info->par;
	struct clcd_regs regs;
	int ret=0;

	writel((5<<9), par->mmio + NUSMART_LCDC_CTRL);
	nusmart_clcd_var_decode(var,&regs);

	writel(regs.hintv, par->mmio + NUSMART_LCDC_HINTV);
	writel(regs.vintv, par->mmio + NUSMART_LCDC_VINTV);
	writel(regs.modulo, par->mmio + NUSMART_LCDC_MODULO);
	writel(regs.hvsize, par->mmio + NUSMART_LCDC_HVSIZE);

	regs.ctrl |= (readl(par->mmio) & NUSMART_LCDC_TEST);
	writel(regs.ctrl, par->mmio + NUSMART_LCDC_CTRL);
	memcpy(&info->var,var,sizeof(struct fb_var_screeninfo));

	return ret;
}

static inline u32 convert_bitfield(int val, struct fb_bitfield *bf)
{
	unsigned int mask = (1 << bf->length) - 1;

	ctrace;
	return (val >> (16 - bf->length) & mask) << bf->offset;
}

/**
 *  	nusmartfb_setcolreg - Optional function. Sets a color register.
 *      @regno: Which register in the CLUT we are programming 
 *      @red: The red value which can be up to 16 bits wide 
 *	@green: The green value which can be up to 16 bits wide 
 *	@blue:  The blue value which can be up to 16 bits wide.
 *	@transp: If supported, the alpha value which can be up to 16 bits wide.
 *      @info: frame buffer info structure
 * 
 *  	Set a single color register. The values supplied have a 16 bit
 *  	magnitude which needs to be scaled in this function for the hardware. 
 *	Things to take into consideration are how many color registers, if
 *	any, are supported with the current color visual. With truecolor mode
 *	no color palettes are supported. Here a pseudo palette is created
 *	which we store the value in pseudo_palette in struct fb_info. For
 *	pseudocolor mode we have a limited color palette. To deal with this
 *	we can program what color is displayed for a particular pixel value.
 *	DirectColor is similar in that we can program each color field. If
 *	we have a static colormap we don't need to implement this function. 
 * 
 *	Returns negative errno on error, or zero on success.
 */
static int nusmartfb_setcolreg(unsigned regno, unsigned red, unsigned green,
		unsigned blue, unsigned transp,
		struct fb_info *info)
{
	/*	ctrace;*/
	if (regno >= 256)  /* no. of hw registers */
		return -EINVAL;
	/*
	 * Program hardware... do anything you want with transp
	 */

	/* grayscale works only partially under directcolor */
	if (info->var.grayscale) {
		/* grayscale = 0.30*R + 0.59*G + 0.11*B */
		red = green = blue = (red * 77 + green * 151 + blue * 28) >> 8;
	}

	/* This is the point were you need to fill up the contents of
	 * info->pseudo_palette. This structure is used _only_ by fbcon, thus
	 * it only contains 16 entries to match the number of colors supported
	 * by the console. The pseudo_palette is used only if the visual is
	 * in directcolor or truecolor mode.  With other visuals, the
	 * pseudo_palette is not used. (This might change in the future.)
	 *
	 * The contents of the pseudo_palette is in raw pixel format.  Ie, each
	 * entry can be written directly to the framebuffer without any conversion.
	 * The pseudo_palette is (void *).  However, if using the generic
	 * drawing functions (cfb_imageblit, cfb_fillrect), the pseudo_palette
	 * must be casted to (u32 *) _regardless_ of the bits per pixel. If the
	 * driver is using its own drawing functions, then it can use whatever
	 * size it wants.
	 */
	if (info->fix.visual == FB_VISUAL_TRUECOLOR ||
			info->fix.visual == FB_VISUAL_DIRECTCOLOR) {

		u32 v;
		if (regno >= 16)
			return -EINVAL;

		v = convert_bitfield(red, &info->var.red) |
			convert_bitfield(green, &info->var.green) |
			convert_bitfield(blue, &info->var.blue) |
			convert_bitfield(transp, &info->var.transp);

		((u32*)(info->pseudo_palette))[regno] = v;
	}

	/* ... */
	return 0;
}

/**
 *      nusmartfb_pan_display - NOT a required function. Pans the display.
 *      @var: frame buffer variable screen structure
 *      @info: frame buffer structure that represents a single frame buffer
 *
 *	Pan (or wrap, depending on the `vmode' field) the display using the
 *  	`xoffset' and `yoffset' fields of the `var' structure.
 *  	If the values don't fit, return -EINVAL.
 *
 *      Returns negative errno on error, or zero on success.
 */
static int nusmartfb_pan_display(struct fb_var_screeninfo *var,
		struct fb_info *info)
{
	/*
	 * If your hardware does not support panning, _do_ _not_ implement this
	 * function. Creating a dummy function will just confuse user apps.
	 */
	struct nusmart_par * par = info->par;
	int offset = var->yoffset * info->fix.line_length;
	unsigned int * base = info->screen_base + offset;
	unsigned int * end; 
	unsigned int * dist;
	unsigned int ctrl;

	if(0 == par->node)
	{
		end = base + DISP_BASE0/2;
		dist = base +  DISP_BASE0;

	}else{
		end = base +  DISP_BASE1/2;
		dist = base + DISP_BASE1;
	}

	info->var.yoffset = var->yoffset;
	ctrace;
	/* Add for LCDC test by weixing */
#ifdef  WAIT_FOR_VSYNC
	/*enable vsync interrupt*/
	ctrl = readl(par->mmio);
	ctrl |= NUSMART_LCDC_INTM_V;	
	writel(ctrl, par->mmio);
#endif	
	switch (par->fbconv) {
		case 1:
			while(base != end) {
				*dist = (*base & 0x001f001f)|((*base & 0xffc0ffc0)>>1);
				base++,dist++;
			}
			/*break;*/
		case 2:
			//			info->var = *var;
			nusmart_clcd_set_hwfb(info, INDIRECT_SHOW);
			break;
		case 0:
			//			info->var = *var;
			/* Modify for LCDC test by weixing */
			//			nusmart_clcd_set_hwfb(info, DIRECT_SHOW);
			//printk(KERN_EMERG "%s var->yoffset %d\n", __func__, var->yoffset);
			if(0 == var->yoffset)
				ctrl &= ~NUSMART_LCDC_PTRF;
			else
				ctrl |= NUSMART_LCDC_PTRF;
			writel(ctrl, par->mmio);
			/*break;*/
		default:
			break;
	}

#ifdef  WAIT_FOR_VSYNC
	/* Modify for LCDC test by weixing */
	init_completion(&par->vsync);
	if(0 == wait_for_completion_timeout(&par->vsync,HZ)){
		printk("%s LINE %d: not get vsync interrupt!\n", __func__, __LINE__);
	}
	/*enable vsync interrupt*/
	//	ctrl = readl(par->mmio);
	//	ctrl |= NUSMART_LCDC_INTM_V;	
	//	writel(ctrl, par->mmio);
#endif	

	/* ... */
	return 0;
}

/**
 *      nusmartfb_blank - NOT a required function. Blanks the display.
 *      @blank_mode: the blank mode we want. 
 *      @info: frame buffer structure that represents a single frame buffer
 *
 *      Blank the screen if blank_mode != FB_BLANK_UNBLANK, else unblank.
 *      Return 0 if blanking succeeded, != 0 if un-/blanking failed due to
 *      e.g. a video mode which doesn't support it.
 *
 *      Implements VESA suspend and powerdown modes on hardware that supports
 *      disabling hsync/vsync:
 *
 *      FB_BLANK_NORMAL = display is blanked, syncs are on.
 *      FB_BLANK_HSYNC_SUSPEND = hsync off
 *      FB_BLANK_VSYNC_SUSPEND = vsync off
 *      FB_BLANK_POWERDOWN =  hsync and vsync off
 *
 *      If implementing this function, at least support FB_BLANK_UNBLANK.
 *      Return !0 for any modes that are unimplemented.
 *
 */
static int nusmartfb_blank(int blank_mode, struct fb_info *info)
{
	/* ... */
	struct nusmart_par *par = info->par;
	printk("blank_mode %d",blank_mode);
	switch(blank_mode) {
		case FB_BLANK_NORMAL:
		case FB_BLANK_POWERDOWN:
			nusmart_clcd_disable(info);
			break;
		case FB_BLANK_UNBLANK:
			nusmart_clcd_enable(info, par->ctrl);
			break;
		default:
			break;
	}
	return 0;
}

static int nusmartfb_mmap(struct fb_info *info,
		struct vm_area_struct *vma)
{
	unsigned long len, off =vma->vm_pgoff << PAGE_SHIFT;
	int ret = -EINVAL;

	ctrace;
	len = info->fix.smem_len;

	if(off <= len && vma->vm_end - vma->vm_start <= len - off) {
#ifdef	FB_RESERV
		unsigned int video_mem_base = info->fix.smem_start;
		printk(KERN_EMERG "%s video mem base 0x%x\n", __func__, video_mem_base);
		vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);
		ret = remap_pfn_range(vma, vma->vm_start, video_mem_base >> PAGE_SHIFT,
				vma->vm_end - vma->vm_start, vma->vm_page_prot);	
#else
		ret = dma_mmap_writecombine(info->dev, vma,
				info->screen_base,
				info->fix.smem_start,
				info->fix.smem_len);
#endif	/*FB_RESERV*/
	}
	return ret;
}

#ifdef NUSMART_PW_CTRL
#include <mach/hardware.h>
#define PRCM_CPUB_CTRL 	0x051c0048
#define PRCM_PWCTRL	0x051b0014
#define PRCM_CLK1_CTRL	0x051b0020
#define PRCM_CLK2_CTRL	0x051b0024
#define SGI_REG		0x05041f00
#define SCU_PM_REG	0x05040008
#define AUDIO_MODE	1
#define POWEROFF_MODE	2
#define WFI_MODE	3
#define CLK_OFF		0
#define CLK_ON		1
#define CLK_CG		2
static int set_pw(struct clk_status * stat)
{
	char __iomem * pw = __io_address(PRCM_PWCTRL);	
	char __iomem * clkctrl1 = __io_address(PRCM_CLK1_CTRL);	
	char __iomem * clkctrl2 = __io_address(PRCM_CLK2_CTRL);	
	char __iomem * cpub = __io_address(PRCM_CPUB_CTRL);	
	char __iomem * sgi = __io_address(SGI_REG);	
	char __iomem * cpupm = __io_address(SCU_PM_REG);	
	unsigned int reg;
	reg = readl(pw);
	if(stat->clk_neon0 == CLK_ON) {
		printk("neon0 on\n");
		reg |=(1<<10);
	} else {
		printk("neon0 off\n");
		reg &=~(1<<10);
	}
	writel(reg,pw);
	reg = readl(clkctrl1);
	if(stat->clk_mali == CLK_ON) {
		printk("mali on\n");
		reg |=(1<<4);
	} else {
		printk("mali off\n");
		reg &=~(1<<4);
	}
	writel(reg,clkctrl1);
	reg = readl(clkctrl2); 
	if(stat->clk_lcd == CLK_CG) {
		printk("lcd off\n");
		reg &= ~(0x1<<27);
	} else {
		printk("lcd on\n");
		reg |= (0x1<<27);
	}	
	if(stat->clk_on2 == CLK_ON) {
		printk("on2 on\n");
		reg |= (0x1<<28);
	} else {
		printk("on2 off\n");
		reg &= ~(0x1<<28);
	}
	writel(reg,clkctrl2);
	reg = readl(cpub);
	if(stat->clk_cpu1 == CLK_ON) {
		if(reg != AUDIO_MODE) {
			if(reg == POWEROFF_MODE)
			{
				printk("snd on\n");
				writel(0x00000000,cpupm);	
				writel(0x00020000,sgi);	
				udelay(1000);
				writel(0x00020000,sgi);	
				reg = 0; 
			} else if(reg == WFI_MODE) {
				printk("snd exit wfi\n");
				writel(0x00020000,sgi);	
				reg = 0; 
			}
		}
	} else {
		printk("snd off\n");
		reg = POWEROFF_MODE; 
	}
	writel(reg,cpub);
	return 0;
}
#endif
/*
 */
static int nusmartfb_ioctl(struct fb_info *info, unsigned int cmd,
		unsigned long arg)
{
	int ret = 0;
	int flag;
	struct nusmart_par * par = info->par;
#ifdef CONFIG_MACH_NS115_STICK
	unsigned long ustart = info->fix.smem_start;
#endif
	void __user *argp = (void __user *)arg;
	struct fb_var_screeninfo var;
	struct fb_videomode  mode;
	unsigned int ctrl;
	unsigned long fb_phy_addr = 0;
	int vmem_base;

#ifdef NUSMART_PW_CTRL
	struct clk_status stat;
#endif
	ctrace;

	switch(cmd) {
#ifdef  WAIT_FOR_VSYNC
		case FBIO_WAITFORVSYNC:
			return 0;
			break;
#endif
		case FBIOPUT_FBCONVFLAG:
			if (copy_from_user(&flag, argp, sizeof(flag)))
				return -EFAULT;
			if (flag >= 0 && flag <= 2) {
				par->fbconv = flag;	
				dbg("FBCONV flag:%d", flag);
			}
			else {
			}
			break;
		case FBIOPUT_FBTUNE:
			if (copy_from_user(&var, argp, sizeof(struct fb_var_screeninfo)))
				return -EFAULT;
			if(set_var(info, &var) < 0)
				return -EFAULT;
			break;
#ifdef NUSMART_PW_CTRL
		case FBIOPUT_PWSTAT:
			if (copy_from_user(&stat, argp, sizeof(struct clk_status))) {
				printk("copy data failed.\n");
				return -EFAULT;
			}
			if(set_pw(&stat) < 0)
				return -EFAULT;
			break;
#endif
		case GET_FB_BUFFER_ADDR:
			ctrl = readl(par->mmio);

			if(ctrl & NUSMART_LCDC_PTRF){
				fb_phy_addr = readl(par->mmio + NUSMART_LCDC_SECPTR);
			}else
				fb_phy_addr = readl(par->mmio + NUSMART_LCDC_PRIPTR);

			if (copy_to_user(argp, &fb_phy_addr, sizeof(unsigned long))) {
				printk("copy data failed.\n");
				return -EFAULT;
			}
			break;

		case SET_RESOLUTION_RATIO:
			if (copy_from_user(&mode, argp, sizeof(struct fb_videomode)))
				return -EFAULT;
			
			ret = set_resolution_ratio(info, &mode);
			if(!ret)
				set_resolution = 1;

			break;

#ifdef CONFIG_MACH_NS115_STICK
		case CHNAGE_FRAMEBUFFER_ADDR:
			if (copy_from_user(&flag, argp, sizeof(int))) {
				printk("copy data failed.\n");
				return -EFAULT;
			}

			if(0 == flag)
			{
				printk("pointer to lcd1 addr");
				writel(ustart, par->mmio + NUSMART_LCDC_PRIPTR);
				ustart += info->var.yres * info->fix.line_length;
				writel(ustart, par->mmio + NUSMART_LCDC_SECPTR);
	
				info->fix.smem_start = vmem_base = VIDEO_MEM_BASE + (1 * VIDEO_MEM_SIZE/2);
				info->screen_base = ioremap(vmem_base, VIDEO_MEM_SIZE/2);
			}else{
				printk("pointer to lcd0 addr");
				writel(ustart, par->mmio + NUSMART_LCDC_PRIPTR);
				ustart += info->var.yres * info->fix.line_length;
				writel(ustart, par->mmio + NUSMART_LCDC_SECPTR);
				info->fix.smem_start = vmem_base = VIDEO_MEM_BASE;
				info->screen_base = ioremap(vmem_base, VIDEO_MEM_SIZE/2);
			}
		
			/*read status*/
			ctrl = readl(par->mmio); 

			/*set first*/	
			ctrl&=~NUSMART_LCDC_PTRF;

			break;
#endif

		default:
			ret = -ENOTTY;
	}
	return ret;
}


static ssize_t nusmartfb_read(struct fb_info *info, char __user *buf,
		size_t count, loff_t *ppos)
{
	int i = 0;
#ifdef CONFIG_FB_DDC
	struct i2c_adapter *adapter = NULL;	
	int 	ddc_adapter = 2;
	unsigned char * edid = NULL;
#endif
	unsigned long pos;
	pos = *ppos;

#ifdef CONFIG_FB_DDC
	printk(KERN_INFO "begin get edid lcd0\n");
	adapter = i2c_get_adapter(ddc_adapter);
	if(adapter) {
		edid=fb_ddc_read_i2c(adapter);
		if(edid) {
			if(copy_to_user(buf, edid, EDID_LENGTH))
			{
				printk("copy data to user failed.\n");
			}
			printk("EDID for lcd 0:\n");
			for(i = 0; i < EDID_LENGTH; i++)
			{
				if ((i % 16) == 0) {
					printk("\n%02X,", edid[i]);
				} else {
					printk(" %02X,", edid[i]);
				}
			}    
			printk("\n");
		}	
		i2c_put_adapter(adapter);
	}else{
		printk(KERN_EMERG "begin get lcd0 adapter failed\n");
	}
#endif
	*ppos += EDID_LENGTH;
	if(hdmi_edid) {
		if(copy_to_user(buf+EDID_LENGTH, hdmi_edid, 256))
		{
			printk("copy data to user failed.\n");
		}
		printk("\nEDID for lcd 1:\n");
		for(i = 0; i < 256; i++)
		{
			if ((i % 16) == 0) {
				printk("\n%02X,", hdmi_edid[i]);
			} else {
				printk(" %02X,", hdmi_edid[i]);
			}
		}    
		printk("\n");
	}	
	*ppos += 256;

	return *ppos;
}

/*
 *  Frame buffer operations
 */

static struct fb_ops nusmartfb_ops = {
	.owner		= THIS_MODULE,
	/*.fb_open	= nusmartfb_open,*/
	/*.fb_read	= nusmartfb_read,*/
	/*.fb_write	= nusmartfb_write,*/
	/*.fb_release	= nusmartfb_release,*/
	.fb_check_var	= nusmartfb_check_var,
	.fb_set_par	= nusmartfb_set_par,
	.fb_setcolreg	= nusmartfb_setcolreg,
	.fb_blank	= nusmartfb_blank,
	.fb_pan_display	= nusmartfb_pan_display,
	.fb_fillrect	= cfb_fillrect, 	/* Needed !!! */
	.fb_copyarea	= cfb_copyarea,	/* Needed !!! */
	.fb_imageblit	= cfb_imageblit,	/* Needed !!! */
	/*.fb_cursor	= nusmartfb_cursor,*/		/* Optional !!! */
	/*.fb_rotate	= nusmartfb_rotate,*/
	/*.fb_sync	= nusmartfb_sync,*/
	.fb_ioctl	= nusmartfb_ioctl,
	.fb_mmap	= nusmartfb_mmap,
};

/* ------------------------------------------------------------------------- */

/*
 *  Initialization
 */
static int video_mem_alloc(struct fb_info *info, unsigned int id)
{
	int vmem_base;
	ctrace;

#ifdef CONFIG_MACH_NS115_STICK
	info->fix.smem_len = VIDEO_MEM_SIZE;
#else
	info->fix.smem_len = VIDEO_MEM_SIZE/2;
#endif

#ifdef 	FB_RESERV 
#ifdef CONFIG_MACH_NS115_STICK
	info->fix.smem_start = vmem_base = VIDEO_MEM_BASE;
	info->screen_base = ioremap(vmem_base, VIDEO_MEM_SIZE);
#else
	info->fix.smem_start = vmem_base = VIDEO_MEM_BASE + (id * VIDEO_MEM_SIZE/2);
	info->screen_base = ioremap(vmem_base, VIDEO_MEM_SIZE/2);
#endif


	printk(KERN_INFO "disp%d: VIDEO_MEM_BASE 0x%08x(virtual 0x%08x), VIDEO_MEM_SIZE 0x%08x(%dMB)\n", 
			id, vmem_base, info->screen_base, VIDEO_MEM_SIZE/2, VIDEO_MEM_SIZE/2/1024/1024);
#else
	info->screen_base = dma_alloc_writecombine(info->device, info->fix.smem_len,
			(dma_addr_t*)&info->fix.smem_start, GFP_KERNEL);
#endif	/*FB_RESERV*/
	if(!info->screen_base) {
		return -ENOMEM;
	}

#ifndef CONFIG_MACH_NS115_STICK
	if(1 == id)
		memset(info->screen_base, 0x0, info->fix.smem_len);
#endif

	return 0;
}

static inline void video_mem_free(struct fb_info *info)
{
	ctrace;
	if(info->screen_base) {
#ifdef 	FB_RESERV
		iounmap(info->screen_base);
#else
		dma_free_writecombine(info->device, info->fix.smem_len,
				info->screen_base, info->fix.smem_start);
#endif	/*FB_RESERV*/
		info->screen_base = NULL;	
	}
}

#ifdef  WAIT_FOR_VSYNC
static irqreturn_t nusmart_lcdc_vert_irq(int irq, void *dev_id)
{
	struct nusmart_par *par = dev_id;
	unsigned int ctrl;

	/*read status*/
	ctrl = readl(par->mmio); 

	/*disable interrupt*/
	ctrl &= ~NUSMART_LCDC_INTM_V;
	writel(ctrl, par->mmio);

	if(ctrl & NUSMART_LCDC_INTF_V) {
		/*handle lcdc vsync*/
		ctrl &= ~NUSMART_LCDC_INTF_V;
		/*complete vsync wait*/
		complete(&par->vsync);	
		ctrl |= NUSMART_LCDC_INTM_V;
		writel(ctrl, par->mmio);
	} else {
		printk("%s: nusmart fb:interrupt detected with vsync not set.\n", __func__);
		/*not complete,reenable interrupt*/
		ctrl |= NUSMART_LCDC_INTM_V;
		writel(ctrl, par->mmio);
	}


	return IRQ_HANDLED;
}
#endif

static irqreturn_t nusmart_lcdc_irq(int irq, void *dev_id)
{
	struct nusmart_par *par = dev_id;
	unsigned int ctrl;

	/*read status*/
	ctrl = readl(par->mmio); 

	/*disable interrupt*/
	ctrl &= ~NUSMART_LCDC_INTM_ERR;
	//ctrl &= ~NUSMART_LCDC_ENABLE;
	ctrl |= NUSMART_LCDC_FRAME_RESET;
	writel(ctrl, par->mmio);

	if(ctrl & (NUSMART_LCDC_UNDERFLOW | NUSMART_LCDC_OVERFLOW)) {
		/*handle lcdc error*/
		if(ctrl & NUSMART_LCDC_UNDERFLOW)
			printk(KERN_INFO "nusmart fb: irq %d lcdc fifo underflow detected!\n", irq);

		if(ctrl & NUSMART_LCDC_OVERFLOW)
			printk(KERN_INFO "nusmart fb:irq %d lcdc fifo overflow detected!\n", irq);

		ctrl &= ~(NUSMART_LCDC_UNDERFLOW | NUSMART_LCDC_OVERFLOW);

		/*set first*/	
		//ctrl&=~NUSMART_LCDC_PTRF;
#if 1
		if(ctrl & NUSMART_LCDC_PTRF) {/*secondary*/
			/*set first*/	
			ctrl&=~NUSMART_LCDC_PTRF;
		} else {/*first*/
			/*set second*/	
			ctrl|=NUSMART_LCDC_PTRF;
		}
#endif
	}else{
		printk("%s: nusmart fb:interrupt detected with vsync not set.\n", __func__);
	}

	/*clear flag & enable interrupt*/
	ctrl |= NUSMART_LCDC_INTM_ERR;
	//ctrl |= NUSMART_LCDC_ENABLE;
	writel(ctrl, par->mmio);

	return IRQ_HANDLED;
}

#ifdef CONFIG_EARLYSUSPEND
#define es_to_fbinfo(x) container_of(x, struct ns2815fb_early_suspend, es) 
struct ns2815fb_early_suspend
{
	struct fb_info * fb;
	struct early_suspend es;
};

static int nusmartfb_save(struct fb_info * info);
static int nusmartfb_restore(struct fb_info * info);

static void nusmartfb_early_resume(struct early_suspend * h)
{
	struct ns2815fb_early_suspend * nes = es_to_fbinfo(h);
	#ifdef CONFIG_MOBILE_PHONE_LCD_NS115
	gpio_direction_output(LCD_ON,1);//LCD_ON
	mdelay(50);
	lcd_init_hx8369();
	#endif
	nusmartfb_restore(nes->fb);
}

static void nusmartfb_early_suspend(struct early_suspend * h)
{
	struct ns2815fb_early_suspend * nes = es_to_fbinfo(h);
	nusmartfb_save(nes->fb);
}

static struct ns2815fb_early_suspend early_suspend[] = {
	{
		.es = {
			.level = EARLY_SUSPEND_LEVEL_DISABLE_FB,
			.suspend = nusmartfb_early_suspend,
			.resume = nusmartfb_early_resume,
		}
	},{
		.es = {
			.level = EARLY_SUSPEND_LEVEL_DISABLE_FB,
			.suspend = nusmartfb_early_suspend,
			.resume = nusmartfb_early_resume,
		}
	},
};

#endif

static struct fb_info *fb1_info = NULL ;
static struct fb_videomode fb1_mode;
static int __devinit nusmartfb_probe(struct platform_device *pdev)
{
	struct fb_info *info;
	struct nusmart_par *par;
	struct device *device = &pdev->dev; 
	int cmap_len=256, retval = 0;	
	struct resource *res = NULL;
	struct fb_monspecs *specs = NULL;
	const struct fb_videomode * m = NULL;
	struct fb_videomode mode;
	struct fb_var_screeninfo stdvar;
	struct lcdc_platform_data * platform_data = NULL;
	int id = pdev->id;
	char *mode_option = NULL;

	unsigned int bits_per_pixel=0, xres=0, yres=0;
	ctrace;

#ifdef CONFIG_FB_DDC
	struct i2c_adapter *adapter = NULL;	
	int 	ddc_adapter = 0, i = 0;
	unsigned char * edid = NULL;
#endif
	printk(KERN_EMERG "lcd %d probe begine!!!!!!!!!!\n", id);

#ifdef CONFIG_MOBILE_PHONE_LCD_NS115
	gpio_request(LCD_ON, "LCD-ON");
	gpio_direction_output(LCD_ON,1);//LCD_ON
	mdelay(50);
	lcd_init_hx8369();
#endif

	/*
	 * Dynamically allocate info and par
	 */
	info = framebuffer_alloc(sizeof(struct nusmart_par), device);

	if (!info) {
		/* goto error path */
		goto out;		
	}

	par = info->par;
	par->fb = info;
	par->fbconv = 0;
	par->mvflag = 0;
	par->node = id;
	specs = &info->monspecs;

	if(id == 0) {
		mode_option = mode_option_disp0;
		par->clk = clk_get(NULL, "ns115_lcdc0");
		par->rgb_clk = clk_get(NULL, "ns115_rgbo");

#ifdef CONFIG_FB_DDC
		/*
		 * get ddc adapter info from platform data
		 */	
		platform_data = pdev->dev.platform_data;
		if(platform_data == NULL){
			err("get platform data failed.");
		}else{

			ddc_adapter = platform_data->ddc_adapter;

			printk(KERN_INFO "begin get edid lcd0\n");
			adapter = i2c_get_adapter(ddc_adapter);
			if(adapter) {
				edid=fb_ddc_read_i2c(adapter);
				if(edid) {
					printk("EDID for lcd 0:\n");
					for(i = 0; i < EDID_LENGTH; i++)
					{
						if ((i % 16) == 0) {
							printk("\n");
							printk("%02X | %02X", i, edid[i]);
						} else {
							printk(" %02X", edid[i]);
						}
					}    
				}	
				i2c_put_adapter(adapter);
			}else{
				printk(KERN_EMERG "begin get lcd0 adapter failed\n");
			}
		}
#endif
		info->pseudo_palette = pseudo_palette; /* The pseudopalette is an
							* 16-member array
							*/
		info->fix = nusmartfb_fix; /* this will be the only time nusmartfb_fix will be
					    * used, so mark it as __devinitdata
					    */

		//for test board screen, this is the screen phyical size, in mm
		info->var.width = 218;
		info->var.height = 131;
	} else {
		mode_option = mode_option_disp1;
		par->clk = clk_get(NULL, "ns115_lcdc1");
		par->rgb_clk = clk_get(NULL, "ns115_hdmi_pxl");
		fb1_info = info;
		info->pseudo_palette = pseudo_palette1; 
		info->fix = nusmartfb_fix1;
		info->mode = &fb1_mode;
	}

	//enable clk as default
	clk_enable(par->rgb_clk);

	if(IS_ERR(par->clk)) {
		printk("get nusmartfb clock failed!\n");
		return PTR_ERR(par->clk);
	}

	/* 
	 * Here we set the screen_base to the virtual memory address
	 * for the framebuffer. Usually we obtain the resource address
	 * from the bus layer and then translate it to virtual memory
	 * space via ioremap. Consult ioport.h. 
	 */
	info->fbops = &nusmartfb_ops;
	retval = video_mem_alloc(info, id);

	if(retval < 0) {
		printk(KERN_EMERG "alloc video mem failed!\n");	
		goto free_fbinfo;
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if(!res) {
		printk(KERN_EMERG "get_resource failed!(mem)\n");
		retval = -ENXIO;
		goto free_video_mem;
	}
	info->fix.mmio_start = res->start;	
	info->fix.mmio_len = res->end - res->start +1;

	par->mmio = ioremap_nocache(info->fix.mmio_start,info->fix.mmio_len);
	if(!par->mmio) {
		printk(KERN_EMERG "ioremap failed!\n");
		retval = -ENOMEM;
		goto free_video_mem;	
	}

	res = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
	if(!res) {
		printk(KERN_EMERG "get_resource failed!(irq)\n");
		retval = -ENXIO;
		goto unmap_mmio;
	}

	par->irq = res->start;	
	if(id == 0)
		retval = request_irq(par->irq, nusmart_lcdc_irq, IRQF_SHARED, DISP0_NAME, par);
	else
		retval = request_irq(par->irq, nusmart_lcdc_irq, IRQF_SHARED, DISP1_NAME, par);
	if(retval < 0) {
		printk(KERN_EMERG "request_irq failed!\n");
		goto unmap_mmio;
	}

#ifdef  WAIT_FOR_VSYNC
	par->irq_vert = res->start+1;	
	retval = request_irq(par->irq_vert, nusmart_lcdc_vert_irq, IRQF_DISABLED|IRQF_SHARED, "nusmartfb_vert", par);
	if(retval < 0) {
		printk(KERN_EMERG "request nusmartfb vert irq failed!\n");
		goto unmap_mmio;
	}
	init_completion(&par->vsync);
#endif

	/*
	 * Set up flags to indicate what sort of acceleration your
	 * driver can provide (pan/wrap/copyarea/etc.) and whether it
	 * is a module -- see FBINFO_* in include/linux/fb.h
	 *
	 * If your hardware can support any of the hardware accelerated functions
	 * fbcon performance will improve if info->flags is set properly.
	 *
	 * FBINFO_HWACCEL_COPYAREA - hardware moves
	 * FBINFO_HWACCEL_FILLRECT - hardware fills
	 * FBINFO_HWACCEL_IMAGEBLIT - hardware mono->color expansion
	 * FBINFO_HWACCEL_YPAN - hardware can pan display in y-axis
	 * FBINFO_HWACCEL_YWRAP - hardware can wrap display in y-axis
	 * FBINFO_HWACCEL_DISABLED - supports hardware accels, but disabled
	 * FBINFO_READS_FAST - if set, prefer moves over mono->color expansion
	 * FBINFO_MISC_TILEBLITTING - hardware can do tile blits
	 *
	 * NOTE: These are for fbcon use only.
	 */
	info->flags = FBINFO_DEFAULT;

	/*
	 * This should give a reasonable default video mode. The following is
	 * done when we can set a video mode. 
	 */
	if (!strcmp(mode_option, DDC_ARGS)) {
		mode_option = "1024x600-16@60";
		info->var.bits_per_pixel = 16;
	}
	else if (!strcmp(mode_option, DDC_ARGS_16)) {
		mode_option = "1024x600-16@60";
		info->var.bits_per_pixel = 16;
	}
	else if (!strcmp(mode_option, DDC_ARGS_32)) {
		mode_option = "1024x600-32@60";
		info->var.bits_per_pixel = 32;
	}else {
		sscanf(mode_option, "%ux%u-%u", &xres, &yres, &bits_per_pixel);

		printk(KERN_INFO "parse mode option %d %d %d\n", xres, yres, bits_per_pixel);
		if(0 != bits_per_pixel){
			info->var.bits_per_pixel = bits_per_pixel;
		}else{
			//set default bits_per_pixel as 32 
			info->var.bits_per_pixel = 32;

			printk(KERN_INFO "nusmartfb%d: set default bits_per_pixel as %d\n", 
				id, info->var.bits_per_pixel );
		}
	}

	if(0 == id){
		fb0_bits_per_pixel = info->var.bits_per_pixel;
		retval = fb_find_mode(&stdvar, info, mode_option, NULL, 0,NULL, info->var.bits_per_pixel);
		fb_var_to_videomode(&mode, &stdvar);
		fb_videomode_to_var(&info->var, &mode);
	}else{
		printk(KERN_ERR "fb1 not fetch edid, set default mode as %s\n", mode_option_disp1);
		//mode_option = mode_option_disp1; 
		retval = fb_find_mode(&stdvar, info, mode_option, NULL, 0,NULL, info->var.bits_per_pixel);
		fb_var_to_videomode(&mode, &stdvar);
		fb_videomode_to_var(&info->var, &mode);
		//sprintf(mode_option, "%dx%d-%d", info->var.xres, info->var.yres, info->var.bits_per_pixel);
		printk(KERN_INFO "%d %s\n", __LINE__, mode_option_disp1);
	}
	if (!retval || retval == 4) {
		printk(KERN_EMERG "find mode failed!\n");
		retval = -EINVAL;	
		goto free_irq;
	}		

	/*add mode list*/
	//fb_videomode_to_modelist(vesa_modes,VESA_MODEDB_SIZE,&info->modelist);

	/* DISP1 can NOT read EDID through I2C, but DISP0 can */

	printk(KERN_EMERG "mode option accept:%dx%d\n", info->var.xres,info->var.yres);
	//printk("mode clk:%fMHz",(1000000/info->var.pixclock));
	printk(KERN_INFO "mode left:%d right:%d hsync:%d\n"
			"upper:%d lower:%d vsync:%d\n"
			"sync:%d\n",info->var.left_margin,info->var.right_margin,
			info->var.hsync_len, info->var.upper_margin,info->var.lower_margin,
			info->var.vsync_len, info->var.sync);
	/* This has to been done !!! */	
	fb_alloc_cmap(&info->cmap, cmap_len, 0);

	nusmartfb_check_var(&info->var, info);
	/*
	 * Does a call to fb_set_par() before register_framebuffer needed?  This
	 * will depend on you and the hardware.  If you are sure that your driver
	 * is the only device in the system, a call to fb_set_par() is safe.
	 *
	 * Hardware in x86 systems has a VGA core.  Calling set_par() at this
	 * point will corrupt the VGA console, so it might be safer to skip a
	 * call to set_par here and just allow fbcon to do it for you.
	 */
	nusmartfb_set_par(info);

	if (register_framebuffer(info) < 0) {
		dbg("register framebuffer failed!");
		retval =  -EINVAL;
		goto free_cmap;
	}
	printk(KERN_INFO "fb%d: %s frame buffer device\n", info->node,
			info->fix.id);

	platform_set_drvdata(pdev, info); /* or platform_set_drvdata(pdev, info) */
#ifdef CONFIG_EARLYSUSPEND
	early_suspend[id].fb = info;
	register_early_suspend(&early_suspend[id].es);
#endif
	
#ifndef CONFIG_MACH_NS115_STICK
	if(1 == id){
		nusmart_clcd_disable(info);
	//	clk_disable(par->clk);
	}
#endif
	
	return 0;
free_cmap:
	fb_dealloc_cmap(&info->cmap);
free_irq:
	free_irq(par->irq, par);
unmap_mmio:
	iounmap(par->mmio);
free_video_mem:
	video_mem_free(info);
free_fbinfo:
	framebuffer_release(info);
out:
	return retval;
}

/*
 *  Cleanup
 */
/* static void __devexit nusmartfb_remove(struct platform_device *pdev) */
static void __devexit nusmartfb_remove(struct platform_device *pdev)
{
	struct fb_info *info = platform_get_drvdata(pdev);
	struct nusmart_par *par = NULL;
	//struct fb_info *info = NULL;

	ctrace;
	if (info) {
		nusmart_clcd_disable(info);
		unregister_framebuffer(info);

		par = info->par;
		iounmap(par->mmio);
		free_irq(par->irq, par);

		if(info->cmap.len>0)
			fb_dealloc_cmap(&info->cmap);
		/* free clk*/
		clk_put(par->clk);
		par->clk = ERR_PTR(-ENOENT);
		if(info->monspecs.modedb)
			fb_destroy_modedb(info->monspecs.modedb);
		video_mem_free(info);
		framebuffer_release(info);
	}
	#ifdef CONFIG_MOBILE_PHONE_LCD_NS115
	gpio_free(LCD_ON);
	#endif
}

#include <linux/platform_device.h>
/* for platform devices */

static ssize_t show_all_vendor(struct device *dev, struct device_attribute *attr,
                               char *buf)
{
       struct fb_info *info = dev_get_drvdata(dev);
       struct fb_monspecs *specs = &info->monspecs;
               
	snprintf(buf, PAGE_SIZE, "%d.%d:%s:%d:%d:%s:%s:%d:%d", (int)specs->version, (int)specs->revision, specs->manufacturer, specs->model, specs->serial, specs->monitor, specs->serial_no, specs->max_x, specs->max_y);
}

static DEVICE_ATTR(vendor, 0444, show_all_vendor, NULL);
static DEVICE_ATTR(edidinfo, 0444, show_modes, NULL);

#ifdef CONFIG_PM
static int nusmartfb_save(struct fb_info * info)
{
	struct nusmart_par *par = info->par;
	int idx = REG_NUM - 1;
	for(;idx >= 0;idx --)
		par->regsave[idx] = ((unsigned int *)(par->mmio))[idx];
	ctrace;
	nusmart_clcd_disable(info);

	if ( 1 == par->node)
	{
		device_remove_file(info->dev, &dev_attr_vendor);
		device_remove_file(info->dev, &dev_attr_edidinfo);
	}

	clk_disable(par->rgb_clk);
		
	/* suspend here */
	return 0;
}

/**
 *	nusmartfb_suspend - Optional but recommended function. Suspend the device.
 *	@dev: platform device
 *	@msg: the suspend event code.
 *
 *      See Documentation/power/devices.txt for more information
 */
static int nusmartfb_suspend(struct platform_device *dev, pm_message_t msg)
{
	struct fb_info *info = platform_get_drvdata(dev);
	nusmartfb_save(info);
	/* suspend here */
	return 0;
}

static int nusmartfb_restore(struct fb_info * info)
{
	struct nusmart_par *par = info->par;
	int idx = REG_NUM - 1;
	for(;idx >= 0;idx --)
		((unsigned int *)(par->mmio))[idx] = par->regsave[idx];
	ctrace;

	clk_enable(par->rgb_clk);

 	nusmart_clcd_enable(info, par->ctrl);

	if((1 == par->node) && (0 == hdmi_state))
		nusmart_clcd_disable(info);

	/* resume here */
	return 0;
}

/**
 *	nusmartfb_resume - Optional but recommended function. Resume the device.
 *	@dev: platform device
 *
 *      See Documentation/power/devices.txt for more information
 */
static int nusmartfb_resume(struct platform_device *dev)
{
	struct fb_info *info = platform_get_drvdata(dev);
	#ifdef CONFIG_MOBILE_PHONE_LCD_NS115
	gpio_direction_output(LCD_ON,1);//LCD_ON
	mdelay(50);
	lcd_init_hx8369();
	#endif
	nusmartfb_restore(info);
	/* resume here */
	return 0;
}
#else
#define nusmartfb_suspend NULL
#define nusmartfb_resume NULL
#endif /* CONFIG_PM */

static struct platform_driver nusmartfb_driver[] = {
	{
		.probe = nusmartfb_probe,
		.remove = nusmartfb_remove,
#ifndef CONFIG_EARLYSUSPEND
		.suspend = nusmartfb_suspend, /* optional but recommended */
		.resume = nusmartfb_resume,   /* optional but recommended */
#endif
		.driver = {
			.name = DISP0_NAME,
			.owner = THIS_MODULE,
		},
	},
#ifdef	DUAL_LCD
	{
		.probe = nusmartfb_probe,
		.remove = nusmartfb_remove,
#ifndef CONFIG_EARLYSUSPEND
		.suspend = nusmartfb_suspend, /* optional but recommended */
		.resume = nusmartfb_resume,   /* optional but recommended */
#endif
		.driver = {
			.name = DISP1_NAME,
			.owner = THIS_MODULE,
		},
	},
#endif
};

void fb_select_modes(const struct fb_videomode *mode,
			 struct list_head *head)
{
	struct list_head *pos, *n;
	struct fb_modelist *modelist;
	struct fb_videomode *m;

//	printk(KERN_INFO "%s : %d\n", __func__, __LINE__);
	dbg();

	list_for_each_safe(pos, n, head) {
		modelist = list_entry(pos, struct fb_modelist, list);
		m = &modelist->mode;
		if (	(((m->xres * m->yres) > (mode->xres * mode->yres)) && (m->xres !=1920 || m->yres != 1080)
			&& (m->xres !=1280 || m->yres != 1024))
		     || (m->refresh > 60)
		     || (m->vmode & FB_VMODE_INTERLACED))
		{
			list_del(pos);
			kfree(pos);
		}
	}
}

void fb_select_modes_new(struct list_head *head)
{
	struct list_head *pos, *n;
	struct fb_modelist *modelist;
	struct fb_videomode *m;

//	printk(KERN_INFO "%s : %d\n", __func__, __LINE__);
	dbg();

	list_for_each_safe(pos, n, head) {
		modelist = list_entry(pos, struct fb_modelist, list);
		m = &modelist->mode;
#ifdef CONFIG_1080I_TO_1080P
		if ( (m->xres !=1920 || m->yres != 1080)
			    && (m->xres !=1280 || m->yres != 720) )
#else
		if ( ((m->xres !=1920 || m->yres != 1080)
			    && (m->xres !=1280 || m->yres != 720))
            || (m->vmode & FB_VMODE_INTERLACED) )

#endif
		{
			list_del(pos);
			kfree(pos);
		}
	}
}

void fb_add_720p(struct list_head *head)
{
	struct fb_videomode mode_720p = {
		/* 1280x720 @ 60 Hz*/
		NULL, 60, 1280, 720, 13468, 220, 110, 20, 5, 40, 5,
		FB_SYNC_HOR_HIGH_ACT | FB_SYNC_VERT_HIGH_ACT,
		FB_VMODE_NONINTERLACED
	};
	struct fb_videomode mode_1080p = {
		/* 1920x1080 @ 60 Hz*/
		NULL, 60, 1920, 1080, 6734, 148, 88, 36, 4, 44, 5,
		FB_SYNC_HOR_HIGH_ACT | FB_SYNC_VERT_HIGH_ACT,
		FB_VMODE_NONINTERLACED
	};

	struct list_head *pos, *n;
	struct fb_modelist *modelist;
	struct fb_videomode *m;
	int flag = 0, flag_1=0;

	if(!list_empty(head))
	{
		list_for_each_safe(pos, n, head) {
			modelist = list_entry(pos, struct fb_modelist, list);
			m = &modelist->mode;
			if( (m->xres == 1280) && (m->yres == 720) )  
			{
				if(50 != m->refresh){
					if(!flag){
						printk(KERN_INFO "720p is already in the modelist");
						memcpy(&modelist->mode, &mode_720p, sizeof(struct fb_videomode));
						flag = 1;
					}else{
						list_del(pos);
						kfree(pos);
					}
				}else{
					if(m->vmode & FB_VMODE_INTERLACED){
						list_del(pos);
						kfree(pos);
					}
				}
			}

			if( (m->xres == 1920) && (m->yres == 1080) )
			{
				if(50 != m->refresh){
					if(!flag_1){
						memcpy(&modelist->mode, &mode_1080p, sizeof(struct fb_videomode));
						flag_1 = 1;
					}else{
						list_del(pos);
						kfree(pos);
					}
				}else{
					if(m->vmode & FB_VMODE_INTERLACED){
						if(!flag_1){
							memcpy(&modelist->mode, &mode_1080p, sizeof(struct fb_videomode));
							flag_1 = 1;
						}else{
							list_del(pos);
							kfree(pos);
						}
					}

				}
			}
		}

		if(!flag){
			fb_add_videomode(&mode_720p, head);
			printk(KERN_INFO "720p is not in the modelist, add it");
		}
	}else{
		fb_add_videomode(&mode_720p, head);
		printk(KERN_INFO "modelist is empty, add 720p to it");
	}
	flag = 0;
	flag_1 = 0;
}

#ifdef DEBUG
void display_modelist(struct list_head *head)
{
	struct list_head *pos;
	struct fb_modelist *modelist;
	struct fb_videomode *m;

	list_for_each(pos, head) {
		modelist = list_entry(pos, struct fb_modelist, list);
		m = &modelist->mode;
		printk(KERN_INFO "xres %d,yres %d,pixclk %d, leftm %d,rm %d,um %d,lowm %d, hsync %d,vsync %d, refresh %d flag=0x%x,vmode=0x%x",
				m->xres,m->yres,m->pixclock,
				m->left_margin,m->right_margin,m->upper_margin,m->lower_margin,
				m->hsync_len,m->vsync_len, m->refresh,m->flag,m->vmode);
	}
}
#endif

static int mode_string(char *buf, unsigned int offset,
		       const struct fb_videomode *mode)
{
	char m = 'U';
	char v = 'p';

	if (mode->flag & FB_MODE_IS_DETAILED)
		m = 'D';
	if (mode->flag & FB_MODE_IS_VESA)
		m = 'V';
	if (mode->flag & FB_MODE_IS_STANDARD)
		m = 'S';

	if (mode->vmode & FB_VMODE_INTERLACED)
		v = 'i';
	if (mode->vmode & FB_VMODE_DOUBLE)
		v = 'd';

	return snprintf(&buf[offset], PAGE_SIZE - offset, "%c:%dx%d%c-%d\n",
	                m, mode->xres, mode->yres, v, mode->refresh);
}

static char buf_mode[1024] = {0,};
static void parepare_edid_buf()
{
	int i = 0, j = 0, k = 0;

	if(hdmi_edid) {
		for(i = 0; i < 256; i++)
		{
			if ((i % 16) == 0) {
				j += snprintf(&buf_mode[j], PAGE_SIZE - j, "\n%02X,", hdmi_edid[i]);
			} else {
				j += snprintf(&buf_mode[j], PAGE_SIZE - j, " %02X,", hdmi_edid[i]);
			}
		}
		snprintf(&buf_mode[1023], PAGE_SIZE - j,"\n");
	}
#ifdef DEBUG
	printk(KERN_INFO "%s ... %s", __func__, buf_mode);
#endif

}

static ssize_t show_modes(struct device *device, struct device_attribute *attr,
			  char *buf)
{
	if(buf_mode)
		snprintf(buf, PAGE_SIZE, "%s", buf_mode);
}

static void bakup_modelist(struct list_head *head, int num)
{
	struct list_head *pos;
	struct fb_modelist *modelist;
	struct fb_videomode *mode, *m;
	int i = 0;

	list_for_each(pos, head) {
		modelist = list_entry(pos, struct fb_modelist, list);
		i += mode_string(buf_mode, i, &modelist->mode);
	}

#ifdef DEBUG
	printk(KERN_INFO "%s ... %s", __func__, buf_mode);
#endif
}

int hdmi_edid_fetch(unsigned char *edid, struct fb_info *info)
{
	struct fb_monspecs *specs = NULL;
	const struct fb_videomode * m = NULL;
	int ret = 0;
	const struct fb_videomode  select_mode = {NULL, 60, 1280, 720,};
	const struct fb_videomode  find_mode = {NULL, 60, 1280, 720,};

	specs = &info->monspecs;

	printk(KERN_INFO "%s............\n", __func__);

	if(edid) {
		parepare_edid_buf();
		if (info->monspecs.modedb) {
			fb_destroy_modedb(info->monspecs.modedb);
			info->monspecs.modedb = NULL;
			fb_destroy_modelist(&info->modelist);
		}

		fb_edid_to_monspecs(edid, specs);
		if(*(edid + 126))
		{
			fb_edid_add_monspecs(edid + 0x80, specs); 
		}

               if(device_create_file(info->dev, &dev_attr_vendor)){
                       device_remove_file(info->dev, &dev_attr_vendor);
               }
		if(specs->modedb) {
			fb_videomode_to_modelist(specs->modedb,
					specs->modedb_len, &info->modelist);

			printk(KERN_INFO "fb%d: %s get Mode list ok! modedb len is %d!\n", info->node,
					info->fix.id, specs->modedb_len);

			//bakup_modelist(&info->modelist, specs->modedb_len);

			if(device_create_file(info->dev, &dev_attr_edidinfo)){
				device_remove_file(info->dev, &dev_attr_edidinfo);
			}


#ifdef DEBUG
			display_modelist(&info->modelist);
#endif
			//fb_select_modes(&select_mode, &info->modelist);
			fb_select_modes_new(&info->modelist);
#ifdef DEBUG
			printk(KERN_INFO "after fb_select_modes_new!!!");
			display_modelist(&info->modelist);
#endif
			fb_add_720p(&info->modelist);

#ifdef DEBUG
			display_modelist(&info->modelist);
#endif
			//m = fb_find_best_display(specs, &info->modelist);
			if( (!set_resolution) && (1 != get_lcd1_args) ){
				m = fb_find_nearest_mode(&find_mode, &info->modelist);


				if (m) {
					printk(KERN_INFO "xres %d,yres %d,pixclk %d\n"
						"leftm %d,rm %d,um %d,lowm %d"
						"hsync %d,vsync,%d",
						m->xres,m->yres,m->pixclock,
						m->left_margin,m->right_margin,m->upper_margin,m->lower_margin,
						m->hsync_len,m->vsync_len);

					fb_videomode_to_var(&info->var, m);
					//info->mode = m;
					memcpy(info->mode, m, sizeof(struct fb_videomode));
#if 0
					if((m->xres == 1920) && (m->yres == 1080))
						info->var.pixclock = 6734;

					if((m->xres == 1280) && (m->yres == 720))
						info->var.pixclock = 13468;
#endif		
				} else {
					printk(KERN_EMERG "fb%d: %s get best display failed!\n", 
							info->node, info->fix.id);
					ret = -2;
				}
			}

		} else {
			printk(KERN_EMERG "fb%d: %s get Mode Database failed!\n",
					info->node, info->fix.id);
			ret = -3;
		}
	}else{
		printk(KERN_EMERG "fb%d: %s not edid fetch!\n", info->node, info->fix.id);
		ret = -4;
	}

	return ret;
}

#include "ns115-hdmi.h"

static int enable_lcd1(struct ns115_hdmi_lcdc_data *hdmi)
{
	struct fb_videomode mode;
	struct fb_var_screeninfo stdvar;
	int retval = 0;
	unsigned char *ptr = hdmi->pedid;
	struct nusmart_par *fb1par = fb1_info->par;

	retval = hdmi_edid_fetch(ptr, fb1_info);
	if(!set_resolution){

		if(retval || (1 == get_lcd1_args)){
			printk(KERN_WARNING "fb1 not fetch edid or get_lcd1_args = %d, set default mode as %s\n", 
				get_lcd1_args, mode_option_disp1);
			retval = fb_find_mode(&stdvar, fb1_info, mode_option_disp1, NULL, 0,NULL, 16);
			fb_var_to_videomode(&mode, &stdvar);
			fb_videomode_to_var(&fb1_info->var, &mode);
			// fb1_info->mode = &mode;
			memcpy(fb1_info->mode, &mode, sizeof(struct fb_videomode));
			hdmi->refresh = 60;
		}else{
			printk(KERN_INFO "%dx%d-%d pixclock %d\n", 
				fb1_info->var.xres, fb1_info->var.yres,
				fb1_info->var.bits_per_pixel, fb1_info->var.pixclock);
			hdmi->refresh = fb1_info->mode->refresh;
		}
	}else{
		printk(KERN_INFO "%dx%d-%d pixclock %d\n", 
			fb1_info->var.xres, fb1_info->var.yres,
			fb1_info->var.bits_per_pixel, fb1_info->var.pixclock);
		hdmi->refresh = fb1_info->mode->refresh;

	}

	if( 1 != get_lcd1_args){
#ifdef CONFIG_MACH_NS115_STICK
#ifdef CONFIG_USES_FAKE_1080P
			fb1_info->var.bits_per_pixel = 32;
#else
		if((fb1_info->mode->xres * fb1_info->mode->yres) <= (1280 * 720))
		{
			fb1_info->var.bits_per_pixel = 32;
		}else{
			fb1_info->var.bits_per_pixel = 16;
		}
#endif
#else
		if(fb0_bits_per_pixel == 16 && (fb1_info->mode->xres * fb1_info->mode->yres) <= (1280 * 720))
		{
			fb1_info->var.bits_per_pixel = 32;
		}else{
			fb1_info->var.bits_per_pixel = 16;
		}
#endif
	}

	nusmartfb_check_var(&fb1_info->var, fb1_info);
	nusmartfb_set_par(fb1_info);

	hdmi->horizontal_res = fb1_info->var.xres;
	hdmi->vertical_res = fb1_info->var.yres;
	hdmi->pixel_clk	= clk_get_rate(fb1par->clk);
	hdmi->mode = fb1_info->mode;

	printk(KERN_INFO "Final set resolution rate to %dx%d-%d",fb1_info->var.xres, fb1_info->var.yres, fb1_info->var.bits_per_pixel);

	return retval;

}

static BLOCKING_NOTIFIER_HEAD(hdmi_notifier_list);
static int hdmi_hotplug_probe_lcd(struct notifier_block *this, unsigned long event, void *ptr)
{
	struct nusmart_par *par;
	struct ns115_hdmi_lcdc_data * hdmi_data = ptr;

	if(!fb1_info)
	{
		printk(KERN_EMERG "lcd1 not probe! exit....");
		return -1;
	}
	par = fb1_info->par;

#ifdef	DUAL_LCD
	if(1 == event){
		printk(KERN_INFO "%s do lcd1 probe!\n", __func__);
		hdmi_state = 1;
		hdmi_edid = hdmi_data->pedid;
		//clk_enable(par->clk);

		hdmi_plugin = 1;
		enable_lcd1(hdmi_data);
		hdmi_plugin = 0;
	}else{
		printk(KERN_INFO "%s do lcd1 unprobe!\n", __func__);
		hdmi_edid = NULL;
		hdmi_state = 0;

		fb_destroy_modelist(&fb1_info->modelist);

		device_remove_file(fb1_info->dev, &dev_attr_vendor);
		device_remove_file(fb1_info->dev, &dev_attr_edidinfo);
#ifndef CONFIG_FB_HDMI_NS115
		nusmart_clcd_disable(fb1_info);
#endif
		//if(par->clk->enabled)
		//	clk_disable(par->clk);
	}
#endif	/* DUAL_LCD */

	return 0;
}

static struct notifier_block hdmi_hotplug_notify = {
	.notifier_call = hdmi_hotplug_probe_lcd,
};

int register_hdmi_notifier(struct notifier_block *nb)
{
	return blocking_notifier_chain_register(&hdmi_notifier_list, nb);
}

int unregister_hdmi_notifier(struct notifier_block *nb)
{
	return blocking_notifier_chain_unregister(&hdmi_notifier_list, nb);
}

int send_hdmi_hotplug_event(unsigned long val, void *v)
{
	return	blocking_notifier_call_chain(&hdmi_notifier_list, val, v);
}

EXPORT_SYMBOL(send_hdmi_hotplug_event);

#ifndef MODULE
/*
 * Only necessary if your driver takes special options,
 * otherwise we fall back on the generic fb_setup().
 */
static int __init nusmartfb_setup(char *options, int id)
{
	/* Parse user speficied options (`video=nusmartfb:') */
	ctrace;
	if(id == 0)
		mode_option_disp0 = options;
	else
		mode_option_disp1 = options;

	printk(KERN_EMERG "%s nusmartfb%d options:%s\n", __func__, id, options);
	return 0;
}
#endif /* MODULE */


static int __init nusmartfb_init(void)
{
	int ret;
	/*
	 *  For kernel boot options (in 'video=DISP0_NAME:<options>' format)
	 */
#ifndef MODULE
	register_hdmi_notifier(&hdmi_hotplug_notify);

	char *option0 = NULL;
	ctrace;
	/* better indicate DISP0's res, or read through I2C, we do NOT retrun even no option get */
	if (!fb_get_options(DISP0_NAME, &option0)) {
		if(option0){

			nusmartfb_setup(option0, 0);
		}else{
			mode_option_disp0 = "1024x600-32";
			err("LINE %d: WARNING: %s get option failed. set as %s default",
					__LINE__, __func__, mode_option_disp0);
		}
	}else{
		mode_option_disp0 = "1024x600-32";
		err("LINE %d: WARNING: %s get option failed. set as %s default",
				__LINE__, __func__, mode_option_disp0);
		//return -ENODEV;
	}
	ret = platform_driver_register(&nusmartfb_driver[0]);

	if (ret) 
		dbg("nusmartfb_driver 0 register failed! retval = %d", ret);

#if 1
#ifdef	DUAL_LCD
//	char *option1 = NULL;
	if (!fb_get_options(DISP1_NAME, &mode_option_disp1)) {
		if(mode_option_disp1){
			nusmartfb_setup(mode_option_disp1, 1);
			get_lcd1_args = 1;
		}else{
			mode_option_disp1 = "1280x720-24";
			err("LINE %d: WARNING: %s get option failed. set as %s default",
					__LINE__, __func__, mode_option_disp1);
		}

	}else{
		mode_option_disp1 = "1280x720-24";
		err("LINE %d: WARNING: %s get option failed. set as %s default",
				__LINE__, __func__, mode_option_disp1);
		//return -ENODEV;
	}

	ret = platform_driver_register(&nusmartfb_driver[1]);

	if (ret) 
		dbg("nusmartfb_driver 0 register failed! retval = %d", ret);
#endif	/* DUAL_LCD */

#endif


#endif	/* MODULE */

	return ret;
}

static void __exit nusmartfb_exit(void)
{
	ctrace;
#ifdef	DUAL_LCD
	platform_driver_unregister(&nusmartfb_driver[1]);
#endif	/* DUAL_LCD */
	//#ifndef DUAL_LCD
	platform_driver_unregister(&nusmartfb_driver[0]);
	//#endif
	unregister_hdmi_notifier(&hdmi_hotplug_notify);
}


module_init(nusmartfb_init);
module_exit(nusmartfb_exit);

MODULE_LICENSE("GPL");
