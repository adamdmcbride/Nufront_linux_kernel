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
#include <linux/earlysuspend.h>
#include <asm/uaccess.h>
#include <mach/memory.h>
#include <mach/get_bootargs.h>
#include <mach/lcdc.h>
#include <asm/irq.h>

#include <asm/mach-types.h>
#define NUSMART_PW_CTRL
#define NUSMART_FB_ON2_SPEC
#define WAIT_FOR_VSYNC

#ifdef WAIT_FOR_VSYNC
#include <linux/completion.h>
#endif
#include <linux/dmaengine.h>

    /*
     *  This is just simple sample code.
     *
     *  No warranty that it actually compiles.
     *  Even less warranty that it actually works :-)
     */

/*define frame buffer size in byte*/
#define VIDEO_MEM_SIZE		nusmart_lcd_len() 
/*define frame buffer alloc by reserve*/
#define FB_RESERV
/*!frame buffer size limited to 4MB if don't reserve*/
#ifdef FB_RESERV
//#define VIDEO_MEM_BASE		0xae000000
//#define VIDEO_MEM_BASE		NS2816_DMA_LCDC_BASE
#define VIDEO_MEM_BASE		nusmart_lcd_base()	
//#define VIDEO_ON2MEM_BASE	NS2816_DMA_LCDC_BASE+SZ_16M
#define VIDEO_ON2MEM_BASE	(VIDEO_MEM_BASE + SZ_16M)
#endif
//#define IRQ_NS2816_DISPLAY_DERR (61)
#define err(x,args...) printk(KERN_ERR"%s:"#x"\n",__func__,##args)
#ifdef DEBUG
//#define ctrace printk("ctrace:%s\n",__func__)
#define ctrace 
#define dbg(x,args...) printk("%s:"#x"\n",__func__,##args)
#define dbg_var(var)	dbg("resolution:%dx%d-%d",var->xres,var->yres,var->bits_per_pixel)
#else
#define ctrace 
#define dbg(x,args...) 
#define dbg_var(var)	
#endif /*DEBUG*/
#include "nusmart-clcd.h"

#define DDC_ARGS	"auto"
#define DDC_ARGS_16	"auto-16"
#define DDC_ARGS_32	"auto-32"
#define LCDC_NAME 	"ns2816fb"
/*
 * Driver data
 */
static char *mode_option __devinitdata;

static u32	pseudo_palette[16];
/*
 *  If your driver supports multiple boards, you should make the  
 *  below data types arrays, or allocate them dynamically (using kmalloc()). 
 */ 

/*
 * Here we define the default structs fb_fix_screeninfo and fb_var_screeninfo
 * if we don't use modedb. If we do use modedb see nusmartfb_init how to use it
 * to get a fb_var_screeninfo. Otherwise define a default var as well. 
 */
static struct fb_fix_screeninfo nusmartfb_fix __devinitdata = {
	.id =		LCDC_NAME,/*"nusmartfb", */
	.type =		FB_TYPE_PACKED_PIXELS,
	.visual =	/*FB_VISUAL_PSEUDOCOLOR |*/ FB_VISUAL_TRUECOLOR,
	.xpanstep =	0,
	.ypanstep =	1,
	.ywrapstep =	0, 
	.accel =	FB_ACCEL_NONE,
};

struct nusmart_size
{
        unsigned int x;
        unsigned int y;
};
struct nusmart_porch
{
        unsigned int sync;
        unsigned int front;
        unsigned int back;
};
struct nusmart_mode
{
	unsigned int pixclk;
        struct nusmart_size screen_size;
        struct nusmart_porch h_porch;
        struct nusmart_porch v_porch;
};
static struct nusmart_mode nusmart_modes[]={
        {
		.pixclk = 25175000,
                .screen_size ={
                                .x      =       640,
                                .y      =       480,
                        },
                .h_porch={
                                .sync   =       96, 
                                .front  =       16,
                                .back   =       48,
                        },
                .v_porch={
                                .sync   =       2,
                                .front  =       10,
                                .back   =       33,
                        },
        },
        {
		.pixclk = 40000000,
                .screen_size ={
                                .x      =       800,
                                .y      =       600,
                        },
                .h_porch={
                                .sync   =       128,
                                .front  =       40,
                                .back   =       88,
                        },
                .v_porch={
                                .sync   =       4,
                                .front  =       1,
                                .back   =       23,
                        },
        },
      {
		.pixclk = 45000000,
                .screen_size ={
                                .x      =       1024,
                                .y      =       600,
                        },
                .h_porch={
                                .sync   =       35,
                                .front  =       53,
                                .back   =       88,
                        },
                .v_porch={
                                .sync   =       5,
                                .front  =       4,
                                .back   =       16,
                        },
        },
        {
		.pixclk = 44900000/*65000000*/,
                .screen_size ={
                                .x      =       1024,
                                .y      =       768,
                        },
                .h_porch={
                                .sync   =       176,
                                .front  =       8,
                                .back   =       56,
                        },
                .v_porch={
                                .sync   =       0,
                                .front  =       4,
                                .back   =       20,
                        },
        },
        {
		.pixclk = 74250000,
                .screen_size ={
                                .x      =       1280,
                                .y      =       720,
                        },
                .h_porch={
                                .sync   =       40,
                                .front  =       110,
                                .back   =       220,
                        },
                .v_porch={
                                .sync   =       5,
                                .front  =       5,
                                .back   =       20,
                        },
        },
        {
		.pixclk = 79500000,
                .screen_size ={
                                .x      =       1280,
                                .y      =       768,
                        },
                .h_porch={
                                .sync   =       128,
                                .front  =       64,
                                .back   =       192,
                        },
                .v_porch={
                                .sync   =       7,
                                .front  =       3,
                                .back   =       20,
                        },
        },
        {
		.pixclk = 85500000,
                .screen_size ={
                                .x      =       1360,
                                .y      =       768,
                        },
                .h_porch={
                                .sync   =       112,
                                .front  =       64,
                                .back   =       256,
                        },
                .v_porch={
                                .sync   =       6,
                                .front  =       3,
                                .back   =       18,
                        },
        },
        {
		.pixclk = 108000000,
                .screen_size ={
                                .x      =       1280,
                                .y      =       960,
                        },
                .h_porch={
                                .sync   =       112,
                                .front  =       96,
                                .back   =       312,
                        },
                .v_porch={
                                .sync   =       1,
                                .front  =       3,
                                .back   =       36,
                        },
        },
        {
		.pixclk = 108000000,
                .screen_size ={
                                .x      =       1280,
                                .y      =       1024,
                        },
                .h_porch={
                                .sync   =       112,
                                .front  =       48,
                                .back   =       248,
                        },
                .v_porch={
                                .sync   =       1,
                                .front  =       3,
                                .back   =       38,
                        },
        },
        {
		.pixclk = 121750000,
                .screen_size ={
                                .x      =       1400,
                                .y      =       1050,
                        },
                .h_porch={
                                .sync   =       144,
                                .front  =       88,
                                .back   =       232,
                        },
                .v_porch={
                                .sync   =       3,
                                .front  =       4,
                                .back   =       32,
                        },
        },
        {
		.pixclk = 106500000,
                .screen_size ={
                                .x      =       1440,
                                .y      =       900,
                        },
                .h_porch={
                                .sync   =       152,
                                .front  =       80,
                                .back   =       232,
                        },
                .v_porch={
                                .sync   =       6,
                                .front  =       3,
                                .back   =       25,
                        },
        },
        {
		.pixclk = 148500000,
                .screen_size ={
                                .x      =       1920,
                                .y      =       1080,
                        },
                .h_porch={
                                .sync   =       44,
                                .front  =       88,
                                .back   =       148,
                        },
                .v_porch={
                                .sync   =       5,
                                .front  =       4,
                                .back   =       36,
                        },
        },
        {
		.pixclk = /*96625000*/193250000,
                .screen_size ={
                                .x      =       1920,
                                .y      =       1200,
                        },
                .h_porch={
                                .sync   =       200,
                                .front  =       136,
                                .back   =       336,
                        },
                .v_porch={
                                .sync   =       6,
                                .front  =       3,
                                .back   =       36,
                        },
        },
        {0,{0,0,},{0,0,0,},{0,0,0,},},

};
static int mode_sel_index = 0;
/*
 *check mode
 */
static int find_mode(struct fb_var_screeninfo *var)
{
	int idx = 0;
	struct nusmart_mode * mode_ptr = NULL;
	for(idx = 0; nusmart_modes[idx].screen_size.x > 0; idx++) {
		mode_ptr = &nusmart_modes[idx];
		if(var->xres == mode_ptr->screen_size.x && 
			var->yres == mode_ptr->screen_size.y) {
			var->hsync_len = mode_ptr->h_porch.sync;
			var->left_margin = mode_ptr->h_porch.front;
			var->right_margin = mode_ptr->h_porch.back;
			var->vsync_len = mode_ptr->v_porch.sync;
			var->upper_margin = mode_ptr->v_porch.front;
			var->lower_margin = mode_ptr->v_porch.back;	
			return idx;
		}
	}	
	return -EINVAL;
}
/*set mode*/
static int set_mode(struct fb_var_screeninfo *var)
{
	int idx = 0;
	struct nusmart_mode * mode_ptr = NULL;
	for(idx = 0; nusmart_modes[idx].screen_size.x > 0; idx++) {
		mode_ptr = &nusmart_modes[idx];
		if(var->xres == mode_ptr->screen_size.x && 
			var->yres == mode_ptr->screen_size.y) {
			mode_ptr->h_porch.sync = var->hsync_len;
			mode_ptr->h_porch.front = var->left_margin;
			mode_ptr->h_porch.back = var->right_margin;
			mode_ptr->v_porch.sync = var->vsync_len;
			mode_ptr->v_porch.front = var->upper_margin;
			mode_ptr->v_porch.back = var->lower_margin;	
			return idx;
		}
	}	
	return -EINVAL;
}
/*add on2 specific function*/
#ifdef NUSMART_FB_ON2_SPEC
static int init_on2buf(struct on2_buf * buf)
{
	int index,ret=0;
	if(!buf)
		return -EINVAL;
	for(index=0;index<ON2BUF_COUNT;index++) {
		buf->buf[index].pptr=(unsigned char *)(VIDEO_ON2MEM_BASE + ON2BUF_SIZE*index);	
/*		buf->buf[index].vptr=ioremap((unsigned int)(buf->buf[index].pptr),ON2BUF_SIZE);
		if(!buf->buf[index].vptr)
			ret = -EINVAL;*/
	}
	return ret;
}
static void release_on2buf(struct on2_buf * buf)
{
	int index;
	if(!buf)
		return; 
	for(index=0;index<ON2BUF_COUNT;index++) {
		if(!buf->buf[index].vptr) {
		/*	iounmap(buf->buf[index].vptr);*/
			buf->buf[index].vptr = 0;
		}
	}
	return;

}
#if 1
static void on2_show(unsigned int hwaddr,void * reg)
{
	unsigned int ctrl;
	ctrl = readl(reg+NUSMART_LCDC_CTRL);
	if(ctrl & NUSMART_LCDC_PTRF) {/*secondary*/
	/*set first*/	
		writel(hwaddr,reg+NUSMART_LCDC_PRIPTR);
		ctrl&=~NUSMART_LCDC_PTRF;
	} else {/*first*/
	/*set second*/	
		writel(hwaddr,reg+NUSMART_LCDC_SECPTR);
		ctrl|=NUSMART_LCDC_PTRF;
	}
	writel(ctrl,reg+NUSMART_LCDC_CTRL);
}
#endif
static int set_on2_res(struct fb_info * info,struct on2_res * res)
{
	struct nusmart_par * par = info->par;
	struct fb_var_screeninfo var;
	struct clcd_regs regs;
	int ret=0;
	return ret;//disable on2 res change
	printk("nusmartfb set on2 res:%dx%d\n",res->x,res->y);
	if(par->on2res.x!=res->x || par->on2res.y!=res->y) {
		var.xres=res->x;
		var.yres=res->y;
		ret=find_mode(&var);
		if(ret < 0)
			return -EINVAL;
		else{
			par->on2resindex = ret;	
			nusmart_clcd_var_decode(&var,&regs);

			writel(regs.hintv, par->mmio + NUSMART_LCDC_HINTV);
			writel(regs.vintv, par->mmio + NUSMART_LCDC_VINTV);
			writel(regs.modulo, par->mmio + NUSMART_LCDC_MODULO);
			writel(regs.hvsize, par->mmio + NUSMART_LCDC_HVSIZE);

			writel((5<<9), par->mmio + NUSMART_LCDC_CTRL);
			if(clk_set_rate(par->clk,nusmart_modes[par->on2resindex].pixclk)<0)
			printk("on2 set clcd clock failed!%d\n",nusmart_modes[par->on2resindex].pixclk);
			
			if(par->on2flag == 2)
				regs.ctrl |= NUSMART_LCDC_RGBF;
			writel(regs.ctrl, par->mmio + NUSMART_LCDC_CTRL);
		}

		par->on2res.x=res->x;
		par->on2res.y=res->y;
	}
	return ret;
}
static int reset_on2_res(struct fb_info * info)
{
	struct nusmart_par * par = info->par;
	struct fb_var_screeninfo var;
	struct clcd_regs regs;
	int ret=0;
	return ret;//disable on2 res change
	if(par->on2res.x!=info->var.xres || par->on2res.y!=info->var.yres) {
		var.xres=info->var.xres;
		var.yres=info->var.yres;
		ret=find_mode(&var);
		if(ret < 0)
			return -EINVAL;
		else{
			par->on2resindex = ret;	
			nusmart_clcd_var_decode(&var,&regs);

			writel(regs.hintv, par->mmio + NUSMART_LCDC_HINTV);
			writel(regs.vintv, par->mmio + NUSMART_LCDC_VINTV);
			writel(regs.modulo, par->mmio + NUSMART_LCDC_MODULO);
			writel(regs.hvsize, par->mmio + NUSMART_LCDC_HVSIZE);

			/*	writel(0, par->mmio + NUSMART_LCDC_CTRL);*/
			if(clk_set_rate(par->clk,nusmart_modes[par->on2resindex].pixclk)<0)
				printk("on2 set clcd clock failed!%d\n",nusmart_modes[par->on2resindex].pixclk);

			writel(regs.ctrl, par->mmio + NUSMART_LCDC_CTRL);
		}

		par->on2res.x=info->var.xres;
		par->on2res.y=info->var.yres;
	}
	return ret;
}
#endif

    /*
     * 	Modern graphical hardware not only supports pipelines but some 
     *  also support multiple monitors where each display can have its  
     *  its own unique data. In this case each display could be  
     *  represented by a separate framebuffer device thus a separate 
     *  struct fb_info. Now the struct nusmart_par represents the graphics
     *  hardware state thus only one exist per card. In this case the 
     *  struct nusmart_par for each graphics card would be shared between 
     *  every struct fb_info that represents a framebuffer on that card. 
     *  This allows when one display changes it video resolution (info->var) 
     *  the other displays know instantly. Each display can always be
     *  aware of the entire hardware state that affects it because they share
     *  the same nusmart_par struct. The other side of the coin is multiple
     *  graphics cards that pass data around until it is finally displayed
     *  on one monitor. Such examples are the voodoo 1 cards and high end
     *  NUMA graphics servers. For this case we have a bunch of pars, each
     *  one that represents a graphics state, that belong to one struct 
     *  fb_info. Their you would want to have *par point to a array of device
     *  states and have each struct fb_ops function deal with all those 
     *  states. I hope this covers every possible hardware design. If not
     *  feel free to send your ideas at jsimmons@users.sf.net 
     */

    /*
     *  If your driver supports multiple boards or it supports multiple 
     *  framebuffers, you should make these arrays, or allocate them 
     *  dynamically using framebuffer_alloc() and free them with
     *  framebuffer_release().
     */ 
/*static struct fb_info info;*/

    /* 
     * Each one represents the state of the hardware. Most hardware have
     * just one hardware state. These here represent the default state(s). 
     */
/*static struct nusmart_par __initdata current_par;*/

static int nusmartfb_init(void);

/**
 *	nusmartfb_open - Optional function. Called when the framebuffer is
 *		     first accessed.
 *	@info: frame buffer structure that represents a single frame buffer
 *	@user: tell us if the userland (value=1) or the console is accessing
 *	       the framebuffer. 
 *
 *	This function is the first function called in the framebuffer api.
 *	Usually you don't need to provide this function. The case where it 
 *	is used is to change from a text mode hardware state to a graphics
 * 	mode state. 
 *
 *	Returns negative errno on error, or zero on success.
 */
/*
static int nusmartfb_open(struct fb_info *info, int user)
{
	return 0;
}
*/
/**
 *	nusmartfb_release - Optional function. Called when the framebuffer 
 *			device is closed. 
 *	@info: frame buffer structure that represents a single frame buffer
 *	@user: tell us if the userland (value=1) or the console is accessing
 *	       the framebuffer. 
 *	
 *	Thus function is called when we close /dev/fb or the framebuffer 
 *	console system is released. Usually you don't need this function.
 *	The case where it is usually used is to go from a graphics state
 *	to a text mode state.
 *
 *	Returns negative errno on error, or zero on success.
 */
/*
static int nusmartfb_release(struct fb_info *info, int user)
{
	return 0;
}
*/
static int set_bitfield(struct fb_var_screeninfo *var, struct fb_info *info)
{
//	struct nusmart_par * par = info->par;
//	struct clcd_regs regs;
	int ret = 0;

	//regs.ctrl = readl(par->mmio + NUSMART_LCDC_CTRL);

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
	/*struct nusmart_par * par = info->par;*/

/*	ctrace;*/
	dbg_var(var);
//	if(par->mvflag == 0)		
//	ret = find_mode(var);
//	if(var->xres*var->yres*var->bits_per_pixel/8>1280*1024*2) /*check for bus width*/
//		ret = -EINVAL;
	if(ret < 0)
		return ret;
	else
		mode_sel_index = ret;


	ret = set_bitfield(var, info);
	
	if(ret < 0)
		return ret;
	/*calculate smem_len after check_bitfield*/
	var->yres_virtual =  var->yres * 2;
	smem_len = var->xres_virtual * var->bits_per_pixel / 8 *
			var->yres_virtual;
	if(smem_len  > info->fix.smem_len) {
		dbg("not enough video mem alloced!");
		return -EINVAL;
	}
	return 0;	   	
}

struct fb_var_screeninfo cached_fb_var;
static int is_fb_var_cached = 0;

static int is_fb_changed(struct fb_info *fb)
{
	ctrace;
    if (!is_fb_var_cached ||
        fb->var.xres != cached_fb_var.xres ||
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
        is_fb_var_cached = 1;

        return 1;
    }
    else

        return 0;
}

static void nusmart_clcd_enable(struct fb_info *fb, u32 ctrl) 
{
	struct nusmart_par * par = fb->par;
	ctrace;
	ctrl |= NUSMART_LCDC_ENABLE;
	writel(ctrl, par->mmio + NUSMART_LCDC_CTRL);

	/*udely(20);*/
	/*enable clk*/
	/*clk_enable(par->clk);*/
	/*
	  * and now apply power.
	  */
	ctrl &= ~NUSMART_LCDC_PSF;
	ctrl &= ~NUSMART_LCDC_BLANK;
	writel(ctrl, par->mmio + NUSMART_LCDC_CTRL);
}
static void nusmart_clcd_disable(struct fb_info *fb)
{
	struct nusmart_par *par = fb->par;
	u32 val;
	
	ctrace;
	val = readl(par->mmio + NUSMART_LCDC_CTRL);
	if (val & NUSMART_LCDC_ENABLE) {
		val &= ~NUSMART_LCDC_ENABLE;
		writel(val, par->mmio + NUSMART_LCDC_CTRL);

		/*udelay(20);*/
	}
	if (!(val & NUSMART_LCDC_PSF)) {
		val |= NUSMART_LCDC_PSF;
		writel(val, par->mmio + NUSMART_LCDC_CTRL);
	}
	/*clk_disable(par->clk);*/
}
/*bpp 16bit & res 1024x768*/
static unsigned int DISP_BASE = 1024*768;
#define DIRECT_SHOW	0
#define INDIRECT_SHOW	1
static void nusmart_clcd_set_hwfb(struct fb_info *fb, int flag)
{
	struct nusmart_par *par = fb->par;
	unsigned long ustart = fb->fix.smem_start;
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
	}else if((800 == var->xres) && (600 == var->yres))
	{
		regs.cfg_axi &= ~(0xf00);
		regs.cfg_axi |= (NUSMART_LCDC_ARBLEN_2);
		writel(regs.cfg_axi, par->mmio + NUSMART_LCDC_CFG_AXI);
		dbg("change burst len to 4!\n");

	//UXGA-1600X1200 60HZ, BurstLen <= 8
	//}else if((1600 == var->xres) && (1200 == var->yres) && ( 6172 == var->pixclock))
	}else if((1600 == var->xres) && (1200 == var->yres))
	{
		regs.cfg_axi &= ~(0xf00);
		regs.cfg_axi |= (NUSMART_LCDC_ARBLEN_2);
		writel(regs.cfg_axi, par->mmio + NUSMART_LCDC_CFG_AXI);
		dbg("change burst len to 8!\n");

	//WSXGA+-1680X1050 60HZ, BurstLen <= 2
	//}else if((1680 == var->xres) && (1050 == var->yres) && ( 6848 == var->pixclock))
	}else if((1680 == var->xres) && (1050 == var->yres))
	{
		regs.cfg_axi &= ~(0xf00);
		regs.cfg_axi |= (NUSMART_LCDC_ARBLEN_2);
		writel(regs.cfg_axi, par->mmio + NUSMART_LCDC_CFG_AXI);
		dbg("change burst len to 2!\n");
	}else{
		regs.cfg_axi &= ~(0xf00);
		regs.cfg_axi |= (NUSMART_LCDC_ARBLEN_2);
		writel(regs.cfg_axi, par->mmio + NUSMART_LCDC_CFG_AXI);
		dbg("change burst len to 2!\n");
	}

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
static int nusmartfb_set_par(struct fb_info *fb)
{
	struct nusmart_par *par = fb->par;
	/* ... */
	struct clcd_regs regs;
	/*struct fb_var_screeninfo * var = &fb->var;*/
	int clk=1000000000;
	/* Add for LCDC test by weixing */
	unsigned int tmp_reg;

	ctrace;
/*	dbg_var(var);*/

	fb->fix.line_length = fb->var.xres_virtual *
	                         fb->var.bits_per_pixel / 8;
	if(fb->var.bits_per_pixel<15) {
		dbg("bit_per_pixel(%d) not supported",fb->var.bits_per_pixel);
	}	
	fb->fix.visual = FB_VISUAL_TRUECOLOR;
#ifdef NUSMART_FB_ON2_SPEC
	if(is_fb_changed(fb)&&(!par->on2flag)) {
#else
	if(is_fb_changed(fb)) {
#endif
		nusmart_clcd_decode(fb, &regs);
		
		nusmart_clcd_disable(fb);
	
		nusmart_fb_set_burstlen(fb);	
		writel(regs.hintv, par->mmio + NUSMART_LCDC_HINTV);
		writel(regs.vintv, par->mmio + NUSMART_LCDC_VINTV);
		writel(regs.modulo, par->mmio + NUSMART_LCDC_MODULO);
		writel(regs.hvsize, par->mmio + NUSMART_LCDC_HVSIZE);
		/* Add for LCDC test by weixing */
		tmp_reg = readl(par->mmio + 0x00);
		tmp_reg |= (unsigned int)(0x1 << 20);
		writel(tmp_reg, par->mmio + 0x00);

		writel(0x3c80c100, par->mmio + 0x20);
		switch(fb->var.xres){
			case 1024:
				if(16 == fb->var.bits_per_pixel){
					writel(0x00000142, par->mmio + 0x24);
					writel(0x000000e6, par->mmio + 0x28);
				}
				else{
					writel(0x00000153, par->mmio + 0x24);
					writel(0x000000e6, par->mmio + 0x28);
				}
				break;
			case 1280:
				if(16 == fb->var.bits_per_pixel){
					writel(0x00000196, par->mmio + 0x24);
					writel(0x0000011a, par->mmio + 0x28);
				}
				else{
					writel(0x000001a7, par->mmio + 0x24);
					writel(0x0000011a, par->mmio + 0x28);
				}
				break;
			case 1360:
				writel(0x00000196, par->mmio + 0x24);
				writel(0x00000112, par->mmio + 0x28);
				break;
			case 1368:
				writel(0x3c80c000, par->mmio + 0x20);
				writel(0x230d508f, par->mmio + 0x08);
				writel(0x000001d1, par->mmio + 0x24);
				writel(0x00000145, par->mmio + 0x28);
				break;
			case 1440:
				writel(0x00000196, par->mmio + 0x24);
				writel(0x00000108, par->mmio + 0x28);
				break;
			case 1680:
				writel(0x00000196, par->mmio + 0x24);
				writel(0x000000ee, par->mmio + 0x28);
				break;
			case 1920:
				if(16 == fb->var.bits_per_pixel){
					writel(0x5809402c, par->mmio + 0x08);
					writel(0x00000196, par->mmio + 0x24);
					writel(0x000000dc, par->mmio + 0x28);
				}
				else{
					writel(0x5810402c, par->mmio + 0x08);
					writel(0x00000196, par->mmio + 0x24);
					writel(0x000000dc, par->mmio + 0x28);
				}
				break;
			default:
				writel(0x00000142, par->mmio + 0x24);
				writel(0x000000e6, par->mmio + 0x28);
				break;
		}
		/* Add for LCDC test by weixing 2011-07-08 */


		nusmart_clcd_set_hwfb(fb, DIRECT_SHOW);
		
		par->ctrl = regs.ctrl;
		/**/
		clk /= fb->var.pixclock;
		clk *= 1000;
		//if(clk_set_rate(par->clk,nusmart_modes[mode_sel_index].pixclk)<0)
		if(clk_set_rate(par->clk,clk)<0)
			printk("set clcd clock failed!%d\n",clk);

		DISP_BASE = fb->var.xres * fb->var.yres;
		nusmart_clcd_enable(fb, regs.ctrl);
	}
#ifdef DEBUG
        printk(KERN_INFO "CLCD: Registers set to\n"
               KERN_INFO "  %08x %08x %08x %08x\n"
               KERN_INFO "  %08x %08x %08x %08x %08x\n",
                readl(par->mmio + NUSMART_LCDC_CTRL), readl(par->mmio + NUSMART_LCDC_RESERV),
                readl(par->mmio + NUSMART_LCDC_HINTV), readl(par->mmio + NUSMART_LCDC_VINTV),
                readl(par->mmio + NUSMART_LCDC_PRIPTR), readl(par->mmio + NUSMART_LCDC_SECPTR),
                readl(par->mmio + NUSMART_LCDC_MODULO), readl(par->mmio + NUSMART_LCDC_HVSIZE),
		readl(par->mmio + NUSMART_LCDC_CFG_AXI));
#endif
	return 0;	
}
static int set_var(struct fb_info * info,struct fb_var_screeninfo * var)
{
	struct nusmart_par * par = info->par;
	struct clcd_regs regs;
	int ret=0;
	ret=set_mode(var);
	if(ret < 0)
		return -EINVAL;
	else{
//		nusmartfb_set_par(info);
		printk("nusmartfb set on2 res:%dx%d\n",var->xres,var->yres);
		writel((5<<9), par->mmio + NUSMART_LCDC_CTRL);
		if(clk_set_rate(par->clk,nusmart_modes[ret].pixclk)<0)
			printk("on2 set clcd clock failed!%d\n",nusmart_modes[ret].pixclk);
		nusmart_clcd_var_decode(var,&regs);

		writel(regs.hintv, par->mmio + NUSMART_LCDC_HINTV);
		writel(regs.vintv, par->mmio + NUSMART_LCDC_VINTV);
		writel(regs.modulo, par->mmio + NUSMART_LCDC_MODULO);
		writel(regs.hvsize, par->mmio + NUSMART_LCDC_HVSIZE);

#ifdef	NUSMART_FB_ON2_SPEC
		if(par->on2flag == 2)
			regs.ctrl |= NUSMART_LCDC_RGBF;
#endif
		regs.ctrl |= (readl(par->mmio) & NUSMART_LCDC_TEST);
		writel(regs.ctrl, par->mmio + NUSMART_LCDC_CTRL);
		memcpy(&info->var,var,sizeof(struct fb_var_screeninfo));
	}
	
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
	
	/* Directcolor:
	 *   var->{color}.offset contains start of bitfield
	 *   var->{color}.length contains length of bitfield
	 *   {hardwarespecific} contains width of DAC
	 *   pseudo_palette[X] is programmed to (X << red.offset) |
	 *                                      (X << green.offset) |
	 *                                      (X << blue.offset)
	 *   RAMDAC[X] is programmed to (red, green, blue)
	 *   color depth = SUM(var->{color}.length)
	 *
	 * Pseudocolor:
	 *    var->{color}.offset is 0
	 *    var->{color}.length contains width of DAC or the number of unique
	 *                        colors available (color depth)
	 *    pseudo_palette is not used
	 *    RAMDAC[X] is programmed to (red, green, blue)
	 *    color depth = var->{color}.length
	 *
	 * Static pseudocolor:
	 *    same as Pseudocolor, but the RAMDAC is not programmed (read-only)
	 *
	 * Mono01/Mono10:
	 *    Has only 2 values, black on white or white on black (fg on bg),
	 *    var->{color}.offset is 0
	 *    white = (1 << var->{color}.length) - 1, black = 0
	 *    pseudo_palette is not used
	 *    RAMDAC does not exist
	 *    color depth is always 2
	 *
	 * Truecolor:
	 *    does not use RAMDAC (usually has 3 of them).
	 *    var->{color}.offset contains start of bitfield
	 *    var->{color}.length contains length of bitfield
	 *    pseudo_palette is programmed to (red << red.offset) |
	 *                                    (green << green.offset) |
	 *                                    (blue << blue.offset) |
	 *                                    (transp << transp.offset)
	 *    RAMDAC does not exist
	 *    color depth = SUM(var->{color}.length})
	 *
	 *  The color depth is used by fbcon for choosing the logo and also
	 *  for color palette transformation if color depth < 4
	 *
	 *  As can be seen from the above, the field bits_per_pixel is _NOT_
	 *  a criteria for describing the color visual.
	 *
	 *  A common mistake is assuming that bits_per_pixel <= 8 is pseudocolor,
	 *  and higher than that, true/directcolor.  This is incorrect, one needs
	 *  to look at the fix->visual.
	 *
	 *  Another common mistake is using bits_per_pixel to calculate the color
	 *  depth.  The bits_per_pixel field does not directly translate to color
	 *  depth. You have to compute for the color depth (using the color
	 *  bitfields) and fix->visual as seen above.
	 */
	
	/*
	 * This is the point where the color is converted to something that
	 * is acceptable by the hardware.
	 */
/*
#define CNVT_TOHW(val,width) ((((val)<<(width))+0x7FFF-(val))>>16)
	red = CNVT_TOHW(red, info->var.red.length);
	green = CNVT_TOHW(green, info->var.green.length);
	blue = CNVT_TOHW(blue, info->var.blue.length);
	transp = CNVT_TOHW(transp, info->var.transp.length);
#undef CNVT_TOHW
*/
	/*
	 * This is the point where the function feeds the color to the hardware
	 * palette after converting the colors to something acceptable by
	 * the hardware. Note, only FB_VISUAL_DIRECTCOLOR and
	 * FB_VISUAL_PSEUDOCOLOR visuals need to write to the hardware palette.
	 * If you have code that writes to the hardware CLUT, and it's not
	 * any of the above visuals, then you are doing something wrong.
	 */
	/*	
	if (info->fix.visual == FB_VISUAL_DIRECTCOLOR ||
	    info->fix.visual == FB_VISUAL_PSEUDOCOLOR)
		hwcmap[regno] = red | green | blue |transp;	
	*/
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
	unsigned int * end = base + DISP_BASE/2; 
	unsigned int * dist = base + DISP_BASE;
	unsigned int ctrl;

	info->var.yoffset = var->yoffset;
	ctrace;
/* Add for LCDC test by weixing */
#ifdef  WAIT_FOR_VSYNC
	/*enable vsync interrupt*/
	ctrl = readl(par->mmio);
	ctrl |= NUSMART_LCDC_INTM_V;	
	writel(ctrl, par->mmio);
#endif	
#ifdef	NUSMART_FB_ON2_SPEC
	if(par->on2flag) {
		//info->var = *var;
	}
	else
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
	wait_for_completion_timeout(&par->vsync,HZ/30); 
	/*enable vsync interrupt*/
//	ctrl = readl(par->mmio);
//	ctrl |= NUSMART_LCDC_INTM_V;	
//	writel(ctrl, par->mmio);
#endif	
	/*
	 * Note that even if this function is fully functional, a setting of
	 * 0 in both xpanstep and ypanstep means that this function will never
	 * get called.
	 */
	
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

/* ------------ Accelerated Functions --------------------- */

/*
 * We provide our own functions if we have hardware acceleration
 * or non packed pixel format layouts. If we have no hardware 
 * acceleration, we can use a generic unaccelerated function. If using
 * a pack pixel format just use the functions in cfb_*.c. Each file 
 * has one of the three different accel functions we support.
 */

/**
 *      nusmartfb_fillrect - REQUIRED function. Can use generic routines if 
 *		 	 non acclerated hardware and packed pixel based.
 *			 Draws a rectangle on the screen.		
 *
 *      @info: frame buffer structure that represents a single frame buffer
 *	@region: The structure representing the rectangular region we 
 *		 wish to draw to.
 *
 *	This drawing operation places/removes a retangle on the screen 
 *	depending on the rastering operation with the value of color which
 *	is in the current color depth format.
 */
/*
void nusmartfb_fillrect(struct fb_info *p, const struct fb_fillrect *region)
{

 *	Meaning of struct fb_fillrect
 *
 *	@dx: The x and y corrdinates of the upper left hand corner of the 
 *	@dy: area we want to draw to. 
 *	@width: How wide the rectangle is we want to draw.
 *	@height: How tall the rectangle is we want to draw.
 *	@color:	The color to fill in the rectangle with. 
 *	@rop: The raster operation. We can draw the rectangle with a COPY
 *	      of XOR which provides erasing effect. 
 *
}
*/
/**
 *      nusmartfb_copyarea - REQUIRED function. Can use generic routines if
 *                       non acclerated hardware and packed pixel based.
 *                       Copies one area of the screen to another area.
 *
 *      @info: frame buffer structure that represents a single frame buffer
 *      @area: Structure providing the data to copy the framebuffer contents
 *	       from one region to another.
 *
 *      This drawing operation copies a rectangular area from one area of the
 *	screen to another area.
 */
/*
void nusmartfb_copyarea(struct fb_info *p, const struct fb_copyarea *area) 
{
 *
 *      @dx: The x and y coordinates of the upper left hand corner of the
 *	@dy: destination area on the screen.
 *      @width: How wide the rectangle is we want to copy.
 *      @height: How tall the rectangle is we want to copy.
 *      @sx: The x and y coordinates of the upper left hand corner of the
 *      @sy: source area on the screen.
 *
}
*/

/**
 *      nusmartfb_imageblit - REQUIRED function. Can use generic routines if
 *                        non acclerated hardware and packed pixel based.
 *                        Copies a image from system memory to the screen. 
 *
 *      @info: frame buffer structure that represents a single frame buffer
 *	@image:	structure defining the image.
 *
 *      This drawing operation draws a image on the screen. It can be a 
 *	mono image (needed for font handling) or a color image (needed for
 *	tux). 
 */

/*
void nusmartfb_imageblit(struct fb_info *p, const struct fb_image *image) 
{
 *
 *      @dx: The x and y coordinates of the upper left hand corner of the
 *	@dy: destination area to place the image on the screen.
 *      @width: How wide the image is we want to copy.
 *      @height: How tall the image is we want to copy.
 *      @fg_color: For mono bitmap images this is color data for     
 *      @bg_color: the foreground and background of the image to
 *		   write directly to the frmaebuffer.
 *	@depth:	How many bits represent a single pixel for this image.
 *	@data: The actual data used to construct the image on the display.
 *	@cmap: The colormap used for color images.   
 *

 *
 * The generic function, cfb_imageblit, expects that the bitmap scanlines are
 * padded to the next byte.  Most hardware accelerators may require padding to
 * the next u16 or the next u32.  If that is the case, the driver can specify
 * this by setting info->pixmap.scan_align = 2 or 4.  See a more
 * comprehensive description of the pixmap below.
 *
}
*/
/**
 *	nusmartfb_cursor - 	OPTIONAL. If your hardware lacks support
 *			for a cursor, leave this field NULL.
 *
 *      @info: frame buffer structure that represents a single frame buffer
 *	@cursor: structure defining the cursor to draw.
 *
 *      This operation is used to set or alter the properities of the
 *	cursor.
 *
 *	Returns negative errno on error, or zero on success.
 */
/*
int nusmartfb_cursor(struct fb_info *info, struct fb_cursor *cursor)
{
 *
 *      @set: 	Which fields we are altering in struct fb_cursor 
 *	@enable: Disable or enable the cursor 
 *      @rop: 	The bit operation we want to do. 
 *      @mask:  This is the cursor mask bitmap. 
 *      @dest:  A image of the area we are going to display the cursor.
 *		Used internally by the driver.	 
 *      @hot:	The hot spot. 
 *	@image:	The actual data for the cursor image.
 *
 *      NOTES ON FLAGS (cursor->set):
 *
 *      FB_CUR_SETIMAGE - the cursor image has changed (cursor->image.data)
 *      FB_CUR_SETPOS   - the cursor position has changed (cursor->image.dx|dy)
 *      FB_CUR_SETHOT   - the cursor hot spot has changed (cursor->hot.dx|dy)
 *      FB_CUR_SETCMAP  - the cursor colors has changed (cursor->fg_color|bg_color)
 *      FB_CUR_SETSHAPE - the cursor bitmask has changed (cursor->mask)
 *      FB_CUR_SETSIZE  - the cursor size has changed (cursor->width|height)
 *      FB_CUR_SETALL   - everything has changed
 *
 *      NOTES ON ROPs (cursor->rop, Raster Operation)
 *
 *      ROP_XOR         - cursor->image.data XOR cursor->mask
 *      ROP_COPY        - curosr->image.data AND cursor->mask
 *
 *      OTHER NOTES:
 *
 *      - fbcon only supports a 2-color cursor (cursor->image.depth = 1)
 *      - The fb_cursor structure, @cursor, _will_ always contain valid
 *        fields, whether any particular bitfields in cursor->set is set
 *        or not.
 *
}
*/
/**
 *	nusmartfb_rotate -  NOT a required function. If your hardware
 *			supports rotation the whole screen then 
 *			you would provide a hook for this. 
 *
 *      @info: frame buffer structure that represents a single frame buffer
 *	@angle: The angle we rotate the screen.   
 *
 *      This operation is used to set or alter the properities of the
 *	cursor.
 */
/*
void nusmartfb_rotate(struct fb_info *info, int angle)
{
* Will be deprecated *
}
*/
/**
 *	nusmartfb_sync - NOT a required function. Normally the accel engine 
 *		     for a graphics card take a specific amount of time.
 *		     Often we have to wait for the accelerator to finish
 *		     its operation before we can write to the framebuffer
 *		     so we can have consistent display output. 
 *
 *      @info: frame buffer structure that represents a single frame buffer
 *
 *      If the driver has implemented its own hardware-based drawing function,
 *      implementing this function is highly recommended.
 */
/*
int nusmartfb_sync(struct fb_info *info)
{
	return 0;
}
*/

static int nusmartfb_mmap(struct fb_info *info,
				struct vm_area_struct *vma)
{
	unsigned long len, off =vma->vm_pgoff << PAGE_SHIFT;
	int ret = -EINVAL;
	
	ctrace;
	len = info->fix.smem_len;
	
	if(off <= len && vma->vm_end - vma->vm_start <= len - off) {
#ifdef	FB_RESERV
	unsigned int video_mem_base = VIDEO_MEM_BASE;
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

static void pl330_callback(void *completion)
{
        complete(completion);
}


/*
 */
static int nusmartfb_ioctl(struct fb_info *info, unsigned int cmd,
			unsigned long arg)
{
	int ret = 0;
	int flag;
	struct nusmart_par * par = info->par;
	void __user *argp = (void __user *)arg;
	struct fb_var_screeninfo var;

#ifdef NUSMART_PL330
       struct pl330_memcpy pl330;
        dma_cap_mask_t mask;
       struct dma_chan *chan;
       struct dma_device *dev;
       struct dma_async_tx_descriptor *tx = NULL;
        enum dma_ctrl_flags     flags;
        struct completion cmp;
        unsigned long tmo = msecs_to_jiffies(3000);
        dma_cookie_t            cookie;

#endif

#ifdef NUSMART_PW_CTRL
	struct clk_status stat;
#endif
#ifdef NUSMART_FB_ON2_SPEC
	unsigned int ctrl;
	unsigned int on2show;
	struct on2_res on2res;
	memset(&on2res,0,sizeof(on2res));
#endif
#ifdef CONFIG_DRM_MALI
	ump_dd_physical_block ump_memory_description;
	ump_secure_id secure_id;
#endif
	ctrace;
	
	switch(cmd) {
#ifdef  WAIT_FOR_VSYNC
	case FBIO_WAITFORVSYNC:
	/* Modify for LCDC test by weixing */
//		if(wait_for_completion_timeout(&par->vsync,HZ/30) > 0)
			return 0;
//		else
//			return -EAGAIN;
		break;
#endif
#ifdef NUSMART_PL330
               case FB_PL330_MEMCPY:
                       if (copy_from_user(&pl330, argp, sizeof(pl330)))
                               return -EFAULT;

                       dma_cap_zero(mask);
                       dma_cap_set(DMA_MEMCPY, mask);

                       chan = dma_request_channel(mask, NULL, NULL);
                       if (chan) {
                       } else
                               break; /* no more channels available */

                       dev = chan->device;

                       flags = DMA_CTRL_ACK | DMA_PREP_INTERRUPT
                               | DMA_COMPL_SKIP_DEST_UNMAP | DMA_COMPL_SRC_UNMAP_SINGLE | DMA_COMPL_SKIP_SRC_UNMAP;

                        tx = dev->device_prep_dma_memcpy(chan,
                                                         pl330.dst,
                                                         pl330.src, pl330.len,
                                                         flags);

                       init_completion(&cmp);
                       tx->callback = pl330_callback;
                       tx->callback_param = &cmp;
                       cookie = tx->tx_submit(tx);

                       dma_async_issue_pending(chan);
                       tmo = wait_for_completion_timeout(&cmp, tmo);
                       dma_async_is_tx_complete(chan, cookie, NULL, NULL);
                       //release
                       dma_release_channel(chan);

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
#ifdef NUSMART_FB_ON2_SPEC
	case FBIOGET_FBON2MODE:
		if (copy_to_user(argp, &par->on2flag, sizeof(int))) 
			return -EFAULT;
		break;
   case FB_LOG_STAGEFRIGHT:
                printk("There is a video frame lost!\n");
                break;
    case FBIOGET_FBON2ADDR:
        if (copy_to_user(argp, &par->on2alpha, sizeof(struct on2_alpha)))
            return -EFAULT;
        break;
    case FBIOSET_FBON2ADDR:
        if (copy_from_user(&par->on2alpha,argp, sizeof(struct on2_alpha)))
			return -EFAULT;
		break;

	case FBIOPUT_FBON2MODE:
		if (copy_from_user(&flag, argp, sizeof(flag)))
			return -EFAULT;
		printk("Set FBON2MODE on2flag:%d\n", flag);
		if (flag == 0) {
			printk("nusmartfb exit on2 mode.\n");
			par->on2flag = flag;	
			writel(0, par->mmio+NUSMART_LCDC_MODULO);
			reset_on2_res(info);
		}else
		if (flag == 1) {
			printk("nusmartfb enter on2 rgb15 mode.\n");
			par->on2flag = flag;	
	//		writel(1280-info->var.xres, par->mmio+NUSMART_LCDC_MODULO);
		}else
		if (flag == 2) {
			ctrl = readl(par->mmio);
			printk("nusmartfb enter on2 rgb24 mode.(ctrl:0x%x)\n",ctrl);
			par->on2flag = flag;	
		}else
		if (flag == 3) {
			printk("nusmartfb enter test mode.\n");
			ctrl = readl(par->mmio);
			ctrl |= NUSMART_LCDC_TEST;	
			writel(ctrl, par->mmio);
		}else
		if (flag == 4) {
			printk("nusmartfb exit test mode.\n");
			ctrl = readl(par->mmio);
			ctrl &= ~NUSMART_LCDC_TEST;	
			writel(ctrl, par->mmio);
		}
		break;
	case FBIOGET_FBON2BUF:
		if (copy_to_user(argp, &par->on2buf, sizeof(struct on2_buf)))
			return -EFAULT;
		break;
	case FBIOPUT_FBON2RES:
		if (copy_from_user(&on2res, argp, sizeof(struct on2_res)))
			return -EFAULT;
		ret = set_on2_res(info,&on2res);
		break;
	case FBIOPUT_FBON2SHOW:
		if (copy_from_user(&on2show, argp, sizeof(on2show)))
			return -EFAULT;
	/*	printk("nusmartfb show on2 buffer %d.0x%x\n",on2show,on2show);*/
		if (on2show >= 0 && on2show < ON2BUF_SIZE) {
		/*	on2_show(par->on2buf.buf[flag].pptr, par->mmio);*/
			writel(par->on2buf.buf[flag].pptr, par->mmio+NUSMART_LCDC_PRIPTR);
		}else
		/*if(on2show >= VIDEO_MEM_BASE && on2show < (VIDEO_MEM_BASE+VIDEO_MEM_SIZE)){*/
		if(on2show >= 0x80000000 && on2show <= 0xffffffff){
		/*	on2_show(on2show, par->mmio);*/
			writel(on2show, par->mmio+NUSMART_LCDC_PRIPTR);
		}else
			ret = -ENOTTY;

		break;
#endif
#ifdef CONFIG_DRM_MALI
	case GET_UMP_SECURE_ID:
		if ( !par->ump_wrapped_buffer ) {
			/* map framebuffer to ump memory area */
			ump_memory_description.addr = info->fix.smem_start;
			ump_memory_description.size = info->fix.smem_len;
			par->ump_wrapped_buffer = ump_dd_handle_create_from_phys_blocks( &ump_memory_description, 1);
		}
		secure_id = ump_dd_secure_id_get( par->ump_wrapped_buffer );
//		return put_user(&secure_id, argp);
		if (copy_to_user(argp, &secure_id, sizeof(ump_secure_id)))
			return -EFAULT;
		break;
#endif

	default:
		ret = -ENOTTY;
	}
	return ret;
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
static int video_mem_alloc(struct fb_info *info)
{
	int maxsize = VIDEO_MEM_SIZE;/*1024*600*8;*/
	ctrace;
		
	info->fix.smem_len = maxsize;
#ifdef 	FB_RESERV 
	info->fix.smem_start = VIDEO_MEM_BASE;
	info->screen_base = ioremap(VIDEO_MEM_BASE, VIDEO_MEM_SIZE);

	printk(KERN_EMERG "VIDEO_MEM_BASE 0x%0lx VIDEO_MEM_SIZE 0x%0x\n", info->fix.smem_start, VIDEO_MEM_SIZE);
#else
	info->screen_base = dma_alloc_writecombine(info->device, info->fix.smem_len,
					 (dma_addr_t*)&info->fix.smem_start, GFP_KERNEL);
#endif	/*FB_RESERV*/
	if(!info->screen_base) {
		return -ENOMEM;
	}
	
	memset(info->screen_base, 0, info->fix.smem_len);
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
/*
	unsigned int tempdata;
	unsigned int i;
	unsigned int ctrl_read;
*/
//	void __iomem * gpioreg = __io_address(0x05130000); 	
//	writel(0x0,gpioreg+0x8);
//	writel(0xf,gpioreg+0x4);
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
		/*
		for(i=0;i<200;i++)
		{
			ctrl_read = readl(par->mmio);
			if(ctrl_read & 0x80000000)
				break;
		}
		*/
		/* modify for LCDC test by weixing */
		ctrl |= NUSMART_LCDC_INTM_V;
		writel(ctrl, par->mmio);
		#if 0
		udelay(4);
		tempdata=0;
		writel(0xf,gpioreg+0x0);
		writel(tempdata,par->mmio);
		udelay(18);
		/*
		for(i=0;i<564;i++)
		{
			ctrl_read = readl(par->mmio);
			if(ctrl_read & 0x80000000)
				break;
		}
		*/
		tempdata=ctrl|0x800|NUSMART_LCDC_INTM_V;
		writel(0x0,gpioreg+0x0);
		writel(tempdata,par->mmio);
		#endif
		/* modify for LCDC test by weixing */
	} else {
		printk("nusmart fb:interrupt detected with vsync not set.\n");
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

	/* unsigned int addr; */
	/*read status*/
	ctrl = readl(par->mmio); 

	/*disable interrupt*/
	ctrl &= ~NUSMART_LCDC_INTM_ERR;
	ctrl &= ~NUSMART_LCDC_ENABLE;
	writel(ctrl, par->mmio);

	if(ctrl & NUSMART_LCDC_ERRF) {
		/*handle lcdc error*/
		printk("nusmart fb:lcdc fifo underflow detected!\n");
		ctrl &= ~NUSMART_LCDC_ERRF;
		
		if(ctrl & NUSMART_LCDC_PTRF) {/*secondary*/
			/*set first*/	
			ctrl&=~NUSMART_LCDC_PTRF;
		} else {/*first*/
			/*set second*/	
			ctrl|=NUSMART_LCDC_PTRF;
		}
		/*addr=readl(par->mmio+NUSMART_LCDC_PRIPTR);
		writel(addr,par->mmio+NUSMART_LCDC_PRIPTR);
		addr=readl(par->mmio+NUSMART_LCDC_SECPTR);
		writel(addr,par->mmio+NUSMART_LCDC_SECPTR);*/
	}else
	{
		printk("nusmart fb:interrupt detected with no flag set.\n");
	}

	/*clear flag & enable interrupt*/
	ctrl |= NUSMART_LCDC_INTM_ERR;
	ctrl |= NUSMART_LCDC_ENABLE;
	writel(ctrl, par->mmio);

        return IRQ_HANDLED;
}

#ifdef CONFIG_EARLYSUSPEND
#define es_to_fbinfo(x) container_of(x, struct ns2816fb_early_suspend, es) 
struct ns2816fb_early_suspend
{
	struct fb_info * fb;
	struct early_suspend es;
};

static int nusmartfb_save(struct fb_info * info);
static int nusmartfb_restore(struct fb_info * info);

static void nusmartfb_early_resume(struct early_suspend * h)
{
	struct ns2816fb_early_suspend * nes = es_to_fbinfo(h);
	nusmartfb_restore(nes->fb);
}

static void nusmartfb_early_suspend(struct early_suspend * h)
{
	struct ns2816fb_early_suspend * nes = es_to_fbinfo(h);
	nusmartfb_save(nes->fb);
}

static struct ns2816fb_early_suspend early_suspend = {
	.es = {
		.level = EARLY_SUSPEND_LEVEL_DISABLE_FB,
		.suspend = nusmartfb_early_suspend,
		.resume = nusmartfb_early_resume,
	}
};

#endif

static int __devinit nusmartfb_probe(struct platform_device *pdev)
{
	struct fb_info *info;
	struct nusmart_par *par;
	struct device *device = &pdev->dev; 
	int cmap_len=256, retval = 0;	
	struct resource *res = NULL;
	struct fb_monspecs *specs = NULL;
	struct i2c_adapter *adapter = NULL;	
#ifdef CONFIG_FB_DDC
	unsigned char * edid = NULL;
	const struct fb_videomode * m = NULL;
	struct fb_videomode mode;
#endif
	struct fb_var_screeninfo stdvar;
	int	kernel_args = 0;
	int 	ddc_adapter = 0;
	struct lcdc_platform_data * platform_data = NULL;
	ctrace;
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
#ifdef CONFIG_DRM_MALI
	par->ump_wrapped_buffer=0;
#endif

	/*
	 *get ddc adapter info from platform data
	 */	
	platform_data = pdev->dev.platform_data;
	if(platform_data == NULL)
		err("get platform data failed.");
	else
		ddc_adapter = platform_data->ddc_adapter;
#ifdef NUSMART_FB_ON2_SPEC
	par->on2flag = 0;
	memset(&par->on2buf, 0, sizeof(struct on2_buf));
	init_on2buf(&par->on2buf);
#endif
	specs = &info->monspecs;
	/*
	 *init clk	
	*/
	par->clk = clk_get(&pdev->dev, LCDC_NAME/*"nusmartfb"*/);
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
	info->fix = nusmartfb_fix; /* this will be the only time nusmartfb_fix will be
	    		    * used, so mark it as __devinitdata
	    		    */
	retval = video_mem_alloc(info);

	if(retval < 0) {
		printk("alloc video mem failed!\n");	
		goto free_fbinfo;
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if(!res) {
		printk("get_resource failed!(mem)\n");
		retval = -ENXIO;
		goto free_video_mem;
	}
	info->fix.mmio_start = res->start;	
	info->fix.mmio_len = res->end - res->start +1;
	
	par->mmio = ioremap_nocache(info->fix.mmio_start,info->fix.mmio_len);
	if(!par->mmio) {
		printk("ioremap failed!\n");
		retval = -ENOMEM;
		goto free_video_mem;	
	}

	res = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
	if(!res) {
		printk("get_resource failed!(irq)\n");
		retval = -ENXIO;
		goto unmap_mmio;
	}
	par->irq = res->start;	
	retval = request_irq(par->irq, nusmart_lcdc_irq, IRQF_SHARED, LCDC_NAME, par);
	if(retval < 0) {
		printk("request_irq failed!\n");
		goto unmap_mmio;
	}

#ifdef  WAIT_FOR_VSYNC
	par->irq_vert = res->start+1;	
	retval = request_irq(par->irq_vert, nusmart_lcdc_vert_irq, IRQF_DISABLED|IRQF_SHARED, "nusmartfb_vert", par);
	if(retval < 0) {
		printk("request nusmartfb vert irq failed!\n");
		goto unmap_mmio;
	}
	init_completion(&par->vsync);
#endif
/*
	res = platform_get_resource(pdev, IORESOURCE_IRQ, 1);
	if(!res) {
		printk("get_resource failed!(irq)\n");
		retval = -ENXIO;
		goto unmap_mmio;
	}
	par->irq_vert = res->start;	
	retval = request_irq(par->irq_vert, nusmart_lcdc_vert_irq, IRQF_DISABLED|IRQF_SHARED, "nusmartfb_vert", par);
	if(retval < 0) {
		printk("request nusmartfb vert irq failed!\n");
		goto unmap_mmio;
	}
*/

	info->pseudo_palette = pseudo_palette; /* The pseudopalette is an
	    				    * 16-member array
	    				    */
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
	    mode_option = "1024x768-16@60";
	    info->var.bits_per_pixel = 16;
	}
	else if (!strcmp(mode_option, DDC_ARGS_16)) {
	    mode_option = "1024x768-16@60";
	    info->var.bits_per_pixel = 16;
	}
	else if (!strcmp(mode_option, DDC_ARGS_32)) {
	    mode_option = "1024x768-32@60";
	    info->var.bits_per_pixel = 32;
	}
	else {
	    kernel_args = 1;
	    info->var.bits_per_pixel = 16;
	}
	 	
	//retval = fb_find_mode(&stdvar, info, mode_option, vesa_modes, VESA_MODEDB_SIZE,NULL, 16);
	retval = fb_find_mode(&stdvar, info, mode_option, NULL, 0,NULL, 16);
	
	if (!retval || retval == 4) {
		printk("find mode failed!\n");
		retval = -EINVAL;	
		goto free_irq;
	}		
	/*add mode list*/
	//fb_videomode_to_modelist(vesa_modes,VESA_MODEDB_SIZE,&info->modelist);
	if(kernel_args == 0) {
			adapter = i2c_get_adapter(ddc_adapter);
	}
	if(adapter) {
#ifdef CONFIG_FB_DDC
		edid=fb_ddc_read_i2c(adapter);
//		edid=fb_ddc_read(adapter);
		if(edid) {
			fb_edid_to_monspecs(edid, specs);
			kfree(edid);
			if(specs->modedb) {
				fb_videomode_to_modelist(specs->modedb,
						specs->modedb_len,
						&info->modelist);
				printk(KERN_ERR "fb%d: %s get Mode list ok!\n", info->node,
						info->fix.id);
				m = fb_find_best_display(specs, &info->modelist);
				if (m) {
		/*			printk("xres %d,yres %d,pixclk %d\n"
						"leftm %d,rm %d,um %d,lowm %d"
						"hsync %d,vsync,%d",
						m->xres,m->yres,m->pixclock,
						m->left_margin,m->right_margin,m->upper_margin,m->lower_margin,
						m->hsync_len,m->vsync_len);
		*/
					fb_videomode_to_var(&info->var, m);
					if(info->var.xres==1024&&info->var.yres==768&&info->var.pixclock<10309) {
						//modify for lp097x02-sla3
						info->var.left_margin = 480;
						info->var.right_margin = 260;
						info->var.hsync_len = 320;
					}
					/* fill all other info->var's fields */
					if (nusmartfb_check_var(&info->var, info) < 0) {
						fb_var_to_videomode(&mode, &stdvar);
						m = fb_find_nearest_mode(&mode, &info->modelist);
						if(m) {
							fb_videomode_to_var(&info->var, m);
							if (nusmartfb_check_var(&info->var, info) < 0) {
								info->var = stdvar;
								printk(KERN_ERR "fb%d: %s get near mode of display failed!\n", info->node,
										info->fix.id);
	
							}
						} else {
							info->var = stdvar;
						}
						printk(KERN_ERR "fb%d: %s check best display failed!\n", info->node,
								info->fix.id);
					}
	
				} else {
					printk(KERN_ERR "fb%d: %s get best display failed!\n", info->node,
							info->fix.id);
					info->var = stdvar;
				}
				
			} else {
				printk(KERN_ERR "fb%d: %s get Mode Database failed!\n", info->node,
						info->fix.id);
				info->var = stdvar;
			}
		} else {
			printk(KERN_ERR "fb%d: %s get edid failed!\n", info->node,
					info->fix.id);
			info->var = stdvar;
		}
		i2c_put_adapter(adapter);
#endif
	} else {
		if(kernel_args == 0) {
			printk(KERN_ERR "fb%d: %s get i2c adapter failed!\n", info->node,
					info->fix.id);
		}
		info->var = stdvar;
	}
	
	printk("mode option accept:%dx%d\n",info->var.xres,info->var.yres);
	printk("mode clk:%dMHz",(1000000/info->var.pixclock));
	printk("mode left:%d right:%d hsync:%d\n"
			"upper:%d lower:%d vsync:%d\n"
			"sync:%d\n",info->var.left_margin,info->var.right_margin,info->var.hsync_len,
			info->var.upper_margin,info->var.lower_margin,info->var.vsync_len,
			info->var.sync);

	/* This has to been done !!! */	
	fb_alloc_cmap(&info->cmap, cmap_len, 0);

	/* 
	 * The following is done in the case of having hardware with a static 
	 * mode. If we are setting the mode ourselves we don't call this. 
	 */	
	/*info->var = nusmartfb_var;*/
	
	/*
	 * For drivers that can...
	 */
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
	early_suspend.fb = info;
	register_early_suspend(&early_suspend.es);
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
static int __devexit nusmartfb_remove(struct platform_device *pdev)
{
	struct fb_info *info = platform_get_drvdata(pdev);
	struct nusmart_par * par = NULL;
	/* or platform_get_drvdata(pdev); */

	ctrace;
	if (info) {
		unregister_framebuffer(info);

		iounmap(par->mmio);
		free_irq(par->irq, par);

		if(info->cmap.len>0)
			fb_dealloc_cmap(&info->cmap);
		/* free clk*/
		par = info->par;	
		clk_put(par->clk);
		par->clk = ERR_PTR(-ENOENT);
		/*free mem*/
#ifdef NUSMART_FB_ON2_SPEC
		release_on2buf(&par->on2buf);
#endif
		if(info->monspecs.modedb)
			fb_destroy_modedb(info->monspecs.modedb);
		video_mem_free(info);
		framebuffer_release(info);
	}
	return 0;
}

#include <linux/platform_device.h>
/* for platform devices */

#ifdef CONFIG_PM
static int nusmartfb_save(struct fb_info * info)
{
	struct nusmart_par *par = info->par;
	int idx = REG_NUM - 1;
	for(;idx >= 0;idx --)
		par->regsave[idx] = ((unsigned int *)(par->mmio))[idx];
	ctrace;
	nusmart_clcd_disable(info);
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
 	nusmart_clcd_enable(info, par->ctrl);
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
	nusmartfb_restore(info);
	/* resume here */
	return 0;
}
#else
#define nusmartfb_suspend NULL
#define nusmartfb_resume NULL
#endif /* CONFIG_PM */

static struct platform_driver nusmartfb_driver = {
	.probe = nusmartfb_probe,
	.remove = nusmartfb_remove,
	.suspend = nusmartfb_suspend, /* optional but recommended */
	.resume = nusmartfb_resume,   /* optional but recommended */
	.driver = {
		.name = LCDC_NAME,/*"nusmartfb",*/
		.owner = THIS_MODULE,
	},
};

/*static struct platform_device *nusmartfb_device;*/

#ifndef MODULE
    /*
     *  Setup
     */
/*
 * Only necessary if your driver takes special options,
 * otherwise we fall back on the generic fb_setup().
 */
static int __init nusmartfb_setup(char *options)
{
    /* Parse user speficied options (`video=nusmartfb:') */
	ctrace;
	mode_option = options;
	printk("nusmartfb options:%s\n",mode_option);
	return 0;
}
#endif /* MODULE */

//#define NUSMART_LCDC_BASE	0x050d0000

static int __init nusmartfb_init(void)
{
	int ret;
	/*
	 *  For kernel boot options (in 'video=LCDC_NAME:<options>' format)
	 */
#ifndef MODULE
	char *option = NULL;
	ctrace;
	if (fb_get_options(LCDC_NAME/*"nusmartfb"*/, &option))
		return -ENODEV;
	nusmartfb_setup(option);
	if(!option) {
		err("%s:get option failed",LCDC_NAME);
		return -EINVAL;
	}
#endif
	ret = platform_driver_register(&nusmartfb_driver);

	if (!ret) {
	}
	else
		dbg("nusmartfb_driver register failed!");
	return ret;
}

static void __exit nusmartfb_exit(void)
{
	ctrace;
	platform_driver_unregister(&nusmartfb_driver);
}

/* ------------------------------------------------------------------------- */


    /*
     *  Modularization
     */

module_init(nusmartfb_init);
module_exit(nusmartfb_exit);

MODULE_LICENSE("GPL");
