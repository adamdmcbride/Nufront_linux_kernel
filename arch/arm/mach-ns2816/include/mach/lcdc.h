#ifndef _MACH_NS2816_LCDC_H_
#define _MACH_NS2816_LCDC_H_
/*extend this if needed*/
#if 0
struct lcdc_platform_ops
{
	int (*init)(void);
	int (*check_var)(struct fb_var_screeninfo * var);
	int (*set_var)(struct fb_var_screeninfo * var);
	int (*set_polarity)(unsigned int polarity);
	int (*disable)(void);
	int (*enable)(void);
};
#endif
struct lcdc_platform_data
{
	int ddc_adapter;
/*	struct lcdc_platform_pos * ops;*/
};
#endif

