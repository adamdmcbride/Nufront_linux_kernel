/*
 * Base driver for CH7033 HDMI chip (I2C)
 *
 * Copyright (C) 2011 NUFRONT INC.
 *
 */

#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/timer.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/gpio.h>
#include <linux/fb.h>
#include <linux/earlysuspend.h>
#include <asm/div64.h>
#include <video/edid.h>
#include "edid.h"

#include "nusmart-hdmi.h"
#include "firmware.h"
#include "hdmi_para.h"

struct hdmi_par
{
        struct fb_info  *fb;
        void __iomem    *mmio;
        u32             ctrl;
        int             fbconv;
        int             mvflag;
        struct clk *    clk;
        int             irq;
        int             irq_vert;
};

struct hdmi_ch7033_para
{
	unsigned int ui_xres;			// X resolution
	unsigned int ui_yres;			// Y resolution
	unsigned int ui_hz;			// Refresh frequency
	unsigned int ui_vmode;			// If interlaced
	unsigned int* puc_table;		// Point to the CH7033B display parameter table
};

struct hdmi_lcdc_para 
{
	unsigned int ui_lcdctype;		// LCD Type: NuLCDC or PL111
	unsigned int ui_restype;		// resolution type
	unsigned int ui_len;			// length of CH7033 display parameter table
	unsigned int* pui_table;		// Point to the CH7033 display parameter table
};

unsigned int displayCH7033(unsigned int* pui_disTable, unsigned int ui_lenTable);


static void hdmi_suspend(struct early_suspend* h);
static void hdmi_resume(struct early_suspend* h);


#define PL111_LCDC				1
#define NULCDC_LCDC				2
#define RES_1024X600_720P			1
#define RES_1024X768_720P			2
#define RES_1280X720_720P			3
#define RES_1920X1080_1080P			4

#define LCD_PL111			"ns2816plfb"
#define LCD_NULCDC			"ns2816fb"

#define FIRMWARE_UPDATE			0

#define MODULE_NAME			"nusmart-hdmi"

#define PLUG_IN				1
#define PLUG_OUT			0
#define EDID_16READ			16
#define EDID_BLOCKLEN			128
#define EDID_LEN			256
#define DEFAULT_DISPLAYOFFSET		2

#define HDMI_ERROR			1
#define HDMI_OK				0

static struct hdmi_lcdc_para hdmi_ch7033_nulcdc[] =
{
	{NULCDC_LCDC, RES_1280X720_720P, (sizeof(CH7033_Nulcdc_1280x720_720p) >> 2), (unsigned int*)CH7033_Nulcdc_1280x720_720p},
	{NULCDC_LCDC, RES_1024X768_720P, (sizeof(CH7033_Nulcdc_1024x768_720p) >> 2), (unsigned int*)CH7033_Nulcdc_1024x768_720p},
	{NULCDC_LCDC, RES_1024X600_720P, (sizeof(CH7033_Nulcdc_1024x600_720p) >> 2), (unsigned int*)CH7033_Nulcdc_1024x600_720p},
	{NULCDC_LCDC, RES_1920X1080_1080P, (sizeof(CH7033_Nulcdc_1920x1080_1080p) >> 2), (unsigned int*)CH7033_Nulcdc_1920x1080_1080p},
};

static struct hdmi_lcdc_para hdmi_ch7033_pl111[] =
{
	{},
};


static struct hdmi_ch7033_para hdmi_ch7033_table[7] = 
{
	{720, 576, 50, FB_VMODE_NONINTERLACED, (unsigned int*)CH7033_576p50},
	{1280, 720, 50, FB_VMODE_NONINTERLACED, (unsigned int*)CH7033_720p50},
	{1280, 720, 60, FB_VMODE_NONINTERLACED, (unsigned int*)CH7033_720p60},
	{1920, 1080, 50, FB_VMODE_NONINTERLACED, (unsigned int*)CH7033_1080p50},
	{1920, 1080, 60, FB_VMODE_NONINTERLACED, (unsigned int*)CH7033_1080p60},
	{1920, 1080, 50, FB_VMODE_INTERLACED, (unsigned int*)CH7033_1080i50},
	{1920, 1080, 60, FB_VMODE_INTERLACED, (unsigned int*)CH7033_1080i60},
};

static struct i2c_client *hdmi_client = NULL;
static struct work_struct kstr_ws;
static struct workqueue_struct* pkstr_wq;

static struct timer_list hdmi_timer;

static unsigned char g_uc_status;
static unsigned int g_ui_dislen;
static unsigned int* g_pui_disTable;

extern unsigned int InitializeCH7033();

static const struct i2c_device_id hdmi_idtable[] = {
    { MODULE_NAME,   0},
    { }
};

MODULE_DEVICE_TABLE(i2c, hdmi_idtable);

unsigned char es_map[16] = { 
0x26,0x27,0x42,0x43,0x44,0x45,0x46,0x47, 0x6A,0x51,0x52,0x53,0x57,0x58,0x59,0x5A 
};

/**
 */
int hdmi_i2c_write(u8 reg, u8 val)
{
        int retval;
        if ((retval = (i2c_smbus_write_byte_data(hdmi_client, reg, val))) < 0)
        {
                printk(KERN_EMERG "FAILED: i2c_smbus_write_byte_data\n");
                return (retval);
        }

        return 0;
}

static u8 hdmi_i2c_read(u8 reg)
{
	u8 val;

	val = i2c_smbus_read_byte_data(hdmi_client, reg);

	//HDMI_DEBUG("read reg 0x%x = %x", reg, val);
	return val;
}


#ifdef FIRMWARE_UPDATE
static int I2CWrite(u8 reg_, u8 val_)
{
	return (hdmi_i2c_write(reg_, val_));
}

static u8 I2CRead(u8 reg_)
{
	return (hdmi_i2c_read(reg_));
}

static int firmware_update(void)
{
	int cnt;
        char data;

        // Update Firmware for CH7033B
        I2CWrite(0x03, 0x04);
        data = I2CRead(0x52);
        data |= 0x20;           // make sure PGM=1
        I2CWrite( 0x52, data & 0xFB); // mcu reset
        I2CWrite( 0x5b, 0x9e);
        I2CWrite( 0x5b, 0xb3);
        I2CWrite( 0x03, 0x04);
        I2CWrite( 0x03, 0x07);
        for (cnt = 0; cnt < FIRMWARE_LEN; cnt ++) {
                I2CWrite( 0x07, uc_firmware[cnt]);
        }
        I2CWrite( 0x03, 0x04); // page 4;
        data = I2CRead(0x52);
        I2CWrite( 0x52, data | 0x24); // mcu reset

        msleep(10);
        I2CWrite(0x03, 0x00);
        I2CWrite(0x4F, 0x5F);
        data = I2CRead(0x4F);
        if (data == 0x1F) {
                printk("Firmware update OK.\r\n");
        }
        else {
                printk("Firmware update Error.\r\n");
        }

	return 0;
}
#endif


static int hdmi_event_notify(struct notifier_block *self, unsigned long action, void *data)
{
        struct fb_event *event = data;
	int i_blank;
        switch(action) {
        case FB_EVENT_BLANK:
		i_blank = *(int*)event->data;
		printk("hdmi event notify: FB_EVENT_BLANK. blank data is 0x%x.\r\n", i_blank);
		if (i_blank) {
                	hdmi_suspend(NULL);
		}
		else {
			hdmi_resume(NULL);
		}
                break;
        }

        return 0;
}

static struct notifier_block hdmi_event_notifier = {
        .notifier_call  = hdmi_event_notify,
};




static unsigned int hdmi_ReadEDID(unsigned char* puc_EdidBuf)
{
	unsigned int ui_ret, ui_timeout, ui_cnt;
	unsigned int ui_blocknum, ui_offset, ui_16cnt;
	unsigned char uc_regdata;
	unsigned char* puc_edidptr;


	ui_ret = HDMI_ERROR;
	ui_timeout = 10;
	ui_cnt = 0;
	puc_edidptr = puc_EdidBuf;
	do {
		hdmi_i2c_write(0x03, 0x00);			
        	// wait untill REG4F[6] equals 0, or timeout
        	do {
        	        uc_regdata = hdmi_i2c_read(0x4F);
        	        if ((uc_regdata & 0x40) == 0x0) {
        	                ui_timeout = 20;
        	        }
        	        else {
        	                msleep(100);
        	        }
        	} while ((ui_timeout <= 10) && (-- ui_timeout));

		if (ui_timeout == 20) {
			ui_blocknum = (ui_cnt >> 7);
			ui_offset = (ui_cnt & 0x7F);
			uc_regdata = (unsigned char)(((ui_blocknum << 7) + ui_offset) >> 4);
			//printk("block num is %d, offset is %d, reg data is %d.\r\n", ui_blocknum, ui_offset, uc_regdata);
			hdmi_i2c_write(0x03, 0x00);
			hdmi_i2c_write(0x50, uc_regdata);
			// Write for EDID request
			hdmi_i2c_write(0x4F, 0x41);
			// wait for EDID data ready
			ui_timeout = 5;
			ui_16cnt = 0;
			while ((-- ui_timeout) && (ui_16cnt == 0)) {
				hdmi_i2c_write(0x03, 0x00);
				//if ((0x7F & hdmi_i2c_read(0x4F)) == 0x01) {
				if ((hdmi_i2c_read(0x4F) & 0x40) == 0x0) {
					// EDID data is ready
					// Read EDID data, 16 bytes each time
					hdmi_i2c_write(0x03, 0x01);
					do {
						*puc_edidptr ++ = hdmi_i2c_read(es_map[ui_16cnt ++]);
					} while (ui_16cnt < EDID_16READ);
					ui_cnt += EDID_16READ;
				}
				else {
					msleep(100);
				}
			}
		}
	} while ((ui_timeout) && (ui_cnt < EDID_LEN));

	if (ui_cnt == EDID_LEN) {
		ui_ret = HDMI_OK;
	}
	printk("End of hdmi_ReadEDID function.\r\n");

	return ui_ret;
}

static unsigned int hdmi_ParseEDID(unsigned char* puc_EdidBuf, unsigned char* puc_TableOffset)
{
	unsigned int ui_ret, ui_cnt;
	struct fb_info* pfb_info;
	struct fb_monspecs* pfb_specs;
	struct fb_videomode* pfb_videomode;
	struct hdmi_ch7033_para* p7033_para;

	ui_ret = HDMI_ERROR;
	pfb_info = framebuffer_alloc(sizeof(struct hdmi_par), NULL);
	pfb_specs = &(pfb_info->monspecs);
	pfb_videomode = NULL;
	// Point to the default offset of hdmi_ch7033_table. Here, it implies to 720P by default.
	*puc_TableOffset = DEFAULT_DISPLAYOFFSET;
	printk("In hdmi_ParseEDID function.\r\n");
	fb_edid_to_monspecs(puc_EdidBuf, pfb_specs);
	if (pfb_specs->modedb) {
		fb_videomode_to_modelist(pfb_specs->modedb, pfb_specs->modedb_len, &(pfb_info->modelist));
		pfb_videomode = fb_find_best_display(pfb_specs, &(pfb_info->modelist));
		if (pfb_videomode) {
			printk("xres %d,yres %d,pixclk %d\n"
				"leftm %d,rm %d,um %d,lowm %d"
				"hsync %d,vsync,%d, vmode,%d",
				pfb_videomode->xres, pfb_videomode->yres, pfb_videomode->pixclock,
				pfb_videomode->left_margin, pfb_videomode->right_margin, pfb_videomode->upper_margin,
				pfb_videomode->lower_margin, pfb_videomode->hsync_len, pfb_videomode->vsync_len,
				pfb_videomode->vmode);

			printk("EDID the last two bytes of first 128 Bytes: 0x%x, 0x%x.\r\n", *(puc_EdidBuf + 126), *(puc_EdidBuf + 127));

#if 1
			ui_ret = HDMI_OK;
			ui_cnt = sizeof(hdmi_ch7033_table);
			p7033_para = hdmi_ch7033_table;
			printk("To scan the hdmi_7033_table to find out the parameters which can match the best display.\r\n");
			// Try to find a mode that can match the best display parameters of the display terminal
			do {
				// Just check X/Y resolution and vmode here.
				if ((p7033_para->ui_xres == pfb_videomode->xres) && (p7033_para->ui_yres == pfb_videomode->yres) && 
					(p7033_para->ui_vmode == pfb_videomode->vmode)) {
					*puc_TableOffset = sizeof(hdmi_ch7033_table) - ui_cnt;
				}
				else {
					++ p7033_para;
				}
				printk("xres %d,yres %d,Hz %d,vmode,%d\r\n",
					p7033_para->ui_xres, p7033_para->ui_yres, p7033_para->ui_hz, p7033_para->ui_vmode);
			} while ((-- ui_cnt) && (*puc_TableOffset != DEFAULT_DISPLAYOFFSET));
			
			// Try to match other display mode supported by the display terminal
			if (*puc_TableOffset == DEFAULT_DISPLAYOFFSET) {
				
			}
#endif


		}		
	}
	
	// Release memory allocated above
	if (pfb_specs->modedb) {
		fb_destroy_modedb(pfb_specs->modedb);
	}
	if (pfb_info) {
		framebuffer_release(pfb_info);
	}


	printk("End of hdmi_ParseEDID function.\r\n");



	return ui_ret;	
}

static void hdmi_workhandler(struct work_struct* work_data)
{
	unsigned int ui_ret;
	unsigned char uc_val, uc_stat, uc_tbloffset;
	unsigned char edid_data[EDID_LEN];

	// Read plugin/out status
	hdmi_i2c_write(0x03, 0x03);	
	uc_val = hdmi_i2c_read(0x25);
	if ((uc_val >> 4) & 0x1) {
		uc_stat = PLUG_IN;
	}
	else {
		uc_stat = PLUG_OUT;
	}

	printk("hdmi_workhandler.g_uc_status is 0x%x, uc_stat is 0x%x.\r\n", g_uc_status, uc_stat);

#if 1
	// Check if plugin/out status is changed
	if (g_uc_status != uc_stat) {
		if (uc_stat == PLUG_IN) {
			printk("HDMI Plugin. g_uc_status is 0x%x. uc_stat is 0x%x.\r\n", g_uc_status, uc_stat);
			// Read EDID info
			ui_ret = hdmi_ReadEDID(edid_data);
			if (ui_ret == HDMI_OK) {
				// Parse display parameters from EDID info
				hdmi_ParseEDID(edid_data, &uc_tbloffset);
				
			}
			else {
				// Use default display parameters
			}
		}
		else {
			printk("HDMI Plugout. g_uc_status is 0x%x. uc_stat is 0x%x.\r\n", g_uc_status, uc_stat);
		}
		g_uc_status = uc_stat;

	}
#endif

	hdmi_timer.expires = jiffies + HZ;		//1000 * HZ / 500;
	add_timer(&hdmi_timer);
}

static int hdmi_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{

	hdmi_client = client;

	//init hdmi
	if(!InitializeCH7033())
	{
		printk(KERN_EMERG "init hdmi error!exit...\n");
		return -1;
	}

	return 0;

}

static int hdmi_i2c_remove(struct i2c_client *client)
{

	//free_irq(client->irq, MODULE_NAME);
	return 0;
}



void hdmi_timerhandler(unsigned int iVec)
{
	queue_work(pkstr_wq, &kstr_ws);
}


static unsigned char g_p07;
static unsigned char g_p09;
static unsigned char g_p0A;

static void hdmi_suspend(struct early_suspend* h)
{
	printk("hdmi suspend\r\n");
        hdmi_i2c_write(0x03, 0x04);
        hdmi_i2c_write(0x52, 0x01);
        hdmi_i2c_write(0x52, 0x03);

        hdmi_i2c_write(0x03, 0x00);
        g_p07 = hdmi_i2c_read(0x07);
        g_p09 = hdmi_i2c_read(0x09);
        g_p0A = hdmi_i2c_read(0x0A);
        hdmi_i2c_write(0x07, 0x1F);
        hdmi_i2c_write(0x09, 0xFF);
        hdmi_i2c_write(0x0A, g_p0A | 0x80);
        hdmi_i2c_write(0x0A, 0xFF);
}


static void hdmi_resume(struct early_suspend* h)
{
	printk("hdmi resume\r\n");
        hdmi_i2c_write(0x03, 0x00);
        hdmi_i2c_write(0x0A, g_p0A);
        hdmi_i2c_write(0x07, g_p07);
        hdmi_i2c_write(0x09, g_p09);

	displayCH7033(g_pui_disTable, g_ui_dislen);
}

static struct i2c_driver hdmi_i2c_driver = {
	.driver = {
		.name   = MODULE_NAME,
		.owner  = THIS_MODULE,
	},
	.probe      = hdmi_i2c_probe,
	.remove     = __devexit_p(hdmi_i2c_remove),
	.id_table   = hdmi_idtable,
};


#ifdef CONFIG_HAS_EARLYSUSPEND
static struct early_suspend hdmi_early_suspend = {
	.level = EARLY_SUSPEND_LEVEL_DISABLE_FB - 1,
	.suspend = hdmi_suspend,
	.resume = hdmi_resume,
};
#endif

unsigned int displayCH7033(unsigned int* pui_disTable, unsigned int ui_lenTable)
{
	ch_uint32 i, val_t, deviceid, revisionid;
	ch_uint32 hinc_reg, hinca_reg, hincb_reg;
	ch_uint32 vinc_reg, vinca_reg, vincb_reg;
	ch_uint32 hdinc_reg, hdinca_reg, hdincb_reg;
	unsigned int* pui_ptr;
	unsigned int ui_len;
	uint64_t tmp = 0;
	

	hdmi_i2c_write(0x03, 0x01);	// page 2
	hdmi_i2c_write(0x39, 0x18);	// Vertical position
	hdmi_i2c_write(0x3B, 0x10);	

	//1. write register table:
	pui_ptr = pui_disTable;
	ui_len = (ui_lenTable >> 1);
	for(i = 0; i < ui_len; ++ i) {
		//hdmi_i2c_write(CH7033_RegTable[i][0], CH7033_RegTable[i][1]);
		hdmi_i2c_write(pui_ptr[0], pui_ptr[1]);
		pui_ptr += 2;
	}


	hdmi_i2c_write(0x03, 0x01);	// page 1
	hdmi_i2c_write(0x39, 0x18);	// Vertical position
	hdmi_i2c_write(0x3B, 0x10);	

	deviceid = hdmi_i2c_read(0x39);
	revisionid = hdmi_i2c_read(0x3B);
	printk("CH7033B V Position register is 0x%x, 0x%x.\r\n", deviceid, revisionid);

	//2. Calculate online parameters:
	hdmi_i2c_write(0x03, 0x00);
	i = hdmi_i2c_read(0x25);
	hdmi_i2c_write(0x03, 0x04);
	//HINCA:
	val_t = hdmi_i2c_read(0x2A);
	hinca_reg = (val_t << 3) | (hdmi_i2c_read(0x2B) & 0x07);
	//HINCB:
	val_t = hdmi_i2c_read(0x2C);
	//hdmi_i2c_write(0x03, 0x04);	// select page 4
	//deviceid = hdmi_i2c_read(0x50);
	//revisionid = hdmi_i2c_read(0x51);
	//printk("CH7033B Device id is 0x%x, revision id is 0x%x.\r\n", deviceid, revisionid);

	//2. Calculate online parameters:
	hdmi_i2c_write(0x03, 0x00);
	i = hdmi_i2c_read(0x25);
	hdmi_i2c_write(0x03, 0x04);
	//HINCA:
	val_t = hdmi_i2c_read(0x2A);
	hinca_reg = (val_t << 3) | (hdmi_i2c_read(0x2B) & 0x07);
	//HINCB:
	val_t = hdmi_i2c_read(0x2C);
	hincb_reg = (val_t << 3) | (hdmi_i2c_read(0x2D) & 0x07);
	//VINCA:
	val_t = hdmi_i2c_read(0x2E);
	vinca_reg = (val_t << 3) | (hdmi_i2c_read(0x2F) & 0x07);
	//VINCB:
	val_t = hdmi_i2c_read(0x30);
	vincb_reg = (val_t << 3) | (hdmi_i2c_read(0x31) & 0x07);
	//HDINCA:
	val_t = hdmi_i2c_read(0x32);
	hdinca_reg = (val_t << 3) | (hdmi_i2c_read(0x33) & 0x07);
	//HDINCB:
	val_t = hdmi_i2c_read(0x34);
	hdincb_reg = (val_t << 3) | (hdmi_i2c_read(0x35) & 0x07);
	//no calculate hdinc if down sample disaled
	if(i & (1 << 6))
	{
		if(hdincb_reg == 0)
		{
			return ch_false;
		}
		//hdinc_reg = (ch_uint32)(((uint64_t)hdinca_reg) * (1 << 20) / hdincb_reg);
		tmp = (((uint64_t)hdinca_reg) * (1 << 20));
		do_div(tmp, hdincb_reg);
		hdinc_reg = tmp;
		//HDMI_DEBUG("hdinc_reg = %d\n", tmp);
		//hdinc_reg = (ch_uint32)((hdinca_reg << 20) / hdincb_reg);
		hdmi_i2c_write(0x3C, (hdinc_reg >> 16) & 0xFF);
		hdmi_i2c_write(0x3D, (hdinc_reg >>  8) & 0xFF);
		hdmi_i2c_write(0x3E, (hdinc_reg >>  0) & 0xFF);
	}
	if(hincb_reg == 0 || vincb_reg == 0)
	{
		return ch_false;
	}
	if(hinca_reg > hincb_reg)
	{
		return ch_false;
	}
	//hinc_reg = (ch_uint32)((uint64_t)hinca_reg * (1 << 20) / hincb_reg);
	//vinc_reg = (ch_uint32)((uint64_t)vinca_reg * (1 << 20) / vincb_reg);
	//hinc_reg = (ch_uint32)((hinca_reg << 20) / hincb_reg);
	//vinc_reg = (ch_uint32)((vinca_reg << 20) / vincb_reg);
	tmp = (uint64_t)hinca_reg * (1 << 20);
	do_div(tmp, hincb_reg);
	hinc_reg = tmp;

	tmp = (uint64_t)vinca_reg * (1 << 20);
	do_div(tmp, vincb_reg);
	vinc_reg = tmp;

	//HDMI_DEBUG("hinca_reg=%d, vinc_reg=%d", hinca_reg, vinc_reg);
	//HDMI_DEBUG("hinc_reg=%d, vinc_reg=%d", hinc_reg, vinc_reg);

	hdmi_i2c_write(0x36, (hinc_reg >> 16) & 0xFF);
	hdmi_i2c_write(0x37, (hinc_reg >>  8) & 0xFF);
	hdmi_i2c_write(0x38, (hinc_reg >>  0) & 0xFF);
	hdmi_i2c_write(0x39, (vinc_reg >> 16) & 0xFF);
	hdmi_i2c_write(0x3A, (vinc_reg >>  8) & 0xFF);
	hdmi_i2c_write(0x3B, (vinc_reg >>  0) & 0xFF);

	//3. Start to running:
	hdmi_i2c_write(0x03, 0x00);
	val_t = hdmi_i2c_read(0x0A);
	hdmi_i2c_write(0x0A, val_t | 0x80);
	hdmi_i2c_write(0x0A, val_t & 0x7F);
	val_t = hdmi_i2c_read(0x0A);
	hdmi_i2c_write(0x0A, val_t & 0xEF);
	hdmi_i2c_write(0x0A, val_t | 0x10);
	hdmi_i2c_write(0x0A, val_t & 0xEF);

	return 0;
}


unsigned int InitializeCH7033()
{
	ch_uint32 val_t;
	char* options;

	options = NULL;
	g_ui_dislen = 0;
	g_pui_disTable = NULL;
	fb_get_options(LCD_NULCDC, &options);
	if (options != NULL) {
		printk("HDMI data from NuLCDC, video option is %s.\r\n", options);
		if ((!strcmp("1280x720", options)) || (!strcmp("1280x720-32@60", options)) || (!strcmp("1280x720-16@60", options)) ||
			(!strcmp("1280x720-16", options)) || (!strcmp("1280x720-32", options))) {
			printk("HDMI display table 1280x720.\r\n");
			g_ui_dislen = hdmi_ch7033_nulcdc[0].ui_len;
			g_pui_disTable = hdmi_ch7033_nulcdc[0].pui_table;
		}
		else if((!strcmp("1024x768", options)) || (!strcmp("1024x768-32@60", options)) || (!strcmp("1024x768-16@60", options)) ||
			(!strcmp("1024x768-16", options)) || (!strcmp("1024x768-32", options))) {
			printk("HDMI display table 1024x768.\r\n");
			g_ui_dislen = hdmi_ch7033_nulcdc[1].ui_len;
			g_pui_disTable = hdmi_ch7033_nulcdc[1].pui_table;
		}
		else if((!strcmp("1024x600", options)) || (!strcmp("1024x600-32@60", options)) || (!strcmp("1024x600-16@60", options)) ||
			(!strcmp("1024x600-16", options)) || (!strcmp("1024x600-32", options))) {
			printk("HDMI display table 1024x600.\r\n");
			g_ui_dislen = hdmi_ch7033_nulcdc[2].ui_len;
			g_pui_disTable = hdmi_ch7033_nulcdc[2].pui_table;
		}
		else {
			printk("Can't find CH7033 parameter for the current display resolution mode, so use default.\r\n");
			g_ui_dislen = hdmi_ch7033_nulcdc[2].ui_len;
			g_pui_disTable = hdmi_ch7033_nulcdc[2].pui_table;
		}
	}
	else {
	
		fb_get_options(LCD_PL111, &options);
		if (options != NULL) {
			printk("HDMI data from PL111, video option is %s.\r\n", options);
		}
		else {
			printk("Can not find any LCDC parameters for HDMI.\r\n");
		}
	}

	if ((g_ui_dislen == 0) || (g_pui_disTable == NULL)) {
		return ch_false;
	}

#ifdef CONFIG_HAS_EARLYSUSPEND
	register_early_suspend(&hdmi_early_suspend);
#endif
//	fb_register_client(&hdmi_event_notifier);


#ifdef FIRMWARE_UPDATE
	firmware_update();
#endif

        hdmi_i2c_write(0x03, 0x04);     // page 4
        val_t = hdmi_i2c_read(0x61);
        val_t &= ~(0x01 << 5);
        hdmi_i2c_write(0x61, val_t);
	val_t = 0;
	val_t = hdmi_i2c_read(0x61);
	printk("Page 4 0x61 register: 0x%x.\r\n", val_t);

	displayCH7033(g_pui_disTable, g_ui_dislen);


	return ch_true;

        printk("hdmi_i2c_probe name = %s, addr = %d, irq = %d\n", hdmi_client->name, hdmi_client->addr, hdmi_client->irq);
        //i_gpio = irq_to_gpio(hdmi_client->irq);
        //i_ret = gpio_request(i_gpio, "hdmi");
        //if (i_ret == 0) {
                pkstr_wq = create_singlethread_workqueue("CH7033B wq");
                if (pkstr_wq) {
                        INIT_WORK(&(kstr_ws), hdmi_workhandler);
			// wait 500ms for CH7033B plugin/out detect
		        msleep(500);
			hdmi_i2c_write(0x03, 0x03);     // page 3
		        val_t = hdmi_i2c_read(0x25);
			if ((val_t >> 4) & 0x1) {
                		printk("CH7033B: Initial Status is HDMI Plugin.\r\n");
                		g_uc_status = PLUG_IN;
        		}
        		else {
                		printk("CH7033B: Initial Status is HDMI Plugout.\r\n");
                		g_uc_status = PLUG_OUT;
        		}
			//i_ret = 0; 
   			//i_ret = request_irq(hdmi_client->irq, hdmi_handler, IRQF_TRIGGER_HIGH | IRQF_SHARED, MODULE_NAME, MODULE_NAME);
		        //if(i_ret == 0)
		        init_timer(&hdmi_timer);
		        hdmi_timer.function = hdmi_timerhandler;
		        hdmi_timer.expires = jiffies + HZ;            //1000 * HZ / 500;
			// run timer
			add_timer(&hdmi_timer);
                }
        //}


	printk("Initial CH7033 done!\n");


	return ch_true;
}


static int __init hdmi_init(void)
{
	int ret;

	ret = i2c_add_driver(&hdmi_i2c_driver);
	if(ret)
	{
		printk(KERN_EMERG "i2c_add_driver failed! exit...\n");
		i2c_del_driver(&hdmi_i2c_driver);
	}

	return ret;
}

static void __exit hdmi_exit(void)
{
    
	i2c_del_driver(&hdmi_i2c_driver);
	printk(KERN_EMERG "hdmi_i2c_exit\n");
}

module_init(hdmi_init);
module_exit(hdmi_exit);

MODULE_AUTHOR("jinying.wu@nufront.com");
MODULE_DESCRIPTION("CH7033 HDMI driver");
MODULE_LICENSE("GPL");
