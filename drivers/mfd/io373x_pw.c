/*
 *  ENE IO373X power driver
 */

#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/init.h>
#include <linux/serio.h>
#include <linux/workqueue.h>
#include <linux/mutex.h>
#include <linux/mfd/io373x.h>
#include <linux/fs.h>
#include <linux/err.h>
#include <linux/kdev_t.h>
#include <linux/fb.h>
#include <linux/backlight.h>
#include <linux/rfkill.h>
#include <asm-generic/uaccess.h>
#include <mach/hardware.h>
//#include <mach/board-ns2816.h>
#include <mach/board-ns115.h>
#include <mach/gpio.h>
#include <mach/soc_power_ctrl.h>
#include "eneec_ioc.h"
#include "io373x_pw.h"

#define TRACE printk

#define TRYS		300
#define DELAY		100

#define SET_ON		1
#define SET_OFF 	2
#define SET_UP		1
#define SET_DOWN	2
#define ESTS_ON 	0x81
#define ESTS_OFF	0x82
#define EVENT_W		1
#define EVENT_R		2
#define EVENT_IDLE	0
#define EVENT_RFIN	0x81

#define SEQ_S3		3
#define SEQ_S5		5
#define SEQ_S6		6

#define EVENT_ADDR	0x8422
#define IDX_ADDR	0x8420
#define DAT_ADDR	0x8421


#define PW_SEQ_WIDX		0x1
#define PW_WIFI_WIDX		0x2
#define PW_BT_WIDX     		0x3
#define PW_G3_WIDX 	     	0x4
#define PW_TOUCH_WIDX		0x5
#define PW_LCD_WIDX		0x6
#define PW_CAM_WIDX		0x7
#define PW_SND_WIDX		0x8
#define PW_BRT_STEP_WIDX	0x9
#define PW_SND_STEP_WIDX	0x10
#define PW_BRT_LEV_WIDX		0x11
#define PW_FAN_LEV_WIDX		0x13
#define PW_GPS_WIDX		0x0a

#define PW_SEQ_RIDX		0x81
#define PW_WIFI_RIDX		0x82
#define PW_BT_RIDX     		0x83
#define PW_G3_RIDX 	     	0x84
#define PW_TOUCH_RIDX		0x85
#define PW_LCD_RIDX		0x86
#define PW_CAM_RIDX		0x87
#define PW_SND_RIDX		0x88
/*
#define PW_BRT_STEP_RIDX	0x89		same as 0x91
#define PW_SND_STEP_RIDX	0x90		no implementation
*/
#define PW_BRT_LEV_RIDX		0x91
#define PW_BRT_MAX_RIDX		0x92
#define PW_FAN_LEV_RIDX		0x93
#define PW_FAN_MAX_RIDX		0x94
#define PW_GPS_RIDX		0x8a

#define WIFI_NAME		"ns-wifi"
#define BT_NAME			"ns-bt"
#define GPS_NAME		"ns-gps"
#define BL_NAME			"ns-backlight"

struct io373x_pw 
{
	struct platform_device *pdev;
	struct io373x *io373x;
	struct backlight_device *backlight;
	int major;
};

struct io373x_pw * g_io373x_pw = NULL;

struct rfkill_ns_data
{
	struct io373x_pw * io373x_pw;
	struct rfkill * rfkill;
	const char * 	name;
	unsigned int	type;
	unsigned char	cmd_on;
	unsigned char	cmd_off;
};

struct rfkill_ns_data g_rfkill_ns_data[] = 
{
	{ NULL, NULL, WIFI_NAME, RFKILL_TYPE_WLAN, WIFI_ON, WIFI_OFF },
	{ NULL, NULL, BT_NAME, RFKILL_TYPE_BLUETOOTH, BT_ON, BT_OFF },
	{ NULL, NULL, GPS_NAME, RFKILL_TYPE_GPS, GPS_ON, GPS_OFF },
};
struct io373x_pw * g_pw;

struct PWCMD
{
	unsigned char cmd;	/*for cmd validate*/
	unsigned char idx;	/*cmd index*/
	unsigned char dat;	/*cmd data*/
	unsigned char event;	/*cmd event*/
	int (*handler)(struct io373x_pw *pw, struct PWPARM *parm, struct PWCMD *cmd);
};

int write_handler(struct io373x_pw *pw, struct PWPARM *parm, struct PWCMD *cmd)
{
	unsigned char evt = 0xff;
	int trys = TRYS;
	while(trys)
	{
		if(io373x_read_regs(pw->io373x, EVENT_ADDR, &evt, 1) < 0) {
			return  -EINVAL;
		} else if(evt != 0){
			udelay(DELAY);
			//TRACE("io373x wh evt:%d\n",evt);
			trys--;
		} else 
			break;
	}
	if(trys) {	/*event equal 0 detected*/
		if((io373x_write_regs(pw->io373x, IDX_ADDR, &cmd->idx, 1) < 0) 
				|| (io373x_write_regs(pw->io373x, DAT_ADDR, &cmd->dat, 1) < 0)
				|| (io373x_write_regs(pw->io373x, EVENT_ADDR, &cmd->event, 1) < 0)) {
			return -EINVAL;
		}	
	} else {
		return -EBUSY;
	}
//	TRACE("write handler ...fin\n");
	return 0;
}
int write_data_handler(struct io373x_pw *pw, struct PWPARM *parm, struct PWCMD *cmd)
{
	unsigned char evt = 0xff;
	int trys = TRYS;
	while(trys)
	{
		if(io373x_read_regs(pw->io373x, EVENT_ADDR, &evt, 1) < 0) {
			return  -EINVAL;
		} else if(evt != 0){
			udelay(DELAY);
			//TRACE("io373x wdh evt:%d\n",evt);
			trys--;
		} else 
			break;
	}
	if(trys) {	/*event equal 0 detected*/
		if((io373x_write_regs(pw->io373x, IDX_ADDR, &cmd->idx, 1) < 0) 
				|| (io373x_write_regs(pw->io373x, DAT_ADDR, &parm->data, 1) < 0)
				|| (io373x_write_regs(pw->io373x, EVENT_ADDR, &cmd->event, 1) < 0)) {
			return -EINVAL;
		}	
	} else {
		return -EBUSY;
	}
	return 0;
}
int read_handler(struct io373x_pw *pw, struct PWPARM *parm, struct PWCMD *cmd)
{
	unsigned char evt = 0xff;
	int trys = TRYS;
	while(trys)
	{
		if(io373x_read_regs(pw->io373x, EVENT_ADDR, &evt, 1) < 0) {
			return  -EINVAL;
		} else if(evt != 0){
			udelay(DELAY);
			//TRACE("io373x rh evt:%d\n",evt);
			trys--;
		} else 
			break;
	}
	if(trys) {	/*event equal 0 detected*/
		if((io373x_write_regs(pw->io373x, IDX_ADDR, &cmd->idx, 1) < 0) 
				|| (io373x_write_regs(pw->io373x, EVENT_ADDR, &cmd->event, 1) < 0)) {
			return -EINVAL;
		}	
		trys = TRYS;
		while(trys)
		{
			if(io373x_read_regs(pw->io373x, EVENT_ADDR, &evt, 1) < 0) {
				return  -EINVAL;
			} else if(evt != EVENT_RFIN){
				udelay(10*DELAY);
				//TRACE("io373x rh cmt evt:%d\n",evt);
				trys--;
			} else 
				break;
		}
		if(trys) {
			if(io373x_read_regs(pw->io373x, DAT_ADDR, &parm->data, 1) < 0)
				return -EINVAL;
			
			evt= 0;
			if(io373x_write_regs(pw->io373x, EVENT_ADDR, &evt, 1) < 0) 
				return -EINVAL;
		}

	} else {
		return -EBUSY;
	}
	return 0;
}

struct PWCMD	g_pwcmds[] = 
{
	{POWER_SEQ_S3,		PW_SEQ_WIDX,		SEQ_S3,		EVENT_W,	write_handler},	/*To S3 State*/	
	{POWER_SEQ_S5,		PW_SEQ_WIDX,		SEQ_S5,		EVENT_W,	write_handler},	/*To S5 State*/		
	{WIFI_ON,		PW_WIFI_WIDX,		SET_ON,		EVENT_W,	write_handler},	/*Power On Wifi*/		
	{WIFI_OFF,		PW_WIFI_WIDX,		SET_OFF,	EVENT_W,	write_handler},	/*Power Off Wifi*/		
	{BT_ON,			PW_BT_WIDX,		SET_ON,		EVENT_W,	write_handler},	/*Power On Bluetooth*/		
	{BT_OFF,		PW_BT_WIDX,		SET_OFF,	EVENT_W,	write_handler},		/*Power off Bluetooth*/		
	{G3_ON,			PW_G3_WIDX,		SET_ON,		EVENT_W,	write_handler},	/*Power On 3G*/		
	{G3_OFF,		PW_G3_WIDX,		SET_OFF,	EVENT_W,	write_handler},		/*Power off 3G*/			
	{TOUCH_ENABLE,		PW_TOUCH_WIDX,		SET_ON,		EVENT_W,	write_handler},	/*Enable Touch panel*/			
	{TOUCH_DISABLE,		PW_TOUCH_WIDX,		SET_OFF,	EVENT_W,	write_handler},	/*Disable Touch Panel*/		
	{LCD_ON,		PW_LCD_WIDX,		SET_ON,		EVENT_W,	write_handler},		/*Power On LCD Screen*/			
	{LCD_OFF,		PW_LCD_WIDX,		SET_OFF,	EVENT_W,	write_handler},	/*Power off LCD Screen*/			
	{CAM_ON,		PW_CAM_WIDX,		SET_ON,		EVENT_W,	write_handler},		/*Power On Camera*/			
	{CAM_OFF,		PW_CAM_WIDX,		SET_OFF,	EVENT_W,	write_handler},	/*Power off Camera*/			
	{SND_ON,		PW_SND_WIDX,		SET_ON,		EVENT_W,	write_handler},		/*Power On Sound*/			
	{SND_OFF,		PW_SND_WIDX,		SET_OFF,	EVENT_W,	write_handler},	/*Power Off Sound*/			
	{BRT_UP,		PW_BRT_STEP_WIDX,	SET_UP,		EVENT_W,	write_handler},		/*Bright Up*/			
	{BRT_DOWN,		PW_BRT_STEP_WIDX,	SET_DOWN,	EVENT_W,	write_handler},	/*Bright Down*/			
	{SND_UP,		PW_SND_STEP_WIDX,	SET_UP,		EVENT_W,	write_handler},		/*Sound Up*/			
	{SND_DOWN,		PW_SND_STEP_WIDX,	SET_DOWN,	EVENT_W,	write_handler},	/*Sound Down*/			
	{BRT_LEV,		PW_BRT_LEV_WIDX,	SET_ON,		EVENT_W,	write_data_handler},	/*Set Bright Level*/			
	{FAN_LEV,		PW_FAN_LEV_WIDX,	SET_ON,		EVENT_W,	write_data_handler},	/*Set Fan Level*/			
	{GPS_ON,		PW_GPS_WIDX,		SET_ON,		EVENT_W,	write_handler},	/*Set Bright Level*/			
	{GPS_OFF,		PW_GPS_WIDX,		SET_OFF,	EVENT_W,	write_handler},	/*Set Fan Level*/			
	{POWER_STS,		PW_SEQ_RIDX,		SET_ON,		EVENT_R,	read_handler},	/*Get Power Status*/			
	{WIFI_STS,		PW_WIFI_RIDX,		SET_ON,		EVENT_R,	read_handler},	/*Get Wifi Status*/			
	{BT_STS,		PW_BT_RIDX,		SET_ON,		EVENT_R,	read_handler},	/*Get Bluetooth Status*/			
	{G3_STS,		PW_G3_RIDX,		SET_ON,		EVENT_R,	read_handler},		/*Get 3G Status*/			
	{TOUCH_STS,		PW_TOUCH_RIDX,		SET_ON,		EVENT_R,	read_handler},	/*Get Touch Status*/			
	{LCD_STS,		PW_LCD_RIDX,		SET_ON,		EVENT_R,	read_handler},	/*Get LCD Status*/			
	{CAM_STS,		PW_CAM_RIDX,		SET_ON,		EVENT_R,	read_handler},	/*Get Camera Status*/			
	{SND_STS,		PW_SND_RIDX,		SET_ON,		EVENT_R,	read_handler},	/*Get Sound Status*/			
	{BRT_RLEV,		PW_BRT_LEV_RIDX,	SET_ON,		EVENT_R,	read_handler},	/*Get Bright Level*/			
	{BRT_RMAX,		PW_BRT_MAX_RIDX,	SET_ON,		EVENT_R,	read_handler},	/*Get Bright Level Max*/			
	{FAN_RLEV,		PW_FAN_LEV_RIDX,	SET_ON,		EVENT_R,	read_handler},	/*Get Fan Level*/			
	{FAN_RMAX,		PW_FAN_MAX_RIDX,	SET_ON,		EVENT_R,	read_handler},	/*Get Fan Level Max*/			
	{GPS_STS,		PW_GPS_RIDX,		SET_ON,		EVENT_R,	read_handler},	/*Get Fan Level Max*/			
	{CMD_MAX,		0,	0,		0,	NULL},	/*Get Fan Level Max*/			
};

int create_io373x_dev(struct io373x_pw *pw)
{
	if(IS_ERR(device_create(getclass(pw->io373x), pw->pdev->dev.parent, MKDEV(pw->major, 0), pw, "io373x_pw"))) {
		printk("Failed in creating dev.(major:%d)\n",pw->major);
		return -ENOMEM;
	}
	return 0;
}

void destory_io373x_dev(struct io373x_pw *pw)
{
	device_destroy(getclass(pw->io373x), MKDEV(pw->major, 0));
}

int handle_pw_cmd(struct io373x_pw *pw, struct PWPARM *parm, struct PWCMD *cmdlist)
{
	int ret = 0;
	struct PWCMD *cur_cmd = &cmdlist[parm->cmd];
	if(parm->cmd >= CMD_MAX) {
		return -ENOTTY;
	}
	if(parm->cmd != cur_cmd->cmd) {
		for(cur_cmd = cmdlist; cur_cmd->cmd!=CMD_MAX; cur_cmd++) {
			if(cur_cmd->cmd == parm->cmd)
				break;
		}
		if(parm->cmd == CMD_MAX)
			return -ENOTTY;
	}	
	if(cur_cmd->handler)
		ret = cur_cmd->handler(pw, parm, cur_cmd);
	return ret;
}

int send_cmd(struct io373x_pw *pw, unsigned int cmd)
{
	struct PWPARM	parm;
	parm.cmd = cmd;
	return handle_pw_cmd(pw, &parm, g_pwcmds);
}
void io373x_poweroff()
{
	struct PWPARM	parm;
	parm.cmd = POWER_SEQ_S5;
	handle_pw_cmd(g_io373x_pw, &parm, g_pwcmds);
}
EXPORT_SYMBOL(io373x_poweroff);


extern void dw_i2c_master_init(void __iomem * , unsigned char );
extern void dw_i2c_smbus_read(void  __iomem * , unsigned char * , unsigned int , unsigned char *, unsigned );
extern void dw_i2c_send_bytes(void  __iomem * , unsigned char *, unsigned int);

int io373x_smbus_set_adr(void __iomem * i2c0_base, unsigned short reg)
{
        unsigned char   wbuf[3];
        wbuf[0] = 0;    //CMD_SET_ADR
        wbuf[1] = reg >> 8;
        wbuf[2] = reg & 0xff;

        dw_i2c_send_bytes(i2c0_base, wbuf, 3);

        return 0;
}


int io373x_smbus_read_reg(void __iomem * i2c0_base, unsigned char slv_addr, unsigned short reg, unsigned char * rbuf, unsigned char len)
{
        unsigned char   wbuf[3];
        wbuf[0] = 0x81; //CMD_READ_BYTE

	dw_i2c_master_init(i2c0_base, slv_addr);
        io373x_smbus_set_adr(i2c0_base, reg);
        dw_i2c_smbus_read(i2c0_base, wbuf, 1, rbuf, len);
        return 0;
}

int io373x_smbus_write_reg(void __iomem * i2c0_base,unsigned char slv_addr, unsigned short reg, unsigned char * buf, unsigned char len)
{
        unsigned char   wbuf[16], i;
        if(len > 14)
                return -1;

        wbuf[0] = 0x03; //CMD_READ_BYTE
        wbuf[1] = len;
        for(i = 0; i< len; i++)
                wbuf[2+i] = buf[i];

	dw_i2c_master_init(i2c0_base, slv_addr);

        io373x_smbus_set_adr(i2c0_base, reg);
        dw_i2c_send_bytes(i2c0_base, wbuf, len + 2);
        return 0;
}

void io373x_raw_seq(unsigned char seq)
{
	unsigned char trys = 10, evt = 0xff, idx, dat, event, slv_addr = 0xc4/2;

	void __iomem * i2c0_base = __io_address(NS115_I2C1_BASE);

	idx = 1;
        dat = seq;
        event = 1;

  	while(trys > 0) {
                io373x_smbus_read_reg(i2c0_base, slv_addr, EVENT_ADDR/*0x8422*/, &evt, 1);                             
                if(evt != 0) {                                             
                        udelay(1000);                                       
                        trys --;                                           
                }                                                          
                else                                                       
                        break;  
        }

 	io373x_smbus_write_reg(i2c0_base, slv_addr, IDX_ADDR /*0x8420*/, &idx, 1);
	io373x_smbus_write_reg(i2c0_base, slv_addr, DAT_ADDR /*0x8421*/, &dat, 1);                   
        io373x_smbus_write_reg(i2c0_base, slv_addr, EVENT_ADDR /*0x8422*/, &event, 1); 
}

void io373x_restart_system(void)
{
	io373x_raw_seq(SEQ_S6);
}

EXPORT_SYMBOL(io373x_restart_system);

void io373x_suspend_system(void)
{
	io373x_raw_seq(SEQ_S3);
}
EXPORT_SYMBOL(io373x_suspend_system);

int send_read_cmd(struct io373x_pw *pw, unsigned int cmd, unsigned char *data)
{
	struct PWPARM	parm;
	int ret;
	parm.data = 0;
	parm.cmd = cmd;
	ret = handle_pw_cmd(pw, &parm, g_pwcmds);
	*data = parm.data;
	return ret;
}
int send_setdata_cmd(struct io373x_pw *pw, unsigned int cmd, unsigned char data)
{
	struct PWPARM	parm;
	parm.cmd = cmd;
	parm.data = data;
	return handle_pw_cmd(pw, &parm, g_pwcmds);
}
#if 1
static int io373x_pw_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	int err = 0;
	struct PWPARM parm;

	if (copy_from_user(&parm, (void __user *) arg, sizeof(struct PWPARM)))
		return -EFAULT; 
	/*
	 * extract the type and number bitfields, and don't decode
	 * wrong cmds: return ENOTTY (inappropriate ioctl) before access_ok()
	 */
	if(_IOC_TYPE(cmd) != ENEEC_IOC_MAGIC)
		return -ENOTTY;
	switch (cmd)
	{
		case ENEEC_IOC_PW:
#ifdef CONFIG_MACH_HUANGSHANS
			if(parm.cmd == CAM_ON) {//camera on
				cam_power(ON);
			} else if(parm.cmd == CAM_OFF) {//camera off
				cam_power(OFF);
			}
#endif
			err = handle_pw_cmd(g_pw, &parm, g_pwcmds);
			break;
/*
		case ENEEC_IOC_PW_R:
			err = io373x_pw(g_pw, &parm, 0);
			if (copy_to_user((void __user *) arg, &parm, sizeof(struct PWPARM)))
				return -EFAULT; 
			printk("read %d %d",parm.gpidx,parm.value);
			break;
*/
		default:
			printk("Unsupported ioctl\n");
			err = -ENOTTY;
			break;
	}
	return err;
}
#endif
static int get_brightness(struct backlight_device *bd)
{
	unsigned char data;
	struct io373x_pw *pw = bl_get_data(bd);
	if(send_read_cmd(pw, BRT_LEV, &data) < 0) {
		data = bd->props.max_brightness;
	}
	return data;
}

static int update_status(struct backlight_device *bd)
{
	struct io373x_pw *pw = bl_get_data(bd);
	send_setdata_cmd(pw,BRT_LEV,bd->props.brightness);

	//gpio_direction_output(15,1);
	//gpio_direction_output(21,1);

	if (bd->props.brightness != 0) {
		if(bd->props.power != 1) {
			bd->props.power	= 1;
			send_cmd(pw, LCD_ON);
			printk("send cmd 'lcd_on' to ec, set backlight: %d\n", bd->props.brightness);
		}
	} else {
		bd->props.power	= 0;
		send_cmd(pw, LCD_OFF);
		printk("send cmd 'lcd_off' to ec, set backlight:%d\n", bd->props.brightness);
	}
	return 0;
}

static struct backlight_ops backlight_ops = {
	.get_brightness	= get_brightness,
	.update_status	= update_status,
};

static int init_backlight(struct platform_device *pdev)
{
	struct io373x_pw * pw = platform_get_drvdata(pdev);
	struct backlight_device * backlight;
	unsigned char data;
	struct backlight_properties props;

	send_read_cmd(pw, BRT_RMAX, &data);

	props.type = BACKLIGHT_RAW;
	props.max_brightness = data;

	backlight = backlight_device_register(BL_NAME, &pdev->dev,
						     pw, &backlight_ops, &props);
	if (IS_ERR(backlight))
		return -ENOMEM;	

	pw->backlight = backlight;

	send_read_cmd(pw, BRT_RMAX, &data);
	backlight->props.max_brightness = data;

	send_read_cmd(pw, BRT_RLEV, &data);
	backlight->props.brightness = data;
	backlight->props.power = FB_BLANK_UNBLANK;
	backlight_update_status(backlight);
	return 0;
}
static void destroy_backlight(struct io373x_pw *pw)
{
	if(pw->backlight)
		backlight_device_unregister(pw->backlight);
	pw->backlight = NULL;
}
static int rfkill_set(void *data, bool blocked)
{
	/* Do something with blocked...*/
	/*
	 * blocked == false is on
	 * blocked == true is off
	 */
	struct rfkill_ns_data * ns_data = data;
	if (blocked)
		send_cmd(ns_data->io373x_pw, ns_data->cmd_off);
	else
		send_cmd(ns_data->io373x_pw, ns_data->cmd_on);

	return 0;
}

static struct rfkill_ops rfkill_ops = {
	.set_block = rfkill_set,
};
static int init_rfkill_item(struct platform_device *pdev, struct rfkill_ns_data * data)
{
	int retval;
	struct io373x_pw * pw = platform_get_drvdata(pdev);
	struct rfkill * rfkill = NULL;

	data->io373x_pw = pw;
	rfkill = rfkill_alloc(data->name, &pdev->dev, data->type,
			   &rfkill_ops, data);
	if (!rfkill)
		return -ENOMEM;

	retval = rfkill_register(rfkill);
	if (retval) {
		rfkill_destroy(rfkill);
		return -ENODEV;
	}
	data->rfkill = rfkill;
	return 0;
}
static int init_rfkill(struct platform_device *pdev)
{
	int idx;
	
	for(idx = 0; idx < ARRAY_SIZE(g_rfkill_ns_data); idx ++) {
		init_rfkill_item(pdev, &g_rfkill_ns_data[idx]);
	}
	return 0;
}

static void destroy_rfkill(void)
{
	int idx;
	for(idx = 0; idx < ARRAY_SIZE(g_rfkill_ns_data); idx++) {
		if(g_rfkill_ns_data->rfkill) {
			rfkill_unregister(g_rfkill_ns_data->rfkill);
			rfkill_destroy(g_rfkill_ns_data->rfkill);
			g_rfkill_ns_data->rfkill = NULL;
		}
	}
}

static struct file_operations io373x_pw_fops = {
    .unlocked_ioctl = io373x_pw_ioctl,
};

static int io373x_pw_probe(struct platform_device *pdev)
{
	struct io373x *io373x = dev_get_drvdata(pdev->dev.parent);
	struct io373x_pw *pw = 0;
	unsigned char evt = 0xff;
	int err = 0;
	int major = 0;

	if (!(pw = kzalloc(sizeof(struct io373x_pw), GFP_KERNEL)))
		return -ENOMEM;

	pw->io373x = io373x;
	pw->pdev = pdev;
	platform_set_drvdata(pdev, pw);
	g_io373x_pw = pw;

	major = register_chrdev(0, "io373x_pw", &io373x_pw_fops);
	if(major < 0) {
		printk(KERN_INFO "io373x_pw: unable to get major %d\n", major);
		err = major;
		goto error_free;
	}else{
		pw->major = major;
	}

	if(0 != create_io373x_dev(pw)) {
		return -ENOMEM;
	}

	g_pw = pw;

	/*clear */
	io373x_read_regs(pw->io373x, EVENT_ADDR, &evt, 1);
	TRACE("io373x init evt:0x%x\n",evt);
	evt = 0;
	io373x_write_regs(pw->io373x, EVENT_ADDR, &evt, 1);
	io373x_read_regs(pw->io373x, EVENT_ADDR, &evt, 1);
	TRACE("io373x clear evt:0x%x\n",evt);

	init_rfkill(pdev);
//#ifndef CONFIG_BACKLIGHT_NS2816
#if 0
	init_backlight(pdev);
#endif
	return 0;
//error
	unregister_chrdev(pw->major, "io373x_pw");
error_free:
	kfree(pw);

	return err;

}

static int io373x_pw_remove(struct platform_device *pdev)
{
	struct io373x_pw *pw = platform_get_drvdata(pdev);
	//destroy_backlight(pw);
	destroy_rfkill();
	destory_io373x_dev(pw);
	unregister_chrdev(pw->major, "io373x_pw");
	kfree(pw);
	g_io373x_pw = NULL;

	return 0;
}

static struct platform_driver io373x_pw_driver = {
	.driver = {
		.name   = "io373x-pw",
		.owner  = THIS_MODULE,
	},
	.probe      = io373x_pw_probe,
	.remove     = __devexit_p(io373x_pw_remove),
};

static int __init io373x_pw_init(void)
{
	printk("io373x_pw_init\n");

	return platform_driver_register(&io373x_pw_driver);
}

static void __exit io373x_pw_exit(void)
{
	printk("io373x_pw_exit\n");

	platform_driver_unregister(&io373x_pw_driver);
}

module_init(io373x_pw_init);
module_exit(io373x_pw_exit);
MODULE_AUTHOR("zeyuan");
MODULE_DESCRIPTION("IO373X Power Control");
MODULE_LICENSE("GPL");
