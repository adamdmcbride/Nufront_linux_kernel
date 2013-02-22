/*
 * drivers/input/touchscreen/ft5x0x_ts.c
 *
 * FocalTech ft5x0x TouchScreen driver.
 *
 * Copyright (c) 2010  Focal tech Ltd.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * VERSION      	DATE			AUTHOR        Note
 *    1.0		  2010-01-05			WenFS    only support mulititouch	Wenfs 2010-10-01
 *    2.0          2011-09-05                   Duxx      Add touch key, and project setting update, auto CLB command
 *
 *
 */

#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/earlysuspend.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/kernel.h>
#include <linux/hrtimer.h>
#include <linux/module.h>
#include <linux/proc_fs.h>
#include <linux/string.h>
#include <asm/uaccess.h>
#include <linux/vmalloc.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <mach/gpio.h>
#include <mach/extend.h>
#include "ft5x06_ts.h"
#include "ft5x06_ex_fun.h"

static struct i2c_client *this_client;

#define CONFIG_FT5X0X_MULTITOUCH 1

static int SYSFS = 0;
static int DEBUG = 0;

module_param(SYSFS,int,0644);
module_param(DEBUG,int,0644);

#define Debug(fmt,args...) 	\
	do 			\
{			\
	if(DEBUG)	\
	printk(KERN_DEBUG fmt,##args);\
}while(0)

#if CFG_SUPPORT_TOUCH_KEY
int tsp_keycodes[CFG_NUMOFKEYS] ={

	KEY_HOME,
	KEY_MENU,
	KEY_BACK,
	KEY_SEARCH
};

char *tsp_keyname[CFG_NUMOFKEYS] ={

	"Home",
	"Menu",
	"Back",
	"Search"
};

static bool tsp_keystatus[CFG_NUMOFKEYS];
#endif

/***********************************************************************************************
Name	:	ft5x0x_i2c_rxdata

Input	:	*rxdata
 *length

Output	:	ret

function	:

 ***********************************************************************************************/
static int ft5x0x_i2c_Read(char *rxdata, int length)
{
	int ret;

	struct i2c_msg msgs[] = {
		{
			.addr	= this_client->addr,
			.flags	= 0,
			.len	= 1,
			.buf	= rxdata,
		},
		{
			.addr	= this_client->addr,
			.flags	= I2C_M_RD,
			.len	= length,
			.buf	= rxdata,
		},
	};

	//msleep(1);
	ret = i2c_transfer(this_client->adapter, msgs, 2);
	if (ret < 0)
		pr_err("msg %s i2c read error: %d\n", __func__, ret);

	return ret;
}
/***********************************************************************************************
Name	:

Input	:


Output	:

function	:

 ***********************************************************************************************/
static int ft5x0x_i2c_Write(char *txdata, int length)
{
	int ret;

	struct i2c_msg msg[] = {
		{
			.addr	= this_client->addr,
			.flags	= 0,
			.len	= length,
			.buf	= txdata,
		},
	};

	//msleep(1);
	ret = i2c_transfer(this_client->adapter, msg, 1);
	if (ret < 0)
		pr_err("%s i2c write error: %d\n", __func__, ret);

	return ret;
}
/***********************************************************************************************
Name	:	 ft5x0x_write_reg

Input	:	addr -- address
para -- parameter

Output	:

function	:	write register of ft5x0x

 ***********************************************************************************************/
static int ft5x0x_write_reg(u8  addr, u8 para)
{
	u8 buf[3];
	int ret = -1;

	buf[0] = addr;
	buf[1] = para;
	ret = ft5x0x_i2c_Write(buf, 2);
	if (ret < 0) {
		pr_err("write reg failed! %#x ret: %d", buf[0], ret);
		return -1;
	}

	return 0;
}


/***********************************************************************************************
Name	:	ft5x0x_read_reg

Input	:	addr
pdata

Output	:

function	:	read register of ft5x0x

 ***********************************************************************************************/
static int ft5x0x_read_reg(u8 addr, u8 *pdata)
{
	int ret;
	u8 buf[2];
	struct i2c_msg msgs[2];

	//
	buf[0] = addr;    //register address

	msgs[0].addr = this_client->addr;
	msgs[0].flags = 0;
	msgs[0].len = 1;
	msgs[0].buf = buf;
	msgs[1].addr = this_client->addr;
	msgs[1].flags = I2C_M_RD;
	msgs[1].len = 1;
	msgs[1].buf = buf;

	ret = i2c_transfer(this_client->adapter, msgs, 2);
	if (ret < 0)
		pr_err("msg %s i2c read error: %d\n", __func__, ret);

	*pdata = buf[0];
	return ret;

}


/***********************************************************************************************
Name	:	 ft5x0x_read_fw_ver

Input	:	 void


Output	:	 firmware version

function	:	 read TP firmware version

 ***********************************************************************************************/
static unsigned char ft5x0x_read_fw_ver(void)
{
	unsigned char ver=' ';
	ft5x0x_read_reg(FT5X0X_REG_FIRMID, &ver);
	return(ver);
}



/***********************************************************************************************
Name	:

Input	:


Output	:

function	:

 ***********************************************************************************************/
static void ft5x0x_ts_release(void)
{
	struct ft5x0x_ts_data *data = i2c_get_clientdata(this_client);

	input_report_abs(data->input_dev, ABS_MT_TOUCH_MAJOR, 0);
	input_report_abs(data->input_dev, ABS_MT_WIDTH_MAJOR, 0);

	input_report_key(data->input_dev, BTN_TOUCH, 0);

	input_sync(data->input_dev);
}


//read touch point information
static int ft5x0x_read_data(void)
{
	struct ft5x0x_ts_data *data = i2c_get_clientdata(this_client);
	struct ts_event *event = &data->event;
	u8 buf[CFG_POINT_READ_BUF] = {0};
	int ret = -1;
	int i;

	ret = ft5x0x_i2c_Read(buf, CFG_POINT_READ_BUF);
	if (ret < 0) {
		pr_err("%s read_data i2c_rxdata failed: %d\n", __func__, ret);
		return ret;
	}
	memset(event, 0, sizeof(struct ts_event));
	event->touch_point = buf[2] & 0x07;

	if (event->touch_point > CFG_MAX_TOUCH_POINTS)
	{
		event->touch_point = CFG_MAX_TOUCH_POINTS;
	}

	for (i = 0; i < event->touch_point; i++)
	{
		event->au16_x[i] = (s16)(buf[3 + 6*i] & 0x0F)<<8 | (s16)buf[4 + 6*i];
		event->au16_y[i] = (s16)(buf[5 + 6*i] & 0x0F)<<8 | (s16)buf[6 + 6*i];
		event->au8_touch_event[i] = buf[0x3 + 6*i] >> 6;
		event->au8_finger_id[i] = (buf[5 + 6*i])>>4;
	}

	event->pressure = 200;

	return 0;
}

/***********************************************************************************************
Name	:

Input	:


Output	:

function	:

 ***********************************************************************************************/


#if CFG_SUPPORT_TOUCH_KEY
int ft5x0x_touch_key_process(struct input_dev *dev, int x, int y, int touch_event)
{
	int i;
	int key_id;

	if ( x < 425&&x > 399)
	{
		key_id = 3;
	}
	else if ( x < 315&&x > 280)
	{
		key_id = 2;
	}

	else if ( x < 205&&x > 179)
	{
		key_id = 1;
	}
	else if (x < 75&&x > 47)
	{
		key_id = 0;
	}

	else
	{
		key_id = 0xf;
	}

	for(i = 0; i <CFG_NUMOFKEYS; i++ )
	{
		if(tsp_keystatus[i])
		{
			input_report_key(dev, tsp_keycodes[i], 0);

			Debug("[FTS] %s key is release. Keycode : %d\n", tsp_keyname[i], tsp_keycodes[i]);

			tsp_keystatus[i] = KEY_RELEASE;
		}
		else if( key_id == i )
		{
			if( touch_event == 0)                                  // detect
			{
				input_report_key(dev, tsp_keycodes[i], 1);

				Debug( "[FTS] %s key is pressed. Keycode : %d\n", tsp_keyname[i], tsp_keycodes[i]);
				tsp_keystatus[i] = KEY_PRESS;
			}
		}
	}
	return 0;

}
#endif

static void ft5x0x_report_value(void)
{
	struct ft5x0x_ts_data *data = i2c_get_clientdata(this_client);
	struct ts_event *event = &data->event;
	int i;


	for (i  = 0; i < event->touch_point; i++)
	{
		if (event->au16_x[i] < SCREEN_MAX_X && event->au16_y[i] < SCREEN_MAX_Y)
			// LCD view area
		{
			input_report_abs(data->input_dev, ABS_MT_POSITION_X, event->au16_x[i]);
			input_report_abs(data->input_dev, ABS_MT_POSITION_Y, event->au16_y[i]);
			input_report_abs(data->input_dev, ABS_MT_WIDTH_MAJOR, 1);
			input_report_abs(data->input_dev, ABS_MT_TRACKING_ID, event->au8_finger_id[i]);
			if (event->au8_touch_event[i]== 0 || event->au8_touch_event[i] == 2)
			{
				input_report_abs(data->input_dev, ABS_MT_TOUCH_MAJOR, event->pressure);
			}
			else
			{
				input_report_abs(data->input_dev, ABS_MT_TOUCH_MAJOR, 0);
			}

			input_report_key(data->input_dev, BTN_TOUCH, 1);
		}
		else //maybe the touch key area
		{
#if CFG_SUPPORT_TOUCH_KEY
			if (event->au16_y[i] >= SCREEN_MAX_Y)
			{
				ft5x0x_touch_key_process(data->input_dev, event->au16_x[i], event->au16_y[i], event->au8_touch_event[i]);
			}
#endif
		}

		input_mt_sync(data->input_dev);
	}
	input_sync(data->input_dev);

	if (event->touch_point == 0) {
		ft5x0x_ts_release();
		return ;
	}


}	/*end ft5x0x_report_value*/


/***********************************************************************************************
Name	:

Input	:


Output	:

function	:

 ***********************************************************************************************/
static void ft5x0x_ts_pen_irq_work(struct work_struct *work)
{
	int ret = -1;

	struct ft5x0x_ts_data  *ts = container_of(work, struct ft5x0x_ts_data, pen_event_work);

	ret = ft5x0x_read_data();
	if (ret == 0) {
		ft5x0x_report_value();
	}

	enable_irq(ts->client->irq);
}
/***********************************************************************************************
Name	:

Input	:


Output	:

function	:

 ***********************************************************************************************/
static irqreturn_t ft5x0x_ts_interrupt(int irq, void *dev_id)
{
	struct ft5x0x_ts_data *ft5x0x_ts = dev_id;

	Debug("[leo]---ft5x0x_ts_interrupt---enter\n");

	disable_irq_nosync(ft5x0x_ts->client->irq);
	if (!work_pending(&ft5x0x_ts->pen_event_work)) {
		queue_work(ft5x0x_ts->ts_workqueue, &ft5x0x_ts->pen_event_work);
	}

	return IRQ_HANDLED;
}
#ifdef CONFIG_HAS_EARLYSUSPEND
/***********************************************************************************************
Name	:

Input	:


Output	:

function	:

 ***********************************************************************************************/
static void ft5x0x_ts_suspend(struct early_suspend *handler)
{
	struct ft5x0x_ts_data *ts;
	ts =  container_of(handler, struct ft5x0x_ts_data, early_suspend);

	Debug("==ft5x0x_ts_suspend=\n");
	disable_irq(this_client->irq);

	cancel_work_sync(&ts->pen_event_work);
	flush_workqueue(ts->ts_workqueue);
	// ==set mode ==,
	ft5x0x_write_reg(FT5X0X_REG_PMODE, PMODE_HIBERNATE);
}
/***********************************************************************************************
Name	:

Input	:


Output	:

function	:

 ***********************************************************************************************/
static void ft5x0x_ts_resume(struct early_suspend *handler)
{
	struct ft5x0x_ts_data *ts;
	ts =  container_of(handler, struct ft5x0x_ts_data, early_suspend);

	Debug("==ft5x0x_ts_resume= ts->reset_gpio = %d this_client->irq = %d \n",ts->reset_gpio,this_client->irq);
	Debug("reset pin level is %d \n",gpio_get_value(ts->reset_gpio));
	gpio_direction_output(ts->reset_gpio,  0);
	mdelay(2);
	gpio_direction_output(ts->reset_gpio,  1);

	msleep(100);
	enable_irq(this_client->irq);
}
#endif  //CONFIG_HAS_EARLYSUSPEND
/***********************************************************************************************
Name	:

Input	:


Output	:

function	:

 ***********************************************************************************************/
	static int
ft5x0x_ts_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct ft5x0x_ts_data *ft5x0x_ts;
	struct input_dev *input_dev;
	int err = 0;
	unsigned char uc_reg_value;

#if CFG_SUPPORT_TOUCH_KEY
	int i;
#endif
	char temp;

	Debug("[FTS] ft5x0x_ts_probe, driver version is %s.\n", CFG_FTS_CTP_DRIVER_VERSION);

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		err = -ENODEV;
		goto exit_check_functionality_failed;
	}

	ft5x0x_ts = kzalloc(sizeof(*ft5x0x_ts), GFP_KERNEL);
	if (!ft5x0x_ts)	{
		err = -ENOMEM;
		goto exit_alloc_data_failed;
	}

	//reset

#ifdef CONFIG_OF
	struct device_node *np;
	const __be32  *ip;
	int reset_gpio;
	np = client->dev.of_node;
	ip = of_get_property(np, "reset_gpio", NULL);
	if(ip)
		reset_gpio = be32_to_cpup(ip);
	ft5x0x_ts->reset_gpio = irq_to_gpio(reset_gpio);
#else
	struct touch_panel_platform_data *tP_platform_data;
	tP_platform_data= (struct touch_panel_platform_data*)client->dev.platform_data;
	ft5x0x_ts->reset_gpio = irq_to_gpio(tP_platform_data->irq_reset);
#endif

	Debug("touch panel reset gpio number is %d\n", ft5x0x_ts->reset_gpio);
	gpio_free(ft5x0x_ts->reset_gpio);
	err = gpio_request(ft5x0x_ts->reset_gpio,"TP_RESET");
	if(err < 0){
		dev_err(&client->dev, "Failed to request RESET GPIO:%d, ERRNO:%d\n", ft5x0x_ts->reset_gpio, err);
		//	goto exit_alloc_data_failed;
	}

	gpio_direction_output(ft5x0x_ts->reset_gpio,  0);
	msleep(2);
	gpio_direction_output(ft5x0x_ts->reset_gpio,  1);
	msleep(100);
	ft5x0x_ts->irq_gpio = irq_to_gpio(client->irq);

	err = gpio_request(ft5x0x_ts->irq_gpio, "TS_INT");
	if(err < 0){
		dev_err(&client->dev, "Failed to request GPIO:%d, ERRNO:%d\n", ft5x0x_ts->irq_gpio, err);
	}
	gpio_direction_input(ft5x0x_ts->irq_gpio);

	ft5x0x_ts->client = client;
	this_client = client;
	i2c_set_clientdata(client, ft5x0x_ts);

	for(uc_reg_value = 0; uc_reg_value <5; ++uc_reg_value){
		err = ft5x0x_read_reg(0x01, &temp);

		if(err < 0){
			pr_err("[leo]---tp test err---");
		}else{
			Debug("[leo]---tp test id=0x%x--\n",temp);
		}
	}

	INIT_WORK(&ft5x0x_ts->pen_event_work, ft5x0x_ts_pen_irq_work);

	ft5x0x_ts->ts_workqueue = create_singlethread_workqueue(dev_name(&client->dev));
	if (!ft5x0x_ts->ts_workqueue) {
		err = -ESRCH;
		goto exit_create_singlethread;
	}

	err = request_irq(client->irq, ft5x0x_ts_interrupt, IRQF_TRIGGER_FALLING, "ft5x0x_ts", ft5x0x_ts);
	if (err < 0) {
		dev_err(&client->dev, "ft5x0x_probe: request irq failed\n");
		goto exit_irq_request_failed;
	}

	disable_irq(client->irq);

	input_dev = input_allocate_device();
	if (!input_dev) {
		err = -ENOMEM;
		dev_err(&client->dev, "failed to allocate input device\n");
		goto exit_input_dev_alloc_failed;
	}

	ft5x0x_ts->input_dev = input_dev;

	input_dev->evbit[0] = BIT_MASK(EV_SYN) | BIT_MASK(EV_KEY) | BIT_MASK(EV_ABS) ;
	input_dev->keybit[BIT_WORD(BTN_TOUCH)] = BIT_MASK(BTN_TOUCH);
	input_dev->propbit[0] = BIT(INPUT_PROP_DIRECT);

	input_dev->absbit[0] =  BIT_MASK(ABS_MT_TRACKING_ID) |
		BIT_MASK(ABS_MT_POSITION_X) | BIT_MASK(ABS_MT_POSITION_Y)|
		BIT_MASK(ABS_MT_TOUCH_MAJOR) | BIT_MASK(ABS_MT_WIDTH_MAJOR);

	input_set_abs_params(input_dev,
			ABS_MT_POSITION_X, 0, SCREEN_MAX_X, 0, 0);
	input_set_abs_params(input_dev,
			ABS_MT_POSITION_Y, 0, SCREEN_MAX_Y, 0, 0);
	input_set_abs_params(input_dev,
			ABS_MT_TOUCH_MAJOR, 0, PRESS_MAX, 0, 0);
	input_set_abs_params(input_dev,
			ABS_MT_WIDTH_MAJOR, 0, 200, 0, 0);
	input_set_abs_params(input_dev,
			ABS_MT_TRACKING_ID, 0, 5, 0, 0);


#if CFG_SUPPORT_TOUCH_KEY
	//setup key code area
	set_bit(EV_SYN, input_dev->evbit);
	set_bit(BTN_TOUCH, input_dev->keybit);
	input_dev->keycode = tsp_keycodes;
	for(i = 0; i < CFG_NUMOFKEYS; i++)
	{
		input_set_capability(input_dev, EV_KEY, ((int*)input_dev->keycode)[i]);
		tsp_keystatus[i] = KEY_RELEASE;
	}
#endif

	input_dev->name	= FT5X0X_NAME;		//dev_name(&client->dev)

	err = input_register_device(input_dev);
	if (err) {
		dev_err(&client->dev,
				"ft5x0x_ts_probe: failed to register input device: %s\n",
				dev_name(&client->dev));
		goto exit_input_register_device_failed;
	}

#ifdef CONFIG_HAS_EARLYSUSPEND
	Debug("==register_early_suspend =\n");
	ft5x0x_ts->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	ft5x0x_ts->early_suspend.suspend = ft5x0x_ts_suspend;
	ft5x0x_ts->early_suspend.resume	= ft5x0x_ts_resume;
	register_early_suspend(&ft5x0x_ts->early_suspend);
#endif

	msleep(150);  //make sure CTP already finish startup process

	//get some register information
	uc_reg_value = ft5x0x_read_fw_ver();
	if(uc_reg_value==' '){
		goto	exit_input_register_device_failed;
	}
	Debug("[FTS] Firmware version = 0x%x\n", uc_reg_value);
	ft5x0x_read_reg(FT5X0X_REG_PERIODACTIVE, &uc_reg_value);
	Debug("[FTS] report rate is %dHz.\n", uc_reg_value * 10);
	ft5x0x_read_reg(FT5X0X_REG_THGROUP, &uc_reg_value);
	Debug("[FTS] touch threshold is %d.\n", uc_reg_value * 4);

#if CFG_SUPPORT_AUTO_UPG
	fts_ctpm_auto_upg();
#endif

#if CFG_SUPPORT_UPDATE_PROJECT_SETTING
	fts_ctpm_update_project_setting();
#endif

	enable_irq(client->irq);

	ft5x0x_create_sysfs(client);


	Debug("[FTS] ==probe over =\n");
	return 0;

exit_input_register_device_failed:
	input_free_device(input_dev);
	gpio_free(ft5x0x_ts->irq_gpio);
	gpio_free(ft5x0x_ts->reset_gpio);
	free_irq(client->irq, ft5x0x_ts);
	unregister_early_suspend(&ft5x0x_ts->early_suspend);
	cancel_work_sync(&ft5x0x_ts->pen_event_work);
	destroy_workqueue(ft5x0x_ts->ts_workqueue);
exit_input_dev_alloc_failed:
	//	free_irq(client->irq, ft5x0x_ts);
	free_irq(client->irq, ft5x0x_ts);
exit_irq_request_failed:
	//exit_platform_data_null:
	cancel_work_sync(&ft5x0x_ts->pen_event_work);
	destroy_workqueue(ft5x0x_ts->ts_workqueue);
exit_create_singlethread:
	pr_err("==singlethread error =\n");
	i2c_set_clientdata(client, NULL);
	kfree(ft5x0x_ts);
exit_alloc_data_failed:
exit_check_functionality_failed:
	return err;
}
/***********************************************************************************************
Name	:

Input	:


Output	:

function	:

 ***********************************************************************************************/
static int __devexit ft5x0x_ts_remove(struct i2c_client *client)
{
	struct ft5x0x_ts_data *ft5x0x_ts;


	Debug("==ft5x0x_ts_remove=\n");
	ft5x0x_ts = i2c_get_clientdata(client);
	unregister_early_suspend(&ft5x0x_ts->early_suspend);
	//	free_irq(client->irq, ft5x0x_ts);
	free_irq(client->irq, ft5x0x_ts);
	input_unregister_device(ft5x0x_ts->input_dev);
	kfree(ft5x0x_ts);
	cancel_work_sync(&ft5x0x_ts->pen_event_work);
	destroy_workqueue(ft5x0x_ts->ts_workqueue);
	i2c_set_clientdata(client, NULL);
	gpio_free(ft5x0x_ts->irq_gpio);
	gpio_free(ft5x0x_ts->reset_gpio);
	return 0;
}

static const struct i2c_device_id ft5x0x_ts_id[] = {
	{ FT5X0X_NAME, 0 },{ }
};


MODULE_DEVICE_TABLE(i2c, ft5x0x_ts_id);

static struct i2c_driver ft5x0x_ts_driver = {
	.probe		= ft5x0x_ts_probe,
	.remove		= __devexit_p(ft5x0x_ts_remove),
	.id_table	= ft5x0x_ts_id,
	.driver	= {
		.name	= FT5X0X_NAME,
		.owner	= THIS_MODULE,
	},
};

/***********************************************************************************************
Name	:

Input	:


Output	:

function	:

 ***********************************************************************************************/
static int __init ft5x0x_ts_init(void)
{
	int ret;


	Debug("==ft5x0x_ts_init==\n");
	ret = i2c_add_driver(&ft5x0x_ts_driver);

	Debug("ret=%d\n",ret);
	return ret;
	//	return i2c_add_driver(&ft5x0x_ts_driver);
}

/***********************************************************************************************
Name	:

Input	:


Output	:

function	:

 ***********************************************************************************************/
static void __exit ft5x0x_ts_exit(void)
{

	Debug("==ft5x0x_ts_exit==\n");
	i2c_del_driver(&ft5x0x_ts_driver);
}

module_init(ft5x0x_ts_init);
module_exit(ft5x0x_ts_exit);

MODULE_AUTHOR("<wenfs@Focaltech-systems.com>");
MODULE_DESCRIPTION("FocalTech ft5x0x TouchScreen driver");
MODULE_LICENSE("GPL");

