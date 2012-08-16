/*
 * wangweilin modify for kernel3.0
 * v0.4 2012.1.14 remove warning
 * v0.3 2012.1.14 add ifdef for suspend and early_suspend
 * v0.2 2012.1.12 add early_suspend and kernel3.0 porting
 * v0.1 wangweilin 2012.1.9 init
 */

#include <linux/mutex.h>
#include <linux/pm_runtime.h>
#include <linux/wait.h>

#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/hwmon.h>
#include <linux/timer.h>
#include <asm/uaccess.h>
#include <asm/errno.h>
#include <linux/delay.h>
#include <asm/io.h>
//#include <linux/gpio.h>
#include <linux/proc_fs.h>
#include <asm/uaccess.h>
#include <linux/ioport.h>
#include <linux/serial_core.h>
#include <linux/platform_device.h>
#include <linux/workqueue.h>
#include <linux/firmware.h>
#include <linux/string.h>
#include <linux/kmod.h>
#include <linux/ctype.h>
#include <linux/input.h>
#include <linux/earlysuspend.h>


#define TR0(fmt, args...) printk(KERN_CRIT "lightsensor_cm3212: " fmt, ##args)
#define TR1(fmt, args...) 

#define DEVICE_NAME			"lightsensor"
#define DEVICE_ID			"lightsensor"
#define CM3212_MAJOR 			242
#define	MAX_LUX				65535

#define CMDREG 0x90

#define LIGHTSENSOR_IOCTL_MAGIC_T 'l'
#define LIGHTSENSOR_IOCTL_GET_ENABLED _IOR(LIGHTSENSOR_IOCTL_MAGIC_T, 1, int *)
#define LIGHTSENSOR_IOCTL_ENABLE _IOW(LIGHTSENSOR_IOCTL_MAGIC_T, 2, int *)
#define LIGHTSENSOR_IOCTL_GET_VALUE _IOW(LIGHTSENSOR_IOCTL_MAGIC_T, 3, int *)

static int lux_v;
static int lux_old;
static int swich;
static int count = 0;
static struct i2c_client *cm3212_client_p;
static struct input_dev *cm3212_dev;
static struct timer_list lightsensor_timer;
enum {OFF = 0, ON};
static int g_switch = OFF;
static int cm3212_probe(struct i2c_client *clientp, const struct i2c_device_id *idp);
#ifndef CONFIG_EARLYSUSPEND
static int cm3212_suspend(struct i2c_client *client, pm_message_t state);
static int cm3212_resume(struct i2c_client *client);
#endif
static int cm3212_remove(struct i2c_client *client);

static int cm3212_open(struct inode *inode, struct file *file);
static int cm3212_release(struct inode *inode, struct file *file);
//static int cm3212_ioctl(struct inode *inode, struct file *file, unsigned int cmd, unsigned long arg);
static long cm3212_ioctl(struct file *file, unsigned int cmd, unsigned long arg);
static int cm3212_chip_on(void);
static int cm3212_chip_off(void);

int cm3212_i2c_read(u8 reg);
int cm3212_i2c_write(u8 reg, u8 val);
int cm3212_i2c_write_command(u8 reg);

static struct i2c_device_id cm3212_idtable[] =
{
	{ "cm3212", 0x90/2},
	{},
};
MODULE_DEVICE_TABLE(i2c, cm3212_idtable);


#ifdef CONFIG_EARLYSUSPEND
static void cm3212_early_suspend(struct early_suspend * h);
static void cm3212_early_resume(struct early_suspend * h);

static struct early_suspend es = {
	.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN,
	.suspend = cm3212_early_suspend,
	.resume = cm3212_early_resume,
};

static void cm3212_early_suspend(struct early_suspend * h)
{
	TR0("early_suspend\n");
	cm3212_chip_off();

	cm3212_i2c_write(CMDREG, 0x0f);
}

static void cm3212_early_resume(struct early_suspend * h)
{
	TR0("early_resume\n");
	cm3212_i2c_write(CMDREG, 0x0e);

	cm3212_chip_on();
}
#endif

static struct i2c_driver cm3212_driver =
{
	.driver =
	{
		.owner = THIS_MODULE, 
		.name = "cm3212",
	}, 
	.id_table = cm3212_idtable,
	.probe = cm3212_probe, 
	.remove = __devexit_p(cm3212_remove), 
#ifdef CONFIG_EARLYSUSPEND
	.suspend = NULL, 
	.resume = NULL,
#else
	.suspend = cm3212_suspend, 
	.resume = cm3212_resume,
#endif
};


static struct file_operations cm3212_fops =
{
	.owner = THIS_MODULE, 
	.open = cm3212_open, 
	.release = cm3212_release, 
	.unlocked_ioctl = cm3212_ioctl,
};

static int cm3212_probe(struct i2c_client *clientp, const struct i2c_device_id *idp)
{
	int ret;
	struct class *ls_class;

	TR0("probe\n");
	
	lux_v = 0;
	lux_old = 0;
	swich = 0;
        init_timer(&lightsensor_timer);
	cm3212_client_p = clientp;

	//////////  register chardev  //////////
	ret = register_chrdev(CM3212_MAJOR,"cm3212",&cm3212_fops);
	if(ret < 0) {
		TR0("register %s char dev error\n", "cm3212");
	}

	//////////  create device file /dev //////////
	ls_class = class_create(THIS_MODULE, "ls_class");
	device_create(ls_class, NULL, MKDEV(CM3212_MAJOR, 0), "lightsensor", "lightsensor");

	//////////  input  ///////////
	cm3212_dev = input_allocate_device();
	if (!cm3212_dev)
	{
		TR0("init: Not enough memory for input_dev\n");
		return(ret);
	}

	cm3212_dev->name = DEVICE_ID;
	cm3212_dev->id.bustype = BUS_I2C;
	cm3212_dev->id.vendor = 0x0089;
	cm3212_dev->id.product = 0x2581;
	cm3212_dev->id.version = 3;
	cm3212_dev->phys = "/dev/lightsensor";

	set_bit(EV_ABS, cm3212_dev->evbit);
	set_bit(ABS_MISC, cm3212_dev->absbit);	//ABS_X to ABS_MISC
/*
	cm3212_dev->absmin[ABS_MISC] = 0;
	cm3212_dev->absmax[ABS_MISC] = MAX_LUX;
	cm3212_dev->absfuzz[ABS_MISC] = 0;
	cm3212_dev->absflat[ABS_MISC] = 0;
*/
	//input_set_abs_params(cm3212_dev, ABS_MISC, -max_val, max_val, fuzz, flat);
	input_set_abs_params(cm3212_dev, ABS_MISC, -MAX_LUX, MAX_LUX, 0, 0);

	//And register..
	ret = input_register_device(cm3212_dev);
	if (ret)
	{
		TR0("ALS button.c: Failed to register input_dev\n");
		goto err_free_dev;
	}

#ifdef CONFIG_EARLYSUSPEND
	register_early_suspend(&es);
#endif
	return 0;

err_free_dev:
	input_free_device(cm3212_dev);

	return ret;
}

#ifndef CONFIG_EARLYSUSPEND
static int cm3212_suspend(struct i2c_client *client, pm_message_t state)
{
	TR0("suspend\n");
	cm3212_chip_off();

	cm3212_i2c_write(CMDREG, 0x0f);

	return 0;
}

static int cm3212_resume(struct i2c_client *client)
{
	TR0("resume\n");
	//schedule_delayed_work(&cm3212_timer_work,HZ*5);
	cm3212_i2c_write(CMDREG, 0x0e);

	cm3212_chip_on();

	return 0;
}
#endif

static int __devexit cm3212_remove(struct i2c_client *client)
{
	TR1("remove\n");	

	return 0;
}

static int cm3212_open(struct inode *inode, struct file *file)
{
	TR1("open\n");	

	return 0;
}

static int cm3212_release(struct inode *inode, struct file *file)
{
	TR1("release\n");	

	return 0;
}

static void lightsensor_bh_handler(struct work_struct *work)
{
	int lux;
	int lux_sub;
	char CM3212_value_MSB, CM3212_value_LSB;
	int CM3212_value_12bit;

	CM3212_value_LSB = cm3212_i2c_read(0x93);
	CM3212_value_MSB = cm3212_i2c_read(0x91);
	CM3212_value_12bit= CM3212_value_MSB;
	CM3212_value_12bit= CM3212_value_12bit << 8;
	CM3212_value_12bit= CM3212_value_12bit | CM3212_value_LSB;
	CM3212_value_12bit= CM3212_value_12bit >> 4;

	lux = CM3212_value_12bit; 
	lux_sub = lux - lux_old;
	lux_v = lux;

	TR1("lux %d\n", lux);
	if (swich < 3) {
		swich++;
		goto out;
	}

	if (lux_sub < 0) {
		lux_sub = lux_sub * -1;
	}
	if (lux_sub < 100) {
		count = 0;
		goto out;
	}

	if(count > 2)
		count = 0;
	else {
		count++;
		goto out;
	}

	lux_old = lux;

	TR1("lux %d++++++++++++++++++++++++\n", lux);

	input_report_abs(cm3212_dev, ABS_MISC, lux);
	input_report_abs(cm3212_dev, ABS_Y, 0);
	input_report_abs(cm3212_dev, ABS_Z, 0);
	input_sync(cm3212_dev);
out:
	if(g_switch == ON)
		mod_timer(&lightsensor_timer, jiffies + (HZ/10) );
}
DECLARE_DELAYED_WORK(lightsensor_work, lightsensor_bh_handler);
static void lightsensor_prepare_handler(struct work_struct *work)
{
	cm3212_i2c_write(CMDREG, 0x0e);
	schedule_delayed_work(&lightsensor_work,HZ);
}

DECLARE_WORK(lightsensor_prepare_work, lightsensor_prepare_handler);

void lightsensor_timer_fn(unsigned long arg)
{
	//TR1("lightsensor_timer_fn\n");
	schedule_work(&lightsensor_prepare_work);
}

static int cm3212_chip_on(void)
{
	TR1("chip on\n");
	TR1("init_timer\n");

	g_switch = ON;
	lightsensor_timer.function = lightsensor_timer_fn;
	mod_timer(&lightsensor_timer, jiffies + (1 * HZ));	//////////////////////////////////////

	return 0; // returns result of last i2cwrite
}

static int cm3212_chip_off(void)
{
	TR1("cm3212 chip_off\n");
	g_switch = OFF;
	//del_timer_sync(&lightsensor_timer);

	return 0;
}

//static int cm3212_ioctl(struct inode *inode, struct file *file, unsigned int cmd, unsigned long arg)
static long cm3212_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{

	int ret = 0, flags = 0;

	TR1("ioctl\n");

	switch (cmd)
	{
		case LIGHTSENSOR_IOCTL_ENABLE:
			copy_from_user(&flags, (void*)arg, sizeof(short));
			TR1("cm3212 On flags:%d\n", flags);

			if(flags == 1) {	///////////////flags == 1	//wwl
				TR1("cm3212 On\n");
				ret = cm3212_chip_on();
				if(ret == -1) {
					ret = 0;
				}
				if(ret == 0) {
					ret = 1;
				}
				TR1("cm3212 On ret:%d\n", ret);

				return ret;
			}
			if(flags == 0) {	///////////////////////////////////////////////////////wwl

				TR1("cm3212 Off\n");
				ret = cm3212_chip_off();
				if(ret == -1)
					ret = 0;
				if(ret == 0)
					ret = 1;

				TR1("cm3212 Off ret:%d\n", ret);
				return ret;
			}
			break;

		case LIGHTSENSOR_IOCTL_GET_VALUE:
			if(copy_to_user((void *)arg, &lux_v, sizeof(int))) {
				ret = -EFAULT;
			}
			
			break;
		default:
			return (-EINVAL);
			break;

	}

	return ret;

}

int cm3212_i2c_read(u8 reg)
{
	int res;

	cm3212_client_p->addr = reg/2;
	res = i2c_smbus_read_byte(cm3212_client_p);

	return res;
}

int cm3212_i2c_write(u8 reg, u8 val)
{
	int retval;

	cm3212_client_p->addr = reg/2;
	if ((retval = (i2c_smbus_write_byte(cm3212_client_p, val))) < 0)
	{
		TR0("FAILED: i2c_smbus_write_byte retval:%d\n", retval);
		return (retval);
	}

	return 0;
}

int cm3212_i2c_write_command(u8 reg)
{
	int result;
	char *p;
	char buf;

	p = &buf;
	*p = reg;
	// write the data
	if ((result = (i2c_smbus_write_byte(cm3212_client_p, (*p)))) < 0)
	{
		TR0("FAILED: i2c_smbus_write_byte reg:0x%x, result:%d\n", *p, result);
		return (result);
	}

	return 0;
}
static int __init cm3212_init(void)
{
	TR0("init\n");

	return i2c_add_driver(&cm3212_driver);
}

static void __exit cm3212_exit(void)
{
	TR0("exit\n");

	return i2c_del_driver(&cm3212_driver);
}

module_init( cm3212_init);
module_exit( cm3212_exit);

MODULE_AUTHOR("wangweilin<wangweilin@nufront.com>");
MODULE_DESCRIPTION("cm3212 lightsensor driver");
MODULE_LICENSE("GPL");
