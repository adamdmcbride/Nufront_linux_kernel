/* drivers/misc/telink_mu600.c
 *
 * Copyright (C) 2012 Nufront, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <mach/gpio.h>
#include <linux/debugfs.h>
#include <linux/miscdevice.h>
#include <mach/irqs-ns115.h>
#include <linux/wakelock.h>
#include <linux/irq.h>
#include <linux/interrupt.h>

//MU600 GPIO PIN
#define		GPIO_3G_RST 		(8+32+21)   	//GPIO_B21
#define		GPIO_3G_PWR 		(8+32+22)  	    //GPIO_B22
#define		GPIO_3G_WKAP 		(3)    		    //WAKEUP_GPIO3
#define		GPIO_3G_WKBP 		(8+19)          //GPIO_A19
//#define		GPIO_3G_UPDATE 		(8+32+9)    //GPIOB9

#define dbg(fmt, args...) printk(KERN_INFO "===MU600_3G_Module: " fmt, ##args)

static unsigned int reset_enable;
static unsigned int power_value;
static unsigned int wkbp_value;
struct wake_lock mdm_wake_lock;

static irqreturn_t telink_wakeup_irq_handler(int irq, void *dev_id)
{
	dbg("==============MU600 wakeup NS115 for 10s================\n");
	wake_lock_timeout(&mdm_wake_lock, 10*HZ);
	return IRQ_HANDLED;
}

void mu600_power_on(void)
{
	dbg("========mu600 power on===========\n");
	gpio_set_value(GPIO_3G_PWR,0);
	msleep(10);
	gpio_set_value(GPIO_3G_PWR,1);
	msleep(2000);
	gpio_set_value(GPIO_3G_PWR,0);
}

void mu600_power_off(void)
{
	dbg("========mu600 power off===========\n");
	gpio_set_value(GPIO_3G_PWR,0);
	msleep(10);
	gpio_set_value(GPIO_3G_PWR,1);
	msleep(10000);
	gpio_set_value(GPIO_3G_PWR,0);
}

void mu600_reset(void)
{
	dbg("========mu600 reset===========\n");
	gpio_set_value(GPIO_3G_RST,0);
	msleep(10);
	gpio_set_value(GPIO_3G_RST,1);
	msleep(200);
	gpio_set_value(GPIO_3G_RST,0);
}

void mu600_gpio_init(void)
{
	int ret = 0;

	dbg("==============3G Power On Start================\n");
	ret = gpio_request(GPIO_3G_RST, "3g-reset");
	if (ret != 0) {
		printk("gpio %d request fail!\n", GPIO_3G_RST);
		return ;
	}

	ret = gpio_request(GPIO_3G_PWR, "3g-pwr");
	if (ret != 0) {
		printk("gpio %d request fail!\n", GPIO_3G_PWR);
		return ;
	}

	ret = gpio_request(GPIO_3G_WKBP, "3g-wakeup-bp");
	if (ret != 0) {
		printk("gpio %d request fail!\n", GPIO_3G_WKBP);
		return;
	}

	ret = gpio_request(GPIO_3G_WKAP, "3g-wakeup-ap");
	if (ret != 0) {
		printk("gpio %d request fail!\n", GPIO_3G_WKAP);
		return;
	}

	//wake up bp first
	gpio_direction_output(GPIO_3G_WKBP, 1);
	gpio_set_value(GPIO_3G_WKBP,1);
	wkbp_value=1;

	dbg("==============3G Reset================\n");
	gpio_direction_output(GPIO_3G_RST, 0);
	gpio_set_value(GPIO_3G_RST,0);
	reset_enable=0;

	dbg("==============3G Power================\n");
	gpio_direction_output(GPIO_3G_PWR, 0);
	gpio_set_value(GPIO_3G_PWR,0);
	mdelay(10);
	gpio_set_value(GPIO_3G_PWR,1);
	mdelay(2000);
	gpio_set_value(GPIO_3G_PWR,0);
	power_value=1;

	gpio_direction_output(GPIO_3G_WKAP, 1);
	wake_lock_init(&mdm_wake_lock, WAKE_LOCK_SUSPEND, "telink_usb_mdm_lock");

	ret  = request_irq(IRQ_NS115_GPIO0_WAKEUP_3, telink_wakeup_irq_handler,	IRQF_TRIGGER_RISING,"mu600_wakeup", NULL);
	if (ret != 0) {
		dbg("request irq failed\n");
	}

	enable_irq_wake(IRQ_NS115_GPIO0_WAKEUP_3);
	disable_irq_nosync(IRQ_NS115_GPIO0_WAKEUP_3);
}

// Power BP
static ssize_t show_pwr_value(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", power_value);
}

static ssize_t set_pwr_value(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t count)
{
	unsigned int tmp;
	tmp = (unsigned int)simple_strtoul(buf, NULL, 10);

	dbg("set_pwr_value:%d\n",tmp);

	if (tmp) {
		tmp=1;
	}
	power_value = tmp;

	if (power_value) {
		mu600_power_on();
	} else {
		mu600_power_off();
	}

	return count;
}

//Reset BP
static ssize_t show_reset_value(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", reset_enable);
}

static ssize_t set_reset_value(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t count)
{
	unsigned int tmp;
	tmp=(unsigned int)simple_strtoul(buf, NULL, 10);
	dbg("set_reset_value:%d\n", tmp);

	if (tmp) {
		tmp=1;
	}
	reset_enable = tmp;

	if (reset_enable) {
		mu600_reset();
		reset_enable=0;
	}

	return count;
}

// Wakp up BP
static ssize_t show_wkbp_value(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", wkbp_value);
}

static ssize_t set_wkbp_value(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t count)
{
	unsigned int tmp;
	tmp = (unsigned int)simple_strtoul(buf, NULL, 10);

	//	dbg("set_wkbp_value:%d\n",tmp);

	if (tmp) {
		tmp=1;
	}
	wkbp_value = tmp;

	if (wkbp_value) {
		gpio_set_value(GPIO_3G_WKBP,1); //wake
	} else {
		gpio_set_value(GPIO_3G_WKBP,0); //sleep
	}

	return count;
}

static DEVICE_ATTR(pwr, S_IRUGO | S_IWUSR, show_pwr_value, set_pwr_value);
static DEVICE_ATTR(reset, S_IRUGO | S_IWUSR, show_reset_value, set_reset_value);
static DEVICE_ATTR(wkbp, S_IRUGO | S_IWUSR, show_wkbp_value, set_wkbp_value);

static struct attribute *mu600_attributes[] = {
	&dev_attr_pwr.attr,
	&dev_attr_reset.attr,
	&dev_attr_wkbp.attr,
    NULL
};

static struct attribute_group mu600_attribute_group = {
	.attrs = mu600_attributes
};

static int mu600_open(struct inode *inode, struct file *file)
{
	return 0;
}

static int mu600_close(struct inode *inode, struct file *file)
{
	return 0;
}

static const struct file_operations mu600_fops = {
	.owner		= THIS_MODULE,
	//.unlocked_ioctl	= mu600_ioctl,
	.open		= mu600_open,
	.release	= mu600_close,
};

static struct miscdevice mu600_miscdev = {
	.minor	= MISC_DYNAMIC_MINOR,
	.name	= "mu600",
	.fops	= &mu600_fops,
};

static int mu600_gpio_probe(struct platform_device *pdev)
{
	int err;

	mu600_gpio_init();

	err = misc_register(&mu600_miscdev);
	if (err) {
			printk("misc. device failed to register.\n");
			return err;
	}

	err = sysfs_create_group(&mu600_miscdev.this_device->kobj, &mu600_attribute_group);
	if (err) {
			printk("sysfs create failed: %d\n", err);
			goto err_misc;
	}

	return 0;
err_misc:
	misc_deregister(&mu600_miscdev);
	return err;
}

static int mu600_gpio_remove(struct platform_device *pdev)
{
	sysfs_remove_group(&pdev->dev.kobj, &mu600_attribute_group);
	misc_deregister(&mu600_miscdev);
	gpio_free(GPIO_3G_RST);
	gpio_free(GPIO_3G_PWR);
	gpio_free(GPIO_3G_WKBP);
	gpio_free(GPIO_3G_WKAP);

	return 0;
}

#ifdef CONFIG_PM
static int mu600_gpio_suspend(struct platform_device *pdev, pm_message_t state)
{
	dbg("=============ENABLE_MU600_IRQ\n");
	enable_irq(IRQ_NS115_GPIO0_WAKEUP_3);
	gpio_direction_input(GPIO_3G_WKAP);
	return 0;
}
static int mu600_gpio_resume(struct platform_device *pdev)
{
	dbg("==============DISABLE_MU600_IRQ\n");
	disable_irq_nosync(IRQ_NS115_GPIO0_WAKEUP_3);
	gpio_direction_output(GPIO_3G_WKAP, 1);
	return 0;
}
#else
#define mu600_gpio_suspend 	NULL
#define mu600_gpio_resume 	NULL
#endif

static struct platform_driver mu600_gpio_driver = {
	.driver = {
		   .name = "mu600_gpio",
		   .owner = THIS_MODULE,
	},
	.probe 		= mu600_gpio_probe,
	.remove 	= __devexit_p(mu600_gpio_remove),
	.suspend	= mu600_gpio_suspend,
	.resume		= mu600_gpio_resume,
};

static int __init mu600_init(void)
{
	return platform_driver_register(&mu600_gpio_driver);
}

static void __exit mu600_exit(void)
{
	platform_driver_unregister(&mu600_gpio_driver);
}

module_init(mu600_init);
module_exit(mu600_exit);

MODULE_DESCRIPTION("Telink MU600 3G Module GPIO Control");
MODULE_AUTHOR("Nufront");
MODULE_LICENSE("GPL");

