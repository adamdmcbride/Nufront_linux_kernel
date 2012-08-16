/*
 * Platform /driver for the Smart Battery
 *
 * Copyright (C) 2010 Nufront Ltd
 *
 * Author: 
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

//#include <linux/kernel.h>
#include <linux/module.h>
//#include <linux/types.h>
//#include <linux/errno.h>
//#include <linux/swab.h>
#include <linux/i2c.h>
#include <linux/idr.h>
#include <linux/slab.h>
#include <linux/power_supply.h>
#include <linux/platform_device.h>
#include <linux/mfd/io373x.h>

//#define	 PRINT_DBG_INFO

#ifdef	PRINT_DBG_INFO 
	#define DBG_PRINT(fmt, args...) printk( KERN_EMERG "smart_bat: " fmt, ## args) 
#else 
	#define DBG_PRINT(fmt, args...) /* not debugging: nothing */ 
#endif 

#define NUFRONT_MANUFACTURER	"Nufront"
#define BATTERY_SERIAL		"EC0112"
#define BATTERY_MODEL		"bq27x00"

#define	AC_PRESENT		(1 << 6)
#define	BATERRY_PRESENT		(1 << 5)
#define	BATERRY_CHARGING	(1 << 4)
#define	BATERRY_CHARGE_FULL	(1 << 3)
#define	IO373X_REG_BAT_IN	(0x8400)
//#define	IO373X_REG_BAT_IN_SIZE	(14)
#define	IO373X_REG_BAT_IN_SIZE	(24)
struct baterry_data {
	unsigned char ctrl;
	unsigned char err;
	unsigned char current_h;	//ua
	unsigned char current_l;	//ua   current
	unsigned char voltage_h;	//mv
	unsigned char voltage_l;	//mv   voltage
	unsigned char design_vol_h;	//uv
	unsigned char design_vol_l;	//uv   design voltage
	unsigned char design_cap_h;	//uwh
	unsigned char design_cap_l;	//uwh  design capablity
	unsigned char remain_cap_h;	//remain capablity
	unsigned char remain_cap_l;
	unsigned char full_cap_h;	//full capablity
	unsigned char full_cap_l;
	unsigned char status_h;
	unsigned char status_l;
	unsigned char rsoc_h;		//raw capablity rate
	unsigned char rsoc_l;
	unsigned char temperature_h;	// (temperature)/10 - 273
	unsigned char temperature_l;
	unsigned char technology_h2;
	unsigned char technology_h1;
	unsigned char technology_l2;
	unsigned char technology_l1;
};

struct smart_info {
	struct power_supply	battery;
	struct power_supply	charger;
	unsigned char		bat_data[32];
	int			id;
	unsigned short		subdev_id;
    	struct io373x 		*io373x;
    	struct work_struct      battery_work;
	struct timer_list 	poll_timer;
};

static DEFINE_IDR(battery_id);
static DEFINE_MUTEX(battery_lock);

unsigned char	g_bat_buf[32];

static void io3730_power_handler(unsigned long data)
{
	struct smart_info * info = (struct smart_info*)data;
	mod_timer(&info->poll_timer, jiffies + msecs_to_jiffies(5000));

	power_supply_changed(&info->battery);	
	power_supply_changed(&info->charger);	
    	queue_work(io373x_wq, &info->battery_work);
}

static void battery_read_work(struct work_struct *work)
{
    struct smart_info * info = 	container_of(work,struct smart_info, battery_work);
    struct io373x *io373x = info->io373x; 
    int	err = 0, i ;
#if 1
	if(io373x != NULL) {
 		//printk(KERN_EMERG "Io3730_power_handler io373x = %x\n", io373x);
		err = io373x_read_regs(io373x, 
			IO373X_REG_BAT_IN, 
			g_bat_buf,//info->bat_data, 
			IO373X_REG_BAT_IN_SIZE); 
		if(err == 0) {
			for(i = 0; i < IO373X_REG_BAT_IN_SIZE; i++) {
		//		printk(KERN_EMERG "%02x ", g_bat_buf[i]);
			}
		//	printk(KERN_EMERG "\n");
			memcpy(info->bat_data, g_bat_buf, IO373X_REG_BAT_IN_SIZE);
		}
	}
#else
	unsigned char* puc_ptr;
	unsigned char uc_count;

	if(io373x != NULL) {
		puc_ptr = g_bat_buf;
		uc_count = IO373X_REG_BAT_IN_SIZE;
		do {
			err = io373x_read_regs(io373x, IO373X_REG_BAT_IN, puc_ptr, 
		} while (err == 0);
		err = io373x_read_regs(io373x, 
			IO373X_REG_BAT_IN, 
			g_bat_buf, 
			IO373X_REG_BAT_IN_SIZE); 
		if(err == 0) {
			memcpy(info->bat_data, g_bat_buf, IO373X_REG_BAT_IN_SIZE);
		}
	}

#endif	    		
}


static int smart_get_current(struct smart_info *info, int *current_uA)
{
	struct  baterry_data * bat  = (struct baterry_data *)info->bat_data;
	int	current_v =  bat->current_l + (bat->current_h << 8);
	if(current_v & 0x8000)
		*current_uA = 0xffff - current_v;
	else
		*current_uA = current_v;

		*current_uA *= 1000;

	DBG_PRINT("smart_get_current: current_uA = %x, l = %x, h=%x \n", 
		*current_uA, 
		bat->current_l, 
		bat->current_h);
	return 0;
}

static int smart_get_voltage(struct smart_info *info, int *voltage_uv)
{
	struct  baterry_data * bat  = (struct baterry_data *)info->bat_data; 
	*voltage_uv = (bat->voltage_l + (bat->voltage_h << 8)) * 1000;
	DBG_PRINT("smart_get_voltage: voltage_uv = %x, l = %x, h=%x \n", 
		*voltage_uv, 
		bat->voltage_l, 
		bat->voltage_h);

	return 0;
}

static int smart_get_capacity(struct smart_info *info, int *capacity)
{
	struct  baterry_data * bat  = (struct baterry_data *)info->bat_data;
	int	rc = bat->remain_cap_l + (bat->remain_cap_h << 8);
	int 	fc = bat->full_cap_l + (bat->full_cap_h << 8);

	if(rc <= fc && fc > 0)
		*capacity = rc * 100 / fc;
	else
		* capacity = 1; 
	DBG_PRINT("smart_get_capacity: capacity = %d, rl = %x, rh=%x, fl = %x, fh = %x \n", 
		*capacity, 
		bat->remain_cap_l, 
		bat->remain_cap_h,
		bat->full_cap_l,
		bat->full_cap_h);

	return 0;
}

static int smart_get_temperature(struct smart_info *info, int *temperature)
{
	struct  baterry_data * bat  = (struct baterry_data *)info->bat_data;
	
	*temperature = (bat->temperature_l + (bat->temperature_h << 8))/10;
	
	DBG_PRINT("smart_get_temperature: temperature= %d, rl = %x, rh=%x\n", 
		*temperature, 
		bat->temperature_l, 
		bat->temperature_h);

	return 0;
}

#define OVER_CHARGED_ALARM		(0x80000)
#define TERMINATE_CHARGE_ALARM		(0x40000)
#define OVER_TEMP_ALARM			(0x10000)
#define TERMINATE_DISCHARGE_ALARM	(0x0800)
#define REMAINING_CAPACITY_ALARM	(0x0200)
#define REMAINING_TIME_ALARM		(0x0100)
#define INITIALIZED			(0x0080)
#define DISCHARGING			(0x0040)
#define FULLY_CHARGED			(0x0020)
#define FULLY_DISCHARGED		(0x0010)

static int smart_get_health(struct smart_info *info, int *health)
{
	struct  baterry_data * bat  = (struct baterry_data *)info->bat_data;
	
	unsigned int status = (bat->status_l + (bat->status_h << 8));
	
	DBG_PRINT("smart_get_health: health = %d, rl = %x, rh=%x\n", 
		status, 
		bat->status_l, 
		bat->status_h);

	if(status & OVER_TEMP_ALARM)
	{
		*health = POWER_SUPPLY_HEALTH_OVERHEAT; 
		return 0;
	}

	*health = POWER_SUPPLY_HEALTH_GOOD;
	return 0; 
}

static int smart_get_technology(struct smart_info *info, int *technology)
{
	struct  baterry_data * bat  = (struct baterry_data *)info->bat_data;

	char type[4];

	sprintf(type,"%c%c%c%c", bat->technology_h2, bat->technology_h1,
			bat->technology_l2,bat->technology_l1);	
	
	DBG_PRINT("smart_get_technology: technology = %s, technology_h2 = %c, technology_h1=%c, \
			technology_l2 = %c, technology_l1 = %c\n",
		type, bat->technology_h2, bat->technology_h1,
		bat->technology_l2,bat->technology_l1);

	if(0 == strcmp("POLY", type)){
		*technology = POWER_SUPPLY_TECHNOLOGY_LION;
		return 0;
	}else if(0 == strcmp("NiCd", type)){
		*technology = POWER_SUPPLY_TECHNOLOGY_NiCd;
		return 0;
	}else if(0 == strcmp("NiMH", type)){
		*technology = POWER_SUPPLY_TECHNOLOGY_NiMH;
		return 0;
	}else if(0 == strcmp("LiP", type)){
		*technology = POWER_SUPPLY_TECHNOLOGY_LIPO;
		return 0;
	}else{
		*technology = POWER_SUPPLY_TECHNOLOGY_UNKNOWN;
		return 0;
	}

}

static int smart_get_batterry_status(struct smart_info *info, int *present)
{
	struct  baterry_data * bat  = (struct baterry_data *)info->bat_data;

	if((bat->ctrl & BATERRY_PRESENT) == 0)
	{
		*present = 0;	
	}else{
		*present = 1;
	}	
	
	DBG_PRINT("smart_get_batterry_status: present= %d\n", *present); 

	return 0;
}

static int smart_get_status(struct smart_info *info, int *status)
{
	struct  baterry_data * bat  = (struct baterry_data *)info->bat_data;

	*status = POWER_SUPPLY_STATUS_UNKNOWN;

	if((bat->ctrl & BATERRY_PRESENT) == 0)
	{	
		return -EINVAL;
	}

	if(bat->ctrl & AC_PRESENT) {
		if(bat->ctrl & BATERRY_CHARGE_FULL)
			*status = POWER_SUPPLY_STATUS_FULL;
		else if(bat->ctrl & BATERRY_CHARGING)
			*status = POWER_SUPPLY_STATUS_CHARGING;
	}
	else if(bat->ctrl & BATERRY_PRESENT) //ac not present
		*status = POWER_SUPPLY_STATUS_DISCHARGING;
	else 	
		*status = POWER_SUPPLY_STATUS_NOT_CHARGING;

	DBG_PRINT("smart_get_status: status= %x, baterry control = %x err = %x\n", *status, bat->ctrl, bat->err); 

	return 0;
}

static int smart_charger_get_property(struct power_supply *psy,
			       enum power_supply_property psp,
			       union power_supply_propval *val)
{
	struct smart_info *info = dev_get_drvdata(psy->dev->parent);
	struct  baterry_data * bat  = (struct baterry_data *)info->bat_data;

	DBG_PRINT("smart_charge_get_property, psp = %x\n", psp);

	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		if(bat->ctrl &  AC_PRESENT)
			val->intval = 1;
		else
			val->intval = 0;
		DBG_PRINT("val->intval = %d\n", val->intval);
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static int smart_battery_get_property(struct power_supply *psy,
				       enum power_supply_property prop,
				       union power_supply_propval *val)
{
	struct smart_info *info = dev_get_drvdata(psy->dev->parent);
	int ret;

	struct  baterry_data * bat  = (struct baterry_data *)info->bat_data;
	DBG_PRINT("smart_battery_get_property, prop = %x\n", prop);

	if((bat->ctrl & BATERRY_PRESENT) == 0)
	{
		DBG_PRINT("battery not present!\n");
		return -EINVAL;
	}

	switch (prop) {



	case POWER_SUPPLY_PROP_HEALTH:
		ret = smart_get_health(info, &val->intval);
		break;
	case POWER_SUPPLY_PROP_PRESENT:
		ret = smart_get_batterry_status(info, &val->intval);
		break;
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = 1;
		break;
	case POWER_SUPPLY_PROP_TECHNOLOGY:
		ret = smart_get_technology(info, &val->intval);		
		break;



	case POWER_SUPPLY_PROP_STATUS:
		ret = smart_get_status(info, &val->intval);
		break;



	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
	case POWER_SUPPLY_PROP_VOLTAGE_AVG:
		//reg9 Voltage (mV)  reg1a SpecificationInfo 
		ret = smart_get_voltage(info, &val->intval);
		break;
	case POWER_SUPPLY_PROP_CURRENT_AVG:
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		ret = smart_get_current(info, &val->intval);
		break;
	case POWER_SUPPLY_PROP_CURRENT_MAX:
		val->intval = 6000000;








	case POWER_SUPPLY_PROP_CAPACITY:
		ret = smart_get_capacity(info, &val->intval);
		break;
        case POWER_SUPPLY_PROP_CAPACITY_LEVEL:
                val->intval = POWER_SUPPLY_CAPACITY_LEVEL_NORMAL;
                break;

	case POWER_SUPPLY_PROP_TEMP: 
		ret = smart_get_temperature(info, &val->intval);
		break;
        case POWER_SUPPLY_PROP_TEMP_AMBIENT:
                val->intval = 316;
                break;

        case POWER_SUPPLY_PROP_TIME_TO_EMPTY_NOW:
                val->intval = 600;
                break;
        case POWER_SUPPLY_PROP_TIME_TO_EMPTY_AVG:
                val->intval = 600; 
                break;
        case POWER_SUPPLY_PROP_TIME_TO_FULL_NOW:
                val->intval = 60;
                break;
        case POWER_SUPPLY_PROP_TIME_TO_FULL_AVG:
                val->intval = 60;
                break;

        case POWER_SUPPLY_PROP_MODEL_NAME:
                val->strval = BATTERY_MODEL;
                break;
        case POWER_SUPPLY_PROP_MANUFACTURER:
                val->strval = NUFRONT_MANUFACTURER;
                break;
        case POWER_SUPPLY_PROP_SERIAL_NUMBER:
                val->strval = BATTERY_SERIAL;
                break;

	default:
		ret = -EINVAL;
	}

	return ret;
}


static enum power_supply_property smart_charger_props[] = {
	POWER_SUPPLY_PROP_ONLINE,
};

static void smart_charger_power_supply_init(struct power_supply *charger)
{
	charger->type			= POWER_SUPPLY_TYPE_MAINS;
	charger->properties		= smart_charger_props;
	charger->num_properties		= ARRAY_SIZE(smart_charger_props);
	charger->get_property		= smart_charger_get_property;
	charger->external_power_changed	= NULL;
}


static enum power_supply_property smart_battery_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_TECHNOLOGY,
};

static void smart_power_supply_init(struct power_supply *battery)
{
	battery->type			= POWER_SUPPLY_TYPE_BATTERY;
	battery->properties		= smart_battery_props;
	battery->num_properties		= ARRAY_SIZE(smart_battery_props);
	battery->get_property		= smart_battery_get_property;
	battery->external_power_changed	= NULL;
}

static int smart_bat_remove(struct platform_device *dev)
{
	struct smart_info *info = platform_get_drvdata(dev);

	if(info->battery.name) {
		power_supply_unregister(&info->battery);
		kfree(info->battery.name);
	}

	if(info->charger.name) {
		power_supply_unregister(&info->charger);
		kfree(info->charger.name);
	}

	mutex_lock(&battery_lock);
	idr_remove(&battery_id, info->id);
	mutex_unlock(&battery_lock);

	kfree(info);
	return 0;
}

static int regiter_smart_charger(struct platform_device *dev, struct smart_info * info , int num)
{
	int	ret = 0;

	//info->charger.name = kasprintf(GFP_KERNEL, "sbcharg-%d", num);
	info->charger.name = "ac";
 
	if (!info->charger.name) {
		ret = -ENOMEM;
		return ret;
	}

	smart_charger_power_supply_init(&info->charger);

	ret = power_supply_register(&dev->dev, &info->charger);
	if (ret) {
		kfree(info->charger.name);
		DBG_PRINT("failed to register charger\n");
		return ret; 
	}

	DBG_PRINT("register smart charger success\n");
	return 0;
}

static int smart_bat_probe(struct platform_device *dev)
{
	struct smart_info *info;
	int ret, err, i;
	int num;
	struct baterry_data * bat;
    	struct io373x *io373x = dev_get_drvdata(dev->dev.parent);


	printk("smart_bat_probe....................................................\r\n");
	io373x = dev_get_drvdata(dev->dev.parent);
	/* Get an ID for this battery */
	ret = idr_pre_get(&battery_id, GFP_KERNEL);
	if (ret == 0) {
		ret = -ENOMEM;
		goto fail_id;
	}

	mutex_lock(&battery_lock);
	ret = idr_get_new(&battery_id, dev, &num);
	mutex_unlock(&battery_lock);
	if (ret < 0)
		goto fail_id;

	info = kzalloc(sizeof(*info), GFP_KERNEL);
	if (!info) {
		ret = -ENOMEM;
		goto fail_info;
	}

	info->battery.name = "battery";
	if (!info->battery.name) {
		ret = -ENOMEM;
		goto fail_name;
	}
	err = io373x_read_regs(io373x, IO373X_REG_BAT_IN, g_bat_buf, IO373X_REG_BAT_IN_SIZE);
	if(err == 0) {
		for(i = 0; i < IO373X_REG_BAT_IN_SIZE; i++) {
			printk("%02x ", g_bat_buf[i]);
		}
		printk("\n");
		memcpy(info->bat_data, g_bat_buf, IO373X_REG_BAT_IN_SIZE);
		// Check if battery exists
		if ((g_bat_buf[0] == 0) || ((g_bat_buf[0] & BATERRY_PRESENT) == 0)) {
			goto fail_name;
		}
	}
	else {
		goto fail_name;
	}

	info->io373x = io373x;
	INIT_WORK(&info->battery_work, battery_read_work);
	platform_set_drvdata(dev, info);

	smart_power_supply_init(&info->battery);
	ret = power_supply_register(&dev->dev, &info->battery);
	if (ret) {
		DBG_PRINT("failed to register battery\n");
		goto fail_register;
	}
	
	DBG_PRINT("register smart battery success, io373x = %x\n", 
			(unsigned int)info->io373x);

	regiter_smart_charger(dev, info, num);

	bat = (struct baterry_data *)info->bat_data;

	init_timer(&info->poll_timer);
	info->poll_timer.function = io3730_power_handler;
	info->poll_timer.data = (unsigned int)info;
	info->poll_timer.expires = jiffies + msecs_to_jiffies(2000);
		
	add_timer(&info->poll_timer);
	return 0;

fail_register:
	kfree(info->battery.name);
fail_name:
	kfree(info);
fail_info:
	mutex_lock(&battery_lock);
	idr_remove(&battery_id, num);
	mutex_unlock(&battery_lock);
fail_id:
	return ret;
}

static struct platform_driver smart_bat_driver = {
	.driver	= {
		.name	= "io373x-battery",
		.owner	= THIS_MODULE,
	},
	.probe		= smart_bat_probe,
	.remove		= __devexit_p(smart_bat_remove),
};

static int __init smart_bat_init(void)
{
	return platform_driver_register(&smart_bat_driver);
}

static void __exit smart_bat_exit(void)
{
	platform_driver_unregister(&smart_bat_driver);
}

module_init(smart_bat_init);
module_exit(smart_bat_exit);

MODULE_AUTHOR("zhijie.hao <zhijie.hao@nufront.com>");
MODULE_DESCRIPTION("Smart Battery driver");
MODULE_LICENSE("GPL");

