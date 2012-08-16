/*
 * BQ27410 battery driver
 *
 * Copyright (C) 2008 Rodolfo Giometti <giometti@linux.it>
 * Copyright (C) 2008 Eurotech S.p.A. <info@eurotech.it>
 * Copyright (C) 2010-2011 Lars-Peter Clausen <lars@metafoo.de>
 * Copyright (C) 2011 Pali Rohár <pali.rohar@gmail.com>
 *
 * Based on a previous work by Copyright (C) 2008 Texas Instruments, Inc.
 *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * THIS PACKAGE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
 * WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 */


#include <linux/module.h>
#include <linux/param.h>
#include <linux/jiffies.h>
#include <linux/workqueue.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/idr.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <asm/unaligned.h>

#include <linux/power/ns115-battery.h>

//#include <linux/power/bq27410_battery.h>

#define DRIVER_VERSION			"1.0.0"

#define BQ27410_REG_CONTROL         0x00

#define BQ27410_REG_TEMP		0x02
#define BQ27410_REG_VOLT		0x04
#define BQ27410_REG_FLAGS		0x06
#define BQ27410_REG_NAC			0x08 /* Nominal available capaciy */
#define BQ27410_REG_FAC			0x0a /* Full available capaciy */
#define BQ27410_REG_RM			0x0c /* Remaining capaciy */
#define BQ27410_REG_FCC			0x0e /* Full charge capaciy */

#define BQ27410_REG_AC			0x10 /* average current */
#define BQ27410_REG_AE			0x16 /* Available energy */

#define BQ27410_REG_SOC			0x1c /* Relative State-of-Charge */

#define BQ27410_STATUS_SS       0x2000

#define BQ27410_FLAG_FC			BIT(9)  /* full charged */
#define BQ27410_FLAG_CHG	    BIT(8)  /* fast-charge capablity*/
#define BQ27410_FLAG_BATIN		BIT(4)  /* full charged */
#define BQ27410_FLAG_DCHG		BIT(0)  /* discharging */


/* extend commands */
#define BQ27410_REG_DCAP            0x3c    /* Designed Capacity. mAh */

#define BQ27410_REG_DATA_FLASH_CLASS         0x3e
#define BQ27410_REG_DATA_FLASH_BLOCK_IDX     0x3f

#define BQ27410_REG_BLOCK_DATA_BASE         0x40
#define BQ27410_REG_BLOCK_DATA_SIZE         32

#define BQ27410_REG_BLOCK_DATA_CHECKSUM     0x60
#define BQ27410_REG_BLOCK_DATA_CONTROL      0x61

#define BQ27410_RS			10 /* Resistor sense */

struct bq27410_gasgauge_info {
	struct device      *dev;    // point to i2c_client->dev
	struct mutex		lock;
};

static struct bq27410_gasgauge_info * g_info;

/*
 * issure a host command to set battery_in detected.
 * because the chip is waiting BATTERY_IN event, after power reset.
 */
static int bq27410_gasgauge_set_bat_in(struct bq27410_gasgauge_info *di)
{
	struct i2c_client *client = to_i2c_client(di->dev);
	int ret;

#ifdef DEBUG
	ret = i2c_smbus_read_word_data(client, BQ27410_REG_FLAGS);
    dev_dbg(di->dev, "flags0: %x\n", ret);

	ret = i2c_smbus_write_word_data(client, BQ27410_REG_CONTROL, 0x000d);

	ret = i2c_smbus_read_word_data(client, BQ27410_REG_FLAGS);
    dev_dbg(di->dev, "flags1: %x\n", ret);
#endif

	ret = i2c_smbus_write_word_data(client, BQ27410_REG_CONTROL, 0x000c);

	mdelay(1);
	ret = i2c_smbus_read_word_data(client, BQ27410_REG_FLAGS);
    dev_dbg(di->dev, "flags2: %x\n", ret);

    return ret;
}

/*
 * set designed capacity of battery.
 * TODO: failed to set capacity by writing the reg BQ27410_REG_DCAP ...
 */
static int bq27410_gasgauge_set_design_capacity(
                    struct bq27410_gasgauge_info *di, u16 capacity_mAh)
{
	struct i2c_client *client = to_i2c_client(di->dev);
    unsigned short status;
    u8 buf[32];
    u8 crc;
    int i;
    int ret;

    ret = i2c_smbus_write_word_data(client, BQ27410_REG_CONTROL, 0x0000);
    if(ret < 0)
        return ret;

    ret = i2c_smbus_read_word_data(client, BQ27410_REG_CONTROL);
    if(ret < 0)
        return ret;

    dev_dbg(di->dev, "BQ27410_REG_CONTROL 0x%x.\n", ret);

    status = ret & 0xffff;
    if(status & BQ27410_STATUS_SS)
    {
        dev_warn(di->dev, "device in SEALED mode. cannot change data flash.\n");
        return -1;
    }

    ret = i2c_smbus_read_word_data(client, BQ27410_REG_DCAP);
    dev_info(di->dev, "orignal designed capacity %d mAh. [0x%04x]\n", ret, ret);

    if(ret == capacity_mAh){
        dev_info(di->dev, "the designed capacity meets requirement.\n");
        return 0;
    }
#if 0
    ret = i2c_smbus_write_word_data(client, BQ27410_REG_DCAP, capacity_mAh);
    if(ret < 0)
    {
        dev_warn(di->dev, "failed to set the designed capacity of battery.\n");
        return ret;
    }
#else
    ret = i2c_smbus_write_byte_data(client, BQ27410_REG_BLOCK_DATA_CONTROL, 0);
    if(ret < 0)
        return ret;

    /* DesignCapacity in Data block: subclass 48, offset 19 */
    ret = i2c_smbus_write_byte_data(client, BQ27410_REG_DATA_FLASH_CLASS, 48);
    if(ret < 0)
        return ret;

    ret = i2c_smbus_write_byte_data(client, BQ27410_REG_DATA_FLASH_BLOCK_IDX, 19/32);
    if(ret < 0)
        return ret;

    ret = i2c_smbus_read_byte_data(client, BQ27410_REG_BLOCK_DATA_CHECKSUM);
    if(ret < 0)
        return ret;

    dev_dbg(di->dev, "old checksum read: %x\n", ret);

    /* read block data */
    ret = i2c_smbus_read_i2c_block_data(client, BQ27410_REG_BLOCK_DATA_BASE, 32, buf);
    if(ret < 0)
        return ret;

    crc = 0;
	buf[19] = buf[20] = 0;
    for(i=0; i<32; i++){
        crc += buf[i];
    }
    dev_dbg(di->dev, "old checksum calculated: %x\n", 0xff-crc);

    /* write new data, MSB stored in data flash */
    ret = i2c_smbus_write_word_data(client, BQ27410_REG_BLOCK_DATA_BASE + 19, cpu_to_be16(capacity_mAh));
    if(ret < 0){
        dev_info(di->dev, "failed to write new data\n");
        return ret;
    }

    /* update new crc */
    crc += capacity_mAh & 0xff;
    crc += capacity_mAh >> 8;

    crc = 0xff - crc;

    ret = i2c_smbus_write_byte_data(client, BQ27410_REG_BLOCK_DATA_CHECKSUM, crc);
    if(ret < 0){
        dev_info(di->dev, "failed to write new crc\n");
        return ret;
    }

#endif
    return 0;
}



/*
 * bq27410_gasgauge_get_mvolts - read battery voltage in mV
 */
static int bq27410_gasgauge_get_mvolts(void)
{
	struct i2c_client *client = NULL;
    int ret;

    if(!g_info){
        pr_warning("Warn: bq27410 gasgauge device not registered.\n");
        return 0;
    }
    client = to_i2c_client(g_info->dev);

    ret = i2c_smbus_read_word_data(client, BQ27410_REG_VOLT);
    if(ret < 0)
        goto out;

    return ret & 0xffff;

out:
    dev_warn(&client->dev, "failed to read voltage. ret %x\n", ret);
    return 0;
}

/*
 * bq27410_gasgauge_get_capacity - read battery capacity in percentage %
 */
static int bq27410_gasgauge_get_capacity(void)
{
    struct i2c_client *client = NULL;
    int ret;

    if(!g_info){
        pr_warning("Warn: bq27410 gasgauge device not registered.\n");
        return 0;
    }
    client = to_i2c_client(g_info->dev);

    ret = i2c_smbus_read_word_data(client, BQ27410_REG_SOC);
    if(ret < 0 || ret > 100)
        goto out;

    return ret;

out:
    dev_warn(&client->dev, "failed to read voltage. ret %x\n", ret);
    return 0;

}

int bq27410_gasgauge_get_status(void)
{
    int status;
    struct i2c_client *client = NULL;
    int ret;

    if(!g_info){
        pr_warning("Warn: bq27410 gasgauge device not registered.\n");
        return 0;
    }
    client = to_i2c_client(g_info->dev);

    /* LSB in chip, but i2c_smbus_read_word_data store the LSB at high byte, so swap the 2 bytes */
    ret = i2c_smbus_read_word_data(client, BQ27410_REG_FLAGS);
    if(ret<0)
        return ret;

    dev_vdbg(&client->dev, "FLAGS 0x%x\n", ret);

    if(ret & BQ27410_FLAG_DCHG){
        status = POWER_SUPPLY_STATUS_DISCHARGING;
    }else if(ret & BQ27410_FLAG_FC){
    	status = POWER_SUPPLY_STATUS_FULL;
    }else{
        status = POWER_SUPPLY_STATUS_CHARGING;
    }
	return status;
}

static struct ns115_battery_gauge bq27410_gasgauge_gauge = {
	.get_battery_mvolts = bq27410_gasgauge_get_mvolts,
	.get_battery_capacity = bq27410_gasgauge_get_capacity,
	.get_battery_status = bq27410_gasgauge_get_status,
};

static __devinit int bq27410_gasgauge_probe(struct i2c_client *client,
				 const struct i2c_device_id *id)
{
	struct bq27410_gasgauge_info *info;
//	struct bq27410_gasgauge_platform_data *pdata;
	int ret;

	info = kzalloc(sizeof(struct bq27410_gasgauge_info), GFP_KERNEL);
	if (!info)
		return -ENOMEM;

	info->dev = &client->dev;
//  pdata = id->driver_data;

    i2c_set_clientdata(client, info);

	mutex_init(&info->lock);
    bq27410_gasgauge_set_bat_in(info);
    bq27410_gasgauge_set_design_capacity(info, 4000);

	g_info = info;

    ret = ns115_battery_gauge_register(&bq27410_gasgauge_gauge);
	if (ret){
		goto out;
	}

	dev_info(info->dev, "%s is OK!\n", __func__);
	return 0;
out:
	g_info = NULL;
	kfree(info);
	return ret;
}

static int __devexit bq27410_gasgauge_remove(struct i2c_client *client)
{
	struct bq27410_device_info *di = i2c_get_clientdata(client);

	kfree(di);
	i2c_set_clientdata(client, NULL);
	return 0;
}

#ifdef CONFIG_PM
static int bq27410_gasgauge_suspend(struct device *dev)
{
	struct bq27410_gasgauge_info *info = dev_get_drvdata(dev);

	return 0;
}

static int bq27410_gasgauge_resume(struct device *dev)
{
	struct bq27410_gasgauge_info *info = dev_get_drvdata(dev);

	return 0;
}

static struct dev_pm_ops bq27410_gasgauge_pm_ops = {
	.suspend	= bq27410_gasgauge_suspend,
	.resume		= bq27410_gasgauge_resume,
};
#endif

static const struct i2c_device_id bq27410_id[] = {
	{ "bq27410-gasgauge", 27410 },
	{},
};
MODULE_DEVICE_TABLE(i2c, bq27410_id);

static struct i2c_driver bq27410_gasgauge_driver = {
	.driver		= {
		.name	= "bq27410_gasgauge",
		.owner	= THIS_MODULE,
#ifdef CONFIG_PM
		.pm	= &bq27410_gasgauge_pm_ops,
#endif
	},
	.probe		= bq27410_gasgauge_probe,
	.remove		= __devexit_p(bq27410_gasgauge_remove),
    .id_table = bq27410_id,
};

static int __init bq27410_gasgauge_init(void)
{
	return i2c_add_driver(&bq27410_gasgauge_driver);
}
device_initcall(bq27410_gasgauge_init);

static void __exit bq27410_gasgauge_exit(void)
{
	i2c_del_driver(&bq27410_gasgauge_driver);
}
module_exit(bq27410_gasgauge_exit);


MODULE_AUTHOR("juntao.feng@nufront.com");
MODULE_DESCRIPTION("BQ27410 battery monitor driver");
MODULE_LICENSE("GPL");
