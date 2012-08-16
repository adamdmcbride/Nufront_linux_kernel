/*
 * driver/mfd/fsa880.c
 *
 * Core driver implementation to access RICOH583 power management chip.
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
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 *
 */
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/delay.h>

//#define DEBUG_EN
#ifdef DEBUG_EN
#define  PDBG(format,...)               printk("[%d:%s]:"format, \
                __LINE__,__func__ , ##__VA_ARGS__ )
#else
#define  PDBG(format,...)               do{}while(0)
#endif

struct fsa880 {
	struct device		*dev;
	struct i2c_client	*client;
};

static struct i2c_client * g_i2c_c = NULL;
static int fsa880_read(struct i2c_client *client, u8 reg, uint8_t *val)
{
	int ret;

	ret = i2c_smbus_read_byte_data(client, reg);
	if (ret < 0) {
		dev_err(&client->dev, "failed reading at 0x%02x.error:%d\n", reg, ret);;
		return ret;
	}

	*val = (uint8_t)ret;
	dev_dbg(&client->dev, "fsa880: reg read  reg=%x, val=%x\n",
			reg, *val);
	return 0;
}

static int fsa880_write(struct i2c_client *client, u8 reg, uint8_t val)
{
	int ret;

	dev_dbg(&client->dev, "fsa880: reg write  reg=%x, val=%x\n",
			reg, val);
	ret = i2c_smbus_write_byte_data(client, reg, val);
	if (ret < 0) {
		dev_err(&client->dev, "failed writing 0x%02x to 0x%02x. error:%d\n",
				val, reg, ret);
		return ret;
	}

	return 0;
}

#ifdef DEBUG_EN
static u8 dbg_addr = 0;

static ssize_t show_addr(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "0x%02X\n", dbg_addr);
}

static ssize_t store_addr(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t n)
{
	dbg_addr = simple_strtol(buf, NULL, 16);
	return n;
}

static ssize_t show_val(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	u8 val;
	int ret = 0;

	ret = fsa880_read(client, dbg_addr, &val);

	if(ret < 0){
		dev_err(&client->dev, "show_val: reg_read failed\n");
		return ret;
	}

	return sprintf(buf, "0x%02X\n", val);
}

static ssize_t store_val(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t n)
{
	struct i2c_client *client = to_i2c_client(dev);
	u8 val;
	int ret = 0;

	val = simple_strtol(buf, NULL, 16);

	ret = fsa880_write(client, dbg_addr, val);

	if(ret < 0){
		dev_err(&client->dev, "store_val: reg_write failed\n");
		return ret;
	}

	return n;
}

static ssize_t store_test(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t n)
{
	struct i2c_client *client = to_i2c_client(dev);

	return n;
}

static DEVICE_ATTR(addr, S_IRUGO | S_IWUGO, show_addr, store_addr);
static DEVICE_ATTR(val, S_IRUGO | S_IWUGO, show_val, store_val);
static DEVICE_ATTR(test, S_IWUGO, NULL, store_test);

static struct attribute *dev_attributes[] = {
	&dev_attr_addr.attr,
	&dev_attr_val.attr,
	&dev_attr_test.attr,
	NULL
};

static struct attribute_group dev_attr_group = {
	.attrs = dev_attributes,
};
#endif

void fsa880_int_func(void)
{
	char val;

	PDBG("fsa880_int_func\n");
	if(!g_i2c_c)
		return;

	fsa880_read(g_i2c_c, 0x03, &val);
	PDBG("reg0x03 = 0x%02x\n", val);
}

static int fsa880_i2c_probe(struct i2c_client *i2c,
		const struct i2c_device_id *id)
{
	struct fsa880 *fsa880;
	int ret;
	uint8_t val;

	fsa880 = kzalloc(sizeof(struct fsa880), GFP_KERNEL);
	if (fsa880 == NULL)
		return -ENOMEM;

	fsa880->client = i2c;
	fsa880->dev = &i2c->dev;
	i2c_set_clientdata(i2c, fsa880);

	ret = fsa880_write(i2c, 0x1b, 0x01);	/* reset */
	if(ret)
		goto err;
	mdelay(5);

	ret = fsa880_write(i2c, 0x02, 0x01);
	if (ret < 0){
		dev_err(&i2c->dev, "%s: write reg 0x02 failed!\n", __func__);
		goto err;
	}

	ret = fsa880_write(i2c, 0x13, 0x24);
	if (ret < 0){
		dev_err(&i2c->dev, "%s: write reg 0x02 failed!\n", __func__);
		goto err;
	}

#ifdef DEBUG_EN
	sysfs_create_group(&i2c->dev.kobj, &dev_attr_group);
#endif

	dev_info(fsa880->dev, "%s is ok!\n", __func__);

	g_i2c_c = i2c;

	return 0;

err:
	kfree(fsa880);
	return ret;
}

static int  __devexit fsa880_i2c_remove(struct i2c_client *i2c)
{
	struct fsa880 *fsa880 = i2c_get_clientdata(i2c);

#ifdef DEBUG_EN
	sysfs_remove_group(&i2c->dev.kobj, &dev_attr_group);
#endif
	g_i2c_c = NULL;
	kfree(fsa880);
	return 0;
}

static int fsa880_suspend(struct i2c_client * client, pm_message_t mesg)
{
	return 0;
}

static int fsa880_resume(struct i2c_client * client)
{
	return 0;
}

static const struct i2c_device_id fsa880_i2c_id[] = {
	{"fsa880_usb_detect", 0},
	{}
};
MODULE_DEVICE_TABLE(i2c, fsa880_i2c_id);

static struct i2c_driver fsa880_i2c_driver = {
	.driver = {
		.name = "fsa880_usb_detect",
		.owner = THIS_MODULE,
	},
	.probe = fsa880_i2c_probe,
	.remove = __devexit_p(fsa880_i2c_remove),
	.suspend = fsa880_suspend,
	.resume = fsa880_resume,
	.id_table = fsa880_i2c_id,
};


static int __init fsa880_i2c_init(void)
{
	int ret = -ENODEV;
	ret = i2c_add_driver(&fsa880_i2c_driver);
	if (ret != 0)
		pr_err("Failed to register I2C driver: %d\n", ret);

	return ret;
}

late_initcall(fsa880_i2c_init);

static void __exit fsa880_i2c_exit(void)
{
	i2c_del_driver(&fsa880_i2c_driver);
}

module_exit(fsa880_i2c_exit);

MODULE_AUTHOR("Wu Jianguo<jianguo.wu@nufront.com>");
MODULE_DESCRIPTION("FSA880 USB charger detect driver");
MODULE_LICENSE("GPL");

