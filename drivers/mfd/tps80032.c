/*
 * driver/mfd/tps80032.c
 *
 * Core driver implementation to access TPS80032 power management chip.
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

#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/workqueue.h>
#include <linux/mfd/core.h>
#include <linux/mfd/tps80032.h>
#include <mach/board-ns115.h>
#include <mach/io.h>
#include <mach/hardware.h>

/** register address, id, address**/
#define TPS80032_REG_INT_STS_A			2, 0XD0
#define TPS80032_REG_INT_STS_B			2, 0XD1
#define TPS80032_REG_INT_STS_C			2, 0XD2
#define TPS80032_REG_INT_MSK_LINE_A		2, 0XD3
#define TPS80032_REG_INT_MSK_LINE_B		2, 0XD4
#define TPS80032_REG_INT_MSK_LINE_C		2, 0XD5
#define TPS80032_REG_INT_MSK_STS_A		2, 0XD6
#define TPS80032_REG_INT_MSK_STS_B		2, 0XD7
#define TPS80032_REG_INT_MSK_STS_C		2, 0XD8
#define TPS80032_REG_PHONENIX_MASK_TRANSITION	1, 0X20
#define TPS80032_REG_SYSEN_CFG_TRANS	1, 0XB4
#define TPS80032_REG_SYSEN_CFG_STATE	1, 0XB5
#define TPS80032_REG_KEY_PRESS_DURATION_CFG		1, 0X2D
#define TPS80032_REG_PHONEIX_DEV_ON		1, 0X25
#define TPS80032_REG_32KAUDIO_STAT		1, 0XC1
#define TPS80032_REG_32KG_STAT			1, 0XBE
#define TPS80032_REG_32KAO_STAT			1, 0XBB

#define SW_RESET	(1 << 6)
#define DEVOFF		1
#define CLK32K_OFF	(0x0)
#define CLK32K_ON	(0x1)

struct tps80032 {
	struct device		*dev;
	struct i2c_client	*client;
	struct mutex		io_lock;
	struct irq_chip		irq_chip;
	struct mutex		irq_lock;
	struct work_struct	work;
	struct workqueue_struct *queue;
	unsigned long		irq_en;
	int irq_base;
};

static struct tps80032 * g_tps80032;

static u8 tps80032_slave_addr[] = {
	0x12,
	0x48,
	0x49,
	0x4A,
};

static inline int tps80032_i2c_addr_set(int id)
{
	if (id < 0 || id >= ARRAY_SIZE(tps80032_slave_addr)){
		return -1;
	}
	g_tps80032->client->addr = tps80032_slave_addr[id];

	return 0;
}

static inline int __tps80032_read(int id, u8 reg, uint8_t *val)
{
	int ret;

	tps80032_i2c_addr_set(id);
	ret = i2c_smbus_read_byte_data(g_tps80032->client, reg);
	if (ret < 0) {
		dev_err(&g_tps80032->client->dev, "failed reading at 0x%02x\n", reg);
		return ret;
	}

	*val = (uint8_t)ret;
	dev_dbg(&g_tps80032->client->dev, "tps80032: reg read  reg=0x%x, val=0x%x\n",
			reg, *val);
	return 0;
}

static inline int __tps80032_bulk_reads(int id, u8 reg, int len, uint8_t *val)
{
	int ret;
	int i;

	tps80032_i2c_addr_set(id);
	ret = i2c_smbus_read_i2c_block_data(g_tps80032->client, reg, len, val);
	if (ret < 0) {
		dev_err(&g_tps80032->client->dev, "failed reading from 0x%02x\n", reg);
		return ret;
	}
	for (i = 0; i < len; ++i) {
		dev_dbg(&g_tps80032->client->dev, "tps80032: reg read  reg=0x%x, val=0x%x\n",
				reg + i, *(val + i));
	}
	return 0;
}

static inline int __tps80032_write(int id, u8 reg, uint8_t val)
{
	int ret;

	tps80032_i2c_addr_set(id);
	dev_dbg(&g_tps80032->client->dev, "tps80032: reg write  reg=0x%x, val=0x%x\n",
			reg, val);
	ret = i2c_smbus_write_byte_data(g_tps80032->client, reg, val);
	if (ret < 0) {
		dev_err(&g_tps80032->client->dev, "failed writing 0x%02x to 0x%02x\n",
				val, reg);
		return ret;
	}

	return 0;
}

static inline int __tps80032_bulk_writes(int id, u8 reg, int len, uint8_t *val)
{
	int ret;
	int i;

	tps80032_i2c_addr_set(id);
	for (i = 0; i < len; ++i) {
		dev_dbg(&g_tps80032->client->dev, "tps80032: reg write  reg=0x%x, val=0x%x\n",
				reg + i, *(val + i));
	}

	ret = i2c_smbus_write_i2c_block_data(g_tps80032->client, reg, len, val);
	if (ret < 0) {
		dev_err(&g_tps80032->client->dev, "failed writings to 0x%02x\n", reg);
		return ret;
	}

	return 0;
}

int tps80032_write(int id, u8 reg, uint8_t val)
{
	int ret = 0;

	mutex_lock(&g_tps80032->io_lock);
	ret = __tps80032_write(id, reg, val);
	mutex_unlock(&g_tps80032->io_lock);

	return ret;
}
EXPORT_SYMBOL_GPL(tps80032_write);

int tps80032_bulk_writes(int id, u8 reg, u8 len, uint8_t *val)
{
	int ret = 0;

	mutex_lock(&g_tps80032->io_lock);
	ret = __tps80032_bulk_writes(id, reg, len, val);
	mutex_unlock(&g_tps80032->io_lock);

	return ret;
}
EXPORT_SYMBOL_GPL(tps80032_bulk_writes);

int tps80032_read(int id, u8 reg, uint8_t *val)
{
	return __tps80032_read(id, reg, val);
}
EXPORT_SYMBOL_GPL(tps80032_read);

int tps80032_bulk_reads(int id, u8 reg, u8 len, uint8_t *val)
{
	return __tps80032_bulk_reads(id, reg, len, val);
}
EXPORT_SYMBOL_GPL(tps80032_bulk_reads);

int tps80032_set_bits(int id, u8 reg, uint8_t bit_mask)
{
	uint8_t reg_val;
	int ret = 0;

	mutex_lock(&g_tps80032->io_lock);

	ret = __tps80032_read(id, reg, &reg_val);
	if (ret)
		goto out;

	if ((reg_val & bit_mask) != bit_mask) {
		reg_val |= bit_mask;
		ret = __tps80032_write(id, reg, reg_val);
	}
out:
	mutex_unlock(&g_tps80032->io_lock);
	return ret;
}
EXPORT_SYMBOL_GPL(tps80032_set_bits);

int tps80032_clr_bits(int id, u8 reg, uint8_t bit_mask)
{
	uint8_t reg_val;
	int ret = 0;

	mutex_lock(&g_tps80032->io_lock);

	ret = __tps80032_read(id, reg, &reg_val);
	if (ret)
		goto out;

	if (reg_val & bit_mask) {
		reg_val &= ~bit_mask;
		ret = __tps80032_write(id, reg, reg_val);
	}
out:
	mutex_unlock(&g_tps80032->io_lock);
	return ret;
}
EXPORT_SYMBOL_GPL(tps80032_clr_bits);

int tps80032_reg_update(int id, u8 reg, uint8_t val, uint8_t mask)
{
	uint8_t reg_val;
	int ret = 0;

	mutex_lock(&g_tps80032->io_lock);

	ret = __tps80032_read(id, reg, &reg_val);
	if (ret)
		goto out;

	if ((reg_val & mask) != val) {
		reg_val = (reg_val & ~mask) | (val & mask);
		ret = __tps80032_write(id, reg, reg_val);
	}
out:
	mutex_unlock(&g_tps80032->io_lock);
	return ret;
}
EXPORT_SYMBOL_GPL(tps80032_reg_update);

int tps80032_clk32kao_switch(int on)
{
	int ret;

	if (on){
		ret = tps80032_write(TPS80032_REG_32KAO_STAT, CLK32K_ON);
	}else{
		ret = tps80032_write(TPS80032_REG_32KAO_STAT, CLK32K_OFF);
	}

	return ret;
}
EXPORT_SYMBOL_GPL(tps80032_clk32kao_switch);

int tps80032_clk32kg_switch(int on)
{
	int ret;

	if (on){
		ret = tps80032_write(TPS80032_REG_32KG_STAT, CLK32K_ON);
	}else{
		ret = tps80032_write(TPS80032_REG_32KG_STAT, CLK32K_OFF);
	}

	return ret;
}
EXPORT_SYMBOL_GPL(tps80032_clk32kg_switch);

int tps80032_clk32kaudio_switch(int on)
{
	int ret;

	if (on){
		ret = tps80032_write(TPS80032_REG_32KAUDIO_STAT, CLK32K_ON);
	}else{
		ret = tps80032_write(TPS80032_REG_32KAUDIO_STAT, CLK32K_OFF);
	}

	return ret;
}
EXPORT_SYMBOL_GPL(tps80032_clk32kaudio_switch);

void tps80032_power_off(void)
{
	int ret;

	dev_emerg(g_tps80032->dev, "%s\n", __func__);
	ret = __tps80032_write(TPS80032_REG_PHONEIX_DEV_ON, DEVOFF);
	if (ret < 0){
		dev_err(g_tps80032->dev, "%s failed: %d\n", __func__, ret);
	}

	return;
}
EXPORT_SYMBOL_GPL(tps80032_power_off);

extern void dw_i2c_master_init(void* i2c_base, unsigned char slv_addr);
extern int dw_i2c_send_bytes(void* i2c_base, unsigned char * out_buf, unsigned int len);

static int tps80032_smbus_write_reg(void __iomem * i2c_base,unsigned char slv_addr, 
		unsigned char reg, unsigned char  val)
{
	char wbuf[2];

	wbuf[0] = reg;
	wbuf[1] = val;
	dw_i2c_master_init(i2c_base, slv_addr);
	dw_i2c_send_bytes(i2c_base, wbuf, 2);
	return 0;
}

void tps80032_restart(char str, const char * cmd)
{
	void __iomem * i2c_base = __io_address(NS115_I2C1_BASE);

	dev_emerg(g_tps80032->dev, "%s\n", __func__);

	tps80032_smbus_write_reg(i2c_base, 0x48, 0x25, SW_RESET);

	return;
}
EXPORT_SYMBOL_GPL(tps80032_restart);

static void tps80032_irq_lock(struct irq_data *irq_data)
{
	mutex_lock(&g_tps80032->irq_lock);
}

static void tps80032_irq_unmask(struct irq_data *irq_data)
{
	unsigned int __irq = irq_data->irq - g_tps80032->irq_base;

	g_tps80032->irq_en |= (1 << __irq);
}

static void tps80032_irq_mask(struct irq_data *irq_data)
{
	unsigned int __irq = irq_data->irq - g_tps80032->irq_base;

	g_tps80032->irq_en &= ~(1 << __irq);
}

static void tps80032_irq_sync_unlock(struct irq_data *irq_data)
{
	u8 mask[3];
	unsigned long irq_mask = ~g_tps80032->irq_en;

	mask[0] = irq_mask & 0xff;
	mask[1] = (irq_mask >> 8) & 0xff;
	mask[2] = (irq_mask >> 16) & 0xff;

	__tps80032_bulk_writes(TPS80032_REG_INT_MSK_LINE_A, 3, mask);
	__tps80032_bulk_writes(TPS80032_REG_INT_MSK_STS_A, 3, mask);

	mutex_unlock(&g_tps80032->irq_lock);
}

static irqreturn_t tps80032_irq(int irq, void *data)
{
	struct tps80032 *tps80032 = data;

	disable_irq_nosync(tps80032->client->irq);
	queue_work(tps80032->queue, &tps80032->work);

	return IRQ_HANDLED;
}

static void tps80032_irq_work(struct work_struct *work)
{
	struct tps80032 *tps80032 = container_of(work, struct tps80032, work);
	u8 int_sts[3] = {0, 0, 0};
	unsigned long irq_en_bit = 0;
	int i;
	int ret;

	ret  = __tps80032_bulk_reads(TPS80032_REG_INT_STS_A, 3, int_sts);
	if (ret < 0) {
		dev_err(tps80032->dev, "Error in reading id:%d reg 0x%02x "
				"error: %d\n", TPS80032_REG_INT_STS_A, ret);
		goto out;
	}
	irq_en_bit = int_sts[2];
	irq_en_bit = (irq_en_bit << 8) | int_sts[1];
	irq_en_bit = (irq_en_bit << 8) | int_sts[0];

	memset(int_sts, 0, sizeof(int_sts));
	ret  = __tps80032_bulk_writes(TPS80032_REG_INT_STS_A, 3, int_sts);
	if (ret < 0) {
		dev_err(tps80032->dev, "Error in writing id:%d reg 0x%02x "
				"error: %d\n", TPS80032_REG_INT_STS_A, ret);
		goto out;
	}
	/* Call interrupt handler if enabled */
	for (i = 0; i < TPS80032_NR_IRQS; ++i) {
		if((tps80032->irq_en & (1 << i)) && (irq_en_bit & (1 << i))){
			dev_dbg(tps80032->dev, "call int handler: %d\n", i);
			handle_nested_irq(tps80032->irq_base + i);
		}
	}
out:
	enable_irq(tps80032->client->irq);
	return;
}

static int __devinit tps80032_irq_init(struct tps80032 *tps80032, int irq,
		int irq_base)
{
	int i, ret;
	uint8_t mask[3] = {0xff, 0xff, 0xff};

	dev_info(tps80032->dev, "irq:%d irq_base:%d\n", irq, irq_base);
	if (!irq_base) {
		dev_warn(tps80032->dev, "No interrupt support on IRQ base\n");
		return -EINVAL;
	}

	mutex_init(&tps80032->irq_lock);
	__tps80032_bulk_writes(TPS80032_REG_INT_STS_A, 3, mask);
	__tps80032_bulk_writes(TPS80032_REG_INT_MSK_LINE_A, 3, mask);
	__tps80032_bulk_writes(TPS80032_REG_INT_MSK_STS_A, 3, mask);
	tps80032->irq_en = 0;

	tps80032->irq_base = irq_base;
	tps80032->irq_chip.name = "tps80032";
	tps80032->irq_chip.irq_mask = tps80032_irq_mask;
	tps80032->irq_chip.irq_unmask = tps80032_irq_unmask;
	tps80032->irq_chip.irq_bus_lock = tps80032_irq_lock;
	tps80032->irq_chip.irq_bus_sync_unlock = tps80032_irq_sync_unlock;

	for (i = 0; i < TPS80032_NR_IRQS; i++) {
		int __irq = i + tps80032->irq_base;
		irq_set_chip_data(__irq, tps80032);
		irq_set_chip_and_handler(__irq, &tps80032->irq_chip,
				handle_simple_irq);
		irq_set_nested_thread(__irq, 1);
#ifdef CONFIG_ARM
		set_irq_flags(__irq, IRQF_VALID);
#endif
	}

	INIT_WORK(&tps80032->work, tps80032_irq_work);
	tps80032->queue = create_workqueue("tps80032 irq queue");
	if (!tps80032->queue){
		return -1;
	}
	ret = request_irq(irq, tps80032_irq, IRQF_TRIGGER_LOW,
			"tps80032", tps80032);
	if (ret < 0){
		dev_err(tps80032->dev, "Error in registering interrupt "
				"error: %d\n", ret);
		goto err;
	}
	device_init_wakeup(tps80032->dev, 1);
	enable_irq_wake(irq);

	return ret;
err:
	destroy_workqueue(tps80032->queue);
	return -1;
}

static int tps80032_remove_subdev(struct device *dev, void *unused)
{
	platform_device_unregister(to_platform_device(dev));
	return 0;
}

static int tps80032_remove_subdevs(struct tps80032 *tps80032)
{
	return device_for_each_child(tps80032->dev, NULL,
			tps80032_remove_subdev);
}

static int __devinit tps80032_add_subdevs(struct tps80032 *tps80032,
		struct tps80032_platform_data *pdata)
{
	struct tps80032_subdev_info *subdev;
	struct platform_device *pdev;
	int i, ret = 0;

	for (i = 0; i < pdata->num_subdevs; i++) {
		subdev = &pdata->subdevs[i];

		pdev = platform_device_alloc(subdev->name, subdev->id);
		dev_info(tps80032->dev, "%s: %s\n", __func__, subdev->name);

		pdev->dev.parent = tps80032->dev;
		pdev->dev.platform_data = subdev->platform_data;

		ret = platform_device_add(pdev);
		if (ret)
			goto failed;
	}
	return 0;

failed:
	tps80032_remove_subdevs(tps80032);
	return ret;
}

int tps80032_suspend_system(void)
{
	int ret;

	ret = tps80032_reg_update(TPS80032_REG_SYSEN_CFG_STATE, 0x03, 0x03);

	return ret;
}
EXPORT_SYMBOL_GPL(tps80032_suspend_system);

static int tps80032_init(void)
{
	int ret = 0;
	int i;
	int reg_data[][4] = {
		//id, address, val , mask
		{TPS80032_REG_PHONENIX_MASK_TRANSITION, 0, 1 << 5},//enable PREQ1
		{TPS80032_REG_SYSEN_CFG_TRANS, 0, 0x0C},//disable sysen when sleep
	};

	for (i = 0; i < sizeof(reg_data) / (4 * sizeof(int)); ++i){
		ret |= tps80032_reg_update(reg_data[i][0], reg_data[i][1], 
				reg_data[i][2], reg_data[i][3]);
	}

	ret |= tps80032_clk32kaudio_switch(1);
	ret |= tps80032_clk32kao_switch(1);
	ret |= tps80032_clk32kg_switch(1);

	return ret;
}

#ifdef TPS80032_DEBUG
static int dbg_id = 0;
static u8 dbg_addr = 0;

static ssize_t show_addr(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "id:%d addr:0x%02X\n", dbg_id, dbg_addr);
}

static ssize_t store_addr(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t n)
{
	int i = 0;
	
	while (buf[i] != ',' && i < n){
		++i;
	}
	dbg_id = simple_strtol(buf, NULL, 16);
	dbg_addr = simple_strtol(buf + i + 1, NULL, 16);

	return n;
}

static ssize_t show_val(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	u8 val;
	int ret = 0;

	ret = tps80032_read(dbg_id, dbg_addr, &val);

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

	ret = tps80032_write(dbg_id, dbg_addr, val);

	if(ret < 0){
		dev_err(&client->dev, "store_val: reg_write failed\n");
		return ret;
	}

	return n;
}

static DEVICE_ATTR(addr, S_IRUGO | S_IWUGO, show_addr, store_addr);
static DEVICE_ATTR(val, S_IRUGO | S_IWUGO, show_val, store_val);

static struct attribute *tps80032_attributes[] = {
	&dev_attr_addr.attr,
	&dev_attr_val.attr,
	NULL,
};

static struct attribute_group tps80032_attr_group = {
	.attrs = tps80032_attributes,
};
#endif

static int tps80032_i2c_probe(struct i2c_client *i2c,
		const struct i2c_device_id *id)
{
	struct tps80032 *tps80032;
	struct tps80032_platform_data *pdata = i2c->dev.platform_data;
	int ret;

	tps80032 = kzalloc(sizeof(struct tps80032), GFP_KERNEL);
	if (tps80032 == NULL)
		return -ENOMEM;

	tps80032->client = i2c;
	tps80032->dev = &i2c->dev;
	i2c_set_clientdata(i2c, tps80032);
	g_tps80032 = tps80032;

	mutex_init(&tps80032->io_lock);
	ret = tps80032_add_subdevs(tps80032, pdata);
	if (ret) {
		dev_err(&i2c->dev, "add devices failed: %d\n", ret);
		goto err_irq_init;
	}

	if (i2c->irq) {
		ret = tps80032_irq_init(tps80032, i2c->irq, pdata->irq_base);
		if (ret) {
			dev_err(&i2c->dev, "IRQ init failed: %d\n", ret);
			goto err_add_devs;
		}
	}

	ret = tps80032_init();
	if(ret){
		dev_err(&i2c->dev, "RICOH init failed: %d\n", ret);
		goto err_add_devs;
	}
#ifdef TPS80032_DEBUG
	sysfs_create_group(&i2c->dev.kobj, &tps80032_attr_group);
#endif
	dev_info(&i2c->dev, "%s is OK!\n", __func__);
	return 0;

err_add_devs:
	if (i2c->irq)
		free_irq(i2c->irq, tps80032);
err_irq_init:
	kfree(tps80032);
	return ret;
}

static int  __devexit tps80032_i2c_remove(struct i2c_client *i2c)
{
	struct tps80032 *tps80032 = i2c_get_clientdata(i2c);

	if (i2c->irq)
		free_irq(i2c->irq, tps80032);

#ifdef TPS80032_DEBUG
	sysfs_remove_group(&i2c->dev.kobj, &tps80032_attr_group);
#endif
	tps80032_remove_subdevs(tps80032);
	destroy_workqueue(tps80032->queue);
	kfree(tps80032);
	return 0;
}


#ifdef CONFIG_PM
static int tps80032_i2c_suspend(struct i2c_client *i2c, pm_message_t state)
{
	int ret;

	ret = tps80032_suspend_system();
	if (ret){
		dev_err(g_tps80032->dev, "%s: sysen low failed: %d\n", __func__, ret);
		return ret;
	}
	if (i2c->irq)
		disable_irq(i2c->irq);
	return 0;
}


static int tps80032_i2c_resume(struct i2c_client *i2c)
{
	if (i2c->irq)
		enable_irq(i2c->irq);
	return 0;
}

#endif

static const struct i2c_device_id tps80032_i2c_id[] = {
	{"tps80032", 0},
	{}
};

MODULE_DEVICE_TABLE(i2c, tps80032_i2c_id);

static struct i2c_driver tps80032_i2c_driver = {
	.driver = {
		.name = "tps80032",
		.owner = THIS_MODULE,
	},
	.probe = tps80032_i2c_probe,
	.remove = __devexit_p(tps80032_i2c_remove),
#ifdef CONFIG_PM
	.suspend = tps80032_i2c_suspend,
	.resume = tps80032_i2c_resume,
#endif
	.id_table = tps80032_i2c_id,
};


static int __init tps80032_i2c_init(void)
{
	int ret = -ENODEV;
	ret = i2c_add_driver(&tps80032_i2c_driver);
	if (ret != 0)
		pr_err("Failed to register I2C driver: %d\n", ret);

	return ret;
}

subsys_initcall(tps80032_i2c_init);

static void __exit tps80032_i2c_exit(void)
{
	i2c_del_driver(&tps80032_i2c_driver);
}

module_exit(tps80032_i2c_exit);

MODULE_AUTHOR("Wu Jianguo<jianguo.wu@nufront.com>");
MODULE_DESCRIPTION("TPS80032 multi-function core driver");
MODULE_LICENSE("GPL");
