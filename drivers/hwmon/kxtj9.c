/*
 * 2012.4.02 modify kxtj9.c for read get_count without i2c.
 * 2012.3.31 modify kxtj9.c for add mutex
 * 2012.3.14 modify kxtj9.c for add interrupt mode by wangweilin 
 * 2012.2.15 modify kxtj9.c for some HAL interface with kxtj9 by wangweilin
 * drivers/input/misc/kxtj9.c - KXTJ9 accelerometer driver
 *
 * Copyright (C) 2011 Kionix, Inc.
 * Written by Kuching Tan <kuchingtan@kionix.com>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/input/kxtj9.h>
#include <linux/version.h>
#include <linux/interrupt.h>
#include <linux/gpio.h>
#include <linux/err.h>
#include <linux/errno.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/miscdevice.h>
#include <linux/earlysuspend.h>
#include <asm/atomic.h>

#define TR0(fmt, args...) printk(KERN_INFO "gsensor_kxtj9: " fmt, ##args)
#define TR1(fmt, args...)

#define NAME			"kxtj9"
#define G_MAX			8000
/* OUTPUT REGISTERS */
#define XOUT_L			0x06
#define WHO_AM_I		0x0F
/* CONTROL REGISTERS */
#define INT_REL			0x1A
#define CTRL_REG1		0x1B
#define INT_CTRL1		0x1E
#define DATA_CTRL		0x21
/* CONTROL REGISTER 1 BITS */
#define PC1_OFF			0x7F
#define PC1_ON			(1 << 7)
/* INPUT_ABS CONSTANTS */
#define FUZZ			3
#define FLAT			3
/* RESUME STATE INDICES */
#define RES_DATA_CTRL		0
#define RES_CTRL_REG1		1
#define RES_INT_CTRL1		2
#define RESUME_ENTRIES		3
#define RESET                   0xCA
/* CONTROL REGISTERS */
#define CTRL_REGC               0x0C
#define CTRL_REGB               0x0D
/* RESUME STATE INDICES */
#define RES_CTRL_REGC           0
#define RES_CTRL_REGB           1
/* Data ready funtion enable bit: set during probe if using irq mode */
#define DRDYE                   (1 << 5)
/* INTERRUPT CONTROL REGISTER 1 BITS */
/* Set these during probe if using irq mode */
#define KXTJ9_IEL               (1 << 3)
#define KXTJ9_IEA               (1 << 4)
#define KXTJ9_IEN               (1 << 5)

#define OFFSET          2048

#define XOUT_H                  0x00
#define AUXOUT_H                0x06
#define RESET_WRITE             0x0A

#define IOCTL_BUFFER_SIZE       64

#define NAME_DEV        "kxud9"
#define DIR_DEV         "/dev/kxud9"

#define ENABLE                  0x40

/* IOCTLs for kxtj9 misc. device library */
#define KXUD9IO                                                                 0x96
#define KXUD9_IOCTL_GET_COUNT                   _IOR(KXUD9IO, 0x01, int)
#define KXUD9_IOCTL_GET_MG                              _IOR(KXUD9IO, 0x02, int)
#define KXUD9_IOCTL_ENABLE_OUTPUT                _IO(KXUD9IO, 0x03)
#define KXUD9_IOCTL_DISABLE_OUTPUT               _IO(KXUD9IO, 0x04)
#define KXUD9_IOCTL_GET_ENABLE                  _IOR(KXUD9IO, 0x05, int)
#define KXUD9_IOCTL_RESET                                _IO(KXUD9IO, 0x06)
#define KXUD9_IOCTL_UPDATE_ODR                  _IOW(KXUD9IO, 0x07, int)

static atomic_t irq_flag;
static atomic_t enable_flag;
volatile static int a_x;
volatile static int a_y;
volatile static int a_z;
static void kxtj9_device_power_off(void);

/*
 * The following table lists the maximum appropriate poll interval for each
 * available output data rate.
 */
static const struct {
	unsigned int cutoff;
	u8 mask;
} kxtj9_odr_table[] = {
	{ 3,	ODR800F },
	{ 5,	ODR400F },
	{ 10,	ODR200F },
	{ 20,	ODR100F },
	{ 40,	ODR50F  },
	{ 80,	ODR25F  },
	{ 0,	ODR12_5F},
};

struct kxtj9_data {
	struct i2c_client *client;
	struct kxtj9_platform_data pdata;
        struct mutex lock;
	struct input_dev *input_dev;
	struct delayed_work poll_work;
	struct workqueue_struct *workqueue;
        struct work_struct irq_work;

	unsigned int poll_interval;
	unsigned int poll_delay;
	u8 shift;
	u8 ctrl_reg1;
	u8 data_ctrl;
        u8 int_ctrl;

        struct delayed_work input_work;

        int sensitivity;
        int hw_initialized;
        atomic_t enabled;
        u8 resume[RESUME_ENTRIES];
        int res_interval;
};

static struct kxtj9_data *tj9 = NULL;

static int kxtj9_i2c_read(u8 addr, u8 *data, int len)
{
	struct i2c_msg msgs[] = {
		{
			.addr = tj9->client->addr,
			.flags = tj9->client->flags,
			.len = 1,
			.buf = &addr,
		},
		{
			.addr = tj9->client->addr,
			.flags = tj9->client->flags | I2C_M_RD,
			.len = len,
			.buf = data,
		},
	};

	return i2c_transfer(tj9->client->adapter, msgs, 2);
}

static int kxtj9_i2c_write(u8 addr, u8 *data, int len)
{
        int err;
        int i;
        u8 buf[len + 1];

        struct i2c_msg msgs[] = {
                {
                 .addr = tj9->client->addr,
                 .flags = tj9->client->flags & I2C_M_TEN,
                 .len = len + 1,
                 .buf = buf,
                 },
        };

        buf[0] = addr;
        for (i = 0; i < len; i++)
                buf[i + 1] = data[i];
        
        err = i2c_transfer(tj9->client->adapter, msgs, 1);
                
        if(err != 1)    
                dev_err(&tj9->client->dev, "write transfer error\n");
        else
                err = 0;
                
                
        return err;     
}      

int kxtj9_get_bits(u8 reg_addr, u8* bits_value, u8 bits_mask)
{
        int err;
        u8 reg_data;

        err = kxtj9_i2c_read(reg_addr, &reg_data, 1);
        if(err < 0)
                return err;

        *bits_value = reg_data & bits_mask;

        return 1;
}

int kxtj9_get_byte(u8 reg_addr, u8* reg_value)
{
        int err;
        u8 reg_data;

        err = kxtj9_i2c_read(reg_addr, &reg_data, 1);
        if(err < 0)
                return err;

        *reg_value = reg_data;

        return 1;
}

int kxtj9_set_bits(int res_index, u8 reg_addr, u8 bits_value, u8 bits_mask)
{
        int err=0, retval=0;
        u8 reg_data = 0x00, reg_bits = 0x00, bits_set = 0x00;

        // Read from device register
        err = kxtj9_i2c_read(reg_addr, &reg_data, 1);
        if(err < 0)
                goto exit0;

        // Apply mask to device register;
        reg_bits = reg_data & bits_mask;

        // Update resume state data
        bits_set = bits_mask & bits_value;
        tj9->resume[res_index] &= ~bits_mask;
        tj9->resume[res_index] |= bits_set;

        // Return 0 if value in device register and value to be written is the same
        if(reg_bits == bits_set)
                retval = 0;
        // Else, return 1
        else
                retval = 1;

        // Write to device register
        err = kxtj9_i2c_write(reg_addr, &tj9->resume[res_index], 1);
        if(err < 0)
                goto exit0;

exit0:
        if(err < 0)
                return err;

        return retval;
}


int kxtj9_set_byte(int res_index, u8 reg_addr, u8 reg_value)
{
        int err, retval=0;
        u8 reg_data;

        // Read from device register
        err = kxtj9_i2c_read(reg_addr, &reg_data, 1);
        if(err < 0)
                goto exit0;

        // Update resume state data
        tj9->resume[res_index] = reg_value;

        // Return 0 if value in device register and value to be written is the same
        if(reg_data == reg_value)
                retval = 0;
        // Else, return 1
        else
                retval = 1;

        // Write to device register
        err = kxtj9_i2c_write(reg_addr, &tj9->resume[res_index], 1);
        if(err < 0)
                goto exit0;

exit0:
        if(err < 0)
                return err;

        return retval;
}

int kxtj9_reset(void)
{
        u8 reg_data = RESET;

        return kxtj9_i2c_write(RESET_WRITE, &reg_data, 1);
}

static int kxtj9_hw_init(void)
{
        int err;
        err = kxtj9_i2c_write(CTRL_REGC, &tj9->resume[RES_CTRL_REGC], 1);
        if(err < 0)
                return err;
        err = kxtj9_i2c_write(CTRL_REGB, &tj9->resume[RES_CTRL_REGB], 1);
        if(err < 0)
                return err;

        tj9->hw_initialized = 1;

        return 0;
}

static void kxtj9_report_acceleration_data(struct kxtj9_data *tj9)
{
	s16 acc_data[3]; /* Data bytes from hardware xL, xH, yL, yH, zL, zH */
	s16 x, y, z;
	int err;
	struct input_dev *input_dev = tj9->input_dev;

	mutex_lock(&input_dev->mutex);
	err = kxtj9_i2c_read(XOUT_L, (u8 *)acc_data, 6);
	mutex_unlock(&input_dev->mutex);
	if (err < 0)
		dev_err(&tj9->client->dev, "accelerometer data read failed\n");

	y = ((s16) le16_to_cpu(acc_data[tj9->pdata.axis_map_x])) >> tj9->shift;
	x = ((s16) le16_to_cpu(acc_data[tj9->pdata.axis_map_y])) >> tj9->shift;
	z = ((s16) le16_to_cpu(acc_data[tj9->pdata.axis_map_z])) >> tj9->shift;
    
    	a_x = x;
    	a_y = y;
    	a_z = z;

	x = x * 819 / 1024;
	y = y * 819 / 1024;
	z = z * 819 / 1024;

	TR1("acc x:%d, y:%d, z:%d\n", x, y, z);

	input_report_abs(tj9->input_dev, ABS_X, tj9->pdata.negate_x ? -x : x);
	input_report_abs(tj9->input_dev, ABS_Y, tj9->pdata.negate_y ? -y : y);
	input_report_abs(tj9->input_dev, ABS_Z, tj9->pdata.negate_z ? -z : z);
	input_sync(tj9->input_dev);
}

static void kxtj9_poll_work(struct work_struct *work)
{
	struct kxtj9_data *tj9 = container_of((struct delayed_work *)work,	struct kxtj9_data, poll_work);

	queue_delayed_work(tj9->workqueue, &tj9->poll_work, tj9->poll_delay);

	kxtj9_report_acceleration_data(tj9);
}

static irqreturn_t kxtj9_isr(int irq, void *dev)
{
        struct kxtj9_data *tj9 = dev;
        TR1("isr\n");
        //disable_irq_nosync(tj9->client->irq);
        queue_work(tj9->workqueue, &tj9->irq_work);
        return IRQ_HANDLED;
}

static void kxtj9_irq_work(struct work_struct *work)
{
        struct kxtj9_data *tj9 = container_of(work,     struct kxtj9_data, irq_work);

        TR1("irq_work\n");
        /* data ready is the only possible interrupt type */
        kxtj9_report_acceleration_data(tj9);
}

static int kxtj9_update_g_range(struct kxtj9_data *tj9, u8 new_g_range)
{
	switch (new_g_range) {
	case KXTJ9_G_2G:
		tj9->shift = 4;
		break;
	case KXTJ9_G_4G:
		tj9->shift = 3;
		break;
	case KXTJ9_G_8G:
		tj9->shift = 2;
		break;
	default:
		return -EINVAL;
	}

	tj9->ctrl_reg1 &= 0xe7;
	tj9->ctrl_reg1 |= new_g_range;

	return 0;
}

static int kxtj9_update_odr(unsigned int poll_interval)
{
	int err;
	int i;

	/* Use the lowest ODR that can support the requested poll interval */
	for (i = 0; i < ARRAY_SIZE(kxtj9_odr_table); i++) {
		tj9->data_ctrl = kxtj9_odr_table[i].mask;
		if (poll_interval < kxtj9_odr_table[i].cutoff)
			break;
	}

	err = i2c_smbus_write_byte_data(tj9->client, CTRL_REG1, 0);
	if (err < 0)
		return err;

        TR0("update_odr :0x%x\n", tj9->data_ctrl);
	err = i2c_smbus_write_byte_data(tj9->client, DATA_CTRL, tj9->data_ctrl);
	if (err < 0)
		return err;

	err = i2c_smbus_write_byte_data(tj9->client, CTRL_REG1, tj9->ctrl_reg1);
	if (err < 0)
		return err;

	return 0;
}

static int kxtj9_device_power_on(void)
{
	int err;

	if (tj9->pdata.power_on)
		return tj9->pdata.power_on();

 	if(!tj9->hw_initialized) {
                mdelay(100);
                err = kxtj9_hw_init();
                if(err < 0) {
                        kxtj9_device_power_off();
                        return err;
                }
        }
 
	return 0;
}

static void kxtj9_device_power_off(void)
{
	int err;

	tj9->ctrl_reg1 &= PC1_OFF;
	err = i2c_smbus_write_byte_data(tj9->client, CTRL_REG1, tj9->ctrl_reg1);
	if (err < 0) {
		TR0("power off error:%d\n", err);
		dev_err(&tj9->client->dev, "soft power off failed\n");
	}

	if (tj9->pdata.power_off)
		tj9->pdata.power_off();
}

static int kxtj9_enable(int resume_t)
{
	if ((atomic_read(&enable_flag) > 0) && (resume_t == 0)) {
		atomic_inc(&enable_flag);
		return 0;
	}
	if(atomic_read(&irq_flag) == 1) {
		int err;

		TR1("enable\n");
		err = kxtj9_device_power_on();
		if (err < 0)
			return err;

		/* ensure that PC1 is cleared before updating control registers */
		err = i2c_smbus_write_byte_data(tj9->client, CTRL_REG1, 0);
		if (err < 0)
			return err;

		TR1("int_ctrl_reg1 1:%x\n", tj9->int_ctrl);
		/* only write INT_CTRL_REG1 if in irq mode */
		if (tj9->client->irq) {
			err = i2c_smbus_write_byte_data(tj9->client,
					INT_CTRL1, tj9->int_ctrl);
			if (err < 0)
				return err;
		}
		TR1("int_ctrl_reg1 2:%x\n", i2c_smbus_read_byte_data(tj9->client, 0x1e));

		err = kxtj9_update_g_range(tj9, tj9->pdata.g_range);
		if (err < 0)
			return err;

		/* turn on outputs */
		tj9->ctrl_reg1 |= PC1_ON;
		tj9->ctrl_reg1 |= DRDYE;
		err = i2c_smbus_write_byte_data(tj9->client, CTRL_REG1, tj9->ctrl_reg1);
		if (err < 0)
			return err;

		err = kxtj9_update_odr(tj9->poll_interval);
		if (err < 0)
			return err;

		TR1("status3:%x\n", i2c_smbus_read_byte_data(tj9->client, 0x18));
		/* clear initial interrupt if in irq mode */
		if (tj9->client->irq) {
			err = i2c_smbus_read_byte_data(tj9->client, INT_REL);
			if (err < 0) {
				dev_err(&tj9->client->dev,
						"error clearing interrupt: %d\n", err);
				goto fail;
			}
		}
		TR1("status4:%x\n", i2c_smbus_read_byte_data(tj9->client, 0x18));

		atomic_inc(&enable_flag);
		return 0;

fail:
		kxtj9_device_power_off();
		return err;
	} else {
		int err;

		err = kxtj9_device_power_on();
		if (err < 0)
			return err;

		/* ensure that PC1 is cleared before updating control registers */
		err = i2c_smbus_write_byte_data(tj9->client, CTRL_REG1, 0);
		if (err < 0)
			return err;

		err = kxtj9_update_g_range(tj9, tj9->pdata.g_range);
		if (err < 0)
			return err;

		/* turn on outputs */
		tj9->ctrl_reg1 |= PC1_ON;
		err = i2c_smbus_write_byte_data(tj9->client, CTRL_REG1, tj9->ctrl_reg1);
		if (err < 0)
			return err;

		err = kxtj9_update_odr(tj9->poll_interval);
		if (err < 0)
			return err;

		queue_delayed_work(tj9->workqueue, &tj9->poll_work, 0);

		atomic_inc(&enable_flag);
		return 0;
	}
}

static void kxtj9_disable(int suspend_t)
{
	if((atomic_read(&enable_flag) > 0) && (suspend_t == 0)) {
		atomic_dec(&enable_flag);
		return;
	}
	if(atomic_read(&irq_flag) == 1) {
		kxtj9_device_power_off();
	} else {
		cancel_delayed_work_sync(&tj9->poll_work);

		kxtj9_device_power_off();
	}
	if(suspend_t == 1)
		atomic_dec(&enable_flag);
}

static int kxtj9_input_open(struct input_dev *input)
{
	tj9 = input_get_drvdata(input);

	return kxtj9_enable(0);
}

static void kxtj9_input_close(struct input_dev *dev)
{
	tj9 = input_get_drvdata(dev);

	kxtj9_disable(0);
}

static void __devinit kxtj9_init_input_device(struct kxtj9_data *tj9,
		struct input_dev *input_dev)
{
	__set_bit(EV_ABS, input_dev->evbit);
	input_set_abs_params(input_dev, ABS_X, -G_MAX, G_MAX, FUZZ, FLAT);
	input_set_abs_params(input_dev, ABS_Y, -G_MAX, G_MAX, FUZZ, FLAT);
	input_set_abs_params(input_dev, ABS_Z, -G_MAX, G_MAX, FUZZ, FLAT);

	input_dev->name = "kxud9_accel";
	input_dev->id.bustype = BUS_I2C;
	input_dev->dev.parent = &tj9->client->dev;
}

static int __devinit kxtj9_setup_input_device(void)
{
	struct input_dev *input_dev;
	int err;

	input_dev = input_allocate_device();
	if (!input_dev) {
		dev_err(&tj9->client->dev, "input device allocate failed\n");
		return -ENOMEM;
	}

	tj9->input_dev = input_dev;

	input_dev->open = kxtj9_input_open;
	input_dev->close = kxtj9_input_close;
	input_set_drvdata(input_dev, tj9);

	kxtj9_init_input_device(tj9, input_dev);

	err = input_register_device(tj9->input_dev);
	if (err) {
		dev_err(&tj9->client->dev,
				"unable to register input polled device %s: %d\n",
				tj9->input_dev->name, err);
		input_free_device(tj9->input_dev);
		return err;
	}

	return 0;
}

/*
 * When IRQ mode is selected, we need to provide an interface to allow the user
 * to change the output data rate of the part.  For consistency, we are using
 * the set_poll method, which accepts a poll interval in milliseconds, and then
 * calls update_odr() while passing this value as an argument.  In IRQ mode, the
 * data outputs will not be read AT the requested poll interval, rather, the
 * lowest ODR that can support the requested interval.  The client application
 * will be responsible for retrieving data from the input node at the desired
 * interval.
 */


/* Returns currently selected poll interval (in ms) */
static ssize_t kxtj9_get_poll(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", tj9->poll_interval);
}

/* Allow users to select a new poll interval (in ms) */
static ssize_t kxtj9_set_poll(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t count)
{
	struct input_dev *input_dev = tj9->input_dev;
	unsigned int interval;

#if (LINUX_VERSION_CODE > KERNEL_VERSION(2,6,35))
	int error;
	error = kstrtouint(buf, 10, &interval);
	if (error < 0)
		return error;
#else
	interval = (unsigned int)simple_strtoul(buf, NULL, 10);
#endif

	/* Lock the device to prevent races with open/close (and itself) */
	mutex_lock(&input_dev->mutex);

	/*
	 * Set current interval to the greater of the minimum interval or
	 * the requested interval
	 */
	tj9->poll_interval = max(interval, tj9->pdata.min_interval);
	tj9->poll_delay = msecs_to_jiffies(tj9->poll_interval);

	kxtj9_update_odr(tj9->poll_interval);

	mutex_unlock(&input_dev->mutex);

	return count;
}

/* Allow users to enable/disable the device */
static ssize_t kxtj9_set_enable(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t count)
{
	struct input_dev *input_dev = tj9->input_dev;
	unsigned int enable;

#if (LINUX_VERSION_CODE > KERNEL_VERSION(2,6,35))
	int error;
	error = kstrtouint(buf, 10, &enable);
	if (error < 0)
		return error;
#else
	enable = (unsigned int)simple_strtoul(buf, NULL, 10);
#endif

	/* Lock the device to prevent races with open/close (and itself) */
	mutex_lock(&input_dev->mutex);

	if(enable)
		kxtj9_enable(1);
	else
		kxtj9_disable(1);

	mutex_unlock(&input_dev->mutex);

	return count;
}

static DEVICE_ATTR(poll, S_IRUGO|S_IWUSR, kxtj9_get_poll, kxtj9_set_poll);
static DEVICE_ATTR(enable, S_IWUSR, NULL, kxtj9_set_enable);

static struct attribute *kxtj9_attributes[] = {
	&dev_attr_poll.attr,
	&dev_attr_enable.attr,
	NULL
};

static struct attribute_group kxtj9_attribute_group = {
	.attrs = kxtj9_attributes
};

static int kxtj9_get_count(char *buf, int bufsize) 
{
    	//printk("get_count x:%d, y:%d, z:%d\n", x, y, z);
    	sprintf(buf, "%04x %04x %04x", a_x, a_y, a_z);

	return 0;
}

static int kxtj9_get_mg(char *buf, int bufsize)
{
#if 1
	struct input_dev *input_dev = tj9->input_dev;
	const char ACC_REG_SIZE = 6;
	int err;
	/* Data bytes from hardware xH, xL, yH, yL, zH, zL */
	u8 acc_data[ACC_REG_SIZE];
	/* x,y,z hardware values */
	int hw_d[3], xyz[3], mg[3];

	if((!buf)||(bufsize<=(sizeof(mg))))
		return -1;

	mutex_lock(&input_dev->mutex);
	err = kxtj9_i2c_read(XOUT_H, acc_data, ACC_REG_SIZE);
	mutex_unlock(&input_dev->mutex);
	if(err < 0)
		return err;

	hw_d[0] = (int) (((acc_data[0] << 4) | (acc_data[1] >> 4)) - OFFSET);
	hw_d[1] = (int) (((acc_data[2] << 4) | (acc_data[3] >> 4)) - OFFSET);
	hw_d[2] = (int) (((acc_data[4] << 4) | (acc_data[5] >> 4)) - OFFSET);

	xyz[0] = ((tj9->pdata.negate_x) ? (-hw_d[tj9->pdata.axis_map_x])
			: (hw_d[tj9->pdata.axis_map_x]));
	xyz[1] = ((tj9->pdata.negate_y) ? (-hw_d[tj9->pdata.axis_map_y])
			: (hw_d[tj9->pdata.axis_map_y]));
	xyz[2] = ((tj9->pdata.negate_z) ? (-hw_d[tj9->pdata.axis_map_z])
			: (hw_d[tj9->pdata.axis_map_z]));

	xyz[0]  = xyz[0] * 819 / 1024;
	xyz[1]  = xyz[1] * 819 / 1024;
	xyz[2]  = xyz[2] * 819 / 1024;

	mg[0] = xyz[0] * 1000 / (tj9->sensitivity);
	mg[1] = xyz[1] * 1000 / (tj9->sensitivity);
	mg[2] = xyz[2] * 1000 / (tj9->sensitivity);

	sprintf(buf, "%d %d %d",mg[0], mg[1], mg[2]);

#if defined DEBUG && DEBUG == 1
	dev_info(&tj9->client->dev, "%s: [%5d] [%5d] [%5d]\n", __FUNCTION__, mg[0], mg[1], mg[2]);
#endif
	return err;
#else
	return 0;
#endif
}


static int __devinit kxtj9_verify(void)
{
	int retval;

	retval = kxtj9_device_power_on();
	if (retval < 0)
		return retval;

	retval = i2c_smbus_read_byte_data(tj9->client, WHO_AM_I);
	if (retval < 0) {
		dev_err(&tj9->client->dev, "read err int source\n");
		goto out;
	}

	//retval = retval != 0x07 ? -EIO : 0;
	retval = 0;

out:
	kxtj9_device_power_off();
	return retval;
}

static int kxtj9_open(struct inode *inode, struct file *file) 
{
	int ret = -1;
	struct input_dev *input_dev = tj9->input_dev;

	TR1("open\n");

	mutex_lock(&input_dev->mutex);

	if(kxtj9_enable(0) < 0) {
		mutex_unlock(&input_dev->mutex);
		return ret;
	}
	mutex_unlock(&input_dev->mutex);

	return 0;
}

static int kxtj9_release(struct inode *inode, struct file *file)
{
	struct input_dev *input_dev = tj9->input_dev;

	mutex_lock(&input_dev->mutex);
	kxtj9_disable(0);
	mutex_unlock(&input_dev->mutex);

	return 0;
}

static long kxtj9_ioctl(struct file *file, unsigned int cmd,
		unsigned long arg)
{
	struct input_dev *input_dev = tj9->input_dev;
    mutex_lock(&input_dev->mutex);
	char buffer[IOCTL_BUFFER_SIZE];
	void __user *data;
	u8 reg_buffer = 0x00;
	//const u8 set = 0xFF, unset = 0x00;
	int retval=0, val_int=0;
	short val_short=0;

	switch (cmd) {
		case KXUD9_IOCTL_GET_COUNT:
			data = (void __user *) arg;
			if(data == NULL){
				retval = -EFAULT;
				goto err_out;
			}
			retval = kxtj9_get_count(buffer, sizeof(buffer));
			if(retval < 0) {
				goto err_out;
            }

			if(copy_to_user(data, buffer, sizeof(buffer))) {
				retval = -EFAULT;
				goto err_out;
			}
            mutex_unlock(&input_dev->mutex);
			break;

		case KXUD9_IOCTL_GET_MG:
			data = (void __user *) arg;
			if(data == NULL){
				retval = -EFAULT;
				goto err_out;
			}
			retval = kxtj9_get_mg(buffer, sizeof(buffer));
			if(retval < 0)
				goto err_out;

			if(copy_to_user(data, buffer, sizeof(buffer))) {
				retval = -EFAULT;
				goto err_out;
			}
            mutex_unlock(&input_dev->mutex);
			break;
		case KXUD9_IOCTL_ENABLE_OUTPUT:
			retval = kxtj9_enable(0);
			if(retval < 0) {
				goto err_out;
			}
			mutex_unlock(&input_dev->mutex);
			break;

		case KXUD9_IOCTL_DISABLE_OUTPUT:
			kxtj9_disable(0);
			mutex_unlock(&input_dev->mutex);
			break;

		case KXUD9_IOCTL_GET_ENABLE:
			data = (void __user *) arg;
			if(data == NULL){
				retval = -EFAULT;
				goto err_out;
			}
			retval = kxtj9_get_bits(CTRL_REGB, &reg_buffer, ENABLE);
			if(retval < 0)
				goto err_out;

			val_short = (short)reg_buffer;

			if(copy_to_user(data, &val_short, sizeof(val_short))) {
				retval = -EFAULT;
				goto err_out;
			}
			mutex_unlock(&input_dev->mutex);
			break;

		case KXUD9_IOCTL_RESET:
			retval = kxtj9_reset();
			if(retval < 0)
				goto err_out;
			mutex_unlock(&input_dev->mutex);
			break;

		case KXUD9_IOCTL_UPDATE_ODR:
			data = (void __user *) arg;
			if(data == NULL){
				retval = -EFAULT;
				goto err_out;
			}
			if(copy_from_user(&val_int, data, sizeof(val_int))) {
				retval = -EFAULT;
				goto err_out;
			}

			tj9->res_interval = max(val_int, tj9->pdata.min_interval);

			retval = kxtj9_update_odr(tj9->res_interval);
			if(retval < 0)
				goto err_out;
			mutex_unlock(&input_dev->mutex);
			break;

		default:
			retval = -ENOIOCTLCMD;
			mutex_unlock(&input_dev->mutex);
			break;
	}

	return retval;
err_out:
	mutex_unlock(&input_dev->mutex);
	return retval;
}


static struct file_operations kxtj9_fops = {
	.owner = THIS_MODULE,
	.open = kxtj9_open,
	.release = kxtj9_release,
	.unlocked_ioctl = kxtj9_ioctl,
};

static struct miscdevice kxtj9_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = NAME_DEV,
	.fops = &kxtj9_fops,
};


#ifdef CONFIG_EARLYSUSPEND

static void kxtj9_early_suspend(struct early_suspend * h)
{
	struct input_dev *input_dev;

	input_dev = tj9->input_dev;

	mutex_lock(&input_dev->mutex);
	kxtj9_disable(1);
	mutex_unlock(&input_dev->mutex);
	TR0("early_suspend\n");
}

static void kxtj9_early_resume(struct early_suspend * h)
{
	struct input_dev *input_dev;

	input_dev = tj9->input_dev;

	mutex_lock(&input_dev->mutex);
	kxtj9_enable(1);
	mutex_unlock(&input_dev->mutex);
	TR0("early_resume\n");
}
static struct early_suspend es = {
	.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN,
	.suspend = kxtj9_early_suspend,
	.resume = kxtj9_early_resume,
};
#endif

static int __devinit kxtj9_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
	const struct kxtj9_platform_data *pdata = client->dev.platform_data;
	int err;

	TR0("probe\n");

	atomic_set(&enable_flag, 0);

	if (!i2c_check_functionality(client->adapter,
				I2C_FUNC_I2C | I2C_FUNC_SMBUS_BYTE_DATA)) {
		dev_err(&client->dev, "client is not i2c capable\n");
		return -ENXIO;
	}

	if (!pdata) {
		dev_err(&client->dev, "platform data is NULL; exiting\n");
		return -EINVAL;
	}

	tj9 = kzalloc(sizeof(*tj9), GFP_KERNEL);
	if (!tj9) {
		dev_err(&client->dev,
				"failed to allocate memory for module data\n");
		return -ENOMEM;
	}

	mutex_init(&tj9->lock);
	tj9->client = client;
	tj9->pdata = *pdata;

	if (pdata->init) {
		err = pdata->init();
		if (err < 0)
			goto err_free_mem;
	}

	err = kxtj9_verify();
	if (err < 0) {
		dev_err(&client->dev, "device not recognized\n");
		goto err_pdata_exit;
	}

	i2c_set_clientdata(client, tj9);

	tj9->ctrl_reg1 = tj9->pdata.res_12bit | tj9->pdata.g_range;
	tj9->poll_interval = tj9->pdata.poll_interval;
	tj9->poll_delay = msecs_to_jiffies(tj9->poll_interval);

#ifdef CONFIG_INPUT_KXTJ9_INT
	gpio_request(irq_to_gpio(client->irq), "kxtj9-irq"); // Replace with appropriate GPIO setup
	gpio_direction_input(irq_to_gpio(client->irq)); // Replace with appropriate GPIO setup
	TR0("irq:%d\n", client->irq);
#else 
	client->irq = 0;
#endif
	atomic_set(&irq_flag, 0);

	if (client->irq) {
		atomic_set(&irq_flag, 1);
		TR0("irq_mode\n");
		err = kxtj9_setup_input_device();
		if (err)
			goto err_pdata_exit;

		tj9->workqueue = create_workqueue("KXTJ9 Workqueue");
		INIT_WORK(&tj9->irq_work, kxtj9_irq_work);
		/* If in irq mode, populate INT_CTRL_REG1 and enable DRDY. */
		tj9->int_ctrl |= KXTJ9_IEN | KXTJ9_IEA | KXTJ9_IEL;
		tj9->ctrl_reg1 |= DRDYE;

		err = request_threaded_irq(client->irq, NULL, kxtj9_isr,
				IRQF_TRIGGER_RISING,
				"kxtj9-irq", tj9);
		if (err) {
			dev_err(&client->dev, "request irq failed: %d\n", err);
			goto err_destroy_input;
		}

		err = sysfs_create_group(&client->dev.kobj, &kxtj9_attribute_group);
		if (err) {
			dev_err(&client->dev, "sysfs create failed: %d\n", err);
			goto err_free_irq;
		}

		err = misc_register(&kxtj9_device);
		if(err) {
			dev_err(&client->dev, "misc. device failed to register.\n");
			goto err_free_irq;
		}
	} else {
		TR0("poll_mode\n");
		err = kxtj9_setup_input_device();
		if (err)
			goto err_pdata_exit;

		tj9->workqueue = create_workqueue("KXTJ9 Workqueue");
		INIT_DELAYED_WORK(&tj9->poll_work, kxtj9_poll_work);

		err = sysfs_create_group(&client->dev.kobj, &kxtj9_attribute_group);
		if (err) {
			dev_err(&client->dev, "sysfs create failed: %d\n", err);
			goto err_destroy_input;
		}

		err = misc_register(&kxtj9_device);
		if(err) {
			dev_err(&client->dev, "misc. device failed to register.\n");
			goto err_destroy_input;
		}
	}


#ifdef CONFIG_EARLYSUSPEND
	register_early_suspend(&es);
#endif


	return 0;
err_free_irq:
	free_irq(client->irq, tj9);
err_destroy_input:
	input_unregister_device(tj9->input_dev);
	destroy_workqueue(tj9->workqueue);
err_pdata_exit:
	if (tj9->pdata.exit)
		tj9->pdata.exit();
err_free_mem:
	kfree(tj9);
	return err;
}

static int __devexit kxtj9_remove(struct i2c_client *client)
{
	struct kxtj9_data *tj9 = i2c_get_clientdata(client);

	sysfs_remove_group(&client->dev.kobj, &kxtj9_attribute_group);
	input_unregister_device(tj9->input_dev);
	misc_deregister(&kxtj9_device);
	destroy_workqueue(tj9->workqueue);

	if (tj9->pdata.exit)
		tj9->pdata.exit();

	kfree(tj9);

	return 0;
}

#ifdef CONFIG_PM_SLEEP
#ifndef CONFIG_EARLYSUSPEND
static int kxtj9_suspend(struct device *dev)
{
	struct input_dev *input_dev;

	input_dev = tj9->input_dev;

	mutex_lock(&input_dev->mutex);

	kxtj9_disable(1);

	mutex_unlock(&input_dev->mutex);
	return 0;
        TR0("suspend\n");
	return 0;	
}

static int kxtj9_resume(struct device *dev)
{
	struct input_dev *input_dev;
	int retval = 0;

	input_dev = tj9->input_dev;

	mutex_lock(&input_dev->mutex);

	kxtj9_enable(1);

	mutex_unlock(&input_dev->mutex);
	return retval;
        TR0("resume\n");
	return 0;
}

static SIMPLE_DEV_PM_OPS(kxtj9_pm_ops, kxtj9_suspend, kxtj9_resume);
#endif
#endif

static const struct i2c_device_id kxtj9_id[] = {
	{ NAME, 0 },
	{ },
};

MODULE_DEVICE_TABLE(i2c, kxtj9_id);

static struct i2c_driver kxtj9_driver = {
	.driver = {
		.name	= NAME,
		.owner	= THIS_MODULE,
#ifdef CONFIG_PM_SLEEP
#ifndef CONFIG_EARLYSUSPEND
		.pm	= &kxtj9_pm_ops,
#endif
#endif
	},
	.probe		= kxtj9_probe,
	.remove		= __devexit_p(kxtj9_remove),
	.id_table	= kxtj9_id,
};

static int __init kxtj9_init(void)
{
	TR0("init\n");
	return i2c_add_driver(&kxtj9_driver);
}
module_init(kxtj9_init);

static void __exit kxtj9_exit(void)
{
	TR0("exit\n");
	i2c_del_driver(&kxtj9_driver);
}
module_exit(kxtj9_exit);

MODULE_DESCRIPTION("KXTJ9 accelerometer driver");
MODULE_AUTHOR("Kuching Tan <kuchingtan@kionix.com>");
MODULE_LICENSE("GPL");
MODULE_VERSION("1.6.1");
