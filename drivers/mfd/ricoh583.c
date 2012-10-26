/*
 * driver/mfd/ricoh583.c
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
/*#define DEBUG			1*/
/*#define VERBOSE_DEBUG		1*/
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/mfd/core.h>
#include <linux/mfd/ricoh583.h>
#include <linux/workqueue.h>
#include <mach/board-ns115.h>
#include <mach/io.h>
#include <mach/hardware.h>

//#define RICOH583_DEBUG_REG

#define RICOH_ONOFFSEL_REG	0x10
#define RICOH_SWCTL_REG		0x5E

/* Interrupt enable register */
#define RICOH583_INT_EN_SYS1	0x19
#define RICOH583_INT_EN_SYS2	0x1D
#define RICOH583_INT_EN_DCDC	0x41
#define RICOH583_INT_EN_RTC	0xED
#define RICOH583_INT_EN_ADC1	0x90
#define RICOH583_INT_EN_ADC2	0x91
#define RICOH583_INT_EN_ADC3	0x92
#define RICOH583_INT_EN_GPIO	0xA8

/* interrupt status registers (monitor regs in Ricoh)*/
#define RICOH583_INTC_INTPOL	0xAD
#define RICOH583_INTC_INTEN	0xAE
#define RICOH583_INTC_INTMON	0xAF

#define RICOH583_INT_MON_GRP	0xAF
#define RICOH583_INT_MON_SYS1	0x1B
#define RICOH583_INT_MON_SYS2	0x1F
#define RICOH583_INT_MON_DCDC	0x43
#define RICOH583_INT_MON_RTC	0xEE

/* interrupt clearing registers */
#define RICOH583_INT_IR_SYS1	0x1A
#define RICOH583_INT_IR_SYS2	0x1E
#define RICOH583_INT_IR_DCDC	0x42
#define RICOH583_INT_IR_RTC	0xEE
#define RICOH583_INT_IR_ADCL	0x94
#define RICOH583_INT_IR_ADCH	0x95
#define RICOH583_INT_IR_ADCEND	0x96
#define RICOH583_INT_IR_GPIOR	0xA9
#define RICOH583_INT_IR_GPIOF	0xAA

/* GPIO register base address */
#define RICOH583_GPIO_IOSEL	0xA0
#define RICOH583_GPIO_PDEN	0xA1
#define RICOH583_GPIO_IOOUT	0xA2
#define RICOH583_GPIO_PGSEL	0xA3
#define RICOH583_GPIO_GPINV	0xA4
#define RICOH583_GPIO_GPDEB	0xA5
#define RICOH583_GPIO_GPEDGE1	0xA6
#define RICOH583_GPIO_GPEDGE2	0xA7
#define RICOH583_GPIO_EN_GPIR	0xA8
#define RICOH583_GPIO_MON_IOIN	0xAB
#define RICOH583_GPIO_GPOFUNC	0xAC
#define RICOH583_INTC_INTEN	0xAE

enum int_type {
	SYS_INT	 = 0x1,
	DCDC_INT = 0x2,
	RTC_INT  = 0x4,
	ADC_INT  = 0x8,
	GPIO_INT = 0x10,
};

struct ricoh583_irq_data {
	u8	int_type;
	u8	master_bit;
	u8	int_en_bit;
	u8	mask_reg_index;
	int	grp_index;
};

struct deepsleep_control_data {
	u8 reg_add;
	u8 ds_pos_bit;
};

static struct i2c_client *ricoh583_i2c_client;

#define RICOH583_IRQ(_int_type, _master_bit, _grp_index, _int_bit, _mask_ind) \
{						\
	.int_type	= _int_type,		\
	.master_bit	= _master_bit,		\
	.grp_index	= _grp_index,		\
	.int_en_bit	= _int_bit,		\
	.mask_reg_index	= _mask_ind,		\
}

static const struct ricoh583_irq_data ricoh583_irqs[] = {
	[RICOH583_IRQ_ONKEY]		= RICOH583_IRQ(SYS_INT,  0, 0, 0, 0),
	[RICOH583_IRQ_ACOK]		= RICOH583_IRQ(SYS_INT,  0, 1, 1, 0),
	[RICOH583_IRQ_LIDOPEN]		= RICOH583_IRQ(SYS_INT,  0, 2, 2, 0),
	[RICOH583_IRQ_PREOT]		= RICOH583_IRQ(SYS_INT,  0, 3, 3, 0),
	[RICOH583_IRQ_CLKSTP]		= RICOH583_IRQ(SYS_INT,  0, 4, 4, 0),
	[RICOH583_IRQ_ONKEY_OFF]	= RICOH583_IRQ(SYS_INT,  0, 5, 5, 0),
	[RICOH583_IRQ_WD]		= RICOH583_IRQ(SYS_INT,  0, 7, 7, 0),
	[RICOH583_IRQ_EN_PWRREQ1]	= RICOH583_IRQ(SYS_INT,  0, 8, 0, 1),
	[RICOH583_IRQ_EN_PWRREQ2]	= RICOH583_IRQ(SYS_INT,  0, 9, 1, 1),
	[RICOH583_IRQ_PRE_VINDET]	= RICOH583_IRQ(SYS_INT,  0, 10, 2, 1),

	[RICOH583_IRQ_DC0LIM]		= RICOH583_IRQ(DCDC_INT, 1, 0, 0, 2),
	[RICOH583_IRQ_DC1LIM]		= RICOH583_IRQ(DCDC_INT, 1, 1, 1, 2),
	[RICOH583_IRQ_DC2LIM]		= RICOH583_IRQ(DCDC_INT, 1, 2, 2, 2),
	[RICOH583_IRQ_DC3LIM]		= RICOH583_IRQ(DCDC_INT, 1, 3, 3, 2),

	[RICOH583_IRQ_CTC]		= RICOH583_IRQ(RTC_INT,  2, 0, 0, 3),
	[RICOH583_IRQ_YALE]		= RICOH583_IRQ(RTC_INT,  2, 5, 5, 3),
	[RICOH583_IRQ_DALE]		= RICOH583_IRQ(RTC_INT,  2, 6, 6, 3),
	[RICOH583_IRQ_WALE]		= RICOH583_IRQ(RTC_INT,  2, 7, 7, 3),

	[RICOH583_IRQ_AIN1L]		= RICOH583_IRQ(ADC_INT,  3, 0, 0, 4),
	[RICOH583_IRQ_AIN2L]		= RICOH583_IRQ(ADC_INT,  3, 1, 1, 4),
	[RICOH583_IRQ_AIN3L]		= RICOH583_IRQ(ADC_INT,  3, 2, 2, 4),
	[RICOH583_IRQ_VBATL]		= RICOH583_IRQ(ADC_INT,  3, 3, 3, 4),
	[RICOH583_IRQ_VIN3L]		= RICOH583_IRQ(ADC_INT,  3, 4, 4, 4),
	[RICOH583_IRQ_VIN8L]		= RICOH583_IRQ(ADC_INT,  3, 5, 5, 4),
	[RICOH583_IRQ_AIN1H]		= RICOH583_IRQ(ADC_INT,  3, 6, 0, 5),
	[RICOH583_IRQ_AIN2H]		= RICOH583_IRQ(ADC_INT,  3, 7, 1, 5),
	[RICOH583_IRQ_AIN3H]		= RICOH583_IRQ(ADC_INT,  3, 8, 2, 5),
	[RICOH583_IRQ_VBATH]		= RICOH583_IRQ(ADC_INT,  3, 9, 3, 5),
	[RICOH583_IRQ_VIN3H]		= RICOH583_IRQ(ADC_INT,  3, 10, 4, 5),
	[RICOH583_IRQ_VIN8H]		= RICOH583_IRQ(ADC_INT,  3, 11, 5, 5),
	[RICOH583_IRQ_ADCEND]		= RICOH583_IRQ(ADC_INT,  3, 12, 0, 6),

	[RICOH583_IRQ_GPIO0]		= RICOH583_IRQ(GPIO_INT, 4, 0, 0, 7),
	[RICOH583_IRQ_GPIO1]		= RICOH583_IRQ(GPIO_INT, 4, 1, 1, 7),
	[RICOH583_IRQ_GPIO2]		= RICOH583_IRQ(GPIO_INT, 4, 2, 2, 7),
	[RICOH583_IRQ_GPIO3]		= RICOH583_IRQ(GPIO_INT, 4, 3, 3, 7),
	[RICOH583_IRQ_GPIO4]		= RICOH583_IRQ(GPIO_INT, 4, 4, 4, 7),
	[RICOH583_IRQ_GPIO5]		= RICOH583_IRQ(GPIO_INT, 4, 5, 5, 7),
	[RICOH583_IRQ_GPIO6]		= RICOH583_IRQ(GPIO_INT, 4, 6, 6, 7),
	[RICOH583_IRQ_GPIO7]		= RICOH583_IRQ(GPIO_INT, 4, 7, 7, 7),
	[RICOH583_NR_IRQS]		= RICOH583_IRQ(GPIO_INT, 4, 8, 8, 7),
};

#define DEEPSLEEP_INIT(_id, _reg, _pos)		\
	[RICOH583_DS_##_id] = {.reg_add = _reg, .ds_pos_bit = _pos}

static struct deepsleep_control_data deepsleep_data[] = {
	DEEPSLEEP_INIT(DC1, 0x21, 4),
	DEEPSLEEP_INIT(DC2, 0x22, 0),
	DEEPSLEEP_INIT(DC3, 0x22, 4),
	DEEPSLEEP_INIT(LDO0, 0x23, 0),
	DEEPSLEEP_INIT(LDO1, 0x23, 4),
	DEEPSLEEP_INIT(LDO2, 0x24, 0),
	DEEPSLEEP_INIT(LDO3, 0x24, 4),
	DEEPSLEEP_INIT(LDO4, 0x25, 0),
	DEEPSLEEP_INIT(LDO5, 0x25, 4),
	DEEPSLEEP_INIT(LDO6, 0x26, 0),
	DEEPSLEEP_INIT(LDO7, 0x26, 4),
	DEEPSLEEP_INIT(LDO8, 0x27, 0),
	DEEPSLEEP_INIT(LDO9, 0x27, 4),
	DEEPSLEEP_INIT(PSO0, 0x28, 0),
	DEEPSLEEP_INIT(PSO1, 0x28, 4),
	DEEPSLEEP_INIT(PSO2, 0x29, 0),
	DEEPSLEEP_INIT(PSO3, 0x29, 4),
	DEEPSLEEP_INIT(PSO4, 0x2A, 0),
	DEEPSLEEP_INIT(PSO5, 0x2A, 4),
	DEEPSLEEP_INIT(PSO6, 0x2B, 0),
	DEEPSLEEP_INIT(PSO7, 0x2B, 4),
};

#define MAX_INTERRUPT_MASKS	8
#define MAX_MAIN_INTERRUPT	5
#define EXT_PWR_REQ		\
	(RICOH583_EXT_PWRREQ1_CONTROL | RICOH583_EXT_PWRREQ2_CONTROL)

struct ricoh583 {
	struct device		*dev;
	struct i2c_client	*client;
	struct mutex		io_lock;
	int			gpio_base;
	struct gpio_chip	gpio;
	int			irq_base;
	struct irq_chip		irq_chip;
	struct mutex		irq_lock;
	struct work_struct	work;
	struct workqueue_struct	*queue;
	unsigned long		group_irq_en[MAX_MAIN_INTERRUPT];

	/* For main interrupt bits in INTC */
	u8			intc_inten_cache;
	u8			intc_inten_reg;

	/* For group interrupt bits and address */
	u8			irq_en_cache[MAX_INTERRUPT_MASKS];
	u8			irq_en_reg[MAX_INTERRUPT_MASKS];
	u8			irq_en_add[MAX_INTERRUPT_MASKS];

	/* Interrupt monitor and clear register */
	u8			irq_mon_add[MAX_INTERRUPT_MASKS + 1];
	u8			irq_clr_add[MAX_INTERRUPT_MASKS + 1];
	u8			main_int_type[MAX_INTERRUPT_MASKS + 1];

	/* For gpio edge */
	u8			gpedge_cache[2];
	u8			gpedge_reg[2];
	u8			gpedge_add[2];
};

static inline int __ricoh583_read(struct i2c_client *client,
		u8 reg, uint8_t *val)
{
	int ret;

	ret = i2c_smbus_read_byte_data(client, reg);
	if (ret < 0) {
		dev_err(&client->dev, "failed reading at 0x%02x\n", reg);
		return ret;
	}

	*val = (uint8_t)ret;
	dev_dbg(&client->dev, "ricoh583: reg read  reg=%x, val=%x\n",
			reg, *val);
	return 0;
}

static inline int __ricoh583_bulk_reads(struct i2c_client *client, u8 reg,
		int len, uint8_t *val)
{
	int ret;
	int i;

	ret = i2c_smbus_read_i2c_block_data(client, reg, len, val);
	if (ret < 0) {
		dev_err(&client->dev, "failed reading from 0x%02x\n", reg);
		return ret;
	}
	for (i = 0; i < len; ++i) {
		dev_dbg(&client->dev, "ricoh583: reg read  reg=%x, val=%x\n",
				reg + i, *(val + i));
	}
	return 0;
}

static inline int __ricoh583_write(struct i2c_client *client,
		u8 reg, uint8_t val)
{
	int ret;

	dev_dbg(&client->dev, "ricoh583: reg write  reg=%x, val=%x\n",
			reg, val);
	ret = i2c_smbus_write_byte_data(client, reg, val);
	if (ret < 0) {
		dev_err(&client->dev, "failed writing 0x%02x to 0x%02x\n",
				val, reg);
		return ret;
	}

	return 0;
}

static inline int __ricoh583_bulk_writes(struct i2c_client *client, u8 reg,
		int len, uint8_t *val)
{
	int ret;
	int i;

	for (i = 0; i < len; ++i) {
		dev_dbg(&client->dev, "ricoh583: reg write  reg=%x, val=%x\n",
				reg + i, *(val + i));
	}

	ret = i2c_smbus_write_i2c_block_data(client, reg, len, val);
	if (ret < 0) {
		dev_err(&client->dev, "failed writings to 0x%02x\n", reg);
		return ret;
	}

	return 0;
}

int ricoh583_write(struct device *dev, u8 reg, uint8_t val)
{
	struct ricoh583 *ricoh583 = dev_get_drvdata(dev);
	int ret = 0;

	mutex_lock(&ricoh583->io_lock);
	ret = __ricoh583_write(to_i2c_client(dev), reg, val);
	mutex_unlock(&ricoh583->io_lock);

	return ret;
}
EXPORT_SYMBOL_GPL(ricoh583_write);

int ricoh583_bulk_writes(struct device *dev, u8 reg, u8 len, uint8_t *val)
{
	struct ricoh583 *ricoh583 = dev_get_drvdata(dev);
	int ret = 0;

	mutex_lock(&ricoh583->io_lock);
	ret = __ricoh583_bulk_writes(to_i2c_client(dev), reg, len, val);
	mutex_unlock(&ricoh583->io_lock);

	return ret;
}
EXPORT_SYMBOL_GPL(ricoh583_bulk_writes);

int ricoh583_read(struct device *dev, u8 reg, uint8_t *val)
{
	return __ricoh583_read(to_i2c_client(dev), reg, val);
}
EXPORT_SYMBOL_GPL(ricoh583_read);

int ricoh583_bulk_reads(struct device *dev, u8 reg, u8 len, uint8_t *val)
{
	return __ricoh583_bulk_reads(to_i2c_client(dev), reg, len, val);
}
EXPORT_SYMBOL_GPL(ricoh583_bulk_reads);

int ricoh583_set_bits(struct device *dev, u8 reg, uint8_t bit_mask)
{
	struct ricoh583 *ricoh583 = dev_get_drvdata(dev);
	uint8_t reg_val;
	int ret = 0;

	mutex_lock(&ricoh583->io_lock);

	ret = __ricoh583_read(to_i2c_client(dev), reg, &reg_val);
	if (ret)
		goto out;

	if ((reg_val & bit_mask) != bit_mask) {
		reg_val |= bit_mask;
		ret = __ricoh583_write(to_i2c_client(dev), reg, reg_val);
	}
out:
	mutex_unlock(&ricoh583->io_lock);
	return ret;
}
EXPORT_SYMBOL_GPL(ricoh583_set_bits);

int ricoh583_clr_bits(struct device *dev, u8 reg, uint8_t bit_mask)
{
	struct ricoh583 *ricoh583 = dev_get_drvdata(dev);
	uint8_t reg_val;
	int ret = 0;

	mutex_lock(&ricoh583->io_lock);

	ret = __ricoh583_read(to_i2c_client(dev), reg, &reg_val);
	if (ret)
		goto out;

	if (reg_val & bit_mask) {
		reg_val &= ~bit_mask;
		ret = __ricoh583_write(to_i2c_client(dev), reg, reg_val);
	}
out:
	mutex_unlock(&ricoh583->io_lock);
	return ret;
}
EXPORT_SYMBOL_GPL(ricoh583_clr_bits);

int ricoh583_update(struct device *dev, u8 reg, uint8_t val, uint8_t mask)
{
	struct ricoh583 *ricoh583 = dev_get_drvdata(dev);
	uint8_t reg_val;
	int ret = 0;

	mutex_lock(&ricoh583->io_lock);

	ret = __ricoh583_read(ricoh583->client, reg, &reg_val);
	if (ret)
		goto out;

	if ((reg_val & mask) != val) {
		reg_val = (reg_val & ~mask) | (val & mask);
		ret = __ricoh583_write(ricoh583->client, reg, reg_val);
	}
out:
	mutex_unlock(&ricoh583->io_lock);
	return ret;
}
EXPORT_SYMBOL_GPL(ricoh583_update);

static int __ricoh583_set_ext_pwrreq1_control(struct device *dev,
		enum ricoh583_deepsleep_control_id id,
		enum ricoh583_ext_pwrreq_control ext_pwr, int slots)
{
	int ret;
	uint8_t sleepseq_val;
	u8 en_bit;
	u8 slot_bit;

	if (!(ext_pwr & RICOH583_EXT_PWRREQ1_CONTROL))
		return 0;

	if (id == RICOH583_DS_DC0) {
		dev_err(dev, "PWRREQ1 is invalid control for rail %d\n", id);
		return -EINVAL;
	}

	en_bit = deepsleep_data[id].ds_pos_bit;
	slot_bit = en_bit + 1;
	ret = ricoh583_read(dev, deepsleep_data[id].reg_add, &sleepseq_val);
	if (ret < 0) {
		dev_err(dev, "Error in reading reg 0x%x\n",
				deepsleep_data[id].reg_add);
		return ret;
	}

	sleepseq_val &= ~(0xF << en_bit);
	sleepseq_val |= (1 << en_bit);
	sleepseq_val |= ((slots & 0x7) << slot_bit);
	ret = ricoh583_set_bits(dev, RICOH_ONOFFSEL_REG, (1 << 1));
	if (ret < 0) {
		dev_err(dev, "Error in updating the 0x%02x register\n",
				RICOH_ONOFFSEL_REG);
		return ret;
	}

	ret = ricoh583_write(dev, deepsleep_data[id].reg_add, sleepseq_val);
	if (ret < 0) {
		dev_err(dev, "Error in writing reg 0x%x\n",
				deepsleep_data[id].reg_add);
		return ret;
	}

#if 0
	if (id == RICOH583_DS_LDO4) {
		ret = ricoh583_write(dev, RICOH_SWCTL_REG, 0x1);
		if (ret < 0)
			dev_err(dev, "Error in writing reg 0x%x\n",
					RICOH_SWCTL_REG);
	}
#endif
	return ret;
}

static int __ricoh583_set_ext_pwrreq2_control(struct device *dev,
		enum ricoh583_deepsleep_control_id id,
		enum ricoh583_ext_pwrreq_control ext_pwr)
{
	int ret;

	if (!(ext_pwr & RICOH583_EXT_PWRREQ2_CONTROL))
		return 0;

	if (id != RICOH583_DS_DC0) {
		dev_err(dev, "PWRREQ2 is invalid control for rail %d\n", id);
		return -EINVAL;
	}

	ret = ricoh583_set_bits(dev, RICOH_ONOFFSEL_REG, (1 << 2));
	if (ret < 0)
		dev_err(dev, "Error in updating the ONOFFSEL 0x10 register\n");
	return ret;
}

int ricoh583_ext_power_req_config(struct device *dev,
		enum ricoh583_deepsleep_control_id id,
		enum ricoh583_ext_pwrreq_control ext_pwr_req,
		int deepsleep_slot_nr)
{
	if ((ext_pwr_req & EXT_PWR_REQ) == EXT_PWR_REQ)
		return -EINVAL;

	if (ext_pwr_req & RICOH583_EXT_PWRREQ1_CONTROL)
		return __ricoh583_set_ext_pwrreq1_control(dev, id,
				ext_pwr_req, deepsleep_slot_nr);

	if (ext_pwr_req & RICOH583_EXT_PWRREQ2_CONTROL)
		return __ricoh583_set_ext_pwrreq2_control(dev,
				id, ext_pwr_req);
	return 0;
}
EXPORT_SYMBOL_GPL(ricoh583_ext_power_req_config);

static int __devinit ricoh583_ext_power_init(struct ricoh583 *ricoh583,
		struct ricoh583_platform_data *pdata)
{
	int ret;
	int i;
	uint8_t on_off_val = 0;

	/*  Clear ONOFFSEL register */
	mutex_lock(&ricoh583->io_lock);
	if (pdata->enable_shutdown_pin)
		on_off_val |= 0x1;

	ret = __ricoh583_write(ricoh583->client, RICOH_ONOFFSEL_REG,
			on_off_val);
	if (ret < 0)
		dev_err(ricoh583->dev, "Error in writing reg %d error: "
				"%d\n", RICOH_ONOFFSEL_REG, ret);

	ret = __ricoh583_write(ricoh583->client, RICOH_SWCTL_REG, 0x0);
	if (ret < 0)
		dev_err(ricoh583->dev, "Error in writing reg %d error: "
				"%d\n", RICOH_SWCTL_REG, ret);

	/* Clear sleepseq register */
	for (i = 0x21; i < 0x2B; ++i) {
		ret = __ricoh583_write(ricoh583->client, i, 0x0);
		if (ret < 0)
			dev_err(ricoh583->dev, "Error in writing reg 0x%02x "
					"error: %d\n", i, ret);
	}
	mutex_unlock(&ricoh583->io_lock);
	return ret;
}

static struct i2c_client *ricoh583_i2c_client;

void ricoh583_power_off(void)
{
	int ret;

	if (!ricoh583_i2c_client)
		return;
	dev_emerg(&ricoh583_i2c_client->dev, "%s\n", __func__);
	ret = __ricoh583_write(ricoh583_i2c_client, 0x13, 1);
	if (ret < 0){
		dev_err(&ricoh583_i2c_client->dev, "Error in writing reg %d error: "
				"%d\n", 0x13, ret);
	}

	return;
}

extern void dw_i2c_master_init(void* i2c_base, unsigned char slv_addr);
extern int  dw_i2c_smbus_read(void* i2c_base, unsigned char * out_buf, unsigned int wlen, unsigned char *rbuf, unsigned rlen);
extern int dw_i2c_send_bytes(void* i2c_base, unsigned char * out_buf, unsigned int len);

static int ricoh583_smbus_read_reg(void __iomem * i2c_base, unsigned char slv_addr, 
		unsigned char reg, unsigned char * rbuf)
{
	dw_i2c_master_init(i2c_base, slv_addr);
	dw_i2c_smbus_read(i2c_base, &reg, 1, rbuf, 1);

	return 0;
}

static int ricoh583_smbus_write_reg(void __iomem * i2c_base,unsigned char slv_addr, 
		unsigned char reg, unsigned char  val)
{
	char wbuf[2];

	wbuf[0] = reg;
	wbuf[1] = val;
	dw_i2c_master_init(i2c_base, slv_addr);
	dw_i2c_send_bytes(i2c_base, wbuf, 2);
	return 0;
}

void ricoh583_restart(char str, const char * cmd)
{
	uint8_t val = 0;
	void __iomem * i2c_base = __io_address(NS115_I2C1_BASE);

	if (!ricoh583_i2c_client)
		return;
	dev_emerg(&ricoh583_i2c_client->dev, "%s\n", __func__);

	ricoh583_smbus_read_reg(i2c_base, ricoh583_i2c_client->addr, 0x14, &val);                             
	val |= 0x1;
	ricoh583_smbus_write_reg(i2c_base, ricoh583_i2c_client->addr, 0x14, val);

	ricoh583_smbus_read_reg(i2c_base, ricoh583_i2c_client->addr, 0x13, &val);                             
	val |= 0x1;
	ricoh583_smbus_write_reg(i2c_base, ricoh583_i2c_client->addr, 0x13, val);

	return;
}

static int ricoh583_gpio_get(struct gpio_chip *gc, unsigned offset)
{
	struct ricoh583 *ricoh583 = container_of(gc, struct ricoh583, gpio);
	uint8_t val;
	int ret;

	ret = __ricoh583_read(ricoh583->client, RICOH583_GPIO_MON_IOIN, &val);
	if (ret < 0)
		return ret;
	//printk("%s:%d\n", __func__, val);

	return ((val & (0x1 << offset)) != 0);
}

static void ricoh583_gpio_set(struct gpio_chip *chip, unsigned offset,
		int value)
{
	struct ricoh583 *ricoh583 = container_of(chip, struct ricoh583, gpio);
	if (value)
		ricoh583_set_bits(ricoh583->dev, RICOH583_GPIO_IOOUT,
				1 << offset);
	else
		ricoh583_clr_bits(ricoh583->dev, RICOH583_GPIO_IOOUT,
				1 << offset);
	//printk("%s offset:%d value:%d\n", __func__, offset, value);
}

static int ricoh583_gpio_input(struct gpio_chip *chip, unsigned offset)
{
	struct ricoh583 *ricoh583 = container_of(chip, struct ricoh583, gpio);

	return ricoh583_clr_bits(ricoh583->dev, RICOH583_GPIO_IOSEL,
			1 << offset);
}

static int ricoh583_gpio_output(struct gpio_chip *chip, unsigned offset,
		int value)
{
	struct ricoh583 *ricoh583 = container_of(chip, struct ricoh583, gpio);

	ricoh583_gpio_set(chip, offset, value);
	return ricoh583_set_bits(ricoh583->dev, RICOH583_GPIO_IOSEL,
			1 << offset);
}

static int ricoh583_gpio_to_irq(struct gpio_chip *chip, unsigned off)
{
	struct ricoh583 *ricoh583 = container_of(chip, struct ricoh583, gpio);

	if ((off >= 0) && (off < 8))
		return ricoh583->irq_base + RICOH583_IRQ_GPIO0 + off;

	return -EIO;
}

static int ricoh583_gpio_request(struct gpio_chip *chip, unsigned offset)
{
	struct ricoh583 *ricoh583 = container_of(chip, struct ricoh583, gpio);
	int ret;

	ret = ricoh583_clr_bits(ricoh583->dev, RICOH583_GPIO_PGSEL,
			1 << offset);
	if (ret < 0)
		dev_err(ricoh583->dev, "%s(): The error in writing register "
				"0x%02x\n", __func__, RICOH583_GPIO_PGSEL);
	return ret;
}

static void ricoh583_gpio_free(struct gpio_chip *chip, unsigned offset)
{
#if 0
	struct ricoh583 *ricoh583 = container_of(chip, struct ricoh583, gpio);
	int ret;

	ret = ricoh583_set_bits(ricoh583->dev, RICOH583_GPIO_PGSEL,
			1 << offset);
	if (ret < 0)
		dev_err(ricoh583->dev, "%s(): The error in writing register "
				"0x%02x\n", __func__, RICOH583_GPIO_PGSEL);
#else
	return;
#endif
}

#define RICOH_GPIO_SLEEP_ID(_id)  (RICOH583_DS_PSO0 + i)
static int __devinit ricoh583_gpio_init(struct ricoh583 *ricoh583,
		struct ricoh583_platform_data *pdata)
{
	int ret;
	int i;
	struct ricoh583_gpio_init_data *ginit;

	if (pdata->gpio_base  <= 0)
		return -1;

	ret = ricoh583_write(ricoh583->dev, RICOH583_GPIO_PGSEL, 0xFF);
	if (ret < 0) {
		dev_err(ricoh583->dev, "%s(): The error in writing register "
				"0x%02x\n", __func__, RICOH583_GPIO_PGSEL);
		return -1;
	}

	for (i = 0; i < pdata->num_gpioinit_data; ++i) {
		ginit = &pdata->gpio_init_data[i];
		if (ginit->ext_pwr_req){
			ret = ricoh583_ext_power_req_config(ricoh583->dev, RICOH_GPIO_SLEEP_ID(i),
					ginit->ext_pwr_req, ginit->deepsleep_slots);
			if (ret < 0){
				return -1;
			}
		}
		if (!ginit->init_apply)
			continue;
		if (ginit->pulldn_en)
			ret = ricoh583_set_bits(ricoh583->dev,
					RICOH583_GPIO_PDEN, 1 << i);
		else
			ret = ricoh583_clr_bits(ricoh583->dev,
					RICOH583_GPIO_PDEN, 1 << i);
		if (ret < 0){
			dev_err(ricoh583->dev, "Gpio %d init "
					"pden configuration failed: %d\n", i, ret);
			return -1;
		}

		if (ginit->output_mode_en) {
			if (ginit->output_val)
				ret = ricoh583_set_bits(ricoh583->dev,
						RICOH583_GPIO_IOOUT, 1 << i);
			else
				ret = ricoh583_clr_bits(ricoh583->dev,
						RICOH583_GPIO_IOOUT, 1 << i);
			if (!ret)
				ret = ricoh583_set_bits(ricoh583->dev,
						RICOH583_GPIO_IOSEL, 1 << i);
		} else
			ret = ricoh583_clr_bits(ricoh583->dev,
					RICOH583_GPIO_IOSEL, 1 << i);

		if (ret < 0){
			dev_err(ricoh583->dev, "Gpio %d init "
					"dir configuration failed: %d\n", i, ret);
			return -1;
		}

		ret = ricoh583_clr_bits(ricoh583->dev, RICOH583_GPIO_PGSEL,
				1 << i);
		if (ret < 0){
			dev_err(ricoh583->dev, "%s(): The error in writing "
					"register 0x%02x\n", __func__,
					RICOH583_GPIO_PGSEL);
			return -1;
		}
	}

	ricoh583->gpio.owner		= THIS_MODULE;
	ricoh583->gpio.label		= ricoh583->client->name;
	ricoh583->gpio.dev		= ricoh583->dev;
	ricoh583->gpio.base		= pdata->gpio_base;
	ricoh583->gpio.ngpio		= RICOH583_NR_GPIO;
	ricoh583->gpio.can_sleep	= 1;

	ricoh583->gpio.request	= ricoh583_gpio_request;
	ricoh583->gpio.free	= ricoh583_gpio_free;
	ricoh583->gpio.direction_input	= ricoh583_gpio_input;
	ricoh583->gpio.direction_output	= ricoh583_gpio_output;
	ricoh583->gpio.set		= ricoh583_gpio_set;
	ricoh583->gpio.get		= ricoh583_gpio_get;
	ricoh583->gpio.to_irq	   = ricoh583_gpio_to_irq;

	ret = gpiochip_add(&ricoh583->gpio);
	if (ret)
		dev_warn(ricoh583->dev, "GPIO registration failed: %d\n", ret);

	ret = ricoh583_write(ricoh583->dev, RICOH583_GPIO_GPOFUNC, 0x20);
    if (ret < 0) {
		dev_err(ricoh583->dev, "%s(): The error in writing register "
					    "0x%02x\n", __func__, RICOH583_GPIO_GPOFUNC);
			    return -1;
	}
	return ret;
}

static void ricoh583_irq_lock(struct irq_data *irq_data)
{
	struct ricoh583 *ricoh583 = irq_data_get_irq_chip_data(irq_data);

	mutex_lock(&ricoh583->irq_lock);
}

static void ricoh583_irq_unmask(struct irq_data *irq_data)
{
	struct ricoh583 *ricoh583 = irq_data_get_irq_chip_data(irq_data);
	unsigned int __irq = irq_data->irq - ricoh583->irq_base;
	const struct ricoh583_irq_data *data = &ricoh583_irqs[__irq];

	ricoh583->group_irq_en[data->grp_index] |= (1 << data->grp_index);
	if (ricoh583->group_irq_en[data->grp_index])
		ricoh583->intc_inten_reg |= 1 << data->master_bit;

	ricoh583->irq_en_reg[data->mask_reg_index] |= 1 << data->int_en_bit;
}

static void ricoh583_irq_mask(struct irq_data *irq_data)
{
	struct ricoh583 *ricoh583 = irq_data_get_irq_chip_data(irq_data);
	unsigned int __irq = irq_data->irq - ricoh583->irq_base;
	const struct ricoh583_irq_data *data = &ricoh583_irqs[__irq];

	ricoh583->group_irq_en[data->grp_index] &= ~(1 << data->grp_index);
	if (!ricoh583->group_irq_en[data->grp_index])
		ricoh583->intc_inten_reg &= ~(1 << data->master_bit);

	ricoh583->irq_en_reg[data->mask_reg_index] &= ~(1 << data->int_en_bit);
}

static void ricoh583_irq_sync_unlock(struct irq_data *irq_data)
{
	struct ricoh583 *ricoh583 = irq_data_get_irq_chip_data(irq_data);
	int i;

	for (i = 0; i < ARRAY_SIZE(ricoh583->gpedge_reg); i++) {
		if (ricoh583->gpedge_reg[i] != ricoh583->gpedge_cache[i]) {
			if (!WARN_ON(__ricoh583_write(ricoh583->client,
							ricoh583->gpedge_add[i],
							ricoh583->gpedge_reg[i])))
				ricoh583->gpedge_cache[i] =
					ricoh583->gpedge_reg[i];
		}
	}

	for (i = 0; i < ARRAY_SIZE(ricoh583->irq_en_reg); i++) {
		if (ricoh583->irq_en_reg[i] != ricoh583->irq_en_cache[i]) {
			if (!WARN_ON(__ricoh583_write(ricoh583->client,
							ricoh583->irq_en_add[i],
							ricoh583->irq_en_reg[i])))
				ricoh583->irq_en_cache[i] =
					ricoh583->irq_en_reg[i];
		}
	}

	if (ricoh583->intc_inten_reg != ricoh583->intc_inten_cache) {
		if (!WARN_ON(__ricoh583_write(ricoh583->client,
						RICOH583_INTC_INTEN, ricoh583->intc_inten_reg)))
			ricoh583->intc_inten_cache = ricoh583->intc_inten_reg;
	}

	mutex_unlock(&ricoh583->irq_lock);
}

static int ricoh583_irq_set_type(struct irq_data *irq_data, unsigned int type)
{
	struct ricoh583 *ricoh583 = irq_data_get_irq_chip_data(irq_data);
	unsigned int __irq = irq_data->irq - ricoh583->irq_base;
	const struct ricoh583_irq_data *data = &ricoh583_irqs[__irq];
	int val = 0;
	int gpedge_index;
	int gpedge_bit_pos;

	if (data->int_type & GPIO_INT) {
		gpedge_index = data->int_en_bit / 4;
		gpedge_bit_pos = data->int_en_bit % 4;

		if (type & IRQ_TYPE_EDGE_FALLING)
			val |= 0x2;

		if (type & IRQ_TYPE_EDGE_RISING)
			val |= 0x1;

		ricoh583->gpedge_reg[gpedge_index] &= ~(3 << gpedge_bit_pos);
		ricoh583->gpedge_reg[gpedge_index] |= (val << gpedge_bit_pos);
		ricoh583_irq_unmask(irq_data);
	}
	return 0;
}

static irqreturn_t ricoh583_irq(int irq, void *data)
{
	struct ricoh583 *ricoh583 = data;

	disable_irq_nosync(ricoh583->client->irq);
	queue_work(ricoh583->queue, &ricoh583->work);

	return IRQ_HANDLED;
}

static void ricoh583_irq_work(struct work_struct *work)
{
	struct ricoh583 *ricoh583 = container_of(work, struct ricoh583, work);
	u8 int_sts[9];
	u8 master_int;
	int i;
	int ret;
	u8 rtc_int_sts = 0;

	/* Clear the status */
	for (i = 0; i < 9; i++)
		int_sts[i] = 0;

	ret  = __ricoh583_read(ricoh583->client, RICOH583_INTC_INTMON,
			&master_int);
	//dev_info(ricoh583->dev, "master_int:0x%x\n", master_int);
	if (ret < 0) {
		dev_err(ricoh583->dev, "Error in reading reg 0x%02x "
				"error: %d\n", RICOH583_INTC_INTMON, ret);
		goto out;
	}

	for (i = 0; i < 9; ++i) {
		if (!(master_int & ricoh583->main_int_type[i]))
			continue;
		ret = __ricoh583_read(ricoh583->client,
				ricoh583->irq_clr_add[i], &int_sts[i]);
		if (ret < 0) {
			dev_err(ricoh583->dev, "Error in reading reg 0x%02x "
					"error: %d\n", ricoh583->irq_clr_add[i], ret);
			int_sts[i] = 0;
			continue;
		}

		if (ricoh583->main_int_type[i] & RTC_INT) {
			rtc_int_sts = 0;
			if (int_sts[i] & 0x1)
				rtc_int_sts |= BIT(6);
			if (int_sts[i] & 0x2)
				rtc_int_sts |= BIT(7);
			if (int_sts[i] & 0x4)
				rtc_int_sts |= BIT(0);
			if (int_sts[i] & 0x8)
				rtc_int_sts |= BIT(5);
		}

		//dev_info(ricoh583->dev, "i:%d clr reg:0x%x mask:0x%x\n", i, ricoh583->irq_clr_add[i] & 0xff, ~int_sts[i] & 0xff);
		ret = __ricoh583_write(ricoh583->client,
				ricoh583->irq_clr_add[i], ~int_sts[i]);
		if (ret < 0) {
			dev_err(ricoh583->dev, "Error in reading reg 0x%02x "
					"error: %d\n", ricoh583->irq_clr_add[i], ret);
		}
		if (ricoh583->main_int_type[i] & RTC_INT)
			int_sts[i] = rtc_int_sts;
	}

	/* Merge gpio interrupts  for rising and falling case*/
	int_sts[7] |= int_sts[8];

	/* Call interrupt handler if enabled */
	for (i = 0; i < RICOH583_NR_IRQS; ++i) {
		const struct ricoh583_irq_data *data = &ricoh583_irqs[i];
		//dev_info(ricoh583->dev, "int_sts:0x%x group_irq_en:0x%x\n", int_sts[data->mask_reg_index],
				//ricoh583->group_irq_en[data->grp_index]);
		if ((int_sts[data->mask_reg_index] & (1 << data->int_en_bit)) &&
				(ricoh583->group_irq_en[data->grp_index] &
				 (1 << data->grp_index))){
			//dev_info(ricoh583->dev, "call int handler: %d\n", i);
			handle_nested_irq(ricoh583->irq_base + i);
		}
	}
out:
	enable_irq(ricoh583->client->irq);
	return ;
}

static int __devinit ricoh583_irq_init(struct ricoh583 *ricoh583, int irq,
		int irq_base)
{
	int i, ret;

	dev_info(ricoh583->dev, "irq:%d irq_base:%d\n", irq, irq_base);
	if (!irq_base) {
		dev_warn(ricoh583->dev, "No interrupt support on IRQ base\n");
		return -EINVAL;
	}

	mutex_init(&ricoh583->irq_lock);

	/* Initialize all locals to 0 */
	for (i = 0; i < MAX_INTERRUPT_MASKS; i++) {
		ricoh583->irq_en_cache[i] = 0;
		ricoh583->irq_en_reg[i] = 0;
	}
	ricoh583->intc_inten_cache = 0;
	ricoh583->intc_inten_reg = 0;
	for (i = 0; i < 2; i++) {
		ricoh583->gpedge_cache[i] = 0;
		ricoh583->gpedge_reg[i] = 0;
	}

	/* Interrupt enable register */
	ricoh583->gpedge_add[0] = RICOH583_GPIO_GPEDGE2;
	ricoh583->gpedge_add[1] = RICOH583_GPIO_GPEDGE1;
	ricoh583->irq_en_add[0] = RICOH583_INT_EN_SYS1;
	ricoh583->irq_en_add[1] = RICOH583_INT_EN_SYS2;
	ricoh583->irq_en_add[2] = RICOH583_INT_EN_DCDC;
	ricoh583->irq_en_add[3] = RICOH583_INT_EN_RTC;
	ricoh583->irq_en_add[4] = RICOH583_INT_EN_ADC1;
	ricoh583->irq_en_add[5] = RICOH583_INT_EN_ADC2;
	ricoh583->irq_en_add[6] = RICOH583_INT_EN_ADC3;
	ricoh583->irq_en_add[7] = RICOH583_INT_EN_GPIO;

	/* Interrupt status monitor register */
	ricoh583->irq_mon_add[0] = RICOH583_INT_MON_SYS1;
	ricoh583->irq_mon_add[1] = RICOH583_INT_MON_SYS2;
	ricoh583->irq_mon_add[2] = RICOH583_INT_MON_DCDC;
	ricoh583->irq_mon_add[3] = RICOH583_INT_MON_RTC;
	ricoh583->irq_mon_add[4] = RICOH583_INT_IR_ADCL;
	ricoh583->irq_mon_add[5] = RICOH583_INT_IR_ADCH;
	ricoh583->irq_mon_add[6] = RICOH583_INT_IR_ADCEND;
	ricoh583->irq_mon_add[7] = RICOH583_INT_IR_GPIOF;
	ricoh583->irq_mon_add[8] = RICOH583_INT_IR_GPIOR;

	/* Interrupt status clear register */
	ricoh583->irq_clr_add[0] = RICOH583_INT_IR_SYS1;
	ricoh583->irq_clr_add[1] = RICOH583_INT_IR_SYS2;
	ricoh583->irq_clr_add[2] = RICOH583_INT_IR_DCDC;
	ricoh583->irq_clr_add[3] = RICOH583_INT_IR_RTC;
	ricoh583->irq_clr_add[4] = RICOH583_INT_IR_ADCL;
	ricoh583->irq_clr_add[5] = RICOH583_INT_IR_ADCH;
	ricoh583->irq_clr_add[6] = RICOH583_INT_IR_ADCEND;
	ricoh583->irq_clr_add[7] = RICOH583_INT_IR_GPIOF;
	ricoh583->irq_clr_add[8] = RICOH583_INT_IR_GPIOR;

	ricoh583->main_int_type[0] = SYS_INT;
	ricoh583->main_int_type[1] = SYS_INT;
	ricoh583->main_int_type[2] = DCDC_INT;
	ricoh583->main_int_type[3] = RTC_INT;
	ricoh583->main_int_type[4] = ADC_INT;
	ricoh583->main_int_type[5] = ADC_INT;
	ricoh583->main_int_type[6] = ADC_INT;
	ricoh583->main_int_type[7] = GPIO_INT;
	ricoh583->main_int_type[8] = GPIO_INT;

	/* Initailize all int register to 0 */
	for (i = 0; i < MAX_INTERRUPT_MASKS; i++)  {
		ret = __ricoh583_write(ricoh583->client,
				ricoh583->irq_en_add[i],
				ricoh583->irq_en_reg[i]);
		if (ret < 0){
			dev_err(ricoh583->dev, "Error in writing reg 0x%02x "
					"error: %d\n", ricoh583->irq_en_add[i], ret);
			return -1;
		}
	}

	for (i = 0; i < 2; i++)  {
		ret = __ricoh583_write(ricoh583->client,
				ricoh583->gpedge_add[i],
				ricoh583->gpedge_reg[i]);
		if (ret < 0){
			dev_err(ricoh583->dev, "Error in writing reg 0x%02x "
					"error: %d\n", ricoh583->gpedge_add[i], ret);
			return -1;
		}
	}

	ret = __ricoh583_write(ricoh583->client, RICOH583_INTC_INTEN, 0x0);
	if (ret < 0){
		dev_err(ricoh583->dev, "Error in writing reg 0x%02x "
				"error: %d\n", RICOH583_INTC_INTEN, ret);
		return -1;
	}

	/* Clear all interrupts in case they woke up active. */
	for (i = 0; i < 9; i++)  {
		ret = __ricoh583_write(ricoh583->client,
				ricoh583->irq_clr_add[i], 0);
		if (ret < 0){
			dev_err(ricoh583->dev, "Error in writing reg 0x%02x "
					"error: %d\n", ricoh583->irq_clr_add[i], ret);
			return -1;
		}
	}

	ricoh583->irq_base = irq_base;
	ricoh583->irq_chip.name = "ricoh583";
	ricoh583->irq_chip.irq_mask = ricoh583_irq_mask;
	ricoh583->irq_chip.irq_unmask = ricoh583_irq_unmask;
	ricoh583->irq_chip.irq_bus_lock = ricoh583_irq_lock;
	ricoh583->irq_chip.irq_bus_sync_unlock = ricoh583_irq_sync_unlock;
	ricoh583->irq_chip.irq_set_type = ricoh583_irq_set_type;

	for (i = 0; i < RICOH583_NR_IRQS; i++) {
		int __irq = i + ricoh583->irq_base;
		irq_set_chip_data(__irq, ricoh583);
		irq_set_chip_and_handler(__irq, &ricoh583->irq_chip,
				handle_simple_irq);
		irq_set_nested_thread(__irq, 1);
#ifdef CONFIG_ARM
		set_irq_flags(__irq, IRQF_VALID);
#endif
	}

	ret = request_irq(irq, ricoh583_irq,  IRQF_TRIGGER_LOW,
			"ricoh583", ricoh583);
	if (ret < 0){
		dev_err(ricoh583->dev, "Error in registering interrupt "
				"error: %d\n", ret);
		return -1;
	}
	device_init_wakeup(ricoh583->dev, 1);
	enable_irq_wake(irq);

	return ret;
}

static int ricoh583_remove_subdev(struct device *dev, void *unused)
{
	platform_device_unregister(to_platform_device(dev));
	return 0;
}

static int ricoh583_remove_subdevs(struct ricoh583 *ricoh583)
{
	return device_for_each_child(ricoh583->dev, NULL,
			ricoh583_remove_subdev);
}

static int __devinit ricoh583_add_subdevs(struct ricoh583 *ricoh583,
		struct ricoh583_platform_data *pdata)
{
	struct ricoh583_subdev_info *subdev;
	struct platform_device *pdev;
	int i, ret = 0;

	for (i = 0; i < pdata->num_subdevs; i++) {
		subdev = &pdata->subdevs[i];

		pdev = platform_device_alloc(subdev->name, subdev->id);
		dev_info(ricoh583->dev, "%s: %s\n", __func__, subdev->name);

		pdev->dev.parent = ricoh583->dev;
		pdev->dev.platform_data = subdev->platform_data;

		ret = platform_device_add(pdev);
		if (ret)
			goto failed;
	}
	return 0;

failed:
	ricoh583_remove_subdevs(ricoh583);
	return ret;
}

#ifdef CONFIG_DEBUG_FS
#include <linux/debugfs.h>
#include <linux/seq_file.h>
static void print_regs(const char *header, struct seq_file *s,
		struct i2c_client *client, int start_offset,
		int end_offset)
{
	uint8_t reg_val;
	int i;
	int ret;

	seq_printf(s, "%s\n", header);
	for (i = start_offset; i <= end_offset; ++i) {
		ret = __ricoh583_read(client, i, &reg_val);
		if (ret >= 0)
			seq_printf(s, "Reg 0x%02x Value 0x%02x\n", i, reg_val);
	}
	seq_printf(s, "------------------\n");
}

static int dbg_tps_show(struct seq_file *s, void *unused)
{
	struct ricoh583 *tps = s->private;
	struct i2c_client *client = tps->client;

	seq_printf(s, "RICOH583 Registers\n");
	seq_printf(s, "------------------\n");

	print_regs("System Regs",	       s, client, 0x0, 0xF);
	print_regs("Power Control Regs",	s, client, 0x10, 0x2B);
	print_regs("DCDC1 Regs",		s, client, 0x30, 0x43);
	print_regs("DCDC1 Regs",		s, client, 0x60, 0x63);
	print_regs("LDO   Regs",		s, client, 0x50, 0x5F);
	print_regs("LDO   Regs",		s, client, 0x64, 0x6D);
	print_regs("ADC  Regs",		 s, client, 0x70, 0x72);
	print_regs("ADC  Regs",		 s, client, 0x74, 0x8B);
	print_regs("ADC  Regs",		 s, client, 0x90, 0x96);
	print_regs("GPIO Regs",		 s, client, 0xA0, 0xAC);
	print_regs("INTC Regs",		 s, client, 0xAD, 0xAF);
	print_regs("RTC  Regs",		 s, client, 0xE0, 0xEE);
	print_regs("RTC  Regs",		 s, client, 0xF0, 0xF4);
	return 0;
}

static int dbg_tps_open(struct inode *inode, struct file *file)
{
	return single_open(file, dbg_tps_show, inode->i_private);
}

static const struct file_operations debug_fops = {
	.open	   = dbg_tps_open,
	.read	   = seq_read,
	.llseek	 = seq_lseek,
	.release	= single_release,
};
static void __init ricoh583_debuginit(struct ricoh583 *tps)
{
	(void)debugfs_create_file("ricoh583", S_IRUGO, NULL,
			tps, &debug_fops);
}
#else
static void __init ricoh583_debuginit(struct ricoh583 *tpsi)
{
	return;
}
#endif

#ifdef RICOH583_DEBUG_REG
static u8 dbg_addr = 0;

static ssize_t show_addr(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "addr:0x%02X\n", dbg_addr);
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

	ret = ricoh583_read(dev, dbg_addr, &val);

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

	ret = ricoh583_write(dev, dbg_addr, val);

	if(ret < 0){
		dev_err(&client->dev, "store_val: reg_write failed\n");
		return ret;
	}

	return n;
}

static DEVICE_ATTR(addr, S_IRUGO | S_IWUGO, show_addr, store_addr);
static DEVICE_ATTR(val, S_IRUGO | S_IWUGO, show_val, store_val);

static struct attribute *ricoh583_attributes[] = {
	&dev_attr_addr.attr,
	&dev_attr_val.attr,
	NULL,
};

static struct attribute_group ricoh583_attr_group = {
	.attrs = ricoh583_attributes,
};
#endif

int ricoh583_suspend_system(void)
{
	int ret;

	ret = ricoh583_clr_bits( &ricoh583_i2c_client->dev, RICOH583_GPIO_IOOUT , 1 << 1);
	if (ret < 0){
		dev_err(&ricoh583_i2c_client->dev, "Error in write reg 0x%d: %d\n",
				RICOH583_GPIO_IOOUT, ret);
	}
	return ret;
}
EXPORT_SYMBOL_GPL(ricoh583_suspend_system);

static int ricoh583_init(struct i2c_client *i2c)
{
	int ret = 0;
	//uint8_t val = 0;
	int i;
	int rw_data[][2] = {
#if 0
		{0x28, 0x10},
		{0x23, 0xb9},
		{0x26, 0xb9},
		{0x27, 0x90},
		{0x24, 0xbb},
		{0x29, 0xbd},
		{0x22, 0x0b},
		{0x21, 0xf0},
#endif
		{0x5E, 0x00},//set LDOSW off
		{0x15, 0x0B},//ONKEY timer: 6s. press time > 6s, power-off
		{0x15, 0x0A},
		{0x10, 0x37},
	};

	for (i = 0; i < sizeof(rw_data) / (2 * sizeof(int)); ++i){
		ret |= __ricoh583_write(i2c, rw_data[i][0], rw_data[i][1]);
		//ret |= __ricoh583_read(i2c, rw_data[i][0], &val);
		//dev_info(&i2c->dev, "reg: 0x%x(0x%x) == 0x%x\n", rw_data[i][0], rw_data[i][1], val);
	}

	return ret;
}

static int ricoh583_i2c_probe(struct i2c_client *i2c,
		const struct i2c_device_id *id)
{
	struct ricoh583 *ricoh583;
	struct ricoh583_platform_data *pdata = i2c->dev.platform_data;
	int ret;

	ricoh583 = kzalloc(sizeof(struct ricoh583), GFP_KERNEL);
	if (ricoh583 == NULL)
		return -ENOMEM;

	ricoh583->client = i2c;
	ricoh583->dev = &i2c->dev;
	i2c_set_clientdata(i2c, ricoh583);
	ricoh583_i2c_client = i2c;

	mutex_init(&ricoh583->io_lock);

	ret = ricoh583_ext_power_init(ricoh583, pdata);
	if (ret < 0){
		goto out;
	}

	ret = ricoh583_add_subdevs(ricoh583, pdata);
	if (ret) {
		dev_err(&i2c->dev, "add devices failed: %d\n", ret);
		goto out;
	}

	INIT_WORK(&ricoh583->work, ricoh583_irq_work);
	ricoh583->queue = create_workqueue("ricoh583_irq_queue");
	if (!ricoh583->queue){
			goto err_subdevs;
	}
	if (i2c->irq) {
		ret = ricoh583_irq_init(ricoh583, i2c->irq, pdata->irq_base);
		if (ret) {
			dev_err(&i2c->dev, "IRQ init failed: %d\n", ret);
			goto err_queue;
		}
	}

	ret = ricoh583_gpio_init(ricoh583, pdata);
	if(ret){
		dev_err(&i2c->dev, "GPIO init failed: %d\n", ret);
		goto err_irq;
	}
	ricoh583_debuginit(ricoh583);

	ret = ricoh583_init(i2c);
	if(ret){
		dev_err(&i2c->dev, "RICOH init failed: %d\n", ret);
		goto err_gpio;
	}
#ifdef RICOH583_DEBUG_REG
	sysfs_create_group(&i2c->dev.kobj, &ricoh583_attr_group);
#endif

	dev_info(&i2c->dev, "%s is OK!\n", __func__);
	return 0;

err_gpio:
	gpiochip_remove(&ricoh583->gpio);
err_irq:
	if (i2c->irq)
		free_irq(i2c->irq, ricoh583);
err_queue:
	destroy_workqueue(ricoh583->queue);
err_subdevs:
	ricoh583_remove_subdevs(ricoh583);
out:
	kfree(ricoh583);
	return ret;
}

static int  __devexit ricoh583_i2c_remove(struct i2c_client *i2c)
{
	struct ricoh583 *ricoh583 = i2c_get_clientdata(i2c);

	if (i2c->irq)
		free_irq(i2c->irq, ricoh583);

#ifdef RICOH583_DEBUG_REG
	sysfs_remove_group(&i2c->dev.kobj, &ricoh583_attr_group);
#endif
	gpiochip_remove(&ricoh583->gpio);
	ricoh583_remove_subdevs(ricoh583);
	destroy_workqueue(ricoh583->queue);
	kfree(ricoh583);
	return 0;
}


#ifdef CONFIG_PM
static int ricoh583_i2c_suspend(struct i2c_client *i2c, pm_message_t state)
{
	int ret;

#if 0
	//set DCDC1 1.1V
	ret = __ricoh583_write(i2c, 0x35, 0x1c);
	if (ret < 0){
		dev_err(&i2c->dev, "Error in writing reg %d error: "
				"%d\n", 0x35, ret);
	}
	// set LDOSW on for DVS
	ret = __ricoh583_write(i2c, RICOH_SWCTL_REG, 0x1);
	if (ret < 0){
		dev_err(&i2c->dev, "Error in writing reg %d error: "
				"%d\n", RICOH_SWCTL_REG, ret);
	}
#endif

	if (i2c->irq)
		disable_irq(i2c->irq);
	return 0;
}


static int ricoh583_i2c_resume(struct i2c_client *i2c)
{
	int ret;

#if 0
	// set LDOSW off for DVS
	ret = __ricoh583_write(i2c, RICOH_SWCTL_REG, 0x0);
	if (ret < 0){
		dev_err(&i2c->dev, "Error in writing reg %d error: "
				"%d\n", RICOH_SWCTL_REG, ret);
	}
#endif

	if (i2c->irq)
		enable_irq(i2c->irq);
	return 0;
}

#endif

static const struct i2c_device_id ricoh583_i2c_id[] = {
	{"ricoh583", 0},
	{}
};

MODULE_DEVICE_TABLE(i2c, ricoh583_i2c_id);

static struct i2c_driver ricoh583_i2c_driver = {
	.driver = {
		.name = "ricoh583",
		.owner = THIS_MODULE,
	},
	.probe = ricoh583_i2c_probe,
	.remove = __devexit_p(ricoh583_i2c_remove),
#ifdef CONFIG_PM
	.suspend = ricoh583_i2c_suspend,
	.resume = ricoh583_i2c_resume,
#endif
	.id_table = ricoh583_i2c_id,
};


static int __init ricoh583_i2c_init(void)
{
	int ret = -ENODEV;
	ret = i2c_add_driver(&ricoh583_i2c_driver);
	if (ret != 0)
		pr_err("Failed to register I2C driver: %d\n", ret);

	return ret;
}

subsys_initcall(ricoh583_i2c_init);

static void __exit ricoh583_i2c_exit(void)
{
	i2c_del_driver(&ricoh583_i2c_driver);
}

module_exit(ricoh583_i2c_exit);

MODULE_DESCRIPTION("RICOH583 multi-function core driver");
MODULE_LICENSE("GPL");
