/*
 * drivers/regulator/tps80032-regulator.c
 *
 * Regulator driver for TPS80032 power management chip.
 *
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
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/err.h>
#include <linux/platform_device.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/machine.h>
#include <linux/mfd/tps80032.h>

struct tps80032_regulator {
	int		id;
	int		reg_id;
	int		vout_reg_id;
	/* Regulator register address.*/
	u8		reg_en_reg;
	u8		en_bit;
	u8		vout_reg;
	u8		vout_mask;
	u8		vout_reg_cache;
	u8		sleep_reg;

	/* chip constraints on regulator behavior */
	int			min_uV;
	int			max_uV;
	int			step_uV;

	/* regulator specific turn-on delay */
	u16			delay;

	/* used by regulator core */
	struct regulator_desc	desc;

	/* Device */
	struct device		*dev;
};

#define SMPS_MAX_CONT_INDEX		0x39
#define SMPS_MAX_CONT_VOUT		0x1300
#define LDO_MAX_CONT_INDEX		0x18
#define LDO_MAX_CONT_VOUT		0x3300
static int smps_discont_vout[] = {
	1350,
	1500,
	1800,
	1900,
	2100,
};

static int tps80032_regulator_enable_time(struct regulator_dev *rdev)
{
	struct tps80032_regulator *ri = rdev_get_drvdata(rdev);

	return ri->delay;
}

static int tps80032_reg_is_enabled(struct regulator_dev *rdev)
{
	struct tps80032_regulator *ri = rdev_get_drvdata(rdev);
	uint8_t control;
	int ret;

	ret = tps80032_read(ri->reg_id, ri->reg_en_reg, &control);
	if (ret < 0) {
		dev_err(&rdev->dev, "Error in reading the control register\n");
		return ret;
	}
	return (((control >> ri->en_bit) & 0x3) == 1);
}

static int tps80032_reg_enable(struct regulator_dev *rdev)
{
	struct tps80032_regulator *ri = rdev_get_drvdata(rdev);
	int ret;

	ret = tps80032_write(ri->reg_id, ri->reg_en_reg, (1 << ri->en_bit));
	if (ret < 0) {
		dev_err(&rdev->dev, "Error in updating the STATE register\n");
		return ret;
	}
	udelay(ri->delay);
	return ret;
}

static int tps80032_reg_disable(struct regulator_dev *rdev)
{
	struct tps80032_regulator *ri = rdev_get_drvdata(rdev);
	int ret;

	ret = tps80032_write(ri->reg_id, ri->reg_en_reg, 0);
	if (ret < 0)
		dev_err(&rdev->dev, "Error in updating the STATE register\n");

	return ret;
}

static int tps80032_list_voltage(struct regulator_dev *rdev, unsigned index)
{
	struct tps80032_regulator *ri = rdev_get_drvdata(rdev);

	if (index > ri->vout_mask){
		return -1;
	}
	if (ri->id <= TPS80032_ID_SMPS5 && index > SMPS_MAX_CONT_INDEX){
		return smps_discont_vout[index - SMPS_MAX_CONT_INDEX - 1];
	}
	if (ri->id > TPS80032_ID_SMPS5){
		if (index == ri->vout_mask){
			return 2750000;
		}else if (index > LDO_MAX_CONT_INDEX){
			return -1;
		}
	}
	return ri->min_uV + (ri->step_uV * index);
}

static int __tps80032_set_voltage(struct tps80032_regulator *ri, 
		int min_uV, int max_uV, unsigned *selector)
{
	int vsel;
	int ret;
	int i;
	uint8_t vout_val;

	if ((min_uV < ri->min_uV) || (max_uV > ri->max_uV))
		return -EDOM;

	if (ri->id <= TPS80032_ID_SMPS5 && min_uV >= smps_discont_vout[0]){
		for(i = 0; i < ARRAY_SIZE(smps_discont_vout); ++i){
			if (min_uV <= smps_discont_vout[i]){
				vsel = SMPS_MAX_CONT_INDEX + i;
				goto done;
			}
		}
	}else if(ri->id > TPS80032_ID_SMPS5 && min_uV == 2750000){
		vsel = ri->vout_mask;
		goto done;
	}
	vsel = (min_uV - ri->min_uV + ri->step_uV)/ri->step_uV;

done:
	if (selector)
		*selector = vsel;

	vout_val = (ri->vout_reg_cache & ~ri->vout_mask) |
		(vsel & ri->vout_mask);
	ret = tps80032_write(ri->vout_reg_id, ri->vout_reg, vout_val);
	if (ret < 0)
		dev_err(ri->dev, "Error in writing the Voltage register\n");
	else
		ri->vout_reg_cache = vout_val;

	return ret;
}

static int tps80032_set_voltage(struct regulator_dev *rdev,
		int min_uV, int max_uV, unsigned *selector)
{
	struct tps80032_regulator *ri = rdev_get_drvdata(rdev);

	return __tps80032_set_voltage(ri, min_uV, max_uV, selector);
}

static int tps80032_get_voltage(struct regulator_dev *rdev)
{
	struct tps80032_regulator *ri = rdev_get_drvdata(rdev);
	uint8_t vsel;

	vsel = ri->vout_reg_cache & ri->vout_mask;
	return tps80032_list_voltage(rdev, vsel);
}

static struct regulator_ops tps80032_ops = {
	.list_voltage	= tps80032_list_voltage,
	.set_voltage	= tps80032_set_voltage,
	.get_voltage	= tps80032_get_voltage,
	.enable		= tps80032_reg_enable,
	.disable	= tps80032_reg_disable,
	.is_enabled	= tps80032_reg_is_enabled,
	.enable_time	= tps80032_regulator_enable_time,
};

#define TPS80032_REG(_id, _reg_id, _vout_reg_id,_en_reg, _en_bit,  _vout_reg, \
		_vout_mask, _ds_reg, _min_mv, _max_mv, _step_uV, _ops, _delay)		\
{								\
	.id		= TPS80032_ID_##_id,			\
	.reg_id = _reg_id,				\
	.vout_reg_id = _vout_reg_id,				\
	.reg_en_reg	= _en_reg,				\
	.en_bit		= _en_bit,				\
	.vout_reg	= _vout_reg,				\
	.vout_mask	= _vout_mask,				\
	.sleep_reg	= _ds_reg,				\
	.min_uV		= _min_mv * 1000,			\
	.max_uV		= _max_mv * 1000,			\
	.step_uV	= _step_uV,				\
	.delay		= _delay,				\
	.desc = {						\
		.name = "tps80032_"#_id,			\
		.id = TPS80032_ID_##_id,			\
		.ops = &_ops,					\
		.type = REGULATOR_VOLTAGE,			\
		.owner = THIS_MODULE,				\
	},							\
}

//_id, _reg_id, _vout_reg_id, _en_reg, _en_bit, _vout_reg, _vout_mask, _ds_reg, 
//_min_mv, _max_mv, _step_uV, _ops, _delay)

static struct tps80032_regulator tps80032_regulator[] = {
	TPS80032_REG(SMPS1, 1, 0, 0x54, 0, 0x56, 0x3F, 0x53,
			600, 2100, 12500, tps80032_ops, 500),
	TPS80032_REG(SMPS2, 1, 0, 0x5A, 0, 0x5C, 0x3F, 0x59,
			600, 2100, 12500, tps80032_ops, 500),
	TPS80032_REG(SMPS3, 1, 1, 0x66, 0, 0x68, 0x3F, 0x65,
			600, 2100, 12500, tps80032_ops, 500),
	TPS80032_REG(SMPS4, 1, 1, 0x42, 0, 0x44, 0x3F, 0x41,
			600, 2100, 12500, tps80032_ops, 500),
	TPS80032_REG(SMPS5, 1, 0, 0x48, 0, 0x4A, 0x3F, 0x47,
			600, 2100, 12500, tps80032_ops, 500),
	TPS80032_REG(LDOLN, 1, 1, 0x96, 0, 0x97, 0x1F, 0x95,
			1000, 3300, 100000, tps80032_ops, 500),
	TPS80032_REG(LDO1, 1, 1, 0x9E, 0, 0x9F, 0x1F, 0x9D,
			1000, 3300, 100000, tps80032_ops, 500),
	TPS80032_REG(LDO2, 1, 1, 0x86, 0, 0x87, 0x1F, 0x85,
			1000, 3300, 100000, tps80032_ops, 500),
	TPS80032_REG(LDO3, 1, 1, 0x8E, 0, 0x8F, 0x1F, 0x8D,
			1000, 3300, 100000, tps80032_ops, 500),
	TPS80032_REG(LDO4, 1, 1, 0x8A, 0, 0x8B, 0x1F, 0x89,
			1000, 3300, 100000, tps80032_ops, 500),
	TPS80032_REG(LDO5, 1, 1, 0x9A, 0, 0x9B, 0x1F, 0x99,
			1000, 3300, 100000, tps80032_ops, 500),
	TPS80032_REG(LDO6, 1, 1, 0x92, 0, 0x93, 0x1F, 0x91,
			1000, 3300, 100000, tps80032_ops, 500),
	TPS80032_REG(LDO7, 1, 1, 0xA6, 0, 0xA7, 0x1F, 0xA5,
			1000, 3300, 100000, tps80032_ops, 500),
	TPS80032_REG(LDOUSB, 1, 1, 0xA2, 0, 0xA3, 0x1F, 0xA1,
			1000, 3300, 100000, tps80032_ops, 500),
};
static inline struct tps80032_regulator *find_regulator_info(int id)
{
	struct tps80032_regulator *ri;
	int i;

	for (i = 0; i < ARRAY_SIZE(tps80032_regulator); i++) {
		ri = &tps80032_regulator[i];
		if (ri->desc.id == id)
			return ri;
	}
	return NULL;
}

static int tps80032_regulator_preinit(struct tps80032_regulator *ri,
		struct tps80032_regulator_platform_data *tps80032_pdata)
{
	int ret = 0;

	if (tps80032_pdata->sleep_apply){
		tps80032_reg_update(ri->reg_id, ri->sleep_reg, 
				tps80032_pdata->sleep_enable << 2, 0x0C);
	}
	if (!tps80032_pdata->init_apply)
		return 0;

	if (tps80032_pdata->init_uV >= 0) {
		ret = __tps80032_set_voltage(ri,
				tps80032_pdata->init_uV,
				tps80032_pdata->init_uV, 0);
		if (ret < 0) {
			dev_err(ri->dev, "Not able to initialize voltage %d "
					"for rail %d err %d\n", tps80032_pdata->init_uV,
					ri->desc.id, ret);
			return ret;
		}
	}

	if (tps80032_pdata->init_enable)
		ret = tps80032_set_bits(ri->reg_id, ri->reg_en_reg,
				(1 << ri->en_bit));
	else
		ret = tps80032_clr_bits(ri->reg_id, ri->reg_en_reg,
				(1 << ri->en_bit));
	if (ret < 0)
		dev_err(ri->dev, "Not able to %s rail %d err %d\n",
				(tps80032_pdata->init_enable) ? "enable" : "disable",
				ri->desc.id, ret);

	return ret;
}

static inline int tps80032_cache_regulator_register(struct tps80032_regulator *ri)
{
	ri->vout_reg_cache = 0;
	return tps80032_read(ri->vout_reg_id, ri->vout_reg, &ri->vout_reg_cache);
}

static int __devinit tps80032_regulator_probe(struct platform_device *pdev)
{
	struct tps80032_regulator *ri = NULL;
	struct regulator_dev *rdev;
	struct tps80032_regulator_platform_data *tps_pdata;
	int id = pdev->id;
	int err;

	dev_dbg(&pdev->dev, "Probing reulator %d\n", id);

	ri = find_regulator_info(id);
	if (ri == NULL) {
		dev_err(&pdev->dev, "invalid regulator ID specified\n");
		return -EINVAL;
	}
	tps_pdata = pdev->dev.platform_data;
	ri->dev = &pdev->dev;

	err = tps80032_cache_regulator_register(ri);
	if (err) {
		dev_err(&pdev->dev, "Fail in caching register\n");
		return err;
	}

	err = tps80032_regulator_preinit(ri, tps_pdata);
	if (err) {
		dev_err(&pdev->dev, "Fail in pre-initialisation\n");
		return err;
	}
	rdev = regulator_register(&ri->desc, &pdev->dev,
			&tps_pdata->regulator, ri);
	if (IS_ERR_OR_NULL(rdev)) {
		dev_err(&pdev->dev, "failed to register regulator %s\n",
				ri->desc.name);
		return PTR_ERR(rdev);
	}

	platform_set_drvdata(pdev, rdev);
	return 0;
}

static int __devexit tps80032_regulator_remove(struct platform_device *pdev)
{
	struct regulator_dev *rdev = platform_get_drvdata(pdev);

	regulator_unregister(rdev);
	return 0;
}

static struct platform_driver tps80032_regulator_driver = {
	.driver	= {
		.name	= "tps80032-regulator",
		.owner	= THIS_MODULE,
	},
	.probe		= tps80032_regulator_probe,
	.remove		= __devexit_p(tps80032_regulator_remove),
};

static int __init tps80032_regulator_init(void)
{
	return platform_driver_register(&tps80032_regulator_driver);
}
subsys_initcall(tps80032_regulator_init);

static void __exit tps80032_regulator_exit(void)
{
	platform_driver_unregister(&tps80032_regulator_driver);
}
module_exit(tps80032_regulator_exit);

MODULE_AUTHOR("Wu Jianguo<jianguo.wu@nufront.com>");
MODULE_DESCRIPTION("TPS80032 regulator driver");
MODULE_ALIAS("platform:tps80032-regulator");
MODULE_LICENSE("GPL");
