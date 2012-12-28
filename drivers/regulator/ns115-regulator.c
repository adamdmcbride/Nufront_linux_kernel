/*
 * drivers/regulator/ns115-regulator.c
 *
 * Regulator driver for internal power switch of NS115 SOC.
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

#include <linux/module.h>
#include <linux/delay.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/err.h>
#include <asm/io.h>
#include <mach/hardware.h>
#include <linux/platform_device.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/ns115-regulator.h>

#define ns115_reg_rails(_name)	"NS115_REG_"#_name
#define ENABLE_DELAY	500//us
#define PRCM_BASE		0x05821000
#define NS115_PWR_ADDR	(PRCM_BASE + 0x8)

struct ns115_regulator {
	int id;
	int voltage;
	int delay;
	void __iomem *addr;
	struct device *dev;
	struct regulator_desc desc;
};

static int ns115_regulator_enable_time(struct regulator_dev *rdev)
{
	struct ns115_regulator *info = rdev_get_drvdata(rdev);

	return info->delay;
}

static int ns115_reg_is_enabled(struct regulator_dev *rdev)
{
	struct ns115_regulator *info = rdev_get_drvdata(rdev);
	u32 val;

	val = readl(info->addr);
	val = (val >> info->id) & 0x1;
	dev_dbg(info->dev, "%s val: 0x%x\n", __func__, val);

	return val;
}

static int ns115_pwr_set_value(struct ns115_regulator *info, int value)
{
	u32 val;

	val = readl(info->addr);
	val &= ~(1 << info->id);
	val |= (!!value) << info->id;
	writel(val, info->addr);

	return 0;
}

static int ns115_reg_enable(struct regulator_dev *rdev)
{
	struct ns115_regulator *info = rdev_get_drvdata(rdev);
	int ret;

	dev_dbg(info->dev, "%s\n", __func__);
	ret = ns115_pwr_set_value(info, 1);

	return ret;
}

static int ns115_reg_disable(struct regulator_dev *rdev)
{
	struct ns115_regulator *info = rdev_get_drvdata(rdev);

	dev_dbg(info->dev, "%s\n", __func__);
	return ns115_pwr_set_value(info, 0);
}

static int ns115_list_voltage(struct regulator_dev *rdev, unsigned index)
{
	struct ns115_regulator *info = rdev_get_drvdata(rdev);

	return info->voltage;
}

static int ns115_get_voltage(struct regulator_dev *rdev)
{
	struct ns115_regulator *info = rdev_get_drvdata(rdev);

	return info->voltage;
}

static struct regulator_ops ns115_pwr_ops = {
	.list_voltage	= ns115_list_voltage,
	.get_voltage	= ns115_get_voltage,
	.enable		= ns115_reg_enable,
	.disable	= ns115_reg_disable,
	.is_enabled	= ns115_reg_is_enabled,
	.enable_time	= ns115_regulator_enable_time,
};

#define NS115_PWR_REG(_id, _voltage)		\
{								\
	.voltage = _voltage * 1000,			\
	.delay = ENABLE_DELAY,				\
	.id		= NS115_PWR_##_id,			\
	.desc = {						\
		.name = ns115_reg_rails(_id),			\
		.id = NS115_PWR_##_id,			\
		.n_voltages = 0,				\
		.ops = &ns115_pwr_ops,					\
		.type = REGULATOR_VOLTAGE,			\
		.owner = THIS_MODULE,				\
	},							\
}

static struct ns115_regulator ns115_regulator[] = {
	NS115_PWR_REG(MALI_GP, 1100),
	NS115_PWR_REG(MALI_L2C, 1100),
	NS115_PWR_REG(MALI_PP0, 1100),
	NS115_PWR_REG(GC300, 1100),
	NS115_PWR_REG(VPU_G1, 1100),
	NS115_PWR_REG(VPU_H1, 1100),
	NS115_PWR_REG(ISP, 1100),
	NS115_PWR_REG(ZSP, 1100),
	NS115_PWR_REG(PLL0, 1100),
	NS115_PWR_REG(PLL1, 1100),
	NS115_PWR_REG(PLL2, 1100),
	NS115_PWR_REG(PLL3, 1100),
	NS115_PWR_REG(PLL4, 1100),
	NS115_PWR_REG(PLL5, 1100),
	NS115_PWR_REG(PLL6, 1100),
};

static inline struct ns115_regulator *find_regulator_info(int id)
{
	struct ns115_regulator *info;
	int i;

	for (i = 0; i < ARRAY_SIZE(ns115_regulator); i++) {
		info = &ns115_regulator[i];
		if (info->id == id)
			return info;
	}
	return NULL;
}

static int ns115_regulator_preinit(struct ns115_regulator *info,
		struct ns115_regulator_platform_data *ns115_pdata)
{
	int ret = 0;

	if (!ns115_pdata->init_apply){
		return 0;
	}

	if (ns115_pdata->init_enable){
		ret = ns115_pwr_set_value(info, 1);
	}else{
		ret = ns115_pwr_set_value(info, 0);
	}
	if (ret < 0){
		dev_err(info->dev, "Not able to %s rail %d err %d\n",
				(ns115_pdata->init_enable) ? "enable" : "disable",
				info->desc.id, ret);
	}

	return ret;
}

static int __devinit ns115_regulator_probe(struct platform_device *pdev)
{
	struct ns115_regulator *info = NULL;
	struct regulator_dev *rdev;
	struct ns115_regulator_platform_data *tps_pdata;
	int id = pdev->id;
	int err;

	dev_dbg(&pdev->dev, "Probing regulator %d\n", id);

	info = find_regulator_info(id);
	if (info == NULL) {
		dev_err(&pdev->dev, "invalid regulator ID:%d specified\n", id);
		return -EINVAL;
	}
	tps_pdata = pdev->dev.platform_data;
	info->dev = &pdev->dev;
	info->addr = __io_address(NS115_PWR_ADDR);

	err = ns115_regulator_preinit(info, tps_pdata);
	if (err) {
		dev_err(&pdev->dev, "Fail in pre-initialisation\n");
		return err;
	}
	rdev = regulator_register(&info->desc, &pdev->dev,
			&tps_pdata->regulator, info);
	if (IS_ERR_OR_NULL(rdev)) {
		dev_err(&pdev->dev, "failed to register regulator %s\n",
				info->desc.name);
		return PTR_ERR(rdev);
	}
	platform_set_drvdata(pdev, rdev);

	return 0;
}

static int __devexit ns115_regulator_remove(struct platform_device *pdev)
{
	struct regulator_dev *rdev = platform_get_drvdata(pdev);

	regulator_unregister(rdev);
	return 0;
}

static struct platform_driver ns115_regulator_driver = {
	.driver	= {
		.name	= "ns115-regulator",
		.owner	= THIS_MODULE,
	},
	.probe		= ns115_regulator_probe,
	.remove		= __devexit_p(ns115_regulator_remove),
};

static int __init ns115_regulator_init(void)
{
	return platform_driver_register(&ns115_regulator_driver);
}
device_initcall(ns115_regulator_init);

static void __exit ns115_regulator_exit(void)
{
	platform_driver_unregister(&ns115_regulator_driver);
}
module_exit(ns115_regulator_exit);

MODULE_AUTHOR("Wu Jianguo<jianguo.wu@nufront.com>");
MODULE_DESCRIPTION("NS115 regulator driver");
MODULE_ALIAS("platform:ns115-regulator");
MODULE_LICENSE("GPL");
