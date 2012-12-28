/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 * 02110-1301, USA.
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/mfd/tps80032.h>
#include <linux/mutex.h>
#include <linux/timer.h>
#include <linux/pm.h>
#include <linux/slab.h>
#include <linux/pm_runtime.h>
#include <linux/workqueue.h>
#include <linux/gpio.h>

#define TPS80032_REG_STS_HW_CONDITIONS	1, 0X21

struct tps80032_pwrkey {
	struct device       * dev;
	struct input_dev    *pwr;
	struct workqueue_struct * workqueue;
	struct work_struct      work;
	unsigned long           delay;
	int         key_irq;
	bool        pressed_first;
	struct      tps80032_pwrkey_platform_data *pdata;
	struct mutex lock;
};
struct tps80032_pwrkey *g_pwrkey;

static int tps80032_pwrkey_stat(void)
{
	int ret;
	uint8_t val;

	ret = tps80032_read(TPS80032_REG_STS_HW_CONDITIONS, &val);
	if (ret < 0){
		dev_err(g_pwrkey->dev, "%s error:%d\n", __func__, ret);
		return ret;
	}
	return !(val & 0x01);
}

static void tps80032_irq_work(struct work_struct * work)
{
	int key_stat;

	mutex_lock(&g_pwrkey->lock);
	key_stat = tps80032_pwrkey_stat();
	if (key_stat < 0){
		return;
	}
	dev_info(g_pwrkey->dev, "pwrkey is %s!\n", key_stat ? "pressed": "released");
	if(key_stat){
		if (!g_pwrkey->pressed_first){
			g_pwrkey->pressed_first = true;
			input_report_key(g_pwrkey->pwr, KEY_POWER, 1);
			input_sync(g_pwrkey->pwr);
		}
	}else{
		if (g_pwrkey->pressed_first){
			g_pwrkey->pressed_first = false;
			input_report_key(g_pwrkey->pwr, KEY_POWER, 0);
			input_sync(g_pwrkey->pwr);
		}
	}
	mutex_unlock(&g_pwrkey->lock);
}

static irqreturn_t pwrkey_irq(int irq, void *_pwrkey)
{
	queue_work(g_pwrkey->workqueue, &g_pwrkey->work);

	return IRQ_HANDLED;
}

static int __devinit tps80032_pwrkey_probe(struct platform_device *pdev)
{
	struct input_dev *pwr;
	int key_irq;
	int err;
	struct tps80032_pwrkey *pwrkey;
	struct tps80032_pwrkey_platform_data *pdata = pdev->dev.platform_data;
	int key_stat;

	if (!pdata) {
		dev_err(&pdev->dev, "power key platform data not supplied\n");
		return -EINVAL;
	}

	key_irq = pdata->irq;

	pwrkey = kzalloc(sizeof(*pwrkey), GFP_KERNEL);
	if (!pwrkey)
		return -ENOMEM;

	pwrkey->dev = &pdev->dev;
	pwrkey->pdata   = pdata;
	pwrkey->pressed_first = false;
	g_pwrkey = pwrkey;

	pwr = input_allocate_device();
	if (!pwr) {
		dev_dbg(&pdev->dev, "Can't allocate power button\n");
		err = -ENOMEM;
		goto free_pwrkey;
	}

	input_set_capability(pwr, EV_KEY, KEY_POWER);

	pwr->name = "tps80032_pwrkey";
	pwr->phys = "tps80032_pwrkey/input0";
	pwr->dev.parent = &pdev->dev;

	mutex_init(&pwrkey->lock);

	err = input_register_device(pwr);
	if (err) {
		dev_dbg(&pdev->dev, "Can't register power key: %d\n", err);
		goto free_input_dev;
	}

	pwrkey->key_irq = key_irq;
	pwrkey->pwr = pwr;

	platform_set_drvdata(pdev, pwrkey);

	/* Check if power-key is pressed at boot up */
	key_stat = tps80032_pwrkey_stat();
	if (key_stat < 0) {
		dev_err(&pdev->dev, "Key-press status at boot failed rc=%d\n",
				key_stat);
		goto unreg_input_dev;
	}
	if (key_stat) {
		input_report_key(pwrkey->pwr, KEY_POWER, 1);
		dev_info(&pdev->dev, "the power key is pressed!\n");
		input_sync(pwrkey->pwr);
		pwrkey->pressed_first = true;
	}

	err = request_threaded_irq(key_irq, NULL, pwrkey_irq,
			IRQF_ONESHOT, "tps80032_pwrkey", pwrkey);
	if (err < 0) {
		dev_err(&pdev->dev, "Can't get %d IRQ for pwrkey: %d\n",
				key_irq, err);
		goto unreg_input_dev;
	}
	pwrkey->workqueue = create_singlethread_workqueue("tps80032_pwrkey");
	INIT_WORK(&pwrkey->work, tps80032_irq_work);

	printk("%s is OK!\n", __func__);

	return 0;

unreg_input_dev:
	input_unregister_device(pwr);
	pwr = NULL;
free_input_dev:
	input_free_device(pwr);
free_pwrkey:
	kfree(pwrkey);
	return err;
}

static int __devexit tps80032_pwrkey_remove(struct platform_device *pdev)
{
	struct tps80032_pwrkey *pwrkey = platform_get_drvdata(pdev);

	flush_workqueue(pwrkey->workqueue);
	destroy_workqueue(pwrkey->workqueue);
	free_irq(pwrkey->key_irq, pwrkey);
	input_unregister_device(pwrkey->pwr);
	kfree(pwrkey);

	return 0;
}

static struct platform_driver tps80032_pwrkey_driver = {
	.probe		= tps80032_pwrkey_probe,
	.remove		= __devexit_p(tps80032_pwrkey_remove),
	.driver		= {
		.name	= "tps80032-pwrkey",
		.owner	= THIS_MODULE,
	},
};

static int __init tps80032_pwrkey_init(void)
{
	return platform_driver_register(&tps80032_pwrkey_driver);
}
module_init(tps80032_pwrkey_init);

static void __exit tps80032_pwrkey_exit(void)
{
	platform_driver_unregister(&tps80032_pwrkey_driver);
}
module_exit(tps80032_pwrkey_exit);

MODULE_AUTHOR("Wu Jianguo<jianguo.wu@nufront.com>");
MODULE_ALIAS("platform:tps80032-pwrkey");
MODULE_DESCRIPTION("tps80032 Power Key");
MODULE_LICENSE("GPL v2");
