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
#include <linux/mfd/ricoh583.h>
#include <linux/mutex.h>
#include <linux/timer.h>
#include <linux/pm.h>
#include <linux/slab.h>
#include <linux/pm_runtime.h>
#include <linux/workqueue.h>
#include <linux/gpio.h>

#define RICOH583_ONKEY_TRIGGER_LEVEL  0
#define RICOH583_ONKEY_OFF_IRQ      0

struct ricoh583_pwrkey {
	struct device       * dev;
	struct input_dev    *pwr;
#if RICOH583_ONKEY_TRIGGER_LEVEL
	struct timer_list   timer;
#endif
	struct workqueue_struct * workqueue;
	struct work_struct      work;
	unsigned long           delay;
	int         key_irq;
	bool        pressed_first;
	struct      ricoh583_pwrkey_platform_data *pdata;
	struct mutex lock;
};
struct ricoh583_pwrkey *g_pwrkey;

//static int test_set = 0;
#if RICOH583_ONKEY_TRIGGER_LEVEL
void ricoh583_pwrkey_timer(unsigned long t)
{
	queue_work(g_pwrkey->workqueue, &g_pwrkey->work);
}
#endif
static void ricoh583_irq_work(struct work_struct * work)
{
	uint8_t val;

	mutex_lock(&g_pwrkey->lock);
	ricoh583_read(g_pwrkey->dev->parent, 0x1b, &val);
	dev_info(g_pwrkey->dev, "pwrkey is pressed?(0x%x): 0x%x\n", 0x1b, val);
	val &= 0x1;
	if(val){
#if (RICOH583_ONKEY_TRIGGER_LEVEL)
		g_pwrkey->timer.expires = jiffies + g_pwrkey->delay;
		add_timer(&g_pwrkey->timer);
#endif
		if (!g_pwrkey->pressed_first){
			g_pwrkey->pressed_first = true;
			input_report_key(g_pwrkey->pwr, KEY_POWER, 1);
			input_sync(g_pwrkey->pwr);
		}else{
			input_report_key(g_pwrkey->pwr, KEY_POWER, 0);
			input_report_key(g_pwrkey->pwr, KEY_POWER, 1);
			input_sync(g_pwrkey->pwr);
		}
	}else{
		if (g_pwrkey->pressed_first){
			g_pwrkey->pressed_first = false;
			input_report_key(g_pwrkey->pwr, KEY_POWER, 0);
			input_sync(g_pwrkey->pwr);
		}else{
			input_report_key(g_pwrkey->pwr, KEY_POWER, 1);
			input_report_key(g_pwrkey->pwr, KEY_POWER, 0);
			input_sync(g_pwrkey->pwr);
		}
	}
	mutex_unlock(&g_pwrkey->lock);
}

static irqreturn_t pwrkey_irq(int irq, void *_pwrkey)
{
#if (RICOH583_ONKEY_TRIGGER_LEVEL)
	g_pwrkey->timer.expires = jiffies + g_pwrkey->delay;
	add_timer(&g_pwrkey->timer);
#else
	queue_work(g_pwrkey->workqueue, &g_pwrkey->work);
#endif

	return IRQ_HANDLED;
}
#if RICOH583_ONKEY_OFF_IRQ
static irqreturn_t pwrkey_irq_off(int irq, void *_pwrkey)
{
	dev_warn(g_pwrkey->dev, "ONKEY is pressed long time!\n");

	return IRQ_HANDLED;
}
#endif
static int __devinit ricoh583_pwrkey_probe(struct platform_device *pdev)
{
	struct input_dev *pwr;
	int key_irq;
	int err;
	struct ricoh583_pwrkey *pwrkey;
	struct ricoh583_pwrkey_platform_data *pdata = pdev->dev.platform_data;
	uint8_t val;

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
	pwrkey->delay = HZ / 1000 * pdata->delay_ms;
	g_pwrkey = pwrkey;

	pwr = input_allocate_device();
	if (!pwr) {
		dev_dbg(&pdev->dev, "Can't allocate power button\n");
		err = -ENOMEM;
		goto free_pwrkey;
	}

	input_set_capability(pwr, EV_KEY, KEY_POWER);

	pwr->name = "ricoh583_pwrkey";
	pwr->phys = "ricoh583_pwrkey/input0";
	pwr->dev.parent = &pdev->dev;

#if RICOH583_ONKEY_TRIGGER_LEVEL
	init_timer(&pwrkey->timer);
	pwrkey->timer.function = ricoh583_pwrkey_timer;
#endif

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
	err = ricoh583_read(pwrkey->dev->parent, 0x1b, &val);
	if (err < 0) {
		dev_err(&pdev->dev, "Key-press status at boot failed rc=%d\n",
				err);
		goto unreg_input_dev;
	}
	val &= 0x1;
	if (val) {
		input_report_key(pwrkey->pwr, KEY_POWER, 1);
		dev_info(&pdev->dev, "the power key is pressed!\n");
		input_sync(pwrkey->pwr);
		pwrkey->pressed_first = true;
	}

#if !(RICOH583_ONKEY_TRIGGER_LEVEL)
	ricoh583_set_bits(pwrkey->dev->parent, 0x1c, 0x1); //trigger both edge
#endif
	err = request_threaded_irq(key_irq, NULL, pwrkey_irq,
			IRQF_ONESHOT, "ricoh583_pwrkey", pwrkey);
	if (err < 0) {
		dev_err(&pdev->dev, "Can't get %d IRQ for pwrkey: %d\n",
				key_irq, err);
		goto unreg_input_dev;
	}
#if RICOH583_ONKEY_OFF_IRQ
	err = request_threaded_irq(key_irq + RICOH583_IRQ_ONKEY_OFF, NULL, pwrkey_irq_off,
			IRQF_ONESHOT, "ricoh583_pwrkey_off", pwrkey);
	if (err < 0) {
		dev_err(&pdev->dev, "Can't get %d IRQ for pwrkey: %d\n",
				key_irq + RICOH583_IRQ_ONKEY_OFF, err);
		free_irq(key_irq, pwrkey);
		goto unreg_input_dev;
	}
#endif
	pwrkey->workqueue = create_singlethread_workqueue("ricoh583_pwrkey");
	INIT_WORK(&pwrkey->work, ricoh583_irq_work);

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

static int __devexit ricoh583_pwrkey_remove(struct platform_device *pdev)
{
	struct ricoh583_pwrkey *pwrkey = platform_get_drvdata(pdev);

	flush_workqueue(pwrkey->workqueue);
	destroy_workqueue(pwrkey->workqueue);
	free_irq(pwrkey->key_irq, pwrkey);
	input_unregister_device(pwrkey->pwr);
	kfree(pwrkey);

	return 0;
}

static struct platform_driver ricoh583_pwrkey_driver = {
	.probe		= ricoh583_pwrkey_probe,
	.remove		= __devexit_p(ricoh583_pwrkey_remove),
	.driver		= {
		.name	= "ricoh583-pwrkey",
		.owner	= THIS_MODULE,
	},
};

static int __init ricoh583_pwrkey_init(void)
{
	return platform_driver_register(&ricoh583_pwrkey_driver);
}
module_init(ricoh583_pwrkey_init);

static void __exit ricoh583_pwrkey_exit(void)
{
	platform_driver_unregister(&ricoh583_pwrkey_driver);
}
module_exit(ricoh583_pwrkey_exit);

MODULE_ALIAS("platform:ricoh583-pwrkey");
MODULE_DESCRIPTION("ricoh583 Power Key");
MODULE_LICENSE("GPL v2");
