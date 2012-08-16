/*
 *
 *  This file is subject to the terms and conditions of the GNU General Public
 *  License. See the file COPYING in the main directory of this archive for
 *  more details.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/string.h>
#include <linux/mm.h>
#include <linux/delay.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/wakelock.h>
#include <linux/pm_runtime.h>
#include <linux/firmware.h>
#include <asm/irq.h>
#include <asm/io.h>
#include <mach/wake_timer.h>
#include <mach/hardware.h>
#include <mach/board-ns115.h>
#include "wake_timer.h"
#ifdef CONFIG_PM_WAKEUP_DEVICE_AUTO_TEST_SUSPEND
#include <linux/input.h>
#endif
#define WAKEUP_TIMER_NAME		"ns115_wk_timer"
#define WAKEUP_TIMER_WAKELOCK_NAME	"ns115_wk_timer_wakelock"

#define SLEEP_MS	10000
#define WAKE_MS		20000

//#define MANU_UNLOCK

#define STAT_ON		1
#define STAT_OFF	0
#define RT_MS		5000

#ifdef CONFIG_PM_AUTO_TEST_SUSPEND
#ifdef CONFIG_PM_WAKEUP_DEVICE_AUTO_TEST_SUSPEND
int register_test_input_dev(struct wake_timer * ts)
{
	int err = 0;

	ts->input_dev = input_allocate_device();
	if (!ts->input_dev) {
		return -ENOMEM;
	}
	input_set_capability(ts->input_dev, EV_KEY, KEY_POWER);

	ts->input_dev->name = "test_input_dev";
	ts->input_dev->phys = "test_input_dev/input0";
	err = input_register_device(ts->input_dev);
	if (err) {
		goto free_input_dev;
	}
	return 0;
free_input_dev:
	input_free_device(ts->input_dev);
	return err;
}
#endif

void enable_timer(struct wake_timer * tm, int ms, int num)
{
	writel(FREQ_REF_KHZ * ms, tm->mmio + num * OFFSET);
	writel(CTRL_EN_BIT | CTRL_MODE_BIT, \
			tm->mmio + CTRL + num * OFFSET);
}

void disable_timer(struct wake_timer * tm, int num) 
{
	writel(0, tm->mmio + CTRL + num * OFFSET);
}

static irqreturn_t wake_timer_irq(int irq, void *dev_id)
{
	struct wake_timer *tm = dev_id;
	int stat = 0;

	if(irq == tm->irq[0]) {
		/*clear interrupt*/
		readl(tm->mmio + EOI); 
		pm_wakeup_event(&tm->pdev->dev, 0);
		pm_relax(&tm->pdev->dev);

#ifdef CONFIG_PM_WAKEUP_DEVICE_AUTO_TEST_SUSPEND
		input_report_key(tm->input_dev, KEY_POWER, 1);
		input_sync(tm->input_dev);

		input_report_key(tm->input_dev, KEY_POWER, 0);
		input_sync(tm->input_dev);
#endif

#ifdef MANU_UNLOCK
 		wake_lock(&tm->wake_lock);
#else
 		wake_lock_timeout(&tm->wake_lock, tm->wake_ms/1000*HZ);
#endif
	} else if(irq == tm->irq[1]) {
		/*clear interrupt*/
		readl(tm->mmio + EOI + OFFSET); 
		if(tm->stat == STAT_ON) {
			stat = pm_runtime_put(&tm->pdev->dev);
		} else {
			stat = pm_runtime_get(&tm->pdev->dev);
		}
		if(stat)
			dev_err(&tm->pdev->dev, "pm runtime ret %d", stat);
#ifdef MANU_UNLOCK
		wake_unlock(&tm->wake_lock);
#endif
	}
	/*dev_info(&tm->pdev->dev, "irq %d", irq);*/

        return IRQ_HANDLED;
}
#endif  /*CONFIG_PM_AUTO_TEST_SUSPEND*/

static int __devinit load_n3s3_fw(struct platform_device *pdev)
{
	/*add n3s3_fw to here for debug */
	const struct firmware *fw = NULL;
	struct wake_timer_data * data = pdev->dev.platform_data;
	const char * fw_name = (data ? data->fw_name : NULL);
	void __iomem * sarram = __io_address(NS115_SARRAM_BASE);
	int ret = 0;
	
	if(!fw_name) {
		dev_err(&pdev->dev,
		       "invalid n3s3_fw name");
		return -ENOENT;
	}
	ret = request_firmware(&fw, fw_name, &pdev->dev);
	if(ret)	{
		dev_err(&pdev->dev,
		       "load %s failed", fw_name);
		return ret;
	}
	if(fw->size < 0x1000) {
		memcpy(sarram, fw->data, fw->size);
		ret = 0;
	} else {
		dev_err(&pdev->dev,
		       "%s %d too large to load", fw_name, fw->size);
		ret = -EINVAL;
	}

	release_firmware(fw);
	return ret;
}

static int __devinit wake_timer_probe(struct platform_device *pdev)
{
	struct resource *res = NULL;
	struct wake_timer * tm = NULL;
	int retval = 0;
	struct wake_timer_data * data = pdev->dev.platform_data;
	
	/*
	 * Dynamically allocate info and par
	 */
	tm = kmalloc(sizeof(struct wake_timer), GFP_KERNEL);
	
	if (!tm) {
		/* goto error path */
		retval = -ENOMEM;
		goto out;
	}

	tm->pdev = pdev;
	tm->stat = STAT_ON;
	tm->count = 0;
	tm->can_suspend = (load_n3s3_fw(pdev) == 0? 1:0);
	if(data) {
		tm->suspend_ms = data->suspend_ms;
		tm->wake_ms = data->wake_ms;
	}
	
#ifdef CONFIG_PM_AUTO_TEST_SUSPEND
	/*
	 *init clk	
	*/
#if 0
	tm->clk = clk_get(&pdev->dev, WAKEUP_TIMER_NAME);
	if(IS_ERR(tm->clk)) {
		dev_err(&pdev->dev,"get wake_timer clock failed");
		return PTR_ERR(tm->clk);
	}
#endif
#ifdef CONFIG_PM_WAKEUP_DEVICE_AUTO_TEST_SUSPEND
	retval = register_test_input_dev(tm);
	if(retval){
		dev_err(&pdev->dev,"register test input dev failed");
		goto free_wake_timer;
	}
#endif

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if(!res) {
		dev_err(&pdev->dev,"get_resource failed");
		retval = -ENXIO;
		goto free_wake_timer;
	}
	
	tm->mmio = ioremap_nocache(res->start,resource_size(res));
	if(!tm->mmio) {
		dev_err(&pdev->dev,"ioremap failed");
		retval = -ENOMEM;
		goto free_wake_timer;
	}

	res = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
	if(!res) {
		dev_err(&pdev->dev,"get_resource failed");
		retval = -ENXIO;
		goto unmap_mmio;
	}

	tm->irq[0] = res->start;
	tm->irq[1] = res->start+1;

	retval = request_irq(tm->irq[0], wake_timer_irq, 0,\
				 WAKEUP_TIMER_NAME, tm);
	if(retval < 0) {
		dev_err(&pdev->dev,"request_irq %d failed",tm->irq[0]);
		retval = -ENXIO;
		goto unmap_mmio;
	}

	retval = request_irq(tm->irq[1], wake_timer_irq, 0, \
				WAKEUP_TIMER_NAME, tm);
	if(retval < 0) {
		dev_err(&pdev->dev,"request_irq %d failed", tm->irq[1]);
		retval = -ENXIO;
		goto free_irq_0;
	}


	wake_lock_init(&(tm->wake_lock), WAKE_LOCK_SUSPEND,
			WAKEUP_TIMER_WAKELOCK_NAME);

	device_init_wakeup(&pdev->dev, false);

	retval = pm_runtime_set_active(&pdev->dev); 
	if(retval) {
		dev_err(&pdev->dev, "pm runtime set active failed %d", retval);
	}
	pm_runtime_enable(&pdev->dev);
	platform_set_drvdata(pdev, tm);

	enable_timer(tm, RT_MS, 1);
	return retval;

free_irq_0:
	free_irq(tm->irq[0], tm);
unmap_mmio:
	iounmap(tm->mmio);
free_wake_timer:
	kfree(tm);
#endif /*CONFIG_PM_AUTO_TEST_SUSPEND*/	
out:
	return retval;
}

static int __devexit wake_timer_remove(struct platform_device *pdev)
{
	struct wake_timer *tm = platform_get_drvdata(pdev);
	if(tm) {
#ifdef CONFIG_PM_AUTO_TEST_SUSPEND
#if 0
		clk_put(tm->clk);
#endif
		disable_timer(tm, 0);
		disable_timer(tm, 1);
		free_irq(tm->irq[0], tm);
		free_irq(tm->irq[1], tm);
		iounmap(tm->mmio);
#ifdef CONFIG_PM_WAKEUP_DEVICE_AUTO_TEST_SUSPEND
		input_unregister_device(tm->input_dev);
#endif
#endif /*CONFIG_PM_AUTO_TEST_SUSPEND*/
		kfree(tm);
	}
	return 0;
}

#ifdef CONFIG_PM_AUTO_TEST_SUSPEND
static int wake_timer_suspend(struct device *dev)
{
	struct wake_timer *tm = dev_get_drvdata(dev);
	if(!(tm->can_suspend && tm->suspend_ms))
		return -EBUSY;

	enable_irq_wake(tm->irq[0]);
	/*enable_irq_wake(tm->irq[1]);*/
	enable_timer(tm, tm->suspend_ms, 0);
#ifdef MANU_UNLOCK
	disable_timer(tm, 1);
#endif
	return 0;
}
static int wake_timer_resume(struct device *dev)
{
	struct wake_timer *tm = dev_get_drvdata(dev);
	disable_irq_wake(tm->irq[0]);
	/*disable_irq_wake(tm->irq[1]);*/
	disable_timer(tm, 0);
#ifdef MANU_UNLOCK
	enable_timer(tm, tm->wake_ms, 1);
#endif
	return 0;
}
int wake_timer_runtime_suspend(struct device *dev)
{
	struct wake_timer *tm = dev_get_drvdata(dev);
	dev_dbg(dev,"%s stat %d", __func__, tm->stat);
	tm->stat = STAT_OFF;
	return 0;
}
int wake_timer_runtime_resume(struct device *dev)
{
	struct wake_timer *tm = dev_get_drvdata(dev);
	dev_dbg(dev,"%s stat %d", __func__, tm->stat);
	tm->stat = STAT_ON;
	return 0;
}
int wake_timer_runtime_idle(struct device *dev)
{
	struct wake_timer *tm = dev_get_drvdata(dev);
	dev_dbg(dev,"%s stat %d", __func__, tm->stat);
	/*return -EBUSY if device is in use*/
	return 0;
}
#else
#define wake_timer_suspend NULL
#define wake_timer_resume NULL
#define wake_timer_runtime_suspend NULL
#define wake_timer_runtime_resume NULL
#define	wake_timer_runtime_idle NULL
#endif /*CONFIG_PM_AUTO_TEST_SUSPEND */

struct dev_pm_ops wake_timer_pm = 
{
	.suspend = wake_timer_suspend,
	.resume = wake_timer_resume,
	.runtime_suspend = wake_timer_runtime_suspend,
	.runtime_resume = wake_timer_runtime_resume,
	.runtime_idle = wake_timer_runtime_idle,
};

static struct platform_driver wake_timer_driver = {
	.probe = wake_timer_probe,
	.remove = wake_timer_remove,
	.driver = {
		.name = WAKEUP_TIMER_NAME,
		.owner = THIS_MODULE,
		.pm = &wake_timer_pm,
	},
};

static int __init wake_timer_init(void)
{
	return platform_driver_register(&wake_timer_driver);
}

static void __exit wake_timer_exit(void)
{
	platform_driver_unregister(&wake_timer_driver);
}

module_init(wake_timer_init);
module_exit(wake_timer_exit);

MODULE_LICENSE("GPL");
