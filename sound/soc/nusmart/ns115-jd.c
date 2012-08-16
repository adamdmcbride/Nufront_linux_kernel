#include <linux/init.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/clk.h>
#include <linux/time.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/switch.h>

#include <linux/interrupt.h>
#include <mach/hardware.h>
#include <mach/board-ns115.h>

#define PRINT_DBG_INFO 
#define DEBOUNCE_CNT 5

#ifdef  PRINT_DBG_INFO 
#define DBG_PRINT(fmt, args...) printk( KERN_INFO "ns115_rt5631_jd " fmt, ## args) 
#else
#define DBG_PRINT(fmt, args...) /* not debugging: nothing */ 
#endif

struct ns115_jd_pridata {
	int irq;
	int gpio;
	int report;
	struct switch_dev sdev;
	struct work_struct work;
};

struct  ns115ref_rt5631_jd_platform_data {
	unsigned int irq;
};
static struct ns115_jd_pridata ns115_jd_data;
struct ns115ref_rt5631_jd_platform_data *ns115re_rt5631_jd_dev_data;
static struct workqueue_struct *ns115_jd_wq;

static irqreturn_t ns115_jd_irq_handler(int irq, void *data)
{
	struct ns115_jd_pridata *jack = &ns115_jd_data;

	disable_irq_nosync(irq);
	queue_work(ns115_jd_wq, &jack->work);

	return IRQ_HANDLED;
}

static void  ns115_jd_work_func(struct work_struct *work)
{
	struct ns115_jd_pridata *jack = container_of(work, struct ns115_jd_pridata, work);
	int irqret, state, state_tmp, deb_cnt = 0;
	unsigned long irqflags;

	//delay 100 ms and check the gpio 5 times to avoid wrong state
	msleep(100);
	state = gpio_get_value(jack->gpio);
	for(deb_cnt = 0; deb_cnt < 5; )
	{
		msleep(60);
		state_tmp = gpio_get_value(jack->gpio);
		if(state == state_tmp)
		{
			deb_cnt++;
		}
		else
		{
			state = state_tmp;
			deb_cnt = 0;
		}
	}
	irqflags = state?IRQF_TRIGGER_FALLING:IRQF_TRIGGER_RISING;
	if(state != jack->report)
	{
		jack->report = state;
		switch_set_state(&jack->sdev, !!state);
	}
	free_irq(jack->irq, NULL);
	irqret = request_irq(jack->irq, ns115_jd_irq_handler, irqflags,
			"ns115-jd", NULL);
}

#ifdef CONFIG_PM
static int ns115_jd_suspend(struct platform_device *pdev)
{
	return 0;
}

static int ns115_jd_resume(struct platform_device *pdev)
{
	int irqret, state;
	unsigned long irqflags;
	struct ns115_jd_pridata *jack = &ns115_jd_data;

	state = gpio_get_value(jack->gpio);
	irqflags = state?IRQF_TRIGGER_FALLING:IRQF_TRIGGER_RISING;

	if(state != jack->report)
	{
		jack->report = state;
		switch_set_state(&jack->sdev, !!state);
	}
	free_irq(jack->irq, NULL);
	irqret = request_irq(jack->irq, ns115_jd_irq_handler, irqflags,
			"ns115-jd", NULL);
	return 0;
}

#else
#define ns115_jd_suspend	NULL
#define ns115_jd_resume	NULL
#endif

static int ns115_jd_probe(struct platform_device *pdev)
{
	int ret, irqret;
	unsigned long irqflags;
	struct ns115_jd_pridata *jack = &ns115_jd_data;

	DBG_PRINT("%s ....\n", __func__);

	ns115re_rt5631_jd_dev_data = (struct ns115ref_rt5631_jd_platform_data *)pdev->dev.platform_data;
	jack->irq = ns115re_rt5631_jd_dev_data->irq;
	jack->gpio = irq_to_gpio(ns115re_rt5631_jd_dev_data->irq);

	INIT_WORK(&jack->work, ns115_jd_work_func);

	ret = gpio_request(jack->gpio, "RT5631_JD");//d15 - CODEC_PWR_EN
	if(ret < 0)
	{
		DBG_PRINT("RT5631_JD request failed \n");
		return ret;
	}
	gpio_direction_input(jack->gpio);
	jack->report = gpio_get_value(jack->gpio);

	irqflags = jack->report?IRQF_TRIGGER_FALLING:IRQF_TRIGGER_RISING;

	DBG_PRINT("gpio = %d, irqflags=0x%x", jack->gpio, irqflags);
	/* switch-class based headset detection */
	jack->sdev.name = "h2w";
	ret = switch_dev_register(&jack->sdev);
	if (ret) {
		DBG_PRINT("error registering switch device %d\n", ret);
		return ret;
	}

	switch_set_state(&jack->sdev, !!jack->report);
	irqret = request_irq(jack->irq, ns115_jd_irq_handler, irqflags,
			"ns115-jd", NULL);

	if (ret)
	{
		DBG_PRINT("JD request irq failed\n");
	}

	return ret;
}
static int __devexit ns115_jd_remove(struct platform_device *pdev)
{
	struct ns115_jd_pridata *jack = &ns115_jd_data;

	DBG_PRINT("%s ....\n", __func__);

	free_irq(jack->irq, NULL);
	cancel_work_sync(&jack->work);
	return 0;
}

static struct platform_driver ns115_jd_driver = {
	.probe = ns115_jd_probe,
	.remove = ns115_jd_remove,
	.suspend = ns115_jd_suspend,
	.resume = ns115_jd_resume,
	.driver = {
		.name = "rt5631-jd",
		.owner = THIS_MODULE,
	},
};

static int __init ns115_jd_init(void)
{
	DBG_PRINT("%s ....\n", __func__);
	//create a work queue for jd
	ns115_jd_wq = create_workqueue("ns115_jd_wq");
	if (!ns115_jd_wq)
	{
		DBG_PRINT("creat ns115_jd_wq faiked\n");
	}
	return platform_driver_register(&ns115_jd_driver);
}

static void __exit ns115_jd_exit(void)
{
	if (ns115_jd_wq)
	{
		destroy_workqueue(ns115_jd_wq);
	}
	platform_driver_unregister(&ns115_jd_driver);
}

module_init(ns115_jd_init);
module_exit(ns115_jd_exit);

/* Module information */
MODULE_AUTHOR("Nufront");
MODULE_DESCRIPTION("Ns115 Jack Detection Interface");
MODULE_LICENSE("GPL");
