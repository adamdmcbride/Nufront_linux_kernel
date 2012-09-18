#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/gpio.h>


#include <linux/mmc/card.h>
#include <linux/mmc/host.h>
#include <linux/regulator/consumer.h>
#include <mach/soc_power_ctrl.h>
#include <mach/mmc.h>

#include "core.h"


#ifdef CONFIG_MACH_NS115_PAD_TEST

#define		GPIO_BT_ON 	(8 + 16)   //low -> high
#define		GPIO_BT_INT 	(8 + 15)   //bt wakeup ap
#define		GPIO_BT_RST 	(8 + 13)   //low -> high
#define		GPIO_BT_WAKEUP 	(8 + 12)   //ap wakeup bt

#else

#define		GPIO_BT_ON 	(8 + 32 + 25)   //low -> high
#define		GPIO_BT_INT 	(8 + 16)   	//bt wakeup ap
#define		GPIO_BT_RST 	(8 + 32 + 13)   //low -> high
#define		GPIO_BT_WAKEUP 	(8 + 32 + 14)  	//ap wakeup bt

#endif

int bt_init()
{
	int ret = 0;

	ret = gpio_request(GPIO_BT_ON, "bt power");
	if(ret != 0) {
		printk("gpio %d request fail!\n", GPIO_BT_ON);
		return ret;
	}
	gpio_direction_output(GPIO_BT_ON, 0);
	mdelay(10);
	gpio_direction_output(GPIO_BT_ON, 1);
	gpio_free(GPIO_BT_ON);

	ret = gpio_request(GPIO_BT_RST, "bt rst");
	if(ret != 0)
		return ret;
	gpio_direction_output(GPIO_BT_RST, 0);
	mdelay(10);
	gpio_direction_output(GPIO_BT_RST, 1);
	gpio_free(GPIO_BT_RST);

	ret = gpio_request(GPIO_BT_WAKEUP, "bt wakeup");
	if(ret != 0)
		return ret;
	gpio_direction_output(GPIO_BT_WAKEUP, 1);
	gpio_free(GPIO_BT_WAKEUP);

	ret = gpio_request(GPIO_BT_INT, "bt int");
	if(ret != 0)
		return ret;
	gpio_direction_input(GPIO_BT_INT);
	gpio_free(GPIO_BT_INT);

	return ret;
}


#ifdef CONFIG_BCMDHD

#if 0
extern struct mmc_host *bcm_slot2_mmc;
static void bcm_detect(void)
{
	//struct mmc_host *mmc = mmc_get_drvdata(&ns115_sdmmc_device);
	struct mmc_host *mmc = bcm_slot2_mmc;
	BUG_ON(mmc == NULL);

	mmc_detect_change(mmc, 0);
}
#endif
void wifi_power_init(int gpio)
{
	int ret;

	ret = gpio_request(gpio, "WIFI_ON");
	if(ret < 0)
		printk(KERN_ERR "WiFi WL_REG_ON gpio request failed \n");

	gpio_direction_output(gpio, 0);
	mdelay(20);
	/* pull-up */
	gpio_direction_output(gpio, 1);
	mdelay(30);

	gpio_free(gpio);

}
EXPORT_SYMBOL(wifi_power_init);

void bcm_wlan_power_on(int flag)
{
	int ret;
	unsigned int gpio;
	struct regulator *regu = NULL;
	struct platform_device  *pdev = &ns115_sdmmc_device;
	struct ns115_mmc_platform_data *pdata = pdev->dev.platform_data;
	gpio = pdata->gpio;

	printk(KERN_INFO "%s: WiFi WL_REG_ON GPIO %d, flag %d >>>>>>>>>\n", __func__, gpio, flag);

	if(flag & 0x1)
		return;

#ifdef CONFIG_MACH_NS115_PAD_REF
	if(!flag) {
		regu = regulator_get(NULL, "vdd_wifi_1v8");
		if(IS_ERR(regu)) {
			printk(KERN_ERR "%s: regulator_get failed", __func__);
			//return;
		} else {
			regulator_enable(regu);
			mdelay(10);
		}
	}
#endif

	ret = gpio_request(gpio, "WIFI_ON");
	if(ret < 0)
		printk(KERN_ERR "WiFi WL_REG_ON gpio request failed \n");

	gpio_direction_output(gpio, 0);
	mdelay(20);
	/* pull-up */
	gpio_direction_output(gpio, 1);
	mdelay(30);

	gpio_free(gpio);
	/**
	 * kernel will stuck if called early,
	 * cuz slot2 hasn't been created.
	 */
	//bcm_detect();
}
EXPORT_SYMBOL(bcm_wlan_power_on);

void bcm_wlan_power_off(int flag)
{
	unsigned int gpio;
	int ret;
	struct regulator *regu = NULL;
	struct platform_device  *pdev = &ns115_sdmmc_device;
	struct ns115_mmc_platform_data *pdata = pdev->dev.platform_data;
	gpio = pdata->gpio;

	printk(KERN_INFO "%s: WiFi WL_REG_ON GPIO %d, flag %d >>>>>>>>>\n", __func__, gpio, flag);

	ret = gpio_request(gpio, "WIFI_ON");
	if(ret < 0)
		printk(KERN_ERR "WiFi WL_REG_ON gpio request failed \n");
	/* pull-down */
	gpio_direction_output(gpio, 0);
	mdelay(20);

	gpio_free(gpio);

#ifdef CONFIG_MACH_NS115_PAD_REF
	if(flag & 0x1) {
		regu = regulator_get(NULL, "vdd_wifi_1v8");
		if(IS_ERR(regu)) {
			printk(KERN_ERR "%s: regulator_get failed", __func__);
			//return;
		} else
			regulator_disable(regu);
	}
#endif
}
EXPORT_SYMBOL(bcm_wlan_power_off);
#endif

/*set the gpio for bt_fm_switch
 * This is just a temporary interface
 * Should be updated according to applications*/
void bt_fm_switch(int value)
{
#ifdef CONFIG_MACH_NS115_PAD_REF
	int ret = 0;
	ret = gpio_request(46, "BT_FM_SWITCH");//B6 - BT_FM_SWITCH
	if(ret < 0)
		printk(KERN_ERR "BT_FM_SWITCH request failed \n");
	gpio_direction_output(46, value);
#endif
}
EXPORT_SYMBOL(bt_fm_switch);

#ifdef CONFIG_SND_SOC_ALC5631
void rt5631_gpio_test_init(void)
{
    int ret = 0;

    ret = gpio_request(65, "CODEC_PWR_EN");//d15 - CODEC_PWR_EN
    if(ret < 0)
        printk(KERN_ERR "RT5631 CODEC_PWR_EN request failed \n");
    gpio_direction_output(65, 1);

    ret = gpio_request(63, "MUTE_AMP");//C15 - MUTE_AMP
    if(ret < 0)
        printk(KERN_ERR "RT5631 MUTE_AMP request failed \n");
    gpio_direction_output(63, 1);

}
EXPORT_SYMBOL(rt5631_gpio_test_init);
#endif

#ifdef CONFIG_SND_SOC_ALC5631
void rt5631_gpio_ref_init(void)
{
    int ret = 0;
    ret = gpio_request(63, "MUTE_AMP");//C15 - MUTE_AMP
    if(ret < 0)
        printk(KERN_ERR "RT5631 MUTE_AMP request failed \n");
    gpio_direction_output(63, 1);
}
EXPORT_SYMBOL(rt5631_gpio_ref_init);
#endif
