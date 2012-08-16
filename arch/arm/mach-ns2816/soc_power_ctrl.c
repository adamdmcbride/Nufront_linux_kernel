#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/gpio.h>

#include <mach/soc_power_ctrl.h>

#define GPIO_CAM_POWER 		(46)
#define GPIO_WIFI_PD 		(50)
#define GPIO_WIFI_RESET		(47)

int wifi_set_power(int val)
{
	//PD=GPIO_B18
	int ret;
	int gpio=GPIO_WIFI_PD;
	static int request=0;
	
	if (!request)
	{
		ret = gpio_request(gpio, "wifi-pd");
		if (ret < 0) 
		{
			printk("request wifi-pd error\n");
			return -1;
		}
		request=1;
	}
	

	//off
	if (!val)
	{	
		gpio_direction_output(gpio, 0);
		//gpio_set_value(gpio, 0);
		printk("================wifi_set_power: work off =======\n");
		return 0;
	}
    
	//on
	gpio_direction_output(gpio, 1);
	//gpio_set_value(gpio, 1);
	printk("================wifi_set_power: work on =======\n");
	
	return 0;
}

int wifi_set_reset(int val)
{
	//RESET=GPIO_B15
	int ret;
	int gpio=GPIO_WIFI_RESET;
	static int request=0;
	
	if (!request)
	{
		ret = gpio_request(gpio, "wifi-reset");
		if (ret < 0) 
		{
			printk("request wifi-reset error\n");
			return -1;
		}
		request=1;
	}
	
	//off
	if (!val)
	{
		gpio_direction_output(gpio, 0);
		//gpio_set_value(gpio, 0);
		printk("================wifi_set_reset: work off=======\n");
		return 0;
	}

	//on	
	gpio_direction_output(gpio, 0);
	//gpio_set_value(gpio, 0);
	mdelay(4);
	gpio_set_value(gpio, 1);
    
	printk("================wifi_set_reset: work on=======\n");
	return 0;
}
int wifi_power(int val)
{
	wifi_set_power(val);
	wifi_set_reset(val);
	
	return 0;	
}

int cam_power(int val) 
{
	int ret = 0;
	ret = gpio_request(GPIO_CAM_POWER, "camera-power");
	if (ret)
		return ret;
	gpio_direction_output(GPIO_CAM_POWER, val);
	gpio_free(GPIO_CAM_POWER);
	return 0;
}

