#ifndef __MACH_SOC_POWER_CTRL_H_
#define __MACH_SOC_POWER_CTRL_H_

int bt_init(void);
void rt5631_gpio_ref_init(void);
void rt5631_gpio_test_init(void);
void bt_fm_switch(int value);
extern void bcm_wlan_power_on(int flag);
extern void bcm_wlan_power_off(int flag);

#endif

