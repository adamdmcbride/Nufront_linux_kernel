#ifndef _NS2816_ARCH_COMMON_H_
#define	_NS2816_ARCH_COMMON_H_

#define PINMUX_GROUP_MAX			(19)

#define PINMUX_SETTING(grp,val) \
	{ .group = (grp), .value = (val),}

#define GPIO_A_15 	PINMUX_SETTING(14,2)
#define GPIO_A_21 	PINMUX_SETTING(16,2)
#define GPIO_A_4_11 	PINMUX_SETTING(18,2)
#define UART_1	 	PINMUX_SETTING(18,0)
#define GPIO_B_14_15 	PINMUX_SETTING(11,2)
#define GPIO_B_18_19 	PINMUX_SETTING(12,2)
#define MMC_SLOT_2   	PINMUX_SETTING(2,2)

struct pinmux_setting
{
	int group;
	int value;
};
extern int pinmux_init(struct pinmux_setting * setting, int len);

extern void common_init(void);
extern void ddr_pm_init(void);
extern void scm_init(void);
#endif
