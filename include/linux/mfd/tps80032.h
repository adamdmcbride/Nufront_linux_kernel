/* include/linux/mfd/tps80032.h
 *
 * Core driver interface to access TPS80032 power management chip.
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

#ifndef __LINUX_MFD_TPS80032_H
#define __LINUX_MFD_TPS80032_H

#include <linux/regulator/machine.h>

//#define TPS80032_DEBUG
#ifdef TPS80032_DEBUG
#define  PDBG(dev, format,...)	\
	dev_err(dev, format, ##__VA_ARGS__)
#define  PINFO(dev, format,...)	\
	dev_err(dev, format, ##__VA_ARGS__)
#else
#define  PDBG(dev, format,...)	do{}while(0)
#define  PINFO(dev, format,...)	\
	dev_info(dev, format, ##__VA_ARGS__)
#endif

/* TPS80032 IRQ definitions */
enum {
	TPS80032_IRQ_PWRON,
	TPS80032_IRQ_RPWRON,
	TPS80032_IRQ_SYS_VLOW,
	TPS80032_IRQ_RTC_ALARM,
	TPS80032_IRQ_RTC_PERIOD,
	TPS80032_IRQ_HOT_DIE,
	TPS80032_IRQ_VXXX_SHORT,
	TPS80032_IRQ_SPDURATION,
	TPS80032_IRQ_WATCHDOG,
	TPS80032_IRQ_BAT,
	TPS80032_IRQ_SIM,
	TPS80032_IRQ_MMC,
	TPS80032_IRQ_GPADC_RT_SW1_EOC,
	TPS80032_IRQ_GPADC_SW2_EOC,
	TPS80032_IRQ_CC_EOC,
	TPS80032_IRQ_CC_AUTOCAL,
	TPS80032_IRQ_ID_WKUP,
	TPS80032_IRQ_VBUS_WKUP,
	TPS80032_IRQ_ID,
	TPS80032_IRQ_VBUS,
	TPS80032_IRQ_CHRG_CTRL,
	TPS80032_IRQ_EXT_CHRG,
	TPS80032_IRQ_INT_CHRG,
	TPS80032_NR_IRQS,
};

enum regulator_id {
	TPS80032_ID_SMPS1,
	TPS80032_ID_SMPS2,
	TPS80032_ID_SMPS3,
	TPS80032_ID_SMPS4,
	TPS80032_ID_SMPS5,
	TPS80032_ID_LDOLN,
	TPS80032_ID_LDO1,
	TPS80032_ID_LDO2,
	TPS80032_ID_LDO3,
	TPS80032_ID_LDO4,
	TPS80032_ID_LDO5,
	TPS80032_ID_LDO6,
	TPS80032_ID_LDO7,
	TPS80032_ID_LDOUSB,
	TPS80032_NR_ID,
};

enum tps80032_adc_channel {
	TPS80032_ADC0_BAT_TYPE,
	TPS80032_ADC1_BAT_TEMP,
	TPS80032_ADC2_AUDIO_GP,
	TPS80032_ADC3_TEMP_DIODE_GP,
	TPS80032_ADC4_TEMP_GP,
	TPS80032_ADC5_GP,
	TPS80032_ADC6_GP,
	TPS80032_ADC7_SYS,
	TPS80032_ADC8_BACKUP_BAT,
	TPS80032_ADC9_EXT_CHG,
	TPS80032_ADC10_VBUS,
	TPS80032_ADC11_DCDC_CUR,
	TPS80032_ADC12_DIE_TEMP,
	TPS80032_ADC13_DIE_TEMP,
	TPS80032_ADC14_USB_ID,
	TPS80032_ADC15_TEST_NET,
	TPS80032_ADC16_TEST_NET,
	TPS80032_ADC17_BAT_CUR,
	TPS80032_ADC18_BAT_VOLT,
	TPS80032_NR_ADC,
};

enum tps80032_leds_current {
	TPS80032_LEDS_0MA,
	TPS80032_LEDS_1MA,
	TPS80032_LEDS_2_5MA,
	TPS80032_LEDS_5MA,
};

enum tps80032_leds_source {
	TPS80032_LEDS_VBUS,
	TPS80032_LEDS_VAC,
	TPS80032_LEDS_EXTPIN,
};

enum tps80032_leds_mode {
	TPS80032_LEDS_HW,
	TPS80032_LEDS_SW,
	TPS80032_LEDS_OFF,
};

struct tps80032_subdev_info {
	int		id;
	const char	*name;
	void		*platform_data;
};

struct tps80032_regulator_platform_data {
	struct regulator_init_data regulator;
	int init_uV;
	unsigned init_enable:1;
	unsigned init_apply:1;
	unsigned sleep_apply:1;
	int sleep_enable;
};

struct tps80032_rtc_platform_data {
	int irq;
};

struct tps80032_pwrkey_platform_data {
	int irq;
	unsigned long delay_ms;
};

struct tps80032_adc_platform_data {
	int sw2_irq;
};

struct tps80032_battery_platform_data {
	int autocal_irq;
	int resistor;//mohm
	int alarm_mvolts; 
	int power_off_mvolts;
	int max_mAh;
	int (*capacity_table)[][2];
	int table_size;
};

struct tps80032_chg_ctrl_platform_data {
	int irq;
};

struct tps80032_charger_platform_data {
	int irq;
};

struct tps80032_leds_platform_data {
	const char *name;
	enum tps80032_leds_current cur;
	enum tps80032_leds_source source;
	enum tps80032_leds_mode mode;
};

struct tps80032_platform_data {
	int		num_subdevs;
	struct	tps80032_subdev_info *subdevs;
	int		irq_base;
};

extern int tps80032_read(int id, uint8_t reg, uint8_t *val);
extern int tps80032_bulk_reads(int id, u8 reg, u8 count, uint8_t *val);
extern int tps80032_write(int id, u8 reg, uint8_t val);
extern int tps80032_bulk_writes(int id, u8 reg, u8 count, uint8_t *val);
extern int tps80032_set_bits(int id, u8 reg, uint8_t bit_mask);
extern int tps80032_clr_bits(int id, u8 reg, uint8_t bit_mask);
extern int tps80032_reg_update(int id, u8 reg, uint8_t val, uint8_t mask);
extern int tps80032_suspend_system(void);
extern void tps80032_power_off(void);
extern void tps80032_restart(char str, const char * cmd);

enum ns115_charger_type;
extern void tps80032_charger_plug(enum ns115_charger_type type);
extern void tps80032_charger_unplug(void);
extern int tps80032_get_adc_value(int channel, int scalar, int cur_source);
extern int tps80032_chg_watchdog_init(int seconds);
extern int tps80032_chg_watchdog_reset(void);
extern int tps80032_clk32kaudio_switch(int on);
extern int tps80032_clk32kao_switch(int on);
extern int tps80032_clk32kg_switch(int on);

#endif
