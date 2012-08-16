#ifndef __ns115_BATTERY_H__
#define __ns115_BATTERY_H__

#include <linux/power_supply.h>

enum ns115_charger_type{
	CHG_TYPE_USB,
	CHG_TYPE_AC,
	CHG_TYPE_DOCK
};

enum ns115_charging_type{
	CHGING_TYPE_PRE,
	CHGING_TYPE_FAST
};

enum ns115_battery_event {
	CHG_PLUG_EVENT,
	CHG_UNPLUG_EVENT,
	CHG_DONE_EVENT,
	CHG_BATT_BEGIN_PRE_CHARGING,
	CHG_BATT_BEGIN_FAST_CHARGING,
	CHG_BATT_TEMP_OUTOFRANGE,
	CHG_BATT_TEMP_INRANGE,
	CHG_BATT_STATUS_CHANGED,
};

/**
 * enum hardware_charger_state
 * @CHG_ABSENT_STATE: charger cable is unplugged
 * @CHG_PRESENT_STATE: charger cable is plugged but charge current isnt drawn
 * @CHG_READY_STATE: charger cable is plugged and kernel knows how much current
 *			it can draw
 * @CHG_CHARGING_STATE: charger cable is plugged and current is drawn for
 *			charging
 */
enum ns115_charger_state {
	CHG_ABSENT_STATE,
	CHG_PRESENT_STATE,
	CHG_READY_STATE,
	CHG_CHARGING_STATE,
	CHG_DONE_STATE
};

struct ns115_charger {
	int type;
	int chg_current;
	const char *name;
	int (*start_charging) (struct ns115_charger *hw_chg, enum ns115_charging_type type);
	int (*stop_charging) (struct ns115_charger *hw_chg);
	int (*get_charging_stat) (enum ns115_charger_state *hw_stat);

	void *charger_private;	/* used by the ns115_battery.c */
};

struct ns115_battery_gauge {
	int resistor_mohm;
	int (*get_battery_mvolts) (void);
	int (*get_battery_temperature) (void);
	int (*get_battery_status)(void);
	int (*get_battery_capacity) (int mvolts);
	int (*is_batt_temp_out_of_range) (void);
};
/**
 * struct ns115_battery_platform_data
 * @safety_time: max charging time in minutes
 * @update_time: how often the userland be updated of the charging progress
 */
struct ns115_battery_platform_data {
	unsigned int update_time;
	unsigned int safety_time;
	int consumption;
	int pre_chg_mvolts;
	int full_mvolts;
};

int ns115_battery_gauge_register(struct ns115_battery_gauge *batt_gauge);
int ns115_battery_gauge_unregister(struct ns115_battery_gauge *batt_gauge);
int ns115_charger_register(struct ns115_charger *hw_chg);
int ns115_charger_unregister(struct ns115_charger *hw_chg);
void ns115_charger_plug(enum ns115_charger_type type);
void ns115_charger_unplug(void);
int ns115_battery_notify_event(enum ns115_battery_event event);

#endif 
