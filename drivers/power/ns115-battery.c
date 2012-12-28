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
 *
 */

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/err.h>
#include <linux/power_supply.h>
#include <linux/delay.h>
#include <linux/bitops.h>
#include <linux/debugfs.h>
#include <linux/power/ns115-battery.h>
#include <linux/time.h>
#include <linux/slab.h>
#include <asm/atomic.h>
#include <asm/uaccess.h>

#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif
#ifdef CONFIG_WAKELOCK
//#define CHARGING_ALWAYS_WAKE
#endif
#ifdef CHARGING_ALWAYS_WAKE
#include <linux/wakelock.h>
#endif

//#define VOLTAGE_DEBOUCE

#define CHARGING_TEOC_MS		9000000
#define UPDATE_TIME_MS			60000
#define RESUME_CHECK_PERIOD_MS		60000

#define NS115_BATT_MISSING_VOLTS 3500
#define NS115_BATT_MISSING_TEMP  35
#define NS115_BATT_PRE_CHG_MVOLTS 3100
#define NS115_BATT_FULL_MVOLTS	 4150
#define NS115_BATT_TRKL_DROP	20
#define NS115_MAX_VOLT_BUF		10
#define NS115_VOLT_WINDOWS		25

#define NS115_CHG_MAX_EVENTS	16
#define RESUME_WAIT_TIME	800 //ms

//#define DEBUG_EN
#ifdef DEBUG_EN
#define  PDBG(format,...)	\
	dev_err(ns115_chg.dev, format, ##__VA_ARGS__)
#define  PINFO(format,...)	\
	dev_err(ns115_chg.dev, format, ##__VA_ARGS__)
#else
#define  PDBG(format,...)	do{}while(0)
#define  PINFO(format,...)	\
	dev_info(ns115_chg.dev, format, ##__VA_ARGS__)
#endif

/**
 * enum ns115_battery_status
 * @BATT_STATUS_ABSENT: battery not present
 * @BATT_STATUS_DISCHARGING: battery is present and is discharging
 * @BATT_STATUS_PRE_CHARGING: battery is being prepare charged
 * @BATT_STATUS_FAST_CHARGING: battery is being fast charged
 * @BATT_STATUS_CHARGING_DONE: finished charging,
 * @BATT_STATUS_TEMP_OUT_OF_RANGE: battery present,
					no charging, temp is hot/cold
 */
enum ns115_battery_status {
	BATT_STATUS_ABSENT,
	BATT_STATUS_DISCHARGING,
	BATT_STATUS_PRE_CHARGING,
	BATT_STATUS_FAST_CHARGING,
	BATT_STATUS_CHARGING_DONE,
	BATT_STATUS_TEMP_OUT_OF_RANGE,
};

enum ns115_system_stat {
	SYS_STAT_NORMAL,
	SYS_STAT_EARLY_SUSPEND,
	SYS_STAT_SUSPEND,
};

struct ns115_charger_priv {
	struct list_head list;
	struct ns115_charger *hw_chg;
	enum ns115_charger_state hw_chg_state;
	struct power_supply psy;
};

struct ns115_battery_mux {
	int inited;
	struct list_head ns115_chargers;
	int count_chargers;
	struct mutex ns115_chargers_lock;

	struct device *dev;

	unsigned int safety_time;
	struct delayed_work teoc_work;

	unsigned int update_time;
	int stop_update;
	struct delayed_work update_heartbeat_work;
	struct delayed_work batt_resume_init_work;

	struct mutex status_lock;
	int	batt_mvolts;
	int	batt_volt_buf[NS115_MAX_VOLT_BUF];
	int batt_volt_pointer;
	int batt_volt_times;
#ifdef VOLTAGE_DEBOUCE
	int batt_left_windows;
	int batt_right_windows;
	int charger_power_on;
#endif
	int power_on;
	int resume_by_charger;
	enum ns115_battery_status batt_status;
	struct ns115_charger_priv *current_chg_priv;

	int event_queue[NS115_CHG_MAX_EVENTS];
	int tail;
	int head;
	spinlock_t queue_lock;
	int queue_count;
	struct work_struct queue_work;
	struct workqueue_struct *event_wq_thread;
#ifdef CHARGING_ALWAYS_WAKE
	struct wake_lock wl;
#endif
	struct wake_lock chg_wl;
	int pre_chg_mvolts;
	int full_mvolts;
	enum ns115_system_stat sys_stat;
	int init_mAh;
	int init_mAh_cache;
	int batt_cc;
	unsigned long init_mAh_time;
	unsigned long init_mAh_time_cache;
};

static struct ns115_battery_mux ns115_chg;

static struct ns115_battery_gauge *ns115_batt_gauge = NULL;


static int is_batt_status_charging(void)
{
	if (ns115_chg.batt_status == BATT_STATUS_FAST_CHARGING
	    || ns115_chg.batt_status == BATT_STATUS_PRE_CHARGING){
		return 1;
	}
	return 0;
}

static int get_about_current(int cur_vol)
{
	int chg_cur = 0;
	int tmp_chg_cur = 0;
	int cur = 0;
	struct ns115_charger_priv *priv;

	priv = ns115_chg.current_chg_priv;
	switch (ns115_chg.sys_stat){
		case SYS_STAT_NORMAL:
			cur = ns115_batt_gauge->normal_pwr * 1000 / cur_vol;
			break;
		case SYS_STAT_EARLY_SUSPEND:
			cur = ns115_batt_gauge->early_pwr * 1000 / cur_vol;
			break;
		case SYS_STAT_SUSPEND:
			cur = ns115_batt_gauge->suspend_pwr * 1000 / cur_vol;
			break;
	}
	if (priv){
		tmp_chg_cur = ns115_chg.current_chg_priv->hw_chg->chg_current;
		if (priv->hw_chg_state == CHG_CHARGING_STATE){
			chg_cur = tmp_chg_cur;
		}else if (priv->hw_chg_state == CHG_DONE_STATE && tmp_chg_cur >= cur){
			return 0;
		}
	}
	cur -= chg_cur;

	return cur;
}

static int get_open_circuit_volt(int mvolts)
{
	int ocv ;
	int resistor = ns115_batt_gauge->resistor_mohm;
	int cur;

	cur = get_about_current(mvolts);
	ocv = mvolts + cur * resistor / 1000;

	return ocv;
}

static int batt_voltage_debouce(int mvolts)
{
	int new_mvolts = mvolts;
	int i, sum;

	if (!mvolts){
		return ns115_chg.batt_mvolts;
	}
	PDBG("votage before Calibration: %d\n", mvolts);

	new_mvolts = get_open_circuit_volt(mvolts);
#ifdef VOLTAGE_DEBOUCE
	if (ns115_chg.batt_status == BATT_STATUS_FAST_CHARGING){
		if (ns115_chg.charger_power_on++ < 3){
			return ns115_chg.batt_mvolts;
		}
		if(ns115_chg.charger_power_on == 4 && 
				ns115_chg.batt_volt_times < NS115_MAX_VOLT_BUF){
			ns115_chg.batt_volt_times = 0;
			ns115_chg.batt_volt_pointer = 0;
		}
	}
	if (ns115_chg.batt_volt_times >= NS115_MAX_VOLT_BUF){
		if (new_mvolts < ns115_chg.batt_mvolts - NS115_VOLT_WINDOWS){
			ns115_chg.batt_right_windows = 0;
			if (++ns115_chg.batt_left_windows > NS115_MAX_VOLT_BUF){
				goto normal;
			}
			return ns115_chg.batt_mvolts;
		}else if (new_mvolts > ns115_chg.batt_mvolts + NS115_VOLT_WINDOWS){
			ns115_chg.batt_left_windows = 0;
			if (++ns115_chg.batt_right_windows > NS115_MAX_VOLT_BUF){
				goto normal;
			}
			return ns115_chg.batt_mvolts;
		}
	}else{
		ns115_chg.batt_volt_times++;
	}
	
	ns115_chg.batt_right_windows = 0;
	ns115_chg.batt_left_windows = 0;
normal:
#else
	if (ns115_chg.batt_volt_times++ >= NS115_MAX_VOLT_BUF){
		ns115_chg.batt_volt_times = NS115_MAX_VOLT_BUF;
	}
#endif
	ns115_chg.batt_volt_buf[ns115_chg.batt_volt_pointer] = new_mvolts;
	if (++ns115_chg.batt_volt_pointer >= NS115_MAX_VOLT_BUF){
		ns115_chg.batt_volt_pointer = 0;
	}
	for (i = 0, sum = 0; i < ns115_chg.batt_volt_times; ++i){
		sum += ns115_chg.batt_volt_buf[i];
	}
	new_mvolts = sum / ns115_chg.batt_volt_times;

	dev_dbg(ns115_chg.dev, "votage after Calibration: %d times:%d\n",
			new_mvolts, ns115_chg.batt_volt_times);

	return new_mvolts;
}

static int get_prop_batt_mvolts(void)
{
	int mvolts;

	if (ns115_batt_gauge && ns115_batt_gauge->get_battery_mvolts){
		mvolts = ns115_batt_gauge->get_battery_mvolts();
		mvolts = batt_voltage_debouce(mvolts);
		ns115_chg.batt_mvolts = mvolts;
		return mvolts;
	}else {
		pr_err("ns115-charger no batt gauge assuming 3.5V\n");
		return NS115_BATT_MISSING_VOLTS;
	}
}

static int get_prop_batt_current(void)
{
	if (ns115_batt_gauge && ns115_batt_gauge->get_battery_current)
		return ns115_batt_gauge->get_battery_current();
	else {
		return -1;
	}
}

static int get_prop_batt_temp(void)
{
	if (ns115_batt_gauge && ns115_batt_gauge->get_battery_temperature)
		return ns115_batt_gauge->get_battery_temperature();
	else {
		pr_debug("ns115-charger no batt gauge assuming 35 deg G\n");
		return NS115_BATT_MISSING_TEMP;
	}
}

static int is_batt_temp_out_of_range(void)
{
	if (ns115_batt_gauge && ns115_batt_gauge->is_batt_temp_out_of_range)
		return ns115_batt_gauge->is_batt_temp_out_of_range();
	else {
		pr_debug("ns115-charger no batt gauge assuming 35 deg G\n");
		return 0;
	}
}

static int ns115_get_batt_vol_cc(int mvolts)
{
	int i;

	for (i = ns115_batt_gauge->table_size -1; i >= 0; --i){
		if (mvolts < (*(ns115_batt_gauge->capacity_table))[i][1]){
			 return (*(ns115_batt_gauge->capacity_table))[i][0];
		}
	}

	return 100;
}

static unsigned long ns115_get_seconds(void)
{
	return jiffies_to_msecs(jiffies) / 1000;
}

static int ns115_batt_init_mAh(void)
{
	int cc, step_cc;
	unsigned long cur_time;

	step_cc = ns115_batt_gauge->table_step;
	cc = ns115_get_batt_vol_cc(get_prop_batt_mvolts());
	ns115_chg.batt_cc = cc;
	ns115_chg.init_mAh = ns115_batt_gauge->max_mAh * cc / 100;
	cur_time = ns115_get_seconds();
	ns115_chg.init_mAh_time = cur_time;
	ns115_chg.init_mAh_time_cache = cur_time;
	ns115_chg.init_mAh_cache = ns115_chg.init_mAh;

	dev_info(ns115_chg.dev, "init_cc: %d%% init_mAh: %dmAh init_time: %ds\n",
			ns115_chg.batt_cc, ns115_chg.init_mAh, (int)ns115_chg.init_mAh_time);

	return 0;
}

static int ns115_get_batt_mAh(int vbat)
{
	int cal_time;
	int cur, cur_mAh;
	unsigned long cur_time;

	cur_time = ns115_get_seconds();
	cal_time = cur_time - ns115_chg.init_mAh_time;
	if (cal_time < 0){
		dev_err(ns115_chg.dev, "seconds: 0x%x error init_time: 0x%x\n",
				(unsigned int)cur_time, (unsigned int)ns115_chg.init_mAh_time);
		ns115_chg.init_mAh_time = cur_time;
		ns115_chg.init_mAh_time_cache = cur_time;
		ns115_chg.init_mAh = ns115_chg.init_mAh_cache;

		return ns115_chg.init_mAh;
	}
	cur = get_about_current(vbat);
	PDBG("current: %dmA\n", cur);

	cur_mAh = cal_time * cur / 3600;
	cur_mAh = ns115_chg.init_mAh - cur_mAh;
	cur_mAh = cur_mAh < 0 ? 0 : cur_mAh;
	ns115_chg.init_mAh_cache = cur_mAh;
	PDBG("batt cur_mAh: %dmAh cal_time: %ds\n",
			cur_mAh, cal_time);

	return cur_mAh;
}

static int ns115_get_capacity(int vbat)
{
	int cur;
	int cc, last_diff_cc, step_cc;
	int cur_mAh, diff_mAh, step_mAh;
	unsigned long cur_time;

	PDBG("*************************\n");
	if (ns115_chg.sys_stat == SYS_STAT_SUSPEND){
		goto out;
	}
	cc = ns115_get_batt_vol_cc(vbat);
	PDBG("voltage: %dmV vol_cc:%d%%\n", vbat, cc);
	cur = get_about_current(vbat);

	last_diff_cc = ns115_chg.batt_cc - cc;
	step_cc = ns115_batt_gauge->table_step;

	cur_mAh = ns115_get_batt_mAh(vbat);
	step_mAh = ns115_batt_gauge->max_mAh * step_cc / 100;
	diff_mAh = cur_mAh - (ns115_batt_gauge->max_mAh * ns115_chg.batt_cc / 100);
	PDBG("diff_mAh: %dmAh step_mAh: %dmAh\n",
			diff_mAh, step_mAh);

	if (cur == 0){
		goto no_change;
	}else if (cur < 0){
		if (ns115_chg.batt_cc > 90 && last_diff_cc > 0){
			if (diff_mAh < step_mAh){
				goto no_change;
			}else{
				goto chg_one_step;
			}
		}else if (diff_mAh < 0 || last_diff_cc > 0){
			goto no_change;
		}
	}else if (cur > 0){
		if (cc == 0){
			if (-diff_mAh > step_mAh / 4){
				goto chg_one_step;
			}else{
				goto no_change;
			}
		}
		if (ns115_chg.batt_cc > 90 && last_diff_cc > 0){
			if (diff_mAh > -step_mAh){
				goto no_change;
			}else{
				goto chg_one_step;
			}
		}else if (diff_mAh > 0 || last_diff_cc < 0){
			goto no_change;
		}
	}
	last_diff_cc = abs(last_diff_cc);
	diff_mAh = abs(diff_mAh);
	if (diff_mAh < step_mAh){
		if (last_diff_cc <= 5){
			goto no_change;
		}else if (last_diff_cc > 5 && last_diff_cc <= 10
				&& diff_mAh > step_mAh / 3 * 2){
			goto chg_one_step;
		}else if (last_diff_cc > 10 && last_diff_cc <= 15
				&& diff_mAh > step_mAh / 2){
			goto chg_one_step;
		}else if (last_diff_cc > 15 && diff_mAh > step_mAh / 3){
			goto chg_one_step;
		}else{
			goto no_change;
		}
	}else{
		if (last_diff_cc == 0){
			if (diff_mAh >= step_mAh + step_mAh / 4){
				goto chg_one_step;
			}else{
				goto no_change;
			}
		}else{
			goto chg_one_step;
		}
	}
chg_one_step:
	PDBG("change one step\n");
	if (cur > 0){
		cc = ns115_chg.batt_cc - step_cc;
	}else{
		cc = ns115_chg.batt_cc + step_cc;
	}
	cur_mAh = ns115_batt_gauge->max_mAh * cc / 100;
	ns115_chg.init_mAh= cur_mAh;
	cur_time = ns115_get_seconds();
	ns115_chg.init_mAh_time = cur_time;
	ns115_chg.init_mAh_time_cache = cur_time;
	ns115_chg.init_mAh_cache = ns115_chg.init_mAh;
	ns115_chg.batt_cc = cc;

no_change:
	if (is_batt_status_charging() && ns115_chg.batt_cc == 100){
		PINFO("capcity 100%%. charge done\n");
		ns115_battery_notify_event(CHG_DONE_EVENT);
	}
	PDBG("battery capacity: %d%%\n", ns115_chg.batt_cc);
	PDBG("*************************\n");
	if (ns115_chg.power_on++ >= 5){
		ns115_chg.power_on = 5;
	}
out:

	return ns115_chg.batt_cc;
}

static int ns115_batt_reinit_mAh(int volt)
{
	unsigned long cur_time;

	ns115_chg.init_mAh = ns115_get_batt_mAh(volt);
	cur_time = ns115_get_seconds();
	ns115_chg.init_mAh_time = cur_time;
	ns115_chg.init_mAh_time_cache = cur_time;

	return 0;
}

static struct power_supply ns115_psy_batt;
static int __ns115_batt_resume_init_mAh(int vbat)
{
	int cur_mAh, cur, step_cc;
	int cal_time;
	int step_mAh, diff_mAh;
	unsigned long cur_time;

	cur = get_about_current(ns115_chg.batt_mvolts);
	cal_time = get_seconds() - ns115_chg.init_mAh_time;
	cur_mAh = ns115_chg.init_mAh - cal_time * cur / 3600;
	PDBG("suspend time: %ds. cur_mAh: %dmAh cur: %dmA\n",
			cal_time, cur_mAh, cur);

	step_cc = ns115_batt_gauge->table_step;
	step_mAh = ns115_batt_gauge->max_mAh * step_cc / 100;
	diff_mAh = cur_mAh - (ns115_batt_gauge->max_mAh * ns115_chg.batt_cc / 100);
	if (abs(diff_mAh) < step_mAh + step_mAh / 4){
		goto out;
	}
	ns115_chg.batt_cc += diff_mAh / step_mAh * step_cc;
	if (ns115_chg.batt_cc < 0){
		ns115_chg.batt_cc = 0;
		cur_mAh = 0;
	}else if (ns115_chg.batt_cc > 100){
		ns115_chg.batt_cc = 100;
		cur_mAh = ns115_batt_gauge->max_mAh;
	}
out:
	ns115_chg.init_mAh = cur_mAh;
	cur_time = ns115_get_seconds();
	ns115_chg.init_mAh_time = cur_time;
	ns115_chg.init_mAh_time_cache = cur_time;
	ns115_chg.init_mAh_cache = ns115_chg.init_mAh;
	if (is_batt_status_charging() && ns115_chg.batt_cc == 100){
		PINFO("capcity 100%%. charge done\n");
		ns115_battery_notify_event(CHG_DONE_EVENT);
	}else{
		power_supply_changed(&ns115_psy_batt);
	}
	PDBG("resume capacity: %d%%\n", ns115_chg.batt_cc);

	return 0;
}

static int ns115_batt_resume_init_mAh(int vbat)
{
	struct ns115_charger_priv *priv;

	priv = ns115_chg.current_chg_priv;
	if (ns115_chg.resume_by_charger && priv){
		if (ns115_chg.resume_by_charger == 1){
			priv->hw_chg_state = CHG_READY_STATE;
			__ns115_batt_resume_init_mAh(vbat);
			priv->hw_chg_state = CHG_CHARGING_STATE;
		}else{
			priv->hw_chg_state = CHG_CHARGING_STATE;
			__ns115_batt_resume_init_mAh(vbat);
			priv->hw_chg_state = CHG_ABSENT_STATE;
			priv = NULL;
		}
		ns115_chg.resume_by_charger = 0;
	}else{
		__ns115_batt_resume_init_mAh(vbat);
	}
	wake_unlock(&ns115_chg.chg_wl);

	return 0;
}

static void ns115_batt_resume_init_work(struct work_struct *work)
{
	int vbat;

	PDBG("%s\n", __func__);
	vbat = get_prop_batt_mvolts();
	if (ns115_chg.sys_stat != SYS_STAT_SUSPEND){
		return;
	}
	ns115_batt_resume_init_mAh(vbat);
	ns115_chg.sys_stat = SYS_STAT_EARLY_SUSPEND;

	return;
}

static int get_prop_batt_capacity(void)
{
	if (ns115_batt_gauge){
		if (ns115_batt_gauge->get_battery_capacity){
			return ns115_batt_gauge->get_battery_capacity(ns115_chg.batt_mvolts);
		}else{
			return ns115_get_capacity(ns115_chg.batt_mvolts);
		}
	}

	return -1;
}

static int get_prop_batt_status(void)
{
	int status = 0;

	if (ns115_batt_gauge && ns115_batt_gauge->get_battery_status) {
		status = ns115_batt_gauge->get_battery_status();
		if (status == POWER_SUPPLY_STATUS_CHARGING ||
			status == POWER_SUPPLY_STATUS_FULL ||
			status == POWER_SUPPLY_STATUS_DISCHARGING)
			return status;
	}

	if (is_batt_status_charging())
		status = POWER_SUPPLY_STATUS_CHARGING;
	else if (ns115_chg.batt_status ==
		 BATT_STATUS_CHARGING_DONE
			 && ns115_chg.current_chg_priv != NULL)
		status = POWER_SUPPLY_STATUS_FULL;
	else
		status = POWER_SUPPLY_STATUS_DISCHARGING;

	return status;
}

#ifdef CONFIG_BATTERY_TPS80032
static int calib_gauge_start_chg(void)
{
	if (ns115_batt_gauge && ns115_batt_gauge->calib_gauge_start_chg) {
		return ns115_batt_gauge->calib_gauge_start_chg();
	}

	return -1;
}

static int calib_gauge_stop_chg(void)
{
	if (ns115_batt_gauge && ns115_batt_gauge->calib_gauge_stop_chg) {
		return ns115_batt_gauge->calib_gauge_stop_chg();
	}

	return -1;
}

static int calib_gauge_chg_done(void)
{
	if (ns115_batt_gauge && ns115_batt_gauge->calib_gauge_chg_done) {
		return ns115_batt_gauge->calib_gauge_chg_done();
	}

	return -1;
}
#endif

static enum power_supply_property ns115_power_props[] = {
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_ONLINE,
};

static char *ns115_power_supplied_to[] = {
	"battery",
};

static int ns115_get_charging_stat(struct ns115_charger_priv *priv)
{
	if (priv->hw_chg->get_charging_stat){
		priv->hw_chg->get_charging_stat(&priv->hw_chg_state);
		return 0;
	}

	return -1;
}

static int ns115_power_get_property(struct power_supply *psy,
				  enum power_supply_property psp,
				  union power_supply_propval *val)
{
	struct ns115_charger_priv *priv;

	priv = container_of(psy, struct ns115_charger_priv, psy);
	switch (psp) {
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = !(priv->hw_chg_state == CHG_ABSENT_STATE);
		break;
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = (priv->hw_chg_state != CHG_ABSENT_STATE);
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static enum power_supply_property ns115_batt_power_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_TEMP,
};

static int ns115_batt_power_get_property(struct power_supply *psy,
				       enum power_supply_property psp,
				       union power_supply_propval *val)
{
	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
		val->intval = get_prop_batt_status();
		break;
	case POWER_SUPPLY_PROP_HEALTH:
		if (ns115_chg.batt_status == BATT_STATUS_TEMP_OUT_OF_RANGE){
			val->intval = POWER_SUPPLY_HEALTH_OVERHEAT;
		}else if (ns115_chg.batt_status == BATT_STATUS_ABSENT){
			val->intval = POWER_SUPPLY_HEALTH_UNKNOWN;
		}else{
			val->intval = POWER_SUPPLY_HEALTH_GOOD;
		}
		break;
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = !(ns115_chg.batt_status == BATT_STATUS_ABSENT);
		break;
	case POWER_SUPPLY_PROP_TECHNOLOGY:
		val->intval = POWER_SUPPLY_TECHNOLOGY_NiMH;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		val->intval = get_prop_batt_mvolts();
		break;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		val->intval = get_prop_batt_current();
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		val->intval = get_prop_batt_capacity();
		break;
	case POWER_SUPPLY_PROP_TEMP:
        val->intval = get_prop_batt_temp();
        break;
	default:
		return -EINVAL;
	}
	return 0;
}

static struct power_supply ns115_psy_batt = {
	.name = "battery",
	.type = POWER_SUPPLY_TYPE_BATTERY,
	.properties = ns115_batt_power_props,
	.num_properties = ARRAY_SIZE(ns115_batt_power_props),
	.get_property = ns115_batt_power_get_property,
};


#ifdef DEBUG
static inline void debug_print(const char *func,
			       struct ns115_charger_priv *hw_chg_priv)
{
	dev_info(ns115_chg.dev,
		"%s current=(%s)(s=%d)(r=%d) new=(%s)(s=%d)(r=%d) batt=%d En\n",
		func,
		ns115_chg.current_chg_priv ? ns115_chg.current_chg_priv->
		hw_chg->name : "none",
		ns115_chg.current_chg_priv ? ns115_chg.
		current_chg_priv->hw_chg_state : -1,
		ns115_chg.current_chg_priv ? ns115_chg.current_chg_priv->
		hw_chg->rating : -1,
		hw_chg_priv ? hw_chg_priv->hw_chg->name : "none",
		hw_chg_priv ? hw_chg_priv->hw_chg_state : -1,
		hw_chg_priv ? hw_chg_priv->hw_chg->rating : -1,
		ns115_chg.batt_status);
}
#else
static inline void debug_print(const char *func,
			       struct ns115_charger_priv *hw_chg_priv)
{
}
#endif

static int ns115_stop_charging(struct ns115_charger_priv *priv)
{
	int ret;

	ret = priv->hw_chg->stop_charging(priv->hw_chg);
#ifdef CONFIG_BATTERY_TPS80032
	calib_gauge_stop_chg();
#endif
#ifdef CHARGING_ALWAYS_WAKE
	if (!ret)
		wake_unlock(&ns115_chg.wl);
#endif

	return ret;
}

/* the best charger has been selected -start charging from current_chg_priv */
static int ns115_start_charging(void)
{
	int ret, charging_type, battery_stat, volt;
	struct ns115_charger_priv *priv;
	int cal_time;

	priv = ns115_chg.current_chg_priv;
	if (ns115_chg.batt_status == BATT_STATUS_ABSENT){
		dev_err(ns115_chg.dev, "%s: battery is absent!\n", __func__);
		return -1;
	}

	volt = get_prop_batt_mvolts();
	if (volt < ns115_chg.pre_chg_mvolts){
		charging_type = CHGING_TYPE_PRE;
		battery_stat = BATT_STATUS_PRE_CHARGING;
	}else{
		charging_type = CHGING_TYPE_FAST;
		battery_stat = BATT_STATUS_FAST_CHARGING;
	}
#ifdef CHARGING_ALWAYS_WAKE
	wake_lock(&ns115_chg.wl);
#endif
	ret = priv->hw_chg->start_charging(priv->hw_chg, charging_type);
	if (ret) {
#ifdef CHARGING_ALWAYS_WAKE
		wake_unlock(&ns115_chg.wl);
#endif
		dev_err(ns115_chg.dev, "%s couldnt start chg error = %d\n",
			priv->hw_chg->name, ret);
	} else{
		if (ns115_chg.power_on < 5){
			cal_time = ns115_get_seconds() - ns115_chg.init_mAh_time;
			dev_info(ns115_chg.dev, "cal_time: %ds\n", cal_time);
			if (cal_time <= 3){
				dev_info(ns115_chg.dev, "the charger plug when capacity init\n");
				priv->hw_chg_state = CHG_CHARGING_STATE;
				ns115_chg.batt_volt_times  = 0;
				ns115_chg.batt_volt_pointer = 0;
				ns115_batt_init_mAh();
			}
		}else{
			if (ns115_chg.sys_stat == SYS_STAT_SUSPEND){
				ns115_chg.resume_by_charger = 1;
				wake_lock(&ns115_chg.chg_wl);
			}else{
				ns115_batt_reinit_mAh(volt);
			}
		}
		priv->hw_chg_state = CHG_CHARGING_STATE;
		ns115_chg.batt_status = battery_stat;
#ifdef CONFIG_BATTERY_TPS80032
		calib_gauge_start_chg();
#endif
	}
#ifdef VOLTAGE_DEBOUCE
	ns115_chg.charger_power_on = 0;
#endif

	power_supply_changed(&ns115_psy_batt);

	return ret;
}

static void handle_charging_done(struct ns115_charger_priv *priv)
{
	if (ns115_chg.current_chg_priv == priv) {
		if (ns115_chg.current_chg_priv->hw_chg_state ==
		    CHG_CHARGING_STATE){
			if (ns115_stop_charging(ns115_chg.current_chg_priv)) {
				dev_err(ns115_chg.dev, "%s couldnt stop chg\n",
					ns115_chg.current_chg_priv->hw_chg->name);
			}
			ns115_batt_reinit_mAh(get_prop_batt_mvolts());
		}
		ns115_chg.current_chg_priv->hw_chg_state = CHG_DONE_STATE;

#ifdef CONFIG_BATTERY_TPS80032
		calib_gauge_chg_done();
#endif
		ns115_chg.batt_status = BATT_STATUS_CHARGING_DONE;
		dev_info(ns115_chg.dev, "%s: stopping safety timer work\n",
				__func__);
		cancel_delayed_work(&ns115_chg.teoc_work);
#ifdef VOLTAGE_DEBOUCE
		ns115_chg.charger_power_on = 0;
#endif

		power_supply_changed(&ns115_psy_batt);
	}
}

static void teoc(struct work_struct *work)
{
	/* we have been charging too long - stop charging */
	dev_info(ns115_chg.dev, "%s: safety timer work expired\n", __func__);
	ns115_battery_notify_event(CHG_DONE_EVENT);
}

static void update_heartbeat(struct work_struct *work)
{
	int volts;
	struct ns115_charger_priv * cur_priv = ns115_chg.current_chg_priv;

	if (cur_priv){
		volts = get_prop_batt_mvolts();
		if (ns115_chg.batt_status == BATT_STATUS_PRE_CHARGING
				&& volts > ns115_chg.pre_chg_mvolts){
			ns115_battery_notify_event(CHG_BATT_BEGIN_FAST_CHARGING);
		}
	}
	if (cur_priv && cur_priv->hw_chg_state == CHG_CHARGING_STATE) {
        if (is_batt_temp_out_of_range()){
            pr_info("the battery temperature is out of range. stop charging!\n");
			ns115_battery_notify_event(CHG_BATT_TEMP_OUTOFRANGE);
        }
	}
	if (cur_priv && ns115_chg.batt_status == BATT_STATUS_TEMP_OUT_OF_RANGE){
        if (!is_batt_temp_out_of_range()){
            pr_info("the battery temperature is OK now. start charging!\n");
			ns115_battery_notify_event(CHG_BATT_TEMP_INRANGE);
        }
    }

	/* notify that the voltage has changed
	 * the read of the capacity will trigger a
	 * voltage read*/
	power_supply_changed(&ns115_psy_batt);

	if (ns115_chg.stop_update) {
		ns115_chg.stop_update = 0;
		return;
	}
	queue_delayed_work(ns115_chg.event_wq_thread,
				&ns115_chg.update_heartbeat_work,
			      round_jiffies_relative(msecs_to_jiffies
						     (ns115_chg.update_time)));

	return;
}

static void __handle_charger_removed(struct ns115_charger_priv
				   *hw_chg_removed, int new_state)
{
	debug_print(__func__, hw_chg_removed);

	if (ns115_chg.current_chg_priv == hw_chg_removed) {
		if (ns115_chg.current_chg_priv->hw_chg_state
						== CHG_CHARGING_STATE) {
			if (ns115_stop_charging(hw_chg_removed)) {
				dev_err(ns115_chg.dev, "%s couldnt stop chg\n",
					ns115_chg.current_chg_priv->hw_chg->name);
			}
		}
		if (ns115_chg.sys_stat == SYS_STAT_SUSPEND){
			ns115_chg.resume_by_charger = 2;
			wake_lock(&ns115_chg.chg_wl);
		}else{
			ns115_batt_reinit_mAh(get_prop_batt_mvolts());
			ns115_chg.current_chg_priv = NULL;
		}
	}

	hw_chg_removed->hw_chg_state = new_state;

	/* if we arent charging stop the safety timer */
	if (is_batt_status_charging()) {
		ns115_chg.batt_status = BATT_STATUS_DISCHARGING;
		dev_info(ns115_chg.dev, "%s: stopping safety timer work\n",
				__func__);
		cancel_delayed_work(&ns115_chg.teoc_work);
	}
}

static void handle_charger_removed(struct ns115_charger_priv
				   *hw_chg_removed, int new_state)
{
	__handle_charger_removed(hw_chg_removed, new_state);
	power_supply_changed(&hw_chg_removed->psy);
	power_supply_changed(&ns115_psy_batt);
}

/* set the charger state to READY before calling this */
static void handle_charger_ready(struct ns115_charger_priv *hw_chg_priv)
{
	debug_print(__func__, hw_chg_priv);

	if (ns115_chg.current_chg_priv){
		__handle_charger_removed(ns115_chg.current_chg_priv, CHG_ABSENT_STATE);
	}
	ns115_chg.current_chg_priv = hw_chg_priv;
	dev_info(ns115_chg.dev,
		 "%s: best charger = %s\n", __func__,
		 ns115_chg.current_chg_priv->hw_chg->name);

	/* start charging from the new charger */
	if (!ns115_start_charging()) {
		/* if we simply switched chg continue with teoc timer
		 * else we update the batt state and set the teoc
		 * timer */
		if (!is_batt_status_charging()) {
			dev_info(ns115_chg.dev,
			       "%s: starting safety timer\n", __func__);
			queue_delayed_work(ns115_chg.event_wq_thread,
						&ns115_chg.teoc_work,
					      round_jiffies_relative
					      (msecs_to_jiffies
					       (ns115_chg.safety_time)));
		}
	} 
	power_supply_changed(&hw_chg_priv->psy);
}

void ns115_charger_plug(enum ns115_charger_type type)
{
	struct ns115_charger_priv *hw_chg_priv = NULL;
	struct ns115_charger_priv *better = NULL;

	mutex_lock(&ns115_chg.status_lock);
	if (!ns115_batt_gauge){
		dev_err(ns115_chg.dev, "%s: there is no battery registered!\n", __func__);
		goto out;
	}
	if (ns115_chg.count_chargers <= 0){
		dev_err(ns115_chg.dev, "%s: there is no charger registered!\n", __func__);
		goto out;
	}
	list_for_each_entry(hw_chg_priv, &ns115_chg.ns115_chargers, list) {
		if(hw_chg_priv->hw_chg->type == type){
			better = hw_chg_priv;
			break;
		}
	}
	if(better){
		handle_charger_ready(better);
		goto out;
	}
	dev_err(ns115_chg.dev, "%s: can't find the charger of type: %d\n", __func__, type);

out:
	mutex_unlock(&ns115_chg.status_lock);
	return;
}
EXPORT_SYMBOL(ns115_charger_plug);

void ns115_charger_unplug(void)
{
	ns115_battery_notify_event(CHG_UNPLUG_EVENT);
}
EXPORT_SYMBOL(ns115_charger_unplug);

static void handle_event(int event)
{
	struct ns115_charger_priv *cur_priv = ns115_chg.current_chg_priv;

	if (event < CHG_BATT_STATUS_CHANGED && !cur_priv){
		return;
	};
	if (event >= CHG_BATT_STATUS_CHANGED && !ns115_batt_gauge){
		return;
	}
	switch (event){
		case CHG_PLUG_EVENT:
			break;
		case CHG_UNPLUG_EVENT:
			handle_charger_removed(cur_priv, CHG_ABSENT_STATE);
			break;
		case CHG_DONE_EVENT:
			handle_charging_done(cur_priv);
			break;
		case CHG_ERROR_EVENT:
			if (ns115_stop_charging(cur_priv)) {
				dev_err(ns115_chg.dev, "%s couldnt stop chg\n", cur_priv->hw_chg->name);
			}else{
			    ns115_chg.batt_status = BATT_STATUS_ABSENT;
		        ns115_chg.current_chg_priv->hw_chg_state = CHG_READY_STATE;
			}
			break;
		case CHG_BATT_BEGIN_PRE_CHARGING:
			cur_priv->hw_chg->start_charging(cur_priv->hw_chg, CHGING_TYPE_PRE);
			ns115_chg.batt_status = BATT_STATUS_PRE_CHARGING;
			break;
		case CHG_BATT_BEGIN_FAST_CHARGING:
			cur_priv->hw_chg->start_charging(cur_priv->hw_chg, CHGING_TYPE_FAST);
			ns115_chg.batt_status = BATT_STATUS_FAST_CHARGING;
			break;
		case CHG_BATT_TEMP_OUTOFRANGE:
			if (ns115_stop_charging(cur_priv)) {
				dev_err(ns115_chg.dev, "%s couldnt stop chg\n", cur_priv->hw_chg->name);
			}else{
				ns115_batt_reinit_mAh(get_prop_batt_mvolts());
			    ns115_chg.batt_status = BATT_STATUS_TEMP_OUT_OF_RANGE;
		        ns115_chg.current_chg_priv->hw_chg_state = CHG_READY_STATE;
            }
			break;
		case CHG_BATT_TEMP_INRANGE:
            if (!ns115_start_charging()){
			    ns115_chg.batt_status = BATT_STATUS_FAST_CHARGING;
		        cur_priv->hw_chg_state = CHG_CHARGING_STATE;
            }
			break;
		case CHG_BATT_REMOVED:
			if (ns115_stop_charging(cur_priv)) {
				dev_err(ns115_chg.dev, "%s couldnt stop chg\n", cur_priv->hw_chg->name);
			}else{
			    ns115_chg.batt_status = BATT_STATUS_ABSENT;
		        ns115_chg.current_chg_priv->hw_chg_state = CHG_READY_STATE;
			}
			break;
		case CHG_BATT_STATUS_CHANGED:
			power_supply_changed(&ns115_psy_batt);
			break;
		default:
			dev_err(ns115_chg.dev, "the %d event isn't defined!\n", event);
	}
	return;
}

static int ns115_dequeue_event(int *event)
{
	unsigned long flags;

	spin_lock_irqsave(&ns115_chg.queue_lock, flags);
	if (ns115_chg.queue_count == 0) {
		spin_unlock_irqrestore(&ns115_chg.queue_lock, flags);
		return -EINVAL;
	}
	*event = ns115_chg.event_queue[ns115_chg.head];
	ns115_chg.head = (ns115_chg.head + 1) % NS115_CHG_MAX_EVENTS;
	pr_debug("%s dequeueing %d\n", __func__, *event);
	ns115_chg.queue_count--;
	spin_unlock_irqrestore(&ns115_chg.queue_lock, flags);

	return 0;
}

static int ns115_enqueue_event(enum ns115_battery_event event)
{
	unsigned long flags;

	spin_lock_irqsave(&ns115_chg.queue_lock, flags);
	if (ns115_chg.queue_count == NS115_CHG_MAX_EVENTS) {
		spin_unlock_irqrestore(&ns115_chg.queue_lock, flags);
		pr_err("%s: queue full cannot enqueue %d\n",
				__func__, event);
		return -EAGAIN;
	}
	pr_debug("%s queueing %d\n", __func__, event);
	ns115_chg.event_queue[ns115_chg.tail] = event;
	ns115_chg.tail = (ns115_chg.tail + 1) % NS115_CHG_MAX_EVENTS;
	ns115_chg.queue_count++;
	spin_unlock_irqrestore(&ns115_chg.queue_lock, flags);

	return 0;
}

static void process_events(struct work_struct *work)
{
	int event;
	int rc;

	do {
		rc = ns115_dequeue_event(&event);
		if (!rc){
			mutex_lock(&ns115_chg.status_lock);
			handle_event(event);
			mutex_unlock(&ns115_chg.status_lock);
		}
	} while (!rc);
}

int ns115_battery_notify_event(enum ns115_battery_event event)
{
	ns115_enqueue_event(event);
	queue_work(ns115_chg.event_wq_thread, &ns115_chg.queue_work);
	return 0;
}
EXPORT_SYMBOL(ns115_battery_notify_event);

static int __init determine_initial_batt_status(void)
{
	int rc;

	ns115_chg.batt_volt_times  = 0;
	ns115_chg.batt_volt_pointer = 0;
	ns115_chg.sys_stat = SYS_STAT_NORMAL;
	ns115_chg.batt_status = BATT_STATUS_DISCHARGING;
	ns115_chg.pre_chg_mvolts = ns115_batt_gauge->pre_chg_mvolts;
	ns115_chg.full_mvolts = ns115_batt_gauge->full_mvolts;

	if (ns115_chg.pre_chg_mvolts == 0){
		ns115_chg.pre_chg_mvolts = NS115_BATT_PRE_CHG_MVOLTS;
	}
	if (ns115_chg.full_mvolts == 0){
		ns115_chg.full_mvolts = NS115_BATT_FULL_MVOLTS;
	}

	ns115_batt_init_mAh();
	ns115_chg.power_on = 0;
	ns115_chg.resume_by_charger = 0;

	rc = power_supply_register(ns115_chg.dev, &ns115_psy_batt);
	if (rc < 0) {
		dev_err(ns115_chg.dev, "%s: power_supply_register failed"
			" rc=%d\n", __func__, rc);
		return rc;
	}

	/* start updaing the battery powersupply every ns115_chg.update_time
	 * milliseconds */
	queue_delayed_work(ns115_chg.event_wq_thread,
				&ns115_chg.update_heartbeat_work,
			      round_jiffies_relative(msecs_to_jiffies
						     (ns115_chg.update_time)));

	pr_debug("%s:OK batt_status=%d\n", __func__, ns115_chg.batt_status);
	return 0;
}

static int __devinit ns115_battery_probe(struct platform_device *pdev)
{
	ns115_chg.dev = &pdev->dev;
	if (pdev->dev.platform_data) {
		unsigned int milli_secs;

		struct ns115_battery_platform_data *pdata =
		    (struct ns115_battery_platform_data *)pdev->dev.platform_data;

		milli_secs = pdata->safety_time * 60 * MSEC_PER_SEC;
		if (milli_secs > jiffies_to_msecs(MAX_JIFFY_OFFSET)) {
			dev_warn(&pdev->dev, "%s: safety time too large"
				 "%dms\n", __func__, milli_secs);
			milli_secs = jiffies_to_msecs(MAX_JIFFY_OFFSET);
		}
		ns115_chg.safety_time = milli_secs;

		milli_secs = pdata->update_time * MSEC_PER_SEC;
		if (milli_secs > jiffies_to_msecs(MAX_JIFFY_OFFSET)) {
			dev_warn(&pdev->dev, "%s: safety time too large"
				 "%dms\n", __func__, milli_secs);
			milli_secs = jiffies_to_msecs(MAX_JIFFY_OFFSET);
		}
		ns115_chg.update_time = milli_secs;
	}
	if (ns115_chg.safety_time == 0){
		ns115_chg.safety_time = CHARGING_TEOC_MS;
	}
	if (ns115_chg.update_time == 0){
		ns115_chg.update_time = UPDATE_TIME_MS;
	}

	mutex_init(&ns115_chg.status_lock);
	INIT_DELAYED_WORK(&ns115_chg.teoc_work, teoc);
	INIT_DELAYED_WORK(&ns115_chg.batt_resume_init_work, ns115_batt_resume_init_work);
	INIT_DELAYED_WORK(&ns115_chg.update_heartbeat_work, update_heartbeat);

#ifdef CHARGING_ALWAYS_WAKE
	wake_lock_init(&ns115_chg.wl, WAKE_LOCK_SUSPEND, "ns115_battery");
#endif
	wake_lock_init(&ns115_chg.chg_wl, WAKE_LOCK_SUSPEND, "ns115_batt_chg");
	dev_info(ns115_chg.dev, "%s is OK!\n", __func__);

	return 0;
}

static int __devexit ns115_battery_remove(struct platform_device *pdev)
{
#ifdef CHARGING_ALWAYS_WAKE
	wake_lock_destroy(&ns115_chg.wl);
#endif
	wake_lock_destroy(&ns115_chg.chg_wl);
	mutex_destroy(&ns115_chg.status_lock);
	return 0;
}

int ns115_charger_register(struct ns115_charger *hw_chg)
{
	struct ns115_charger_priv *priv;
	int rc = 0;

	if (!ns115_chg.inited) {
		pr_err("%s: ns115_chg is NULL,Too early to register\n", __func__);
		return -EAGAIN;
	}

	if (hw_chg->start_charging == NULL
		|| hw_chg->stop_charging == NULL
		|| hw_chg->name == NULL){
		pr_err("%s: invalid hw_chg\n", __func__);
		return -EINVAL;
	}

	priv = kzalloc(sizeof *priv, GFP_KERNEL);
	if (priv == NULL) {
		dev_err(ns115_chg.dev, "%s kzalloc failed\n", __func__);
		return -ENOMEM;
	}

	priv->psy.name = hw_chg->name;
	if (hw_chg->type == CHG_TYPE_USB)
		priv->psy.type = POWER_SUPPLY_TYPE_USB;
    else if (hw_chg->type == CHG_TYPE_DOCK)
		priv->psy.type = POWER_SUPPLY_TYPE_UPS;
	else
		priv->psy.type = POWER_SUPPLY_TYPE_MAINS;

	priv->psy.supplied_to = ns115_power_supplied_to;
	priv->psy.num_supplicants = ARRAY_SIZE(ns115_power_supplied_to);
	priv->psy.properties = ns115_power_props;
	priv->psy.num_properties = ARRAY_SIZE(ns115_power_props);
	priv->psy.get_property = ns115_power_get_property;

	rc = power_supply_register(NULL, &priv->psy);
	if (rc) {
		dev_err(ns115_chg.dev, "%s power_supply_register failed\n",
			__func__);
		goto out;
	}

	priv->hw_chg = hw_chg;
	priv->hw_chg_state = CHG_ABSENT_STATE;
	INIT_LIST_HEAD(&priv->list);
	mutex_lock(&ns115_chg.ns115_chargers_lock);
	list_add_tail(&priv->list, &ns115_chg.ns115_chargers);
	mutex_unlock(&ns115_chg.ns115_chargers_lock);
	hw_chg->charger_private = (void *)priv;
	ns115_chg.count_chargers++;

    dev_info(ns115_chg.dev, "%s: %s\n", __func__, hw_chg->name);
	return 0;

out:
	kfree(priv);
	return rc;
}
EXPORT_SYMBOL(ns115_charger_register);

int ns115_charger_unregister(struct ns115_charger *hw_chg)
{
	struct ns115_charger_priv *priv;

	priv = (struct ns115_charger_priv *)(hw_chg->charger_private);
	mutex_lock(&ns115_chg.ns115_chargers_lock);
	list_del(&priv->list);
	mutex_unlock(&ns115_chg.ns115_chargers_lock);
	power_supply_unregister(&priv->psy);
	kfree(priv);
	ns115_chg.count_chargers--;
	return 0;
}
EXPORT_SYMBOL(ns115_charger_unregister);

int ns115_battery_gauge_register(struct ns115_battery_gauge *batt_gauge)
{
	if (!ns115_chg.inited) {
		pr_err("%s: ns115_chg is NULL,Too early to register\n", __func__);
		return -EAGAIN;
	}

	if (ns115_batt_gauge) {
		return -EAGAIN;
		pr_err("ns115-charger %s multiple battery gauge called\n",
								__func__);
	} else {
		ns115_batt_gauge = batt_gauge;
		return determine_initial_batt_status();
	}
}
EXPORT_SYMBOL(ns115_battery_gauge_register);

int ns115_battery_gauge_unregister(struct ns115_battery_gauge *batt_gauge)
{
	power_supply_unregister(&ns115_psy_batt);
	ns115_batt_gauge = NULL;

	return 0;
}
EXPORT_SYMBOL(ns115_battery_gauge_unregister);


static int ns115_battery_suspend(struct device *dev)
{
	unsigned long cur_time;
	int cur_mAh, step_mAh, diff_mAh;
	int step_cc;

	dev_dbg(ns115_chg.dev, "%s suspended\n", __func__);
	ns115_chg.stop_update = 1;
	cancel_delayed_work(&ns115_chg.update_heartbeat_work);

	step_cc = ns115_batt_gauge->table_step;
	step_mAh = ns115_batt_gauge->max_mAh * step_cc / 100;
	cur_mAh = ns115_get_batt_mAh(ns115_chg.batt_mvolts);
	diff_mAh = cur_mAh - (ns115_batt_gauge->max_mAh * ns115_chg.batt_cc / 100);
	if (abs(diff_mAh) > step_mAh + step_mAh / 4){
		if (diff_mAh > 0){
			cur_mAh = ns115_batt_gauge->max_mAh * ns115_chg.batt_cc / 100
				+ (step_mAh + step_mAh / 4);
		}else{
			cur_mAh = ns115_batt_gauge->max_mAh * ns115_chg.batt_cc / 100
				- (step_mAh + step_mAh / 4);
		}
	}
	ns115_chg.init_mAh = cur_mAh;
	PDBG("suspend battery cur_mAh: %dmAh\n", cur_mAh);

	cur_time = get_seconds();
	ns115_chg.init_mAh_time = cur_time;
	ns115_chg.init_mAh_time_cache = cur_time;

	ns115_chg.sys_stat = SYS_STAT_SUSPEND;

	return 0;
}

static int ns115_battery_resume(struct device *dev)
{
	dev_dbg(ns115_chg.dev, "%s resumed\n", __func__);
	ns115_chg.stop_update = 0;
	ns115_chg.batt_volt_times  = 0;
	ns115_chg.batt_volt_pointer = 0;

	/* start updaing the battery powersupply every ns115_chg.update_time
	 * milliseconds */
	queue_delayed_work(ns115_chg.event_wq_thread,
				&ns115_chg.batt_resume_init_work, msecs_to_jiffies(RESUME_WAIT_TIME));
	queue_delayed_work(ns115_chg.event_wq_thread,
				&ns115_chg.update_heartbeat_work,
			      round_jiffies_relative(msecs_to_jiffies
						     (ns115_chg.update_time)));
	return 0;
}

static SIMPLE_DEV_PM_OPS(ns115_battery_pm_ops,
		ns115_battery_suspend, ns115_battery_resume);

static struct platform_driver ns115_battery_driver = {
	.probe = ns115_battery_probe,
	.remove = __devexit_p(ns115_battery_remove),
	.driver = {
		   .name = "ns115_battery",
		   .owner = THIS_MODULE,
		   .pm = &ns115_battery_pm_ops,
	},
};

#ifdef CONFIG_HAS_EARLYSUSPEND
static void ns115_batt_early_suspend(struct early_suspend *h)
{
	dev_dbg(ns115_chg.dev, "%s\n", __func__);
	ns115_batt_reinit_mAh(get_prop_batt_mvolts());
	ns115_chg.sys_stat = SYS_STAT_EARLY_SUSPEND;
}
static void ns115_batt_late_resume(struct early_suspend *h)
{
	int vbat;

	PDBG("%s\n", __func__);
	vbat = get_prop_batt_mvolts();
	if (ns115_chg.sys_stat == SYS_STAT_EARLY_SUSPEND){
		ns115_batt_reinit_mAh(vbat);
	}else{
		ns115_batt_resume_init_mAh(vbat);
	}
	ns115_chg.sys_stat = SYS_STAT_NORMAL;
}
static struct early_suspend ns115_batt_early_suspend_desc = {
	.level = 1,
	.suspend = ns115_batt_early_suspend,
	.resume = ns115_batt_late_resume,
};
#endif
static int __init ns115_battery_init(void)
{
	int rc;

	INIT_LIST_HEAD(&ns115_chg.ns115_chargers);
	ns115_chg.count_chargers = 0;
	ns115_chg.batt_status = BATT_STATUS_ABSENT;
	mutex_init(&ns115_chg.ns115_chargers_lock);

	ns115_chg.tail = 0;
	ns115_chg.head = 0;
	spin_lock_init(&ns115_chg.queue_lock);
	ns115_chg.queue_count = 0;
	INIT_WORK(&ns115_chg.queue_work, process_events);
	ns115_chg.event_wq_thread = create_workqueue("ns115_battery_eventd");
	if (!ns115_chg.event_wq_thread) {
		rc = -ENOMEM;
		goto out;
	}
	rc = platform_driver_register(&ns115_battery_driver);
	if (rc < 0) {
		pr_err("%s: FAIL: platform_driver_register. rc = %d\n",
		       __func__, rc);
		goto destroy_wq_thread;
	}
#ifdef CONFIG_HAS_EARLYSUSPEND
	register_early_suspend(&ns115_batt_early_suspend_desc);
#endif

	ns115_chg.inited = 1;
	return 0;

destroy_wq_thread:
	destroy_workqueue(ns115_chg.event_wq_thread);
out:
	return rc;
}

static void __exit ns115_battery_exit(void)
{
#ifdef CONFIG_HAS_EARLYSUSPEND
	unregister_early_suspend(&ns115_batt_early_suspend_desc);
#endif
	flush_workqueue(ns115_chg.event_wq_thread);
	destroy_workqueue(ns115_chg.event_wq_thread);
	platform_driver_unregister(&ns115_battery_driver);
}

fs_initcall(ns115_battery_init);
module_exit(ns115_battery_exit);

MODULE_AUTHOR("Wu Jianguo<jianguo.wu@nufront.com>");
MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("Battery driver for ns115 chipsets.");
MODULE_VERSION("1.0");
