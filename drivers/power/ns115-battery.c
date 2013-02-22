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
#include <linux/init.h>
#include "ns115_battery_parameter.h"

#if defined(CONFIG_HAS_EARLYSUSPEND) && defined(CONFIG_NS115_ADC_GAUGE)
#include <linux/earlysuspend.h>
#endif

#ifndef CONFIG_NS115_CHG_SUSPEND
#define CHARGING_ALWAYS_WAKE
#endif
#ifdef CHARGING_ALWAYS_WAKE
#include <linux/wakelock.h>
#endif

#define CHARGING_TEOC_MS		9000000
#define UPDATE_TIME_MS			60000
#define RESUME_CHECK_PERIOD_MS		60000

#define NS115_BATT_MISSING_VOLTS 3500
#define NS115_BATT_MISSING_TEMP  35
#define NS115_BATT_PRE_CHG_MVOLTS 3100
#define NS115_MAX_VOLT_BUF		10

#define NS115_CHG_MAX_EVENTS	16
#define RESUME_WAIT_TIME	800 //ms
#define RESISTOR_BUF_LEN	5

#define SAVE_GAPACITY_TIME	(2*60*1000) //2Minute
#define GET_SAVE_CAP_TIME	(10 * 1000)

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
#define PERR(format,...)	\
	dev_err(ns115_chg.dev, format, ##__VA_ARGS__)

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
	struct mutex status_lock;
	struct ns115_charger_priv *current_chg_priv;

	int event_queue[NS115_CHG_MAX_EVENTS];
	int tail;
	int head;
	spinlock_t queue_lock;
	int queue_count;
	struct work_struct queue_work;
	struct workqueue_struct *event_wq_thread;
	enum ns115_battery_status batt_status;
#ifdef CHARGING_ALWAYS_WAKE
	struct wake_lock wl;
#endif
	int	batt_mvolts;
	int pre_chg_mvolts;
#ifdef CONFIG_NS115_ADC_GAUGE
	int batt_cc;
	int get_cap_record;
	int resistor_mohm;
	int resistor_buf[RESISTOR_BUF_LEN];
	int resistor_pointer;
	int resistor_times;
	int ocv_less_vol;
	int ocv_more_vol;
	int max_chg_current;

	struct mutex vol_lock;
	int	batt_volt_buf[NS115_MAX_VOLT_BUF];
	int batt_volt_pointer;
	int batt_volt_times;
	int power_on;
	int i_term_times;
	int resume_by_charger;
	int init_mAs;
	int init_mAs_cache;
#ifdef CONFIG_WAKELOCK
	struct wake_lock chg_wl;
#endif
	enum ns115_system_stat sys_stat;
	unsigned long init_mAs_time;
	unsigned long init_mAs_time_cache;
	struct work_struct resistor_work;
	struct delayed_work save_capacity_work;
	struct delayed_work batt_resume_init_work;
#endif
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

#ifdef CONFIG_NS115_ADC_GAUGE
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
	int resistor = ns115_chg.resistor_mohm;
	int cur;

	cur = get_about_current(mvolts);
	ocv = mvolts + cur * resistor / 1000;

	return ocv;
}

static int get_batt_average_voltage(int mvolts)
{
	int new_mvolts = mvolts;
	int i, sum;

	if (!mvolts){
		return ns115_chg.batt_mvolts;
	}
	PDBG("votage before Calibration: %d\n", mvolts);

	mutex_lock(&ns115_chg.vol_lock);
	if (ns115_chg.batt_volt_times++ >= NS115_MAX_VOLT_BUF){
		ns115_chg.batt_volt_times = NS115_MAX_VOLT_BUF;
	}
	ns115_chg.batt_volt_buf[ns115_chg.batt_volt_pointer] = new_mvolts;
	if (++ns115_chg.batt_volt_pointer >= NS115_MAX_VOLT_BUF){
		ns115_chg.batt_volt_pointer = 0;
	}
	mutex_unlock(&ns115_chg.vol_lock);

	for (i = 0, sum = 0; i < ns115_chg.batt_volt_times; ++i){
		sum += ns115_chg.batt_volt_buf[i];
	}
	new_mvolts = sum / ns115_chg.batt_volt_times;

	dev_dbg(ns115_chg.dev, "votage after Calibration: %d times:%d\n",
			new_mvolts, ns115_chg.batt_volt_times);

	return new_mvolts;
}
#endif

static int get_prop_batt_mvolts(void)
{
	int mvolts;

	if (ns115_batt_gauge && ns115_batt_gauge->get_battery_mvolts){
		mvolts = ns115_batt_gauge->get_battery_mvolts();
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

#ifdef CONFIG_NS115_ADC_GAUGE
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

static int ns115_get_batt_cc_vol(int cc)
{
	int i;
	int vol=0;
	for (i =  0; i < ns115_batt_gauge->table_size; ++i){
		if (cc >= (*(ns115_batt_gauge->capacity_table))[i][0]){
			 vol = (*(ns115_batt_gauge->capacity_table))[i][1];
			 break;
		}
	}
	if(!vol)
		return 3900;
	return vol;
}

static unsigned long ns115_get_seconds(void)
{
	return jiffies_to_msecs(jiffies) / 1000;
}

int ns115_calbt_mah_full_vol(int full_mAh, int full_vol)
{
	int vol_offset;
	int i;

	ns115_batt_gauge->max_mAh = full_mAh;
	PINFO("calibate battery max_mAh: %d\n", full_mAh);
	(*(ns115_batt_gauge->capacity_table))[0][1] = full_vol;
	PINFO("calibate capacity table[100:%d]\n", full_vol);

	for (i = 1; (*(ns115_batt_gauge->capacity_table))[i - 1][1] <=
			(*(ns115_batt_gauge->capacity_table))[i][1]; ++i){

		vol_offset = (*(ns115_batt_gauge->capacity_table))[i][1]
			- (*(ns115_batt_gauge->capacity_table))[i + 1][1];
		(*(ns115_batt_gauge->capacity_table))[i][1] =
			(*(ns115_batt_gauge->capacity_table))[i - 1][1] - vol_offset / 2;
		PINFO("calibate capacity table[%d:%d]\n",
				(*(ns115_batt_gauge->capacity_table))[i][0],
				(*(ns115_batt_gauge->capacity_table))[i][1]);
	}

	return 0;
}

static int bootargs_cc = -1;
static int get_batt_bootargs(char *str)
{
	if (str){
		bootargs_cc = simple_strtol(str, NULL, 10);
	}else{
		bootargs_cc = -1;
	}

	return 1;
}
__setup("batt=", get_batt_bootargs);

static int fixed_capacity_from_record(battery_parameter_t *record)
{
	int record_cc, cur_cc;
	int vol, ocv;
	unsigned long cur_time;
	int check_no;

	if(!record){
		goto err;
	}
	if (record->version != BAT_PARA_VERSION){
		PERR("%s: battery_parameter version error!\n", __func__);
		goto err;
	}
	check_no = record->capacity ^ record->resistor
		^ record->max_mAh ^ record->full_vol;
	if (record->check_no != check_no){
		PERR("%s: the check numbers is error!\n", __func__);
		goto err;
	}
	PINFO("%s: max_mAh: %d full_vol: %d\n", __func__,
			record->max_mAh, record->full_vol);
	ns115_calbt_mah_full_vol(record->max_mAh, record->full_vol);

	PINFO("%s: resistor: %dmohm\n", __func__, record->resistor);
	ns115_chg.resistor_mohm = record->resistor;
	ns115_chg.resistor_buf[ns115_chg.resistor_pointer] = record->resistor;
	if (++ns115_chg.resistor_pointer >= RESISTOR_BUF_LEN){
		ns115_chg.resistor_pointer = 0;
	}
	if (++ns115_chg.resistor_times >= RESISTOR_BUF_LEN){
		ns115_chg.resistor_times = RESISTOR_BUF_LEN;
	}
	PINFO("bootargs: batt=%d\n", bootargs_cc);
	if (bootargs_cc > 0){
		goto done;
	}
	record_cc = record->capacity;
	vol = get_prop_batt_mvolts();
	ocv = get_open_circuit_volt(vol);
	cur_cc = ns115_get_batt_vol_cc(ocv);
	if (abs(record_cc - cur_cc) >= 25){
		goto err;
	}

	ns115_chg.batt_cc = record_cc;
	ns115_chg.init_mAs = ns115_batt_gauge->max_mAh * record_cc * 36;
	cur_time = ns115_get_seconds();
	ns115_chg.init_mAs_time = cur_time;
	ns115_chg.init_mAs_time_cache = cur_time;
	ns115_chg.init_mAs_cache = ns115_chg.init_mAs;

	PINFO("%s: init_cc: %d%% init_mAh: %dmAh init_time: %ds\n",
			__func__, ns115_chg.batt_cc, ns115_chg.init_mAs / 3600,
			(int)ns115_chg.init_mAs_time);

done:
	return 0;
err:
	PERR("%s: don't use the record battery parameter!\n", __func__);
	return -1;
}

static int ns115_save_capacity(void)
{
	battery_parameter_t para;

	para.version = BAT_PARA_VERSION;
	para.capacity = ns115_chg.batt_cc;
	para.resistor = ns115_chg.resistor_mohm;
	para.max_mAh = ns115_batt_gauge->max_mAh;
	para.full_vol = (*(ns115_batt_gauge->capacity_table))[0][1];

	para.check_no = para.capacity ^ para.resistor
		^ para.max_mAh ^ para.full_vol;
	if(ns115_battery_set_config(&para) < 0){
		PDBG("fatal error:%s:set config error!!\n", __func__);
		return 0;
	}
	PDBG("%s:save resistor[%dmohm] capcacity[%d%%]!\n",
			__func__, para.resistor, para.capacity);
	PDBG("%s: save max_mAh[%dmAh] full_vol[%dmV]\n",
			__func__, para.max_mAh, para.full_vol);

	return 0;
}

static void ns115_save_capacity_work(struct work_struct *work)
{
	battery_parameter_t para;

	if (!ns115_chg.get_cap_record){
		ns115_chg.get_cap_record = 1;
		if (ns115_battery_get_config(&para) < 0){
			PINFO("get config failed!\n");
			goto err;
		}
		if (fixed_capacity_from_record(&para) < 0){
			PINFO("fixed capacity form record failed!\n");
			goto err;
		}
	}else{
		ns115_save_capacity();
	}
err:
	queue_delayed_work(ns115_chg.event_wq_thread,
				&ns115_chg.save_capacity_work,
				msecs_to_jiffies(SAVE_GAPACITY_TIME));

	return;
}

static int ns115_batt_init_mAs(void)
{
	int cc, vol;
	unsigned long cur_time;

	PINFO("bootargs: batt=%d\n", bootargs_cc);
	if (bootargs_cc > 0){
		cc = bootargs_cc;
		goto done;
	}
	vol = get_prop_batt_mvolts();
	vol = get_open_circuit_volt(vol);
	cc = ns115_get_batt_vol_cc(vol);

done:
	ns115_chg.batt_cc = cc;
	ns115_chg.init_mAs = ns115_batt_gauge->max_mAh * cc * 36;
	cur_time = ns115_get_seconds();
	ns115_chg.init_mAs_time = cur_time;
	ns115_chg.init_mAs_time_cache = cur_time;
	ns115_chg.init_mAs_cache = ns115_chg.init_mAs;

	dev_info(ns115_chg.dev, "init_cc: %d%% init_mAs: %dmAh init_time: %ds\n",
			ns115_chg.batt_cc, ns115_chg.init_mAs / 3600, (int)ns115_chg.init_mAs_time);

	return 0;
}

static int ns115_calculate_resistor(void)
{
	int resistor_sum = 0;
	int resistor = 0;
	int diff_vol = 0;
	int max_diff_vol = 0;
	int i, j;
	int vol_buf[10];
	int bef_vol_len;
	struct ns115_charger_priv *priv;

	priv = ns115_chg.current_chg_priv;
	if (ns115_chg.batt_cc > 90){
		goto out;
	}
	if (!priv || priv->hw_chg->chg_current != ns115_chg.max_chg_current){
		goto out;
	}
	mutex_lock(&ns115_chg.vol_lock);
	bef_vol_len = NS115_MAX_VOLT_BUF > ns115_chg.batt_volt_times ?
		ns115_chg.batt_volt_times : NS115_MAX_VOLT_BUF;

	for (j = 0, i = ns115_chg.batt_volt_pointer; i < bef_vol_len && j < 5; ++i, ++j){
		vol_buf[j] = ns115_chg.batt_volt_buf[i];
		PDBG("vol_buf[%d] = %dmV. i: %d\n", j, vol_buf[j], i);
	}
	for (i = 0; i < ns115_chg.batt_volt_pointer && j < 5; ++i, ++j){
		vol_buf[j] = ns115_chg.batt_volt_buf[i];
		PDBG("vol_buf[%d] = %dmV. i: %d\n", j, vol_buf[j], i);
	}
	mutex_unlock(&ns115_chg.vol_lock);

	do {
		msleep(1000);
		vol_buf[j] = get_prop_batt_mvolts();
		PDBG("vol_buf[%d] = %dmV\n", j, vol_buf[j]);
	}while (++j < 10);

	for (i = 1; i < 10; ++i){
		diff_vol = vol_buf[i] - vol_buf[i - 1];
		if (diff_vol > max_diff_vol){
			max_diff_vol = diff_vol;
		}
	}
	resistor = max_diff_vol * 1000 / ns115_chg.max_chg_current;
	PINFO("%s: max_diff_vol: %d", __func__, max_diff_vol);
	if (resistor < ns115_chg.resistor_mohm / 2){
		PINFO("%s: %d is too small!\n", __func__, resistor);
		goto out;
	}

	ns115_chg.resistor_buf[ns115_chg.resistor_pointer] = resistor;
	if (++ns115_chg.resistor_pointer >= RESISTOR_BUF_LEN){
		ns115_chg.resistor_pointer = 0;
	}
	if (++ns115_chg.resistor_times >= RESISTOR_BUF_LEN){
		ns115_chg.resistor_times = RESISTOR_BUF_LEN;
	}
	for (i = 0; i < ns115_chg.resistor_times; ++i){
		resistor_sum += ns115_chg.resistor_buf[i];
	}
	ns115_chg.resistor_mohm = resistor_sum / i;
	PINFO("%s: %dmohm average: %d\n", __func__, resistor, ns115_chg.resistor_mohm);
	ns115_save_capacity();

out:
	return 0;
}

static void ns115_resistor_work(struct work_struct *work)
{
	ns115_calculate_resistor();
}

static int ns115_calculate_current(int vbat)
{
	int ocv, cur;

	ocv = ns115_get_batt_cc_vol(ns115_chg.batt_cc);
	cur = (ocv - vbat) * 1000 / ns115_chg.resistor_mohm;
	PDBG("ocv:%dmV vbat:%dmV\n", ocv, vbat);
	PDBG("resistor:%dmohm current:%dmA\n", ns115_chg.resistor_mohm, cur);

	if (vbat < ns115_batt_gauge->power_off_mvolts && !is_batt_status_charging()){
		cur += cur * (ns115_batt_gauge->power_off_mvolts - vbat) / 100;
		PDBG("vol < power_off_vol. calibate current: %dmA\n", cur);
	}
	if (is_batt_status_charging() && cur > 0){
		cur = 0;
	}
	if (!is_batt_status_charging() && cur < 0){
		cur = 0;
	}

	return cur;
}

static int ns115_get_batt_mAs(int vbat)
{
	int cal_time;
	int cur, cur_mAs, cal_mAs;
	unsigned long cur_time;
	struct ns115_charger_priv *priv;

	priv = ns115_chg.current_chg_priv;
	cur_time = ns115_get_seconds();
	cal_time = cur_time - ns115_chg.init_mAs_time;
	if (cal_time < 0){
		PINFO("seconds: 0x%x error init_time: 0x%x\n", (unsigned)cur_time,
				(unsigned)ns115_chg.init_mAs_time);
		ns115_chg.init_mAs_time = cur_time;
		ns115_chg.init_mAs_time_cache = cur_time;
		ns115_chg.init_mAs = ns115_chg.init_mAs_cache;

		return ns115_chg.init_mAs;
	}
	cur = ns115_calculate_current(vbat);

	cal_mAs = cal_time * cur;
	cur_mAs = ns115_chg.init_mAs - cal_mAs;
	ns115_chg.init_mAs_cache = cur_mAs;
	PDBG("batt cur_mAh: %dmAh cal_mAs: %dmAs cal_time: %ds\n",
			cur_mAs / 3600, cal_mAs, cal_time);

	if (priv){
		if (is_batt_status_charging() && ns115_chg.batt_cc > 90 &&
					cur < 0 && -cur < priv->hw_chg->chg_current / 10){
			if (++ns115_chg.i_term_times < NS115_MAX_VOLT_BUF){
				goto out;
			}
			PINFO("current %d < %d(chg_current 10%%). charge done\n",
						cur, priv->hw_chg->chg_current / 10);
			ns115_calbt_mah_full_vol(cur_mAs / 3600, vbat);
			ns115_battery_notify_event(CHG_DONE_EVENT);
			ns115_chg.i_term_times = 0;
		}else if (ns115_chg.batt_status == BATT_STATUS_CHARGING_DONE
				&& cur < 0 && -cur > priv->hw_chg->chg_current / 10){
			if (++ns115_chg.i_term_times < NS115_MAX_VOLT_BUF){
				goto out;
			}
			ns115_chg.i_term_times = 0;
			PINFO("current %d > %d(chg_current 10%%). full_vol is small\n",
						cur, priv->hw_chg->chg_current / 10);
			ns115_calbt_mah_full_vol(cur_mAs / 3600, vbat);
		}else{
			ns115_chg.i_term_times = 0;
		}
	}else{
		ns115_chg.i_term_times = 0;
	}
out:

	return cur_mAs;
}

static int ns115_get_capacity(void)
{
	int aver_vol;
	int cur_mAs;
	int cc;
	unsigned long cur_time;

	PDBG("*************************\n");
	if (ns115_chg.sys_stat == SYS_STAT_SUSPEND){
		goto out;
	}

	aver_vol = get_batt_average_voltage(ns115_chg.batt_mvolts);
	cur_mAs = ns115_get_batt_mAs(aver_vol);

	ns115_chg.init_mAs= cur_mAs;
	cur_time = ns115_get_seconds();
	ns115_chg.init_mAs_time = cur_time;
	ns115_chg.init_mAs_time_cache = cur_time;
	ns115_chg.init_mAs_cache = ns115_chg.init_mAs;

	cc = (cur_mAs / 36  + ns115_batt_gauge->max_mAh - 100)
		/ ns115_batt_gauge->max_mAh;
	if (cc < 0){
		cc = 0;
	}else if (cc > 100){
		cc = 100;
		if (is_batt_status_charging()){
			PINFO("capacity 100%%. charge done!\n");
			ns115_battery_notify_event(CHG_DONE_EVENT);
		}
	}
	if (ns115_chg.batt_status == BATT_STATUS_CHARGING_DONE){
		cc = 100;
	}
	ns115_chg.batt_cc  = cc;

	PDBG("battery capacity: %d%%\n", ns115_chg.batt_cc);
	PDBG("*************************\n");
	if (ns115_chg.power_on++ >= 5){
		ns115_chg.power_on = 5;
	}
out:

	return ns115_chg.batt_cc;
}

static int ns115_batt_reinit_mAs(int volt)
{
	unsigned long cur_time;

	ns115_chg.init_mAs = ns115_get_batt_mAs(volt);
	cur_time = ns115_get_seconds();
	ns115_chg.init_mAs_time = cur_time;
	ns115_chg.init_mAs_time_cache = cur_time;

	return 0;
}

static struct power_supply ns115_psy_batt;
static int __ns115_batt_resume_init_mAs(int vbat)
{
	int cur_mAs, cur;
	int cal_time;
	unsigned long cur_time;

	cur = get_about_current(ns115_chg.batt_mvolts);
	cal_time = get_seconds() - ns115_chg.init_mAs_time;
	cur_mAs = ns115_chg.init_mAs - cal_time * cur;
	PDBG("suspend time: %ds. cur_mAs: %dmAh cur: %dmA\n",
			cal_time, cur_mAs / 3600, cur);

	ns115_chg.batt_cc = cur_mAs /36 / ns115_batt_gauge->max_mAh;
	if (ns115_chg.batt_cc < 0){
		ns115_chg.batt_cc = 0;
		cur_mAs = 0;
	}else if (ns115_chg.batt_cc > 100){
		ns115_chg.batt_cc = 100;
		cur_mAs = ns115_batt_gauge->max_mAh * 3600;
	}

	ns115_chg.init_mAs = cur_mAs;
	cur_time = ns115_get_seconds();
	ns115_chg.init_mAs_time = cur_time;
	ns115_chg.init_mAs_time_cache = cur_time;
	ns115_chg.init_mAs_cache = ns115_chg.init_mAs;
	if (is_batt_status_charging() && ns115_chg.batt_cc == 100){
		PINFO("capacity 100%%. charge done\n");
		ns115_battery_notify_event(CHG_DONE_EVENT);
	}else{
		power_supply_changed(&ns115_psy_batt);
	}
	PDBG("resume capacity: %d%%\n", ns115_chg.batt_cc);

	return 0;
}

static int ns115_batt_resume_init_mAs(int vbat)
{
	struct ns115_charger_priv *priv;

	priv = ns115_chg.current_chg_priv;
	if (ns115_chg.resume_by_charger && priv){
		if (ns115_chg.resume_by_charger == 1){
			priv->hw_chg_state = CHG_READY_STATE;
			__ns115_batt_resume_init_mAs(vbat);
			priv->hw_chg_state = CHG_CHARGING_STATE;
		}else{
			priv->hw_chg_state = CHG_CHARGING_STATE;
			__ns115_batt_resume_init_mAs(vbat);
			priv->hw_chg_state = CHG_ABSENT_STATE;
			priv = NULL;
		}
		ns115_chg.resume_by_charger = 0;
	}else{
		__ns115_batt_resume_init_mAs(vbat);
	}
#ifdef CONFIG_WAKELOCK
	wake_unlock(&ns115_chg.chg_wl);
#endif

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
	ns115_batt_resume_init_mAs(vbat);
	ns115_chg.sys_stat = SYS_STAT_EARLY_SUSPEND;

	return;
}
#endif

static int get_prop_batt_capacity(void)
{
	if (ns115_batt_gauge){
		if (ns115_batt_gauge->get_battery_capacity){
			return ns115_batt_gauge->get_battery_capacity(ns115_chg.batt_mvolts);
		}
#ifdef CONFIG_NS115_ADC_GAUGE
		else{
			return ns115_get_capacity();
		}
#endif
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
#ifdef CONFIG_NS115_ADC_GAUGE
	int cal_time;
#endif

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
		PDBG("%s start charging! chg current: %dmA!\n",
				priv->hw_chg->name, priv->hw_chg->chg_current);
#ifdef CONFIG_NS115_ADC_GAUGE
		if (ns115_chg.power_on < 5){
			cal_time = ns115_get_seconds() - ns115_chg.init_mAs_time;
			dev_info(ns115_chg.dev, "cal_time: %ds\n", cal_time);
			if (cal_time <= 3){
				dev_info(ns115_chg.dev, "the charger plug when capacity init\n");
				priv->hw_chg_state = CHG_CHARGING_STATE;

				mutex_lock(&ns115_chg.vol_lock);
				ns115_chg.batt_volt_times  = 0;
				ns115_chg.batt_volt_pointer = 0;
				mutex_unlock(&ns115_chg.vol_lock);

				ns115_batt_init_mAs();
			}else{
				queue_work(ns115_chg.event_wq_thread, &ns115_chg.resistor_work);
			}
		}else{
			if (ns115_chg.sys_stat == SYS_STAT_SUSPEND){
				ns115_chg.resume_by_charger = 1;
#ifdef CONFIG_WAKELOCK
				wake_lock(&ns115_chg.chg_wl);
#endif
			}else{
				ns115_batt_reinit_mAs(volt);
				queue_work(ns115_chg.event_wq_thread, &ns115_chg.resistor_work);
			}
		}
#endif
		priv->hw_chg_state = CHG_CHARGING_STATE;
		ns115_chg.batt_status = battery_stat;
#ifdef CONFIG_BATTERY_TPS80032
		calib_gauge_start_chg();
#endif
	}

	power_supply_changed(&ns115_psy_batt);

	return ret;
}

static void handle_charging_done(struct ns115_charger_priv *priv)
{
#ifdef CONFIG_NS115_ADC_GAUGE
	unsigned long cur_time;
#endif
	if (ns115_chg.current_chg_priv == priv) {
		if (ns115_chg.current_chg_priv->hw_chg_state ==
		    CHG_CHARGING_STATE){
			if (ns115_stop_charging(ns115_chg.current_chg_priv)) {
				dev_err(ns115_chg.dev, "%s couldnt stop chg\n",
					ns115_chg.current_chg_priv->hw_chg->name);
			}
#ifdef CONFIG_NS115_ADC_GAUGE
			ns115_chg.init_mAs = ns115_batt_gauge->max_mAh * 3600;
			cur_time = ns115_get_seconds();
			ns115_chg.init_mAs_time = cur_time;
			ns115_chg.init_mAs_time_cache = cur_time;
#endif
		}
		ns115_chg.current_chg_priv->hw_chg_state = CHG_DONE_STATE;

#ifdef CONFIG_BATTERY_TPS80032
		calib_gauge_chg_done();
#endif
		ns115_chg.batt_status = BATT_STATUS_CHARGING_DONE;
		dev_info(ns115_chg.dev, "%s: stopping safety timer work\n",
				__func__);
		cancel_delayed_work(&ns115_chg.teoc_work);

		power_supply_changed(&ns115_psy_batt);
	}
}

static void teoc(struct work_struct *work)
{
	/* we have been charging too long - stop charging */
	dev_info(ns115_chg.dev, "%s: safety timer work expired\n", __func__);
	ns115_battery_notify_event(CHG_ERROR_EVENT);
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

	PDBG("%s removed!\n", hw_chg_removed->hw_chg->name);
	if (ns115_chg.current_chg_priv == hw_chg_removed) {
		if (ns115_chg.current_chg_priv->hw_chg_state
						== CHG_CHARGING_STATE) {
			if (ns115_stop_charging(hw_chg_removed)) {
				dev_err(ns115_chg.dev, "%s couldnt stop chg\n",
					ns115_chg.current_chg_priv->hw_chg->name);
			}
		}
#ifdef CONFIG_NS115_ADC_GAUGE
		if (ns115_chg.sys_stat == SYS_STAT_SUSPEND){
			ns115_chg.resume_by_charger = 2;
#ifdef CONFIG_WAKELOCK
			wake_lock(&ns115_chg.chg_wl);
#endif
		}else{
			ns115_batt_reinit_mAs(get_prop_batt_mvolts());
			ns115_chg.current_chg_priv = NULL;
		}
#else
		ns115_chg.current_chg_priv = NULL;
#endif
	}

	hw_chg_removed->hw_chg_state = new_state;
	ns115_chg.batt_status = BATT_STATUS_DISCHARGING;

	/* if we arent charging stop the safety timer */
	dev_info(ns115_chg.dev, "%s: stopping safety timer work\n",
			__func__);
	cancel_delayed_work(&ns115_chg.teoc_work);
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
#ifdef CONFIG_NS115_ADC_GAUGE
				ns115_batt_reinit_mAs(get_prop_batt_mvolts());
#endif
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

	ns115_chg.batt_status = BATT_STATUS_DISCHARGING;
	ns115_chg.pre_chg_mvolts = ns115_batt_gauge->pre_chg_mvolts;

	if (ns115_chg.pre_chg_mvolts == 0){
		ns115_chg.pre_chg_mvolts = NS115_BATT_PRE_CHG_MVOLTS;
	}
#ifdef CONFIG_NS115_ADC_GAUGE
	ns115_chg.batt_volt_times  = 0;
	ns115_chg.batt_volt_pointer = 0;
	ns115_chg.resistor_pointer = 0;
	ns115_chg.resistor_times = 0;
	ns115_chg.ocv_less_vol = 0;
	ns115_chg.ocv_more_vol = 0;
	ns115_chg.get_cap_record = 0;
	ns115_chg.resistor_mohm = ns115_batt_gauge->resistor_mohm;

	ns115_chg.i_term_times = 0;
	ns115_chg.sys_stat = SYS_STAT_NORMAL;
	ns115_chg.power_on = 0;
	ns115_chg.resume_by_charger = 0;
	ns115_batt_init_mAs();
#endif

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
#ifdef CONFIG_NS115_ADC_GAUGE
	queue_delayed_work(ns115_chg.event_wq_thread,
				&ns115_chg.save_capacity_work,
				msecs_to_jiffies(GET_SAVE_CAP_TIME));
#endif

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
#ifdef CONFIG_NS115_ADC_GAUGE
	mutex_init(&ns115_chg.vol_lock);
	INIT_DELAYED_WORK(&ns115_chg.batt_resume_init_work, ns115_batt_resume_init_work);
	INIT_DELAYED_WORK(&ns115_chg.save_capacity_work, ns115_save_capacity_work);
	INIT_WORK(&ns115_chg.resistor_work, ns115_resistor_work);
#endif
	INIT_DELAYED_WORK(&ns115_chg.update_heartbeat_work, update_heartbeat);

#ifdef CHARGING_ALWAYS_WAKE
	wake_lock_init(&ns115_chg.wl, WAKE_LOCK_SUSPEND, "ns115_battery");
#endif
#if defined(CONFIG_NS115_ADC_GAUGE) && defined(CONFIG_WAKELOCK)
	wake_lock_init(&ns115_chg.chg_wl, WAKE_LOCK_SUSPEND, "ns115_batt_chg");
#endif
	dev_info(ns115_chg.dev, "%s is OK!\n", __func__);

	return 0;
}

static int __devexit ns115_battery_remove(struct platform_device *pdev)
{
#ifdef CHARGING_ALWAYS_WAKE
	wake_lock_destroy(&ns115_chg.wl);
#endif
#if defined(CONFIG_NS115_ADC_GAUGE) && defined(CONFIG_WAKELOCK)
	mutex_destroy(&ns115_chg.vol_lock);
	wake_lock_destroy(&ns115_chg.chg_wl);
#endif
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
#ifdef CONFIG_NS115_ADC_GAUGE
	if (hw_chg->chg_current > ns115_chg.max_chg_current){
		ns115_chg.max_chg_current = hw_chg->chg_current;
	}
#endif

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
#ifdef CONFIG_NS115_ADC_GAUGE
	unsigned long cur_time;
	int cur_mAs;
#endif

	dev_dbg(ns115_chg.dev, "%s suspended\n", __func__);
	ns115_chg.stop_update = 1;
	cancel_delayed_work(&ns115_chg.update_heartbeat_work);

#ifdef CONFIG_NS115_ADC_GAUGE
	cancel_delayed_work(&ns115_chg.save_capacity_work);
	cur_mAs = ns115_get_batt_mAs(ns115_chg.batt_mvolts);
	ns115_chg.init_mAs = cur_mAs;
	PDBG("suspend battery cur_mAh: %dmAh\n", cur_mAs / 3600);

	cur_time = get_seconds();
	ns115_chg.init_mAs_time = cur_time;
	ns115_chg.init_mAs_time_cache = cur_time;

	ns115_chg.sys_stat = SYS_STAT_SUSPEND;
#endif

	return 0;
}

static int ns115_battery_resume(struct device *dev)
{
	dev_dbg(ns115_chg.dev, "%s resumed\n", __func__);
	ns115_chg.stop_update = 0;

#ifdef CONFIG_NS115_ADC_GAUGE
	mutex_lock(&ns115_chg.vol_lock);
	ns115_chg.batt_volt_times  = 0;
	ns115_chg.batt_volt_pointer = 0;
	mutex_unlock(&ns115_chg.vol_lock);

	/* start updaing the battery powersupply every ns115_chg.update_time
	 * milliseconds */
	queue_delayed_work(ns115_chg.event_wq_thread,
				&ns115_chg.batt_resume_init_work,
				msecs_to_jiffies(RESUME_WAIT_TIME));
	queue_delayed_work(ns115_chg.event_wq_thread,
				&ns115_chg.save_capacity_work,
				msecs_to_jiffies(SAVE_GAPACITY_TIME));
#endif
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

#if defined(CONFIG_HAS_EARLYSUSPEND) && defined(CONFIG_NS115_ADC_GAUGE)
static void ns115_batt_early_suspend(struct early_suspend *h)
{
	dev_dbg(ns115_chg.dev, "%s\n", __func__);
	ns115_batt_reinit_mAs(get_prop_batt_mvolts());
	ns115_chg.sys_stat = SYS_STAT_EARLY_SUSPEND;
}
static void ns115_batt_late_resume(struct early_suspend *h)
{
	int vbat;

	PDBG("%s\n", __func__);
	vbat = get_prop_batt_mvolts();
	if (ns115_chg.sys_stat == SYS_STAT_EARLY_SUSPEND){
		ns115_batt_reinit_mAs(vbat);
	}else{
		ns115_batt_resume_init_mAs(vbat);
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
#ifdef CONFIG_NS115_ADC_GAUGE
	ns115_chg.max_chg_current = 0;
#endif
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
#if defined(CONFIG_HAS_EARLYSUSPEND) && defined(CONFIG_NS115_ADC_GAUGE)
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
#if defined(CONFIG_HAS_EARLYSUSPEND) && defined(CONFIG_NS115_ADC_GAUGE)
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
