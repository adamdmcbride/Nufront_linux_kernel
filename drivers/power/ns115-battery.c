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

#ifdef CONFIG_WAKELOCK
//#define CHARGING_ALWAYS_WAKE
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
#define NS115_BATT_FULL_MVOLTS	 4150
#define NS115_BATT_TRKL_DROP	20
#define NS115_MAX_VOLT_BUF		10
#define NS115_VOLT_WINDOWS		25

#define NS115_CHG_MAX_EVENTS	16

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
	int	batt_mvolts;
	int	batt_volt_buf[NS115_MAX_VOLT_BUF];
	int batt_volt_pointer;
	int batt_volt_times;
	int charger_power_on;
	int batt_left_windows;
	int batt_right_windows;
	int consumption;
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
	int pre_chg_mvolts;
	int full_mvolts;
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

static int batt_voltage_debouce(int mvolts)
{
	int new_mvolts = mvolts;
	int resistor = ns115_batt_gauge->resistor_mohm;
	int chg_current;
	int i, sum;

	if (!mvolts){
		return ns115_chg.batt_mvolts;
	}
	dev_dbg(ns115_chg.dev, "votage before Calibration: %d\n", mvolts);
	if (ns115_chg.batt_status == BATT_STATUS_FAST_CHARGING){
		if (ns115_chg.charger_power_on++ < 3){
			return ns115_chg.batt_mvolts;
		}
		if(ns115_chg.charger_power_on == 4 && 
				ns115_chg.batt_volt_times < NS115_MAX_VOLT_BUF){
			ns115_chg.batt_volt_times = 0;
			ns115_chg.batt_volt_pointer = 0;
		}
		if (mvolts < ns115_chg.full_mvolts){
			chg_current = ns115_chg.current_chg_priv->hw_chg->chg_current;
			new_mvolts = mvolts - (chg_current * resistor / 1000);
		}
	}
	new_mvolts += (ns115_chg.consumption * resistor) / new_mvolts;

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
	ns115_chg.batt_volt_buf[ns115_chg.batt_volt_pointer] = new_mvolts;
	if (++ns115_chg.batt_volt_pointer >= NS115_MAX_VOLT_BUF){
		ns115_chg.batt_volt_pointer = 0;
	}
	for (i = 0, sum = 0; i < ns115_chg.batt_volt_times; ++i){
		sum += ns115_chg.batt_volt_buf[i];
	}
	new_mvolts = sum / ns115_chg.batt_volt_times;

	ns115_chg.batt_mvolts = new_mvolts;
	dev_dbg(ns115_chg.dev, "votage after Calibration: %d times:%d\n",
			new_mvolts, ns115_chg.batt_volt_times);

	return new_mvolts;
}

static int get_prop_batt_mvolts(void)
{
	int mvolts;

	if (ns115_batt_gauge && ns115_batt_gauge->get_battery_mvolts){
		mvolts = ns115_batt_gauge->get_battery_mvolts();
		return batt_voltage_debouce(mvolts);
	}else {
		pr_err("ns115-charger no batt gauge assuming 3.5V\n");
		return NS115_BATT_MISSING_VOLTS;
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

static int get_prop_batt_capacity(void)
{
	if (ns115_batt_gauge && ns115_batt_gauge->get_battery_capacity)
		return ns115_batt_gauge->get_battery_capacity(ns115_chg.batt_mvolts);

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
		priv->hw_chg_state = CHG_CHARGING_STATE;
		ns115_chg.batt_status = battery_stat;
	}
	ns115_chg.charger_power_on = 0;
	power_supply_changed(&ns115_psy_batt);

	return ret;
}

static void handle_charging_done(struct ns115_charger_priv *priv)
{
	if (ns115_chg.current_chg_priv == priv) {
		if (ns115_chg.current_chg_priv->hw_chg_state ==
		    CHG_CHARGING_STATE)
			if (ns115_stop_charging(ns115_chg.current_chg_priv)) {
				dev_err(ns115_chg.dev, "%s couldnt stop chg\n",
					ns115_chg.current_chg_priv->hw_chg->name);
			}
		ns115_chg.current_chg_priv->hw_chg_state = CHG_READY_STATE;

		ns115_chg.batt_status = BATT_STATUS_CHARGING_DONE;
		dev_info(ns115_chg.dev, "%s: stopping safety timer work\n",
				__func__);
		cancel_delayed_work(&ns115_chg.teoc_work);
		ns115_chg.charger_power_on = 0;
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
		/*this a backup solotion, because of the hardware design, 
		 * the bq24170 charger can't charge full*/
		if (is_batt_status_charging() && volts >= ns115_chg.full_mvolts){
			ns115_battery_notify_event(CHG_DONE_EVENT);
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
		ns115_chg.current_chg_priv = NULL;
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
	ns115_chg.batt_status = BATT_STATUS_DISCHARGING;
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
		ns115_chg.consumption = pdata->consumption;
		ns115_chg.pre_chg_mvolts = pdata->pre_chg_mvolts;
		ns115_chg.full_mvolts = pdata->full_mvolts;
	}
	if (ns115_chg.safety_time == 0){
		ns115_chg.safety_time = CHARGING_TEOC_MS;
	}
	if (ns115_chg.update_time == 0){
		ns115_chg.update_time = UPDATE_TIME_MS;
	}
	if (ns115_chg.pre_chg_mvolts == 0){
		ns115_chg.pre_chg_mvolts = NS115_BATT_PRE_CHG_MVOLTS;
	}
	if (ns115_chg.full_mvolts == 0){
		ns115_chg.full_mvolts = NS115_BATT_FULL_MVOLTS;
	}

	mutex_init(&ns115_chg.status_lock);
	INIT_DELAYED_WORK(&ns115_chg.teoc_work, teoc);
	INIT_DELAYED_WORK(&ns115_chg.update_heartbeat_work, update_heartbeat);

#ifdef CHARGING_ALWAYS_WAKE
	wake_lock_init(&ns115_chg.wl, WAKE_LOCK_SUSPEND, "ns115_battery");
#endif
	dev_info(ns115_chg.dev, "%s is OK!\n", __func__);

	return 0;
}

static int __devexit ns115_battery_remove(struct platform_device *pdev)
{
#ifdef CHARGING_ALWAYS_WAKE
	wake_lock_destroy(&ns115_chg.wl);
#endif
	mutex_destroy(&ns115_chg.status_lock);
	power_supply_unregister(&ns115_psy_batt);
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
	ns115_batt_gauge = NULL;

	return 0;
}
EXPORT_SYMBOL(ns115_battery_gauge_unregister);


static int ns115_battery_suspend(struct device *dev)
{
	dev_dbg(ns115_chg.dev, "%s suspended\n", __func__);
	ns115_chg.stop_update = 1;
	cancel_delayed_work(&ns115_chg.update_heartbeat_work);
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

	ns115_chg.inited = 1;
	return 0;

destroy_wq_thread:
	destroy_workqueue(ns115_chg.event_wq_thread);
out:
	return rc;
}

static void __exit ns115_battery_exit(void)
{
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
