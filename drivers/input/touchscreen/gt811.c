/* drivers/input/touchscreen/gt811.c
 *
 * 2010 - 2012 Goodix Technology.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be a reference
 * to you, when you are integrating the GOODiX's CTP IC into your system,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * Version:1.0
 * Author:andrew@goodix.com
 * Release Date:2012/05/01
 * Revision record:
 *      V1.0:2012/05/01,create file,by andrew
 */

#include <linux/irq.h>
#include "gt811.h"
#if GTP_AUTO_UPDATE
#include "gt811_firmware.h"
#endif

static const char *goodix_ts_name = "Goodix Capacitive TouchScreen";
static struct workqueue_struct *goodix_wq;
struct i2c_client *i2c_connect_client = NULL;
static u8 config[GTP_CONFIG_LENGTH + GTP_ADDR_LENGTH] =
    { (u8) (GTP_REG_CONFIG_DATA >> 8),
	(u8) GTP_REG_CONFIG_DATA,
};

#if GTP_HAVE_TOUCH_KEY
static const u16 touch_key_array[] = GTP_KEY_TAB;
#define GTP_MAX_KEY_NUM	 (sizeof(touch_key_array)/sizeof(touch_key_array[0]))
#endif

void gtp_reset_guitar(s32 ms);
static s8 gtp_i2c_test(struct i2c_client *client);
#ifdef CONFIG_HAS_EARLYSUSPEND
static void goodix_ts_early_suspend(struct early_suspend *h);
static void goodix_ts_late_resume(struct early_suspend *h);
#endif

#ifdef GTP_CREATE_WR_NODE
extern s32 init_wr_node(struct i2c_client *);
extern void uninit_wr_node(void);
#endif

#ifdef GTP_ESD_PROTECT
static struct delayed_work gtp_esd_check_work;
static struct workqueue_struct *gtp_esd_check_workqueue = NULL;
static void gtp_esd_check_func(struct work_struct *);
#endif

/*******************************************************
Function:
Read data from the i2c slave device.

Input:
client:	i2c device.
buf[0]:operate address.
buf[1]~buf[len]:read data buffer.
len:operate length.

Output:
numbers of i2c_msgs to transfer
 *********************************************************/
s32 gtp_i2c_read(struct i2c_client *client, u8 * buf, s32 len)
{
	struct i2c_msg msgs[2];
	s32 ret = -1;
	s32 retries = 0;

	GTP_DEBUG_FUNC();

	msgs[0].flags = !I2C_M_RD;
	msgs[0].addr = client->addr;
	msgs[0].len = GTP_ADDR_LENGTH;
	msgs[0].buf = &buf[0];

	msgs[1].flags = I2C_M_RD;
	msgs[1].addr = client->addr;
	msgs[1].len = len - GTP_ADDR_LENGTH;
	msgs[1].buf = &buf[GTP_ADDR_LENGTH];

	while (retries < 5) {
		ret = i2c_transfer(client->adapter, msgs, 2);
		if (!(ret <= 0))
			break;
		retries++;
	}
	return ret;
}

/*******************************************************
Function:
write data to the i2c slave device.

Input:
client:	i2c device.
buf[0]:operate address.
buf[1]~buf[len]:write data buffer.
len:operate length.

Output:
numbers of i2c_msgs to transfer.
 *********************************************************/
s32 gtp_i2c_write(struct i2c_client * client, u8 * data, s32 len)
{
	struct i2c_msg msg;
	s32 ret = -1;
	s32 retries = 0;

	GTP_DEBUG_FUNC();

	msg.flags = !I2C_M_RD;
	msg.addr = client->addr;
	msg.len = len;
	msg.buf = data;

	while (retries < 5) {
		ret = i2c_transfer(client->adapter, &msg, 1);
		if (ret == 1)
			break;
		retries++;
	}
	return ret;
}

/*******************************************************
Function:
Enable IRQ Function.

Input:
ts:	i2c client private struct.

Output:
None.
 *******************************************************/
void gtp_irq_disable(struct goodix_ts_data *ts)
{
	unsigned long irqflags;

	GTP_DEBUG_FUNC();

	spin_lock_irqsave(&ts->irq_lock, irqflags);
	if (!ts->irq_is_disable) {
		disable_irq_nosync(ts->client->irq);
		ts->irq_is_disable = 1;
	}
	spin_unlock_irqrestore(&ts->irq_lock, irqflags);
}

/*******************************************************
Function:
Disable IRQ Function.

Input:
ts:	i2c client private struct.

Output:
None.
 *******************************************************/
void gtp_irq_enable(struct goodix_ts_data *ts)
{
	unsigned long irqflags;

	GTP_DEBUG_FUNC();

	spin_lock_irqsave(&ts->irq_lock, irqflags);
	if (ts->irq_is_disable) {
		enable_irq(ts->client->irq);
		ts->irq_is_disable = 0;
	}
	spin_unlock_irqrestore(&ts->irq_lock, irqflags);
}

/*******************************************************
Function:
Send config Function.

Input:
client:	i2c client.

Output:
Executive outcomes.0！！success,non-0！！fail.
 *******************************************************/
s32 gtp_send_cfg(struct i2c_client *client)
{
#if GTP_DRIVER_SEND_CFG
	s32 retry = 0;
	s32 ret = -1;

	for (retry = 0; retry < 5; retry++) {
		ret =
		    gtp_i2c_write(client, config,
				  GTP_CONFIG_LENGTH + GTP_ADDR_LENGTH);
		if (ret > 0) {
			return 0;
		}
	}

	return retry;
#else
	return 0;
#endif
}

/*******************************************************
Function:
GTP initialize function.

Input:
ts:	i2c client private struct.

Output:
Executive outcomes.0---succeed.
 *******************************************************/
s32 gtp_init_panel(struct goodix_ts_data * ts)
{
	s32 ret = -1;
	u8 rd_cfg_buf[4];
	u8 cfg_info_group1[] = CTP_CFG_GROUP1;
	u8 cfg_info_group2[] = CTP_CFG_GROUP2;
	u8 cfg_info_group3[] = CTP_CFG_GROUP3;
	u8 *send_cfg_buf[3] =
	    { cfg_info_group1, cfg_info_group2, cfg_info_group3 };
	u8 cfg_info_len[3] =
	    { sizeof(cfg_info_group1) / sizeof(cfg_info_group1[0]),
		sizeof(cfg_info_group2) / sizeof(cfg_info_group2[0]),
		sizeof(cfg_info_group3) / sizeof(cfg_info_group3[0])
	};
	GTP_DEBUG("len1=%d,len2=%d,len3=%d", cfg_info_len[0], cfg_info_len[1],
		  cfg_info_len[2]);
	if ((!cfg_info_len[1]) && (!cfg_info_len[2])) {
		rd_cfg_buf[GTP_ADDR_LENGTH] = 0;
	} else {
		rd_cfg_buf[0] = (u8) (GTP_REG_SENSOR_ID >> 8);
		rd_cfg_buf[1] = (u8) GTP_REG_SENSOR_ID;
		ret = gtp_i2c_read(ts->client, rd_cfg_buf, 3);
		if (ret <= 0) {
			GTP_ERROR
			    ("Read SENSOR ID failed,default use group1 config!");
			rd_cfg_buf[GTP_ADDR_LENGTH] = 0;
		}
		rd_cfg_buf[GTP_ADDR_LENGTH] &= 0x03;
	}
	GTP_DEBUG("SENSOR ID:%d", rd_cfg_buf[GTP_ADDR_LENGTH]);
	memcpy(config, send_cfg_buf[rd_cfg_buf[GTP_ADDR_LENGTH]],
	       (GTP_CONFIG_LENGTH + GTP_ADDR_LENGTH));
#if GTP_CUSTOM_CFG
	config[57] &= 0xf7;
	if (GTP_INT_TRIGGER & 0x01) {
		config[57] += 0x08;
	}
	config[59] = GTP_REFRESH;
	config[60] = GTP_MAX_TOUCH > 5 ? 5 : GTP_MAX_TOUCH;
	config[61] = (u8) GTP_MAX_WIDTH;
	config[62] = (u8) (GTP_MAX_WIDTH >> 8);
	config[63] = (u8) GTP_MAX_HEIGHT;
	config[64] = (u8) (GTP_MAX_HEIGHT >> 8);
#endif
	ts->abs_x_max = (config[62] << 8) + config[61];
	ts->abs_y_max = (config[64] << 8) + config[63];
	ts->max_touch_num = config[60];
	ts->int_trigger_type = ((config[57] >> 3) & 0x01);
	if ((!ts->abs_x_max) || (!ts->abs_y_max) || (!ts->max_touch_num)) {
		GTP_ERROR
		    ("GTP resolution & max_touch_num invalid, use default value!");
		ts->abs_x_max = GTP_MAX_HEIGHT;
		ts->abs_y_max = GTP_MAX_WIDTH;
		ts->max_touch_num = GTP_MAX_TOUCH;
	}

	ret = gtp_send_cfg(ts->client);
	if (ret) {
		GTP_ERROR("Send config error.");
	}
	GTP_DEBUG("X_MAX = %d,Y_MAX = %d,MAX_TOUCH = %d,TRIGGER=%d",
		  ts->abs_x_max, ts->abs_y_max, ts->max_touch_num,
		  ts->int_trigger_type);
	msleep(10);
	return 0;
}

/*******************************************************
Function:
Read goodix touchscreen version function.

Input:
client:	i2c client struct.

Output:
Executive outcomes.0---succeed.
 *******************************************************/
s32 gtp_read_version(struct goodix_ts_data * ts)
{
	s32 ret = -1;
	s32 count = 0;
	u8 version_data[6] =
	    { (u8) (GTP_REG_VERSION >> 8), (u8) GTP_REG_VERSION, 0, 0, 0, 0 };
	u8 version_comfirm[6] =
	    { (u8) (GTP_REG_VERSION >> 8), (u8) GTP_REG_VERSION, 0, 0, 0, 0 };

	GTP_DEBUG_FUNC();

	ret = gtp_i2c_read(ts->client, version_data, 6);
	if (ret <= 0) {
		GTP_ERROR("GTP read version failed");
		return ret;
	}
	GTP_DEBUG("Read version:%02x%02x", version_data[4], version_data[5]);

	while (count++ < 10) {
		gtp_i2c_read(ts->client, version_comfirm, 6);
		if ((version_data[4] != version_comfirm[4]) || (version_data[5]
                                                        != version_comfirm[5])) {
			GTP_DEBUG("Comfirm version:%02x%02x",
				  version_comfirm[4], version_comfirm[5]);
			version_data[4] = version_comfirm[4];
			version_data[5] = version_comfirm[5];
			break;
		}
		msleep(5);
	}
	if (count == 11) {
		GTP_INFO("GTP chip version:%02x%02x_%02x%02x",
			 version_data[3], version_data[2], version_data[4],
			 version_data[5]);
		ts->version = (version_data[4] << 8) + version_data[5];
		ret = 0;
	} else {
		GTP_ERROR("GTP read version confirm error!");
		ret = 1;
	}
	return ret;
}

/*******************************************************
Function:
Touch down report function.

Input:
ts:private data.
id:tracking id.
x:input x.
y:input y.
w:input weight.

Output:
None.
 *******************************************************/
static void gtp_touch_down(struct goodix_ts_data *ts, s32 id, s32 x, s32 y,
			   s32 w)
{
#if GTP_CHANGE_X2Y
	GTP_SWAP(x, y);
#endif
	input_report_abs(ts->input_dev, ABS_MT_POSITION_X, x);
	input_report_abs(ts->input_dev, ABS_MT_POSITION_Y, y);
	input_report_abs(ts->input_dev, ABS_MT_TRACKING_ID, id);
	input_report_abs(ts->input_dev, ABS_PRESSURE, w);
	input_report_key(ts->input_dev, BTN_TOUCH, 1);
	input_mt_sync(ts->input_dev);
	GTP_DEBUG("ID=%d, X=%d, Y=%d, W=%d ", id, x, y, w);
}

/*******************************************************
Function:
Touch up report function.

Input:
ts:private data.

Output:
None.
 *******************************************************/
static void gtp_touch_up(struct goodix_ts_data *ts)
{
	input_report_abs(ts->input_dev, ABS_PRESSURE, 0);
	input_report_key(ts->input_dev, BTN_TOUCH, 0);
	input_mt_sync(ts->input_dev);
}

/*******************************************************
Function:
Goodix touchscreen work function.

Input:
work:	work_struct of goodix_wq.

Output:
None.
 *******************************************************/
static void goodix_ts_work_func(struct work_struct *work)
{
	u8 point_data[GTP_READ_BYTES] =
	    { (u8) (GTP_REG_COOR >> 8), (u8) GTP_REG_COOR, 0 };
	u8 point_test[4] = { 0x7, 0x41, 0, 0 };
	u8 check_sum = 0;
	u8 read_position = 0;
	u8 track_id[GTP_MAX_TOUCH];
	u8 point_index = 0;
	u8 point_tmp = 0;
	u8 touch_num = 0;
	u8 key_value = 0;
	u8 input_w = 0;
	u16 input_x = 0;
	u16 input_y = 0;
	u32 count = 0;
	u32 position = 0;
	s32 ret = -1;

	struct goodix_ts_data *ts;

	GTP_DEBUG_FUNC();

	ts = container_of(work, struct goodix_ts_data, work);
#if 1
	ret =
	    gtp_i2c_read(ts->client, point_data,
			 (sizeof(point_data) / sizeof(point_data[0]) - 2));
	//ret = gtp_i2c_read(ts->client, point_data, sizeof(point_data)/sizeof(point_data[0]));
	if (ret <= 0) {
		goto exit_work_func;
	}
	GTP_DEBUG_ARRAY(point_data,
			(sizeof(point_data) / sizeof(point_data[0]) - 2));
	//GTP_DEBUG_ARRAY(point_data, sizeof(point_data)/sizeof(point_data[0]));
#else

#endif

#if 1
	ret = gtp_i2c_read(ts->client, point_test, 4);
	if (ret <= 0) {
		goto exit_work_func;
	}
	GTP_DEBUG_ARRAY(point_test, 4);

	point_data[34] = point_test[2];
	point_data[35] = point_test[3];
#endif

	if (point_data[GTP_ADDR_LENGTH] & 0x20) {
		if (point_data[3] == 0xF0) {
			GTP_DEBUG("Reload config!");
			ret = gtp_send_cfg(ts->client);
			if (ret) {
				GTP_ERROR("Send config error.");
			}
			goto exit_work_func;
		}
	}

	point_index = point_data[2] & 0x1f;
	point_tmp = point_index;
	for (position = 0; (position < GTP_MAX_TOUCH) && point_tmp; position++) {
		if (point_tmp & 0x01) {
			track_id[touch_num++] = position;
		}
		point_tmp >>= 1;
	}
	GTP_DEBUG("Touch num:%d", touch_num);
	if (touch_num) {
		switch (point_data[2] & 0x1f) {
		case 0:
			read_position = 3;
			break;
		case 1:
			for (count = 2; count < 9; count++) {
				check_sum += (s32) point_data[count];
			}
			read_position = 9;
			break;
		case 2:
		case 3:
			for (count = 2; count < 14; count++) {
				check_sum += (s32) point_data[count];
			}
			read_position = 14;
			break;

		default:
			for (count = 2; count < 35; count++) {
				check_sum += (s32) point_data[count];
			}
			read_position = 35;
		}
		if (check_sum != point_data[read_position]) {
			GTP_DEBUG("Cal_chksum:%d,  Read_chksum:%d",
				  check_sum, point_data[read_position]);
			//GTP_ERROR("Coordinate checksum error!");
			printk("Coordinate checksum error!");
			goto exit_work_func;
		}

		for (count = 0; count < touch_num; count++) {
			if (track_id[count] != 3) {
				if (track_id[count] < 3) {
					position = 4 + track_id[count] * 5;
				} else {
					position = 30;
				}
				input_x = (u16) (point_data[position] << 8)
				    + (u16) point_data[position + 1];
				input_y = (u16) (point_data[position + 2] << 8)
				    + (u16) point_data[position + 3];
				input_w = point_data[position + 4];
			} else {
				input_x =
				    (u16) (point_data[19] << 8) +
				    (u16) point_data[26];
				input_y =
				    (u16) (point_data[27] << 8) +
				    (u16) point_data[28];
				input_w = point_data[29];
			}

			if ((input_x > ts->abs_x_max)
			    || (input_y > ts->abs_y_max)) {
				continue;
			}
			gtp_touch_down(ts, track_id[count], input_x, input_y,
				       input_w);
		}
	} else {
		GTP_DEBUG("Touch Release!");
		gtp_touch_up(ts);
	}

#if GTP_HAVE_TOUCH_KEY
	key_value = point_data[3] & 0x0F;
	for (count = 0; count < GTP_MAX_KEY_NUM; count++) {
		input_report_key(ts->input_dev, touch_key_array[count],
				 !!(key_value & (0x01 << count)));
	}
#endif
	input_report_key(ts->input_dev, BTN_TOUCH, (touch_num || key_value));
	input_sync(ts->input_dev);

exit_work_func:
	if (ts->use_irq) {
		gtp_irq_enable(ts);
	}
}

/*******************************************************
Function:
Timer interrupt service routine.

Input:
timer:	timer struct pointer.

Output:
Timer work mode. HRTIMER_NORESTART---not restart mode
 *******************************************************/
static enum hrtimer_restart goodix_ts_timer_handler(struct hrtimer *timer)
{
	struct goodix_ts_data *ts =
	    container_of(timer, struct goodix_ts_data, timer);

	GTP_DEBUG_FUNC();

	queue_work(goodix_wq, &ts->work);
	hrtimer_start(&ts->timer, ktime_set(0, (GTP_POLL_TIME + 6) * 1000000),
		      HRTIMER_MODE_REL);
	return HRTIMER_NORESTART;
}

/*******************************************************
Function:
External interrupt service routine.

Input:
irq:	interrupt number.
dev_id: private data pointer.

Output:
irq execute status.
 *******************************************************/
static irqreturn_t goodix_ts_irq_handler(int irq, void *dev_id)
{
	struct goodix_ts_data *ts = dev_id;

	GTP_DEBUG_FUNC();

	gtp_irq_disable(ts);
	queue_work(goodix_wq, &ts->work);

	return IRQ_HANDLED;
}

/*******************************************************
Function:
Eter sleep function.

Input:
ts:private data.

Output:
Executive outcomes.0！！success,non-0！！fail.
 *******************************************************/
static s8 gtp_enter_sleep(struct goodix_ts_data *ts)
{
	s8 ret = -1;
	s8 retry = 0;
	u8 i2c_control_buf[3] =
	    { (u8) (GTP_REG_SLEEP >> 8), (u8) GTP_REG_SLEEP, 1 };

	GTP_DEBUG_FUNC();

	while (retry++ < 5) {
		ret = gtp_i2c_write(ts->client, i2c_control_buf, 3);
		if (ret == 1) {
			GTP_DEBUG("GTP enter sleep!");
			return 0;
		}
		msleep(10);
	}
	GTP_ERROR("GTP send sleep cmd failed.");
	return retry;
}

/*******************************************************
Function:
Wakeup from sleep mode Function.

Input:
ts:	private data.

Output:
Executive outcomes.0！！success,non-0！！fail.
 *******************************************************/
static s8 gtp_wakeup_sleep(struct goodix_ts_data *ts)
{
	u8 retry = 0;
	s8 ret = -1;

	GTP_DEBUG_FUNC();

#if GTP_POWER_CTRL_SLEEP
	while (retry++ < 5) {
		gtp_reset_guitar(20);
		ret = gtp_send_cfg(ts->client);
		if (ret) {
			GTP_ERROR("Wakeup sleep send config failed.");
			continue;
		}
		GTP_DEBUG("Wakeup sleep send config success.");
		break;
	}
	return ret;
#else
	while (retry++ < 10) {
		GTP_GPIO_OUTPUT(GTP_INT_PORT, 0);
		msleep(2);
		GTP_GPIO_OUTPUT(GTP_INT_PORT, 1);
		msleep(2);
		GTP_GPIO_OUTPUT(GTP_INT_PORT, 0);
		msleep(10);
		GTP_GPIO_AS_INT(GTP_INT_PORT);
		msleep(200);
		ret = gtp_i2c_test(ts->client);
		if (!ret) {
			GTP_DEBUG("GTP wakeup sleep.");
			return 0;
		}
		msleep(10);
	}
	GTP_ERROR("GTP wakeup sleep failed.");
	return retry;
#endif
}

/*******************************************************
Function:
I2c test Function.

Input:
client:i2c client.

Output:
Executive outcomes.0！！success,non-0！！fail.
 *******************************************************/
static s8 gtp_i2c_test(struct i2c_client *client)
{
	u8 retry = 0;
	u8 test_data = 1;
	s8 ret = -1;

	GTP_DEBUG_FUNC();

	while (retry++ < 5) {
		ret = gtp_i2c_write(client, &test_data, 1);
		if (ret > 0) {
			return 0;
		}
		GTP_ERROR("GTP i2c test failed time %d.", retry);
		msleep(10);
	}
	return retry;
}

/*******************************************************
Function:
Reset chip Function.

Input:
ms:reset time.

Output:
None.
 *******************************************************/
void gtp_reset_guitar(s32 ms)
{
	GTP_GPIO_OUTPUT(GTP_RST_PORT, 0);
	msleep(ms);

	GTP_GPIO_AS_INPUT(GTP_RST_PORT);
	msleep(200);
}

/*******************************************************
Function:
Request gpio Function.

Input:
ts:private data.

Output:
Executive outcomes.0！！success,non-0！！fail.
 *******************************************************/
static s8 gtp_request_io_port(struct goodix_ts_data *ts)
{
	s32 ret = 0;

	ret = GTP_GPIO_REQUEST(GTP_INT_PORT, "GTP_INT_IRQ");
	if (ret < 0) {
		GTP_ERROR("Failed to request GPIO:%d, ERRNO:%d",
			  (s32) GTP_INT_PORT, ret);
		ret = -ENODEV;
	} else {
		GTP_GPIO_AS_INT(GTP_INT_PORT);
		ts->client->irq = GTP_INT_IRQ;
	}

	ret = GTP_GPIO_REQUEST(GTP_RST_PORT, "GTP_RST_PORT");
	if (ret < 0) {
		GTP_ERROR("Failed to request GPIO:%d, ERRNO:%d",
			  (s32) GTP_RST_PORT, ret);
		ret = -ENODEV;
	}

	GTP_GPIO_AS_INPUT(GTP_RST_PORT);
	gtp_reset_guitar(20);

	if (ret < 0) {
		GTP_GPIO_FREE(GTP_RST_PORT);
		GTP_GPIO_FREE(GTP_INT_PORT);
	}

	return ret;
}

/*******************************************************
Function:
Request irq Function.

Input:
ts:private data.

Output:
Executive outcomes.0！！success,non-0！！fail.
 *******************************************************/
static s8 gtp_request_irq(struct goodix_ts_data *ts)
{
	s32 ret = -1;
	const u8 irq_table[2] = GTP_IRQ_TAB;

	ret = request_irq(ts->client->irq,
			  goodix_ts_irq_handler,
			  irq_table[ts->int_trigger_type],
			  ts->client->name, ts);
	if (ret) {
		GTP_ERROR("Request IRQ failed!ERRNO:%d.", ret);
		GTP_GPIO_AS_INPUT(GTP_INT_PORT);
		GTP_GPIO_FREE(GTP_INT_PORT);
		hrtimer_init(&ts->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
		ts->timer.function = goodix_ts_timer_handler;
		hrtimer_start(&ts->timer, ktime_set(1, 0), HRTIMER_MODE_REL);
		return 1;
	} else {
		gtp_irq_disable(ts);
		ts->use_irq = 1;
		return 0;
	}
}

/*******************************************************
Function:
Request input device Function.

Input:
ts:private data.

Output:
Executive outcomes.0！！success,non-0！！fail.
 *******************************************************/
static s8 gtp_request_input_dev(struct goodix_ts_data *ts)
{
	s8 ret = -1;
	s8 phys[32];
#if GTP_HAVE_TOUCH_KEY
	u8 index = 0;
#endif
	u16 input_dev_x_max = ts->abs_x_max;
	u16 input_dev_y_max = ts->abs_y_max;

	GTP_DEBUG_FUNC();

	ts->input_dev = input_allocate_device();
	if (ts->input_dev == NULL) {
		GTP_ERROR("Failed to allocate input device.");
		return -ENOMEM;
	}

	ts->input_dev->evbit[0] =
	    BIT_MASK(EV_SYN) | BIT_MASK(EV_KEY) | BIT_MASK(EV_ABS);
	ts->input_dev->keybit[BIT_WORD(BTN_TOUCH)] = BIT_MASK(BTN_TOUCH);
	ts->input_dev->propbit[0] = BIT(INPUT_PROP_DIRECT);

#if GTP_HAVE_TOUCH_KEY
	for (index = 0; index < GTP_MAX_KEY_NUM; index++) {
		input_set_capability(ts->input_dev, EV_KEY,
				     touch_key_array[index]);
	}
#endif

#if GTP_CHANGE_X2Y
	GTP_SWAP(input_dev_x_max, input_dev_y_max);
#endif

#ifdef GTP_MULTI_TOUCH
	ts->input_dev->absbit[0] = BIT_MASK(ABS_MT_TRACKING_ID) |
	    BIT_MASK(ABS_MT_POSITION_X) | BIT_MASK(ABS_MT_POSITION_Y) |
	    BIT(ABS_PRESSURE);

	input_set_abs_params(ts->input_dev, ABS_PRESSURE, 0, 255, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_POSITION_X, 0,
			     input_dev_x_max, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_POSITION_Y, 0,
			     input_dev_y_max, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_TRACKING_ID, 0,
			     ts->max_touch_num, 0, 0);
#else
	ts->input_dev->absbit[0] = BIT(ABS_X) | BIT(ABS_Y) | BIT(ABS_PRESSURE);

	input_set_abs_params(ts->input_dev, ABS_X, 0, ts->abs_x_max, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_Y, 0, ts->abs_y_max, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_PRESSURE, 0, 255, 0, 0);
#endif

	sprintf(phys, "input/ts");
	ts->input_dev->name = goodix_ts_name;
	ts->input_dev->phys = phys;
	ts->input_dev->id.bustype = BUS_I2C;
	ts->input_dev->id.vendor = 0xDEAD;
	ts->input_dev->id.product = 0xBEEF;
	ts->input_dev->id.version = 10427;

	ret = input_register_device(ts->input_dev);
	if (ret) {
		GTP_ERROR("Register %s input device failed",
			  ts->input_dev->name);
		return -ENODEV;
	}
#ifdef CONFIG_HAS_EARLYSUSPEND
	ts->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	ts->early_suspend.suspend = goodix_ts_early_suspend;
	ts->early_suspend.resume = goodix_ts_late_resume;
	register_early_suspend(&ts->early_suspend);
#endif
	return 0;
}

/*******************************************************
Function:
Goodix touchscreen probe function.

Input:
client:	i2c device struct.
id:device id.

Output:
Executive outcomes. 0---succeed.
 *******************************************************/
static int goodix_ts_probe(struct i2c_client *client,
			   const struct i2c_device_id *id)
{
	s32 ret = -1;
	struct goodix_ts_data *ts;

	GTP_DEBUG_FUNC();
	//do NOT remove these output log
	GTP_INFO("GTP Driver Version:%s", GTP_DRIVER_VERSION);
	GTP_INFO("GTP Driver build@%s,%s", __TIME__, __DATE__);

	i2c_connect_client = client;
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		GTP_ERROR("I2C check functionality failed.");
		return -ENODEV;
	}
	ts = kzalloc(sizeof(*ts), GFP_KERNEL);
	if (ts == NULL) {
		GTP_ERROR("Alloc GFP_KERNEL memory failed.");
		return -ENOMEM;
	}

	memset(ts, 0, sizeof(*ts));
	INIT_WORK(&ts->work, goodix_ts_work_func);
	ts->client = client;
	i2c_set_clientdata(client, ts);

	ret = gtp_request_io_port(ts);
	if (ret) {
		GTP_ERROR("GTP request IO port failed.");
		kfree(ts);
		return ret;
	}

	ret = gtp_i2c_test(client);
	if (ret) {
		GTP_ERROR("I2C communication ERROR!.");
	}
#if GTP_AUTO_UPDATE
	ret = gtp_read_version(ts);
	if (ret) {
		GTP_ERROR("Read version failed.");
	} else {
		ret = gup_downloader(ts, goodix_gt811_firmware);
		if (ret < 0) {
			GTP_ERROR("Warnning:update might be ERROR!\n");
		}
	}
#endif
	ret = gtp_init_panel(ts);
	if (ret) {
		GTP_ERROR("GTP init panel failed.");
	}

	ret = gtp_request_input_dev(ts);
	if (ret) {
		GTP_ERROR("GTP request input dev failed");
	}

	ret = gtp_request_irq(ts);
	if (ret) {
		GTP_INFO("GTP works in polling mode.");
	} else {
		GTP_INFO("GTP works in interrupt mode.");
	}

	ret = gtp_read_version(ts);
	if (ret) {
		GTP_ERROR("Read version failed.");
	}
	gtp_irq_enable(ts);

#ifdef GTP_CREATE_WR_NODE
	init_wr_node(client);
#endif

#if GTP_ESD_PROTECT
	gtp_esd_check_workqueue = create_workqueue("gtp_esd_check");
	INIT_DELAYED_WORK(&gtp_esd_check_work, gtp_esd_check_func);
	ret = queue_delayed_work(gtp_esd_check_workqueue, &gtp_esd_check_work,
				 GTP_ESD_CHECK_CIRCLE);
	if (ret) {
		GTP_ERROR("Create ESD check thread failed!");
	}
#endif

	return 0;
}

/*******************************************************
Function:
Goodix touchscreen driver release function.

Input:
client:	i2c device struct.

Output:
Executive outcomes. 0---succeed.
 *******************************************************/
static int goodix_ts_remove(struct i2c_client *client)
{
	struct goodix_ts_data *ts = i2c_get_clientdata(client);

	GTP_DEBUG_FUNC();

#ifdef CONFIG_HAS_EARLYSUSPEND
	unregister_early_suspend(&ts->early_suspend);
#endif

#ifdef GTP_CREATE_WR_NODE
	uninit_wr_node();
#endif

#if GTP_ESD_PROTECT
	destroy_workqueue(gtp_esd_check_workqueue);
#endif

	if (ts) {
		if (ts->use_irq) {
			GTP_GPIO_AS_INPUT(GTP_INT_PORT);
			GTP_GPIO_FREE(GTP_INT_PORT);
			free_irq(client->irq, ts);
		} else {
			hrtimer_cancel(&ts->timer);
		}
	}

	GTP_INFO("GTP driver is removing...");
	i2c_set_clientdata(client, NULL);
	input_unregister_device(ts->input_dev);
	kfree(ts);
	return 0;
}

/*******************************************************
Function:
Early suspend function.

Input:
h:early_suspend struct.

Output:
None.
 *******************************************************/
#ifdef CONFIG_HAS_EARLYSUSPEND
static void goodix_ts_early_suspend(struct early_suspend *h)
{
	struct goodix_ts_data *ts;
	s8 ret = -1;
	ts = container_of(h, struct goodix_ts_data, early_suspend);

	GTP_DEBUG_FUNC();

#if GTP_ESD_PROTECT
	ts->gtp_is_suspend = 1;
	cancel_delayed_work_sync(&gtp_esd_check_work);
#endif

	if (ts->use_irq) {
		gtp_irq_disable(ts);
	} else {
		hrtimer_cancel(&ts->timer);
	}
	ret = gtp_enter_sleep(ts);
	if (ret) {
		GTP_ERROR("GTP early suspend failed.");
	}
}

/*******************************************************
Function:
Late resume function.

Input:
h:early_suspend struct.

Output:
None.
 *******************************************************/
static void goodix_ts_late_resume(struct early_suspend *h)
{
	struct goodix_ts_data *ts;
	s8 ret = -1;
	ts = container_of(h, struct goodix_ts_data, early_suspend);

	GTP_DEBUG_FUNC();

	ret = gtp_wakeup_sleep(ts);
	if (ret) {
		GTP_ERROR("GTP later resume failed.");
	}

	if (ts->use_irq) {
		gtp_irq_enable(ts);
	} else {
		hrtimer_start(&ts->timer, ktime_set(1, 0), HRTIMER_MODE_REL);
	}

#if GTP_ESD_PROTECT
	ts->gtp_is_suspend = 0;
	queue_delayed_work(gtp_esd_check_workqueue, &gtp_esd_check_work,
			   GTP_ESD_CHECK_CIRCLE);
#endif
}
#endif

#if GTP_ESD_PROTECT
static void gtp_esd_check_func(struct work_struct *work)
{
	int i = 0;
	int ret = -1;
	struct goodix_ts_data *ts = NULL;
	//    u8 buf[2] = {GTP_REG_CONFIG_DATA >> 8 & 0xff, GTP_REG_CONFIG_DATA & 0xff};

	GTP_DEBUG_FUNC();

	ts = i2c_get_clientdata(i2c_connect_client);

	for (i = 0; ts->irq_is_disable; i++) {
		msleep(1);

		if (i >= 5) {
			return;
		}
	}

	for (i = 0; i < 3; i++) {
		ret = gtp_i2c_end_cmd(i2c_connect_client);
		if (ret >= 0) {
			break;
		}
	}

	if (i >= 3) {
		gtp_reset_guitar(50);
	}

	if (!ts->gtp_is_suspend) {
		queue_delayed_work(gtp_esd_check_workqueue, &gtp_esd_check_work,
				   GTP_ESD_CHECK_CIRCLE);
	}

	return;
}
#endif

static const struct i2c_device_id goodix_ts_id[] = {
	{GTP_I2C_NAME, 0},
	{}
};

static struct i2c_driver goodix_ts_driver = {
	.probe = goodix_ts_probe,
	.remove = goodix_ts_remove,
#ifndef CONFIG_HAS_EARLYSUSPEND
	.suspend = goodix_ts_early_suspend,
	.resume = goodix_ts_late_resume,
#endif
	.id_table = goodix_ts_id,
	.driver = {
		   .name = GTP_I2C_NAME,
		   .owner = THIS_MODULE,
		   },
};

/*******************************************************
Function:
Driver Install function.
Input:
None.
Output:
Executive Outcomes. 0---succeed.
 ********************************************************/
static int __devinit goodix_ts_init(void)
{
	s32 ret;

	GTP_DEBUG_FUNC();
	GTP_INFO("GTP driver install.");
	goodix_wq = create_singlethread_workqueue("goodix_wq");
	if (!goodix_wq) {
		GTP_ERROR("Creat workqueue failed.");
		return -ENOMEM;
	}
	ret = i2c_add_driver(&goodix_ts_driver);
	return ret;
}

/*******************************************************
Function:
Driver uninstall function.
Input:
None.
Output:
Executive Outcomes. 0---succeed.
 ********************************************************/
static void __exit goodix_ts_exit(void)
{
	GTP_DEBUG_FUNC();
	GTP_INFO("GTP driver exited.");
	i2c_del_driver(&goodix_ts_driver);
	if (goodix_wq) {
		destroy_workqueue(goodix_wq);
	}
}

late_initcall(goodix_ts_init);
module_exit(goodix_ts_exit);

MODULE_DESCRIPTION("GTP Series Driver");
MODULE_LICENSE("GPL");
