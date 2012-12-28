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
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/mfd/tps80032.h>
#include <linux/mutex.h>
#include <linux/timer.h>
#include <linux/slab.h>
#include <linux/completion.h>

//reg id = 2
#define GPADC_REG_ID	2
#define TPS80032_REG_GPADC_CTRL		0x2E
#define TPS80032_REG_GPADC_CTRL2	0x2F
#define TPS80032_REG_RTSELECT_LSB	0x32
#define TPS80032_REG_CTRL_P1		0x36
#define TPS80032_REG_GPSELECT_ISB	0x35
#define TPS80032_REG_RTCH0_LSB		0x37
#define TPS80032_REG_RTCH1_LSB		0x39
#define TPS80032_REG_GPCH0_LSB		0x3B
#define TPS80032_REG_GPCH0_MSB		0x3C
#define TPS80032_REG_TOGGLE1		0x90
#define TPS80032_REG_VBUS_CTRL_SET	0x04
#define TPS80032_REG_USB_ID_CTRL_SET	0x06
//reg id = 1
#define TPS80032_REG_MISC1			0xE4

#define TPS80032_GPADC_TRIM1	3, 0xCD
#define TPS80032_GPADC_TRIM2	3, 0xCE
#define TPS80032_GPADC_TRIM3	3, 0xCF
#define TPS80032_GPADC_TRIM4	3, 0xD0
#define TPS80032_GPADC_TRIM5	3, 0xD1
#define TPS80032_GPADC_TRIM6	3, 0xD2
#define TPS80032_GPADC_TRIM7	3, 0xD3
#define TPS80032_GPADC_TRIM8	3, 0xD4
#define TPS80032_GPADC_TRIM9	3, 0xD5
#define TPS80032_GPADC_TRIM10	3, 0xD6
#define TPS80032_GPADC_TRIM11	3, 0xD7
#define TPS80032_GPADC_TRIM12	3, 0xD8
#define TPS80032_GPADC_TRIM13	3, 0xD9
#define TPS80032_GPADC_TRIM14	3, 0xDA
#define TPS80032_GPADC_TRIM15	3, 0xDB
#define TPS80032_GPADC_TRIM16	3, 0xDC
#define TPS80032_GPADC_TRIM17	3, 0xDD
#define TPS80032_GPADC_TRIM18	3, 0xDE
#define TPS80032_GPADC_TRIM19	3, 0xFD
#define TPS80032_GPADC_TRIM20	3, 0xFE
#define TPS80032_GPADC_TRIM21	3, 0xFF

#define GPADCCS		1 << 1
#define GPADCCR		1
#define SP1			1 << 3
#define COLLISION_GP	1 << 4
#define GPADC_SAMP_WINDOW 1 << 2

#define SENSE_RESISTOR	20 //mohm
#define TPS80032_GPADC_VREF	1250
#define TPS80032_GPADC_TIMEOUT	(3 * HZ)

struct tps80032_adc_reg_set {
	u8 enable;
	u8 reg_id;
	u8 reg_addr;
	u8 pos;
	u8 mask;
};

struct tps80032_adc_scalar_map {
	int rang;
	u8 reg_value;
};

struct tps80032_adc_channel_data {
	int channel;
	struct tps80032_adc_reg_set scalar;
	struct tps80032_adc_scalar_map scalar_map[2];
	struct tps80032_adc_reg_set cur_source;
	struct tps80032_adc_reg_set enable;
	int gain;
	int offset;
};

#define TPS80032_ADC_DATA(_chan, _sclr_en, _sclr_reg_id, _sclr_reg_addr, _sclr_pos, _sclr_mask,	\
		_map0_rang, _map0_val, _map1_rang, _map1_val,	\
		_cur_en, _cur_reg_id, _cur_reg_addr, _cur_pos, _cur_mask,	\
		_en_en, _en_reg_id, _en_reg_addr, _en_pos, _en_mask)		\
{	\
	.channel = _chan,	\
	.scalar = {	\
		.enable = _sclr_en,	\
		.reg_id = _sclr_reg_id, 	\
		.reg_addr = _sclr_reg_addr, 	\
		.pos = _sclr_pos,	\
		.mask = _sclr_mask,	\
	},	\
	.scalar_map = {	\
		{_map0_rang, _map0_val},	\
		{_map1_rang, _map1_val},	\
	},	\
	.cur_source = {	\
		.enable = _cur_en,	\
		.reg_id = _cur_reg_id,	\
		.reg_addr = _cur_reg_addr,	\
		.pos = _cur_pos,	\
		.mask = _cur_mask,	\
	},	\
	.enable = {	\
		.enable = _en_en,	\
		.reg_id = _en_reg_id,	\
		.reg_addr = _en_reg_addr,	\
		.pos = _en_pos,	\
		.mask = _en_mask,	\
	},	\
}


/*_chan, _sclr_en, _sclr_reg_id, _sclr_reg_addr, _sclr_pos, _sclr_mask,
		_map0_rang, _map0_val, _map1_rang, _map1_val,
		_cur_en, _cur_reg_id, _cur_reg_addr, _cur_pos, _cur_mask,
		_en_en, _en_reg_id, _en_reg_addr, _en_pos, _en_mask)*/
static struct tps80032_adc_channel_data tps80032_adc_channel_map[] = {
	TPS80032_ADC_DATA(TPS80032_ADC0_BAT_TYPE, 0, 0, 0, 0, 0,
			1250, 0, 0, 0,  1, 2, TPS80032_REG_GPADC_CTRL, 7, 0x80,  0, 0, 0, 0, 0),
	TPS80032_ADC_DATA(TPS80032_ADC1_BAT_TEMP, 0, 0, 0, 0, 0,
			1250, 0, 0, 0,  0, 0, 0, 0, 0,  1, 2, TPS80032_REG_GPADC_CTRL, 0, 0x01),
	TPS80032_ADC_DATA(TPS80032_ADC2_AUDIO_GP, 1, 2, TPS80032_REG_GPADC_CTRL, 2, 0x04,
			1250, 0, 1875, 1,  0, 0, 0, 0, 0,  0, 0, 0, 0, 0),
	TPS80032_ADC_DATA(TPS80032_ADC3_TEMP_DIODE_GP, 0, 0, 0, 0, 0,
			1250, 0, 0, 0,  1, 2, TPS80032_REG_GPADC_CTRL2, 0, 0x03,  0, 0, 0, 0, 0),
	TPS80032_ADC_DATA(TPS80032_ADC4_TEMP_GP, 0, 0, 0, 0, 0,
			1250, 0, 0, 0,  0, 0, 0, 0, 0,  0, 0, 0, 0, 0),
	TPS80032_ADC_DATA(TPS80032_ADC5_GP, 0, 0, 0, 0, 0,
			1250, 0, 0, 0,  0, 0, 0, 0, 0,  0, 0, 0, 0, 0),
	TPS80032_ADC_DATA(TPS80032_ADC6_GP, 0, 0, 0, 0, 0,
			1250, 0, 0, 0,  0, 0, 0, 0, 0,  0, 0, 0, 0, 0),
	TPS80032_ADC_DATA(TPS80032_ADC7_SYS, 1, 2, TPS80032_REG_GPADC_CTRL, 3, 0x08,
			6250, 0, 5000, 1,  0, 0, 0, 0, 0,  1, 1, TPS80032_REG_MISC1, 1, 0x02),
	TPS80032_ADC_DATA(TPS80032_ADC8_BACKUP_BAT, 1, 1, TPS80032_REG_MISC1, 0, 0x01,
			1250, 0, 6250, 1,  0, 0, 0, 0, 0,  0, 0, 0, 0, 0),
	TPS80032_ADC_DATA(TPS80032_ADC9_EXT_CHG, 1, 1, TPS80032_REG_MISC1, 2, 0x04,
			1250, 0, 11250, 1,  0, 0, 0, 0, 0,  0, 0, 0, 0, 0),
	TPS80032_ADC_DATA(TPS80032_ADC10_VBUS, 0, 0, 0, 0, 0,
			6875, 0, 0, 0,  0, 0, 0, 0, 0,  1, 2, TPS80032_REG_VBUS_CTRL_SET, 0, 0x01),
	TPS80032_ADC_DATA(TPS80032_ADC11_DCDC_CUR, 1, 2, TPS80032_REG_GPADC_CTRL, 4, 0x10,
			1250, 0, 1875, 1,  0, 0, 0, 0, 0,  0, 0, 0, 0, 0),
	TPS80032_ADC_DATA(TPS80032_ADC12_DIE_TEMP, 0, 0, 0, 0, 0,
			1250, 0, 0, 0,  0, 0, 0, 0, 0,  1, 2, TPS80032_REG_GPADC_CTRL, 5, 0x20),
	TPS80032_ADC_DATA(TPS80032_ADC13_DIE_TEMP, 0, 0, 0, 0, 0,
			1250, 0, 0, 0,  0, 0, 0, 0, 0,  1, 2, TPS80032_REG_GPADC_CTRL, 6, 0x40),
	TPS80032_ADC_DATA(TPS80032_ADC14_USB_ID, 0, 0, 0, 0, 0,
			6875, 0, 0, 0,  0, 0, 0, 0, 0,  1, 2, TPS80032_REG_USB_ID_CTRL_SET, 0, 0x01),
	TPS80032_ADC_DATA(TPS80032_ADC15_TEST_NET, 0, 0, 0, 0, 0,
			6250, 0, 0, 0,  0, 0, 0, 0, 0,  0, 0, 0, 0, 0),
	TPS80032_ADC_DATA(TPS80032_ADC16_TEST_NET, 0, 0, 0, 0, 0,
			4750, 0, 0, 0,  0, 0, 0, 0, 0,  0, 0, 0, 0, 0),
	TPS80032_ADC_DATA(TPS80032_ADC17_BAT_CUR, 0, 0, 0, 0, 0,
			1250, 0, 0, 0,  0, 0, 0, 0, 0,  0, 0, 0, 0, 0),
	TPS80032_ADC_DATA(TPS80032_ADC18_BAT_VOLT, 1, 2, TPS80032_REG_GPADC_CTRL2, 3, 0x08,
			6250, 0, 5000, 1,  0, 0, 0, 0, 0,  1, 2, TPS80032_REG_GPADC_CTRL2, 2, 0x04),
};

struct tps80032_cur_map {
	int cur;
	u8 reg_value;
};

#define TPS80032_CUR_MAP(_cur, _val)	\
{	\
	.cur = _cur,	\
	.reg_value = _val,	\
}

static struct tps80032_cur_map tps80032_adc0_cur_map[] = {
	TPS80032_CUR_MAP(7,  0),
	TPS80032_CUR_MAP(15, 1),
};

static struct tps80032_cur_map tps80032_adc3_cur_map[] = {
	TPS80032_CUR_MAP(0,  0),
	TPS80032_CUR_MAP(10, 1),
	TPS80032_CUR_MAP(400,2),
	TPS80032_CUR_MAP(800,3),
};

struct tps80032_current_source {
	int channel;
	int map_size;
	struct tps80032_cur_map *map;
};

#define TPS80032_CUR_SOURCE(_chan, _map) \
{	\
	.channel = _chan,	\
	.map_size =	ARRAY_SIZE(_map),	\
	.map = _map,	\
}

static struct tps80032_current_source tps80032_cur_source_map[] = {
	TPS80032_CUR_SOURCE(TPS80032_ADC0_BAT_TYPE, tps80032_adc0_cur_map),
	TPS80032_CUR_SOURCE(TPS80032_ADC3_TEMP_DIODE_GP, tps80032_adc3_cur_map),
};

struct tps80032_adc_info {
	struct device * dev;
	struct completion sw2_comp;
	struct mutex sw2_lock;
	struct tps80032_adc_platform_data *pdata;
	int sw2_irq;
	int tmp_scalar;
	int tmp_cur;
	int tmp_value;
};

static struct tps80032_adc_info *g_info = NULL;

static int tps80032_adc_channel_init(struct tps80032_adc_info * info,
		struct tps80032_adc_channel_data * chan_data)
{
	int ret, i, j;
	u8 val;

	if(info->tmp_scalar > 0){
		for (i = 0, val = 0; i < 2; ++i){
			if (info->tmp_scalar == chan_data->scalar_map[i].rang){
				val = chan_data->scalar_map[i].reg_value;
				break;
			}
		}
		if (i >= 2){
			dev_err(info->dev, "channel:%d: wrong scalar:%d! use defualt: %d\n", 
					chan_data->channel, info->tmp_scalar, chan_data->scalar_map[0].rang);
			info->tmp_scalar = chan_data->scalar_map[0].rang;
		}
	}else{
		info->tmp_scalar = chan_data->scalar_map[0].rang;
	}
	if (chan_data->scalar.enable){
		ret = tps80032_reg_update(chan_data->scalar.reg_id, chan_data->scalar.reg_addr, 
				val << chan_data->scalar.pos, chan_data->scalar.mask);
		if (ret < 0){
			dev_err(info->dev, "channel:%d set scalar failed:%d\n", chan_data->channel, ret);
			return -1;
		}
	}
	if (chan_data->cur_source.enable && info->tmp_cur){
		for (i = 0, val = 0; i < ARRAY_SIZE(tps80032_cur_source_map); ++i){
			if (chan_data->channel == tps80032_cur_source_map[i].channel){
				for (j = 0; j < tps80032_cur_source_map[i].map_size; ++j){
					if (info->tmp_cur== tps80032_cur_source_map[i].map[j].cur){
						val = tps80032_cur_source_map[i].map[j].reg_value;
						goto find_cur;
					}
				}
			}
		}
		dev_err(info->dev, "channel:%d wrong current source: %d. use defualt: 0\n",
				chan_data->channel, info->tmp_cur);
		info->tmp_cur = 0;
find_cur:
		ret = tps80032_reg_update(chan_data->cur_source.reg_id, chan_data->cur_source.reg_addr, 
				val << chan_data->cur_source.pos, chan_data->cur_source.mask);
		if (ret < 0){
			dev_err(info->dev, "channel:%d set current source failed:%d\n", chan_data->channel, ret);
			return -1;
		}
	}
	if (chan_data->enable.enable){
		ret = tps80032_reg_update(chan_data->enable.reg_id, chan_data->enable.reg_addr, 
				1 << chan_data->enable.pos, chan_data->enable.mask);
		if (ret < 0){
			dev_err(info->dev, "channel:%d set enable failed:%d\n", chan_data->channel, ret);
			return -1;
		}
	}

	return 0;
}

static int tps80032_adc_channel_disable(struct tps80032_adc_info * info,
		struct tps80032_adc_channel_data * chan_data)
{
	int ret;

	if (chan_data->enable.enable){
		ret = tps80032_reg_update(chan_data->enable.reg_id, chan_data->enable.reg_addr, 
				0, chan_data->enable.mask);
		if (ret < 0){
			dev_err(info->dev, "channel:%d set enable failed:%d\n", chan_data->channel, ret);
			return -1;
		}
	}
	ret = tps80032_set_bits(GPADC_REG_ID, TPS80032_REG_TOGGLE1, GPADCCR);
	if (ret < 0){
		dev_err(info->dev, "disable GPADC failed: %d!\n", ret);
	}

	return 0;
}

static int tps80032_adc_channel_start(struct tps80032_adc_info * info,
		struct tps80032_adc_channel_data * chan_data)
{
	int ret;

	ret = tps80032_set_bits(GPADC_REG_ID, TPS80032_REG_TOGGLE1, GPADC_SAMP_WINDOW);
	if (ret < 0){
		dev_err(info->dev, "set GPADC sample window failed: %d!\n", ret);
		return -1;
	}
	ret = tps80032_set_bits(GPADC_REG_ID, TPS80032_REG_TOGGLE1, GPADCCS);
	if (ret < 0){
		dev_err(info->dev, "enable GPADC failed: %d!\n", ret);
		return -1;
	}
	ret = tps80032_write(GPADC_REG_ID, TPS80032_REG_GPSELECT_ISB, chan_data->channel);
	if (ret < 0){
		dev_err(info->dev, "select ADC channel failed: %d!\n", ret);
		return -1;
	}
	ret = tps80032_set_bits(GPADC_REG_ID, TPS80032_REG_CTRL_P1, SP1);
	if (ret < 0){
		dev_err(info->dev, "start GPADC failed: %d!\n", ret);
		return -1;
	}

	return 0;
}

static int tps80032_adc_channel_value(struct tps80032_adc_info * info,
		struct tps80032_adc_channel_data * chan_data)
{
	u8 adc_data[2];
	const char *obj;
	const char *unit;
	int ret = 0;
	int data = 0;

	init_completion(&info->sw2_comp);
	ret = wait_for_completion_interruptible_timeout(&info->sw2_comp,
			TPS80032_GPADC_TIMEOUT);
	if (!ret){
		dev_err(info->dev, "wait GPADC channel:%d conversion time out!\n",
				chan_data->channel);
		return -1;
	}

	ret = tps80032_bulk_reads(GPADC_REG_ID, TPS80032_REG_GPCH0_LSB, 2, adc_data);
	if (ret < 0){
		dev_err(info->dev, "read adc channel:%d data failed: %d!\n",
				chan_data->channel, ret);
		return -1;
	}
	if (adc_data[1] & COLLISION_GP){
		dev_err(info->dev, "the adc channel:%d conversion isn't end!\n",
				chan_data->channel);
		return -1;
	}
	if (chan_data->channel == TPS80032_ADC17_BAT_CUR){
		data = adc_data[0] & 0x7f;
		if (adc_data[0] & 0x80){
			data = -data;
		}
		data = (data - chan_data->offset) * 100 / chan_data->gain;
		data = data * TPS80032_GPADC_VREF * 1000 / SENSE_RESISTOR / 4095;
		obj = "current";
		unit = "mA";
	}else{
		data = adc_data[1] & 0x0f;
		data = (data << 8) | adc_data[0];
		data = (data - chan_data->offset) * 100 / chan_data->gain;
		data = data * TPS80032_GPADC_VREF / 4095;
		data *= info->tmp_scalar / TPS80032_GPADC_VREF;
		obj = "voltage";
		unit = "mV";
	}
	dev_info(info->dev, "adc channel:%d GPCH0_LSB(0x3b): 0x%x\n",
			chan_data->channel, adc_data[0]);
	dev_info(info->dev, "adc channel:%d GPCH0_MSB(0x3c): 0x%x\n",
			chan_data->channel, adc_data[1]);
	dev_info(info->dev, "channel:%d %s: %d%s\n",
			chan_data->channel, obj, data, unit);

	info->tmp_value = data;

	return 0;
}

static struct tps80032_adc_channel_data * tps80032_find_channel(
		struct tps80032_adc_info *info, int channel)
{
	int i;

	if (channel < 0 || channel >= TPS80032_NR_ADC){
		dev_err(info->dev, "the channel %d isn't in range(0-%d).\n",
				channel, TPS80032_NR_ADC);
		return NULL;
	}
	for (i = 0; i < ARRAY_SIZE(tps80032_adc_channel_map); ++i){
		if (channel == tps80032_adc_channel_map[i].channel){
			return &tps80032_adc_channel_map[i];
		}
	}
	dev_err(info->dev, "Can't find the channel: %d information!\n", channel);

	return NULL;
}

int tps80032_get_adc_value(int channel, int scalar, int cur_source)
{
	struct tps80032_adc_info *info = g_info;
	struct tps80032_adc_channel_data * chan_data;
	int ret;

	mutex_lock(&info->sw2_lock);
	if (!info){
		printk(KERN_ERR "tps80032_adc device is NULL\n");
		goto err;
	}
	chan_data = tps80032_find_channel(info, channel);
	if (!chan_data){
		goto err;
	}
	info->tmp_scalar = scalar;
	info->tmp_cur = cur_source;
	ret = tps80032_adc_channel_init(info, chan_data);
	if (ret < 0){
		goto err;
	}
	ret = tps80032_adc_channel_start(info, chan_data);
	if (ret < 0){
		goto err;
	}
	ret = tps80032_adc_channel_value(info, chan_data);
	if (ret < 0){
		goto err;
	}
	ret = tps80032_adc_channel_disable(info, chan_data);
	if (ret < 0){
		goto err;
	}
	mutex_unlock(&info->sw2_lock);

	return info->tmp_value;
err:
	mutex_unlock(&info->sw2_lock);
	return 0;
}
EXPORT_SYMBOL_GPL(tps80032_get_adc_value);

static irqreturn_t tps80032_sw2_irq(int irq, void * _info)
{
	struct tps80032_adc_info * info = _info;

	complete(&info->sw2_comp);

	return IRQ_HANDLED;
}

static int tps80032_adc_offset_init(struct tps80032_adc_info *info)
{
	int i, ret, gain, offset;
	int d1, d2, d11, d22;
	u8 reg_val[4];

	ret = tps80032_bulk_reads(TPS80032_GPADC_TRIM1, 4, reg_val);
	if (ret < 0){
		dev_err(info->dev, "read GPADC TRIM failed: %d\n", ret);
		return -1;
	}
	d1 = (reg_val[2] & 0x1f) * 4 + (reg_val[0] & 0x06);
	if (reg_val[0] & 0x01){
		d1 = -d1;
	}
	d2 = (reg_val[3] & 0x3f) * 4 + (reg_val[1] & 0x06);
	if (reg_val[1] & 0x01){
		d2 = -d2;
	}
	gain = (d2 - d1) * 100 / (3276 - 1441);
	offset = d1 - 1441 * gain / 100;
	gain += 100;
	tps80032_adc_channel_map[0].gain = gain;
	tps80032_adc_channel_map[0].offset = offset;
	tps80032_adc_channel_map[1].gain = gain;
	tps80032_adc_channel_map[1].offset = offset;
	tps80032_adc_channel_map[2].gain = gain;
	tps80032_adc_channel_map[2].offset = offset;
	tps80032_adc_channel_map[3].gain = gain;
	tps80032_adc_channel_map[3].offset = offset;
	tps80032_adc_channel_map[4].gain = gain;
	tps80032_adc_channel_map[4].offset = offset;
	tps80032_adc_channel_map[5].gain = gain;
	tps80032_adc_channel_map[5].offset = offset;
	tps80032_adc_channel_map[6].gain = gain;
	tps80032_adc_channel_map[6].offset = offset;
	tps80032_adc_channel_map[11].gain = gain;
	tps80032_adc_channel_map[11].offset = offset;
	tps80032_adc_channel_map[12].gain = gain;
	tps80032_adc_channel_map[12].offset = offset;
	tps80032_adc_channel_map[13].gain = gain;
	tps80032_adc_channel_map[13].offset = offset;
	tps80032_adc_channel_map[14].gain = gain;
	tps80032_adc_channel_map[14].offset = offset;
	dev_info(info->dev, "channel 1,2,3,4,5,6,11,12,13,14: gain:%d%%"
			"offset:%d\n", gain, offset);

	ret = tps80032_bulk_reads(TPS80032_GPADC_TRIM7, 4, reg_val);
	if (ret < 0){
		dev_err(info->dev, "read GPADC TRIM failed: %d\n", ret);
		return -1;
	}
	d11 = (reg_val[1] & 0x18) * 16 + (reg_val[0] & 0x1e);
	if (reg_val[0] & 0x01){
		d11 = -d11;
	}
	d11 += d1;
	d22 = (reg_val[3] & 0x1f) * 4 + (reg_val[1] & 0x06);
	if (reg_val[1] & 0x01){
		d22 = -d22;
	}
	d22 += d2;
	gain = (d22 - d11) * 100 / (3276 - 1441);
	offset = d11 - 1441 * gain / 100;
	gain += 100;
	tps80032_adc_channel_map[8].gain = gain;
	tps80032_adc_channel_map[8].offset = offset;
	dev_info(info->dev, "channel 8: gain:%d%%, offset:%d\n", gain, offset);

	ret = tps80032_read(TPS80032_GPADC_TRIM12, reg_val);
	ret |= tps80032_read(TPS80032_GPADC_TRIM14, reg_val + 1);
	ret |= tps80032_read(TPS80032_GPADC_TRIM16, reg_val + 2);
	if (ret < 0){
		dev_err(info->dev, "read GPADC TRIM failed: %d\n", ret);
		return -1;
	}
	d11 = (reg_val[1] & 0x18) * 16 + (reg_val[0] & 0x1e);
	if (reg_val[0] & 0x01){
		d11 = -d11;
	}
	d11 += d1;
	d22 = (reg_val[2] & 0x1f) * 4 + (reg_val[1] & 0x06);
	if (reg_val[1] & 0x01){
		d22 = -d22;
	}
	d22 += d2;
	gain = (d22 - d11) * 100 / (3276 - 1441);
	offset = d11 - 1441 * gain / 100;
	gain += 100;
	tps80032_adc_channel_map[9].gain = gain;
	tps80032_adc_channel_map[9].offset = offset;
	dev_info(info->dev, "channel 9: gain:%d%%, offset:%d\n", gain, offset);

	ret = tps80032_read(TPS80032_GPADC_TRIM9, reg_val);
	ret |= tps80032_read(TPS80032_GPADC_TRIM11, reg_val + 1);
	ret |= tps80032_read(TPS80032_GPADC_TRIM13, reg_val + 2);
	ret |= tps80032_read(TPS80032_GPADC_TRIM15, reg_val + 3);
	if (ret < 0){
		dev_err(info->dev, "read GPADC TRIM failed: %d\n", ret);
		return -1;
	}
	d11 = (reg_val[1] & 0x0f) * 8 + (reg_val[0] & 0x0e);
	if (reg_val[0] & 0x01){
		d11 = -d11;
	}
	d22 = (reg_val[3] & 0x0f) * 8 + (reg_val[2] & 0x0e);
	if (reg_val[2] & 0x01){
		d22 = -d22;
	}
	gain = (d22 - d11) * 100 / (745 - 149);
	offset = d11 - 149 * gain / 100;
	gain += 100;
	tps80032_adc_channel_map[10].gain = gain;
	tps80032_adc_channel_map[10].offset = offset;
	dev_info(info->dev, "channel 10: gain:%d%%, offset:%d\n", gain, offset);

	ret = tps80032_bulk_reads(TPS80032_GPADC_TRIM5, 2, reg_val);
	if (ret < 0){
		dev_err(info->dev, "read GPADC TRIM failed: %d\n", ret);
		return -1;
	}
	d11 = reg_val[0] & 0x7e;
	if (reg_val[0] & 0x01){
		d11 = -d11;
	}
	d11 += d1;
	d22 = reg_val[1] & 0xfe;
	if (reg_val[1] & 0x01){
		d22 = -d22;
	}
	d22 += d2;
	gain = (d22 - d11) * 100 / (3276 - 1441);
	offset = d11 - 1441 * gain / 100;
	gain += 100;
	tps80032_adc_channel_map[7].gain = gain;
	tps80032_adc_channel_map[7].offset = offset;
	tps80032_adc_channel_map[18].gain = gain;
	tps80032_adc_channel_map[18].offset = offset;
	dev_info(info->dev, "channel 7,18: gain:%d%%, offset:%d\n", gain, offset);

	for (i = 0; i < TPS80032_NR_ADC; ++i){
		if (tps80032_adc_channel_map[i].gain == 0){
			tps80032_adc_channel_map[i].gain = 100;
			tps80032_adc_channel_map[i].offset = 0;
		}
	}

	return 0;
}

static int __devinit tps80032_adc_probe(struct platform_device *pdev)
{
	int ret;
	struct tps80032_adc_info * info;
	struct tps80032_adc_platform_data *pdata = pdev->dev.platform_data;

	if (!pdata) {
		dev_err(&pdev->dev, "gpadc platform data not supplied\n");
		return -EINVAL;
	}

	info = kzalloc(sizeof(*info), GFP_KERNEL);
	if (!info){
		return -ENOMEM;
	}
	info->dev = &pdev->dev;
	info->pdata   = pdata;
	info->sw2_irq = pdata->sw2_irq;

	platform_set_drvdata(pdev, info);

	mutex_init(&info->sw2_lock);
	ret = request_threaded_irq(info->sw2_irq, NULL, tps80032_sw2_irq,
			IRQF_ONESHOT, "tps80032_adc", info);
	if (ret < 0){
		dev_err(info->dev, "Can't get %d IRQ for tps80032 adc: %d\n",
				info->sw2_irq, ret);
		goto free_adc;
	}
	ret = tps80032_adc_offset_init(info);
	if (ret < 0){
		dev_err(info->dev, "offset init failed!\n");
		goto irq;
	}
	g_info = info;
	printk("%s is OK!\n", __func__);

	return 0;

irq:
	free_irq(info->sw2_irq, info);
free_adc:
	kfree(info);
	return ret;
}

static int __devexit tps80032_adc_remove(struct platform_device *pdev)
{
	struct tps80032_adc_info * info = platform_get_drvdata(pdev);

	free_irq(info->sw2_irq, info);
	kfree(info);

	return 0;
}

static struct platform_driver tps80032_adc_driver = {
	.probe		= tps80032_adc_probe,
	.remove		= __devexit_p(tps80032_adc_remove),
	.driver		= {
		.name	= "tps80032-adc",
		.owner	= THIS_MODULE,
	},
};

static int __init tps80032_adc_init(void)
{
	return platform_driver_register(&tps80032_adc_driver);
}
module_init(tps80032_adc_init);

static void __exit tps80032_adc_exit(void)
{
	platform_driver_unregister(&tps80032_adc_driver);
}
module_exit(tps80032_adc_exit);

MODULE_AUTHOR("Wu Jianguo<jianguo.wu@nufront.com>");
MODULE_ALIAS("platform:tps80032-gpadc");
MODULE_DESCRIPTION("tps80032 general purpose ADC");
MODULE_LICENSE("GPL v2");
