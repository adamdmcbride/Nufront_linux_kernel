/* drivers/input/touchscreen/gt811_update.c
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
 *
 */
#include <linux/kthread.h>
#include "gt811.h"

static s8 *fw_path;
u16 show_len;
u16 total_len;

static struct file *gtp_file_open(u8 * path, mm_segment_t * old_fs_p);
static int gtp_get_file_length(char *path);
static void gtp_file_close(struct file *filp, mm_segment_t old_fs);

#pragma pack(1)
struct tpd_firmware_info_t {
	u8 chip_type;
	u16 version;
	u8 rom_version;
	u8 reserved[3];
	u16 start_addr;
	u16 length;
	u8 checksum[3];
	u8 mandatory_flag[6];
	u8 data;
};
#pragma pack()

/*******************************************************
Function:
Enter update mode function.

Input:
client:i2c client.

Output:
Executive outcomes.1---succeed.
 *********************************************************/
s32 gup_enter_update_mode(struct i2c_client *client)
{
	return 0;
}

/*******************************************************
Function:
Leave update mode function.

Input:
client:i2c client.

Output:
Executive outcomes.1---succeed.
 *********************************************************/
void gup_leave_update_mode(void)
{
}

/*******************************************************
Function:
The entrance of GTP update function.

Input:
data:the data to update.

Output:
Executive outcomes.1---succeed.
 *********************************************************/
s32 gup_update_proc(void *data)
{
	struct goodix_ts_data *ts;
	struct file *file_data = NULL;
	mm_segment_t old_fs;
	u32 file_len = 0;
	s8 *cmd;
	u8 fw_path_len = 0;
	u8 *file_ptr = NULL;
	s32 ret = -1;
	u8 retry = 0;

	GTP_DEBUG_FUNC();
	show_len = 10;
	total_len = 100;
	cmd = (s8 *) data;
	ts = i2c_get_clientdata(i2c_connect_client);
	if (ts->enter_update) {
		GTP_ERROR("Updating...");
		goto exit_update_proc;
	}

	if (ts == NULL) {
		GTP_ERROR("NULL ts,Exit update.");
		goto exit_update_proc;
	}

	if (data == NULL) {
		GTP_ERROR("Null path,Exit Update.");
		goto exit_update_proc;
	}
	GTP_INFO("Begin update......");

	fw_path_len = strlen(data);
	fw_path = (s8 *) vmalloc(fw_path_len);
	memcpy(fw_path, data, fw_path_len + 1);
	GTP_DEBUG("fw_path:%s, fw_len:%d", fw_path, fw_path_len);

	file_data = gtp_file_open(fw_path, &old_fs);
	if (file_data == NULL) {
		GTP_ERROR("Cannot open update file,Exit update.");
		goto exit_update_proc;
	}

	file_len = gtp_get_file_length(fw_path);
	GTP_DEBUG("Update file length:%d.", file_len);
	file_ptr = (u8 *) vmalloc(file_len);
	if (file_ptr == NULL) {
		GTP_ERROR("Cannot malloc memory,Exit update.");
		goto exit_update_proc;
	}

	ret =
	    file_data->f_op->read(file_data, file_ptr, file_len,
				  &file_data->f_pos);
	if (ret <= 0) {
		GTP_ERROR("Read file data failed,Exit update.");
		goto exit_update_proc;
	}
	gtp_file_close(file_data, old_fs);

	ret = gup_downloader(ts, file_ptr);
	vfree(file_ptr);
	if (ret < 0) {
		GTP_ERROR("Update failed!Exit.");
		goto exit_update_proc;
	}
	show_len = 100;
	total_len = 100;
	GTP_GPIO_OUTPUT(GTP_RST_PORT, 0);
	msleep(5);
	GTP_GPIO_AS_INPUT(GTP_RST_PORT);
	msleep(20);
	for (retry = 0; retry < 3; retry++) {
		ret = gtp_init_panel(ts);
		msleep(2);
		if (ret != 0) {
			GTP_ERROR("Init panel failed!.");
			continue;
		} else {
			break;
		}
	}
	GTP_GPIO_AS_INT(GTP_INT_PORT);
	return 1;

exit_update_proc:
	show_len = 200;
	total_len = 100;
	return 0;
}

/*******************************************************
Function:
Open file function.

Input:
path:the file path.
old_fs_p:old fs point.

Output:
File point.
 *********************************************************/
static struct file *gtp_file_open(u8 * path, mm_segment_t * old_fs_p)
{
	s32 errno = -1;
	struct file *filp = NULL;

	filp = filp_open(path, O_RDONLY, 0644);
	if (!filp || IS_ERR(filp)) {
		if (!filp)
			errno = -ENOENT;
		else
			errno = PTR_ERR(filp);
		GTP_ERROR("Open update file error.");
		return NULL;
	}
	*old_fs_p = get_fs();
	set_fs(get_ds());

	filp->f_op->llseek(filp, 0, 0);
	return filp;
}

/*******************************************************
Function:
Close file function.

Input:
filp:the file point.
old_fs_p:old fs point.

Output:
None.
 *********************************************************/
static void gtp_file_close(struct file *filp, mm_segment_t old_fs)
{
	set_fs(old_fs);
	if (filp)
		filp_close(filp, NULL);
}

/*******************************************************
Function:
Get file length function.

Input:
path:the file path.

Output:
Executive outcomes.
 *********************************************************/
static int gtp_get_file_length(char *path)
{
	struct file *file_ck = NULL;
	mm_segment_t old_fs;
	s32 length;

	file_ck = gtp_file_open(path, &old_fs);
	if (file_ck == NULL)
		return 0;

	length = file_ck->f_op->llseek(file_ck, 0, SEEK_END);
	if (length < 0)
		length = 0;
	gtp_file_close(file_ck, old_fs);
	return length;
}

/*******************************************************
Function:
Judge equal function.

Input:
src:source data.
dst:destination data.
len:judge length.

Output:
Executive outcomes.
 *********************************************************/
static u8 is_equal(u8 * src, u8 * dst, s32 len)
{
	s32 i;

	GTP_DEBUG_ARRAY(src, len);
	GTP_DEBUG_ARRAY(dst, len);
	for (i = 0; i < len; i++) {
		if (src[i] != dst[i]) {
			return 0;
		}
	}
	return 1;
}

/*******************************************************
Function:
Nvram store function.

Input:
ts:private data.

Output:
Executive outcomes.
 *********************************************************/
static u8 gup_nvram_store(struct goodix_ts_data *ts)
{
	int ret;
	int i;
	u8 inbuf[3] = { REG_NVRCS_H, REG_NVRCS_L, 0 };
	ret = gtp_i2c_read(ts->client, inbuf, 3);
	if (ret < 0) {
		return 0;
	}

	if ((inbuf[2] & BIT_NVRAM_LOCK) == BIT_NVRAM_LOCK) {
		return 0;
	}

	inbuf[2] = NVRAM_STORE_CMD;	//store command

	for (i = 0; i < 300; i++) {
		ret = gtp_i2c_write(ts->client, inbuf, 3);
		if (ret < 0)
			break;
	}

	return ret;
}

/*******************************************************
Function:
Recall function.

Input:
ts:private data.

Output:
Executive outcomes.
 *********************************************************/
static u8 gup_nvram_recall(struct goodix_ts_data *ts)
{
	int ret;
	u8 inbuf[3] = { REG_NVRCS_H, REG_NVRCS_L, 0 };

	ret = gtp_i2c_read(ts->client, inbuf, 3);
	if (ret < 0) {
		return 0;
	}

	if ((inbuf[2] & BIT_NVRAM_LOCK) == BIT_NVRAM_LOCK) {
		return 0;
	}

	inbuf[2] = (1 << BIT_NVRAM_RECALL);
	ret = gtp_i2c_write(ts->client, inbuf, 3);
	return ret;
}

/*******************************************************
Function:
Update reset function.

Input:
ts:private data.

Output:
Executive outcomes.
 *********************************************************/
static int gup_update_reset(struct goodix_ts_data *ts, u8 check)
{
	s32 ret = 1;
	u8 retry;
	u8 outbuf[3] = { 0, 0xff, 0 };
	u8 inbuf[3] = { 0, 0xff, 0 };

	GTP_GPIO_OUTPUT(GTP_RST_PORT, 0);
	msleep(20);
	GTP_GPIO_AS_INPUT(GTP_RST_PORT);
	msleep(100);
	for (retry = 0; retry < 80; retry++) {
		ret = gtp_i2c_write(ts->client, inbuf, 0);
		if (ret > 0) {
			msleep(10);
			ret = gtp_i2c_read(ts->client, inbuf, 3);
			if (ret > 0) {
				if (check) {
					if (inbuf[2] == 0x55) {
						ret =
						    gtp_i2c_write(ts->client,
								  outbuf, 3);
						msleep(10);
						break;
					}
				} else {
					break;
				}
			}
		} else {
			if (check) {
				GTP_GPIO_OUTPUT(GTP_RST_PORT, 0);
				msleep(20);
				GTP_GPIO_AS_INPUT(GTP_RST_PORT);
				msleep(20);
				GTP_ERROR
				    ("GTP update reset i2c address failed\n");
			}
		}
	}
	GTP_INFO("Detect address 0x%0X\n", ts->client->addr);
	return ret;
}

/*******************************************************
Function:
Update set address function.

Input:
ts:private data.

Output:
Executive outcomes.
 *********************************************************/
static int gup_set_address_2(struct goodix_ts_data *ts)
{
	unsigned char inbuf[3] = { 0, 0, 0 };
	int i;

	for (i = 0; i < 12; i++) {
		if (gtp_i2c_read(ts->client, inbuf, 3)) {
			GTP_INFO("Got response\n");
			return 1;
		}
		GTP_INFO("wait for retry\n");
		msleep(50);
	}
	return 0;
}

/*******************************************************
Function:
Update firmware function.

Input:
nvram:data to update.
start_addr:update start address.
length:update length.
ts:private data.

Output:
Executive outcomes.
 *********************************************************/
static u8 gup_update_firmware(u8 * nvram, u16 start_addr, u16 length,
			      struct goodix_ts_data *ts)
{
	u8 ret, err, retry_time, i;
	u16 cur_code_addr;
	u16 cur_frame_num, total_frame_num, cur_frame_len;
	u32 gt80x_update_rate;

	u8 i2c_data_buf[PACK_SIZE + 2] = { 0, };
	u8 i2c_chk_data_buf[PACK_SIZE + 2] = { 0, };

	if (length > NVRAM_STORE_MAX_LEN) {
		GTP_ERROR("Fw length %d is bigger than limited length %d\n",
			  length, NVRAM_LEN - NVRAM_BOOT_SECTOR_LEN);
		return 0;
	}

	total_frame_num = (length + PACK_SIZE - 1) / PACK_SIZE;
	gt80x_update_rate = 0;
	for (cur_frame_num = 0; cur_frame_num < total_frame_num;
	     cur_frame_num++) {
		retry_time = 5;

		GTP_INFO("PACK[%d].", cur_frame_num);
		cur_code_addr = start_addr + cur_frame_num * PACK_SIZE;
		i2c_data_buf[0] = (cur_code_addr >> 8) & 0xff;
		i2c_data_buf[1] = cur_code_addr & 0xff;

		i2c_chk_data_buf[0] = i2c_data_buf[0];
		i2c_chk_data_buf[1] = i2c_data_buf[1];

		if (cur_frame_num == total_frame_num - 1) {
			cur_frame_len = length - cur_frame_num * PACK_SIZE;
		} else {
			cur_frame_len = PACK_SIZE;
		}

		for (i = 0; i < cur_frame_len; i++) {
			i2c_data_buf[2 + i] =
			    nvram[cur_frame_num * PACK_SIZE + i];
		}
		do {
			err = 0;
			ret =
			    gtp_i2c_write(ts->client, i2c_data_buf,
					  (cur_frame_len + 2));
			if (ret <= 0) {
				GTP_ERROR("GTP update write frame failed");
				err = 1;
			}

			ret =
			    gtp_i2c_read(ts->client, i2c_chk_data_buf,
					 (cur_frame_len + 2));
			if (ret <= 0) {
				GTP_ERROR("GTP update read frame failed");
				err = 1;
			}

			if (is_equal
			    (&i2c_data_buf[2], &i2c_chk_data_buf[2],
			     cur_frame_len) == 0) {
				GTP_ERROR
				    ("GTP update read frame is not equal write frame\n");
				err = 1;
			}

		} while (err == 1 && (--retry_time) > 0);

		if (err == 1) {
			break;
		}
		gt80x_update_rate = (cur_frame_num + 1) * 128 / total_frame_num;
	}

	if (err == 1) {
		GTP_ERROR("GTP update write nvram failed\n");
		return 0;
	}

	ret = gup_nvram_store(ts);

	msleep(20);

	if (ret == 0) {
		GTP_ERROR("GTP update nvram store failed\n");
		return 0;
	}

	ret = gup_nvram_recall(ts);
	if (ret == 0) {
		GTP_ERROR("nvram recall failed\n");
		return 0;
	}
	msleep(20);

	for (cur_frame_num = 0; cur_frame_num < total_frame_num;
	     cur_frame_num++) {
		cur_code_addr = start_addr + cur_frame_num * PACK_SIZE;
		retry_time = 5;
		i2c_chk_data_buf[0] = (cur_code_addr >> 8) & 0xff;
		i2c_chk_data_buf[1] = cur_code_addr & 0xff;

		if (cur_frame_num == total_frame_num - 1) {
			cur_frame_len = length - cur_frame_num * PACK_SIZE;
		} else {
			cur_frame_len = PACK_SIZE;
		}

		do {
			err = 0;
			ret =
			    gtp_i2c_read(ts->client, i2c_chk_data_buf,
					 (cur_frame_len + 2));
			if (ret == 0) {
				err = 1;
			}

			if (is_equal
			    (&nvram[cur_frame_num * PACK_SIZE],
			     &i2c_chk_data_buf[2], cur_frame_len) == 0) {
				err = 1;
			}
		} while (err == 1 && (--retry_time) > 0);

		if (err == 1) {
			break;
		}
		gt80x_update_rate =
		    127 + (cur_frame_num + 1) * 128 / total_frame_num;
	}
	gt80x_update_rate = 255;
	if (err == 1) {
		GTP_ERROR("GTP update nvram validate failed\n");
		return 0;
	}
	return 1;
}

/*******************************************************
Function:
Write nvram function.

Input:
nvram:data to update.
start_addr:update start address.
length:update length.
ts:private data.

Output:
Executive outcomes.
 *********************************************************/
static u8 gup_write_nvram(u8 * nvram, u16 start_addr, u16 length,
			  struct goodix_ts_data *ts)
{
	u8 ret;
	u8 error = 0;
	GTP_GPIO_OUTPUT(GTP_INT_PORT, 0);
	msleep(20);
	ret = gup_update_reset(ts, 1);
	if (ret < 0) {
		error = 1;
		GTP_ERROR("GTP update reset fail\n");
		goto end_update_proc;
	}

	ret = gup_set_address_2(ts);
	if (ret == 0) {
		error = 1;
		GTP_ERROR("GTP update set address fail\n");
		goto end_update_proc;
	}

	ret = gup_update_firmware(nvram, start_addr, length, ts);
	if (!ret) {
		error = 1;
		GTP_ERROR("GTP firmware update fail\n");
		goto end_update_proc;
	}

end_update_proc:
	GTP_GPIO_OUTPUT(GTP_INT_PORT, 1);
	msleep(500);
	ret = gup_update_reset(ts, 0);
	if (ret < 0) {
		error = 1;
		GTP_ERROR("GTP update final reset fail\n");
		goto end_update_proc;
	}
	if (error == 1) {
		return 0;
	}
	while (gtp_read_version(ts) < 0) ;
	return 1;
}

/*******************************************************
Function:
Little endian tranfer to big endian function.

Input:
Little_endian:data to tranfer.

Output:
Executive outcomes.
 *********************************************************/
u16 Little2BigEndian(u16 little_endian)
{
	u16 temp = 0;
	temp = little_endian & 0xff;
	return (temp << 8) + ((little_endian >> 8) & 0xff);
}

/*******************************************************
Function:
Update downloader function.

Input:
ts:private date.
data:data to download.

Output:
Executive outcomes.
 *********************************************************/
s32 gup_downloader(struct goodix_ts_data * ts, u8 * data)
{
	struct tpd_firmware_info_t *fw_info =
	    (struct tpd_firmware_info_t *)data;
	u32 fw_checksum = 0;
	u16 fw_version;
	u16 fw_start_addr;
	u16 fw_length;
	u8 *data_ptr;
	u8 rd_buf[4] = { 0 };
	u8 *mandatory_base = "GOODIX";
	u8 rd_rom_version;
	u8 rd_chip_type;
	u8 rd_nvram_flag;
	s32 retry = 0;
	s32 ret = -1;
	s32 err = 0;

	rd_buf[0] = 0x14;
	rd_buf[1] = 0x00;
	rd_buf[2] = 0x80;
	ret = gtp_i2c_write(ts->client, rd_buf, 3);
	if (ret < 0) {
		GTP_ERROR("I2c write clean flag failed");
		goto exit_downloader;
	}
	rd_buf[0] = 0x40;
	rd_buf[1] = 0x11;
	ret = gtp_i2c_read(ts->client, rd_buf, 3);
	if (ret <= 0) {
		GTP_ERROR("I2c read failed!");
		goto exit_downloader;
	}
	rd_chip_type = rd_buf[2];
	rd_buf[0] = 0xFB;
	rd_buf[1] = 0xED;
	ret = gtp_i2c_read(ts->client, rd_buf, 3);
	if (ret <= 0) {
		GTP_ERROR("I2c read failed!");
		goto exit_downloader;
	}
	rd_rom_version = rd_buf[2];
	rd_buf[0] = 0x06;
	rd_buf[1] = 0x94;
	ret = gtp_i2c_read(ts->client, rd_buf, 3);
	if (ret <= 0) {
		GTP_ERROR("I2c read failed!");
		goto exit_downloader;
	}
	rd_nvram_flag = rd_buf[2];

	fw_version = fw_info->version;
	fw_start_addr = Little2BigEndian(fw_info->start_addr);
	fw_length = Little2BigEndian(fw_info->length);
	data_ptr = &(fw_info->data);

	GTP_INFO("FW:chip_type = %02x", fw_info->chip_type);
	GTP_INFO("FW:version = 0x%04x", fw_info->version);
	GTP_INFO("FW:rom_version = 0x%02x", fw_info->rom_version);
	GTP_INFO("FW:start_addr = 0x%04x", fw_start_addr);
	GTP_INFO("FW:file_size = 0x%04x", fw_length);
	fw_checksum =
	    ((u32) fw_info->checksum[0] << 16) +
	    ((u32) fw_info->checksum[1] << 8)
	    + ((u32) fw_info->checksum[2]);
	GTP_INFO("FW:checksum=0x%06x", fw_checksum);
	GTP_INFO("current version 0x%04X, target verion 0x%04X", ts->version,
		 fw_version);

	if (rd_chip_type != fw_info->chip_type) {
		GTP_INFO("Chip type not match,exit downloader\n");
		goto exit_downloader;
	}

	if (!rd_rom_version) {
		if (fw_info->rom_version != 0x45) {
			GTP_ERROR("Rom version not match,exit downloader\n");
			goto exit_downloader;
		}
		GTP_INFO("Rom version E..");
		goto chk_fw_version;
	} else if (rd_rom_version != fw_info->rom_version) {
		GTP_ERROR("Rom version not match,exidownloader\n");
		goto exit_downloader;
	}
	GTP_INFO("Rom version %c\n", rd_rom_version);

	if (rd_nvram_flag == 0x55) {
		GTP_INFO("NVRAM correct!.");
		goto chk_fw_version;
	} else if (rd_nvram_flag == 0xAA) {
		GTP_ERROR("NVRAM incorrect!Need update..");
		goto begin_upgrade;
	} else {
		GTP_ERROR("NVRAM other error![0x694]=0x%02x\n", rd_nvram_flag);
		goto begin_upgrade;
	}
chk_fw_version:
	if (ts->version >= fw_version) {
		GTP_ERROR("Fw verison not match..");
		goto chk_mandatory_upgrade;
	}
	GTP_INFO("Need to upgrade\n");
	goto begin_upgrade;
chk_mandatory_upgrade:
	GTP_DEBUG("%s\n", mandatory_base);
	GTP_DEBUG("%s\n", fw_info->mandatory_flag);
	ret = memcmp(mandatory_base, fw_info->mandatory_flag, 6);
	if (ret) {
		GTP_ERROR("Not meet mandatory upgrade,exit downloader!ret:%d\n",
			  ret);
		goto exit_downloader;
	}
	GTP_INFO("Mandatory upgrade!.");
begin_upgrade:
	GTP_INFO("Begin upgrade!.");
	GTP_INFO("STEP_0:request gpio");

	GTP_INFO("STEP_1:begin update");
	err = -1;
	while (retry < 3) {
		ret = gup_write_nvram(data_ptr, fw_start_addr, fw_length, ts);
		if (ret == 1) {
			err = 1;
			break;
		}
		retry++;
	}

exit_downloader:

	GTP_GPIO_AS_INT(GTP_INT_PORT);
	return err;
}
