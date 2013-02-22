/*
 * @file drivers/power/ns115_battery_parameter.c
 * @brief adc battery gague Linux device driver
 * @author hujian@nufront
 * @date 2012/12/6
 *
 * 	This software is licensed under the terms of the GNU General Public
 * 	License version 2, as published by the Free Software Foundation, and
 * 	may be copied, distributed, and modified under those terms.
 *
 * 	This program is distributed in the hope that it will be useful,
 * 	but WITHOUT ANY WARRANTY; without even the implied warranty of
 * 	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * 	GNU General Public License for more details.
 *
 *
 */
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/sched.h>
#include <linux/cdev.h>
#include <linux/wakelock.h>
#include <asm/uaccess.h>
#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/clk.h>
#include <linux/mutex.h>
#include <linux/types.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/poll.h>
#include <linux/string.h>
//***********************************
#include <linux/syscalls.h>
#include <linux/wait.h>
#include <linux/workqueue.h>
#include "ns115_battery_parameter.h"

DEFINE_MUTEX(batt_par_lock);//define lock

/*
 *return:
 *-3:operation error
 *-2:open error or file not exist
 *-1:parameter invalid
 *0:failed with unknown reseason!
 *above 0;filed discricptor
 */
int ns115_battery_get_config(battery_parameter_t *para){
	long int fd,len;
	unsigned int orgfs;
	int ret = 0;
	const char *filename = BATTER_PARAMETER_CONFIG_FILE_PATH;

	if(!filename || !para)
		return -1;

	mutex_lock(&batt_par_lock);

	orgfs = get_fs();
	// Set segment descriptor associated to kernel space
	set_fs(KERNEL_DS);

	fd = sys_open(filename,O_RDONLY, 0600);
	if(fd<0){
		ret = -2;
		printk(KERN_ERR"%s:sys_open %s error!!. %ld\n",__func__,filename,fd);
		goto error_index0;
	}

	len = sys_read(fd,(void*)para, sizeof(battery_parameter_t));
	if(len<0 || (len != sizeof(battery_parameter_t)) ){
		ret = -3;
		printk(KERN_ERR"%s:sys_read %s error!!. %ld\n",__func__,filename,fd);
		goto error_index0;
	}
	sys_close(fd);

error_index0:
	set_fs(orgfs);
	mutex_unlock(&batt_par_lock);
	return ret;
}

int ns115_battery_set_config(battery_parameter_t *para){
	long int fd,len;
	unsigned int orgfs;
	int ret = 0;
	const char *filename = BATTER_PARAMETER_CONFIG_FILE_PATH;

	if(!filename || !para)
		return -1;

	mutex_lock(&batt_par_lock);
	orgfs = get_fs();
	// Set segment descriptor associated to kernel space
	set_fs(KERNEL_DS);

	fd = sys_open(filename, O_WRONLY | O_CREAT | O_TRUNC, 0600);
	if(fd<0){
		ret = -2;
		printk(KERN_ERR"%s:sys_open %s error!!. %ld\n",__func__,filename,fd);
		goto error_index0;
	}

	len = sys_write(fd, (void*)para, sizeof(battery_parameter_t)+1);
	if(len<0 || (len != (sizeof(battery_parameter_t)+1) ) ){
		ret = -3;
		printk(KERN_ERR"%s:sys_write %s error!!. %ld\n",__func__,filename,fd);
		goto error_index0;
	}
	sys_close(fd);

error_index0:
	set_fs(orgfs);
	mutex_unlock(&batt_par_lock);
	return ret;
}

EXPORT_SYMBOL(ns115_battery_get_config);
EXPORT_SYMBOL(ns115_battery_set_config);
//*********************************************************************************************************
