/*
 * @file drivers/power/ns115_batter_parameter.h
 * @brief adc batter gague Linux device driver
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
#ifndef NS115_BATTER_PARAMETER_H
#define NS115_BATTER_PARAMETER_H

#define BATTER_PARAMETER_CONFIG_FILE_PATH "/data/misc/batter_config.ini"
#define BAT_PARA_VERSION	3	//add the version when modfiy the battery_parameter

typedef struct battery_parameter{
	int version;
	int check_no;
	int capacity;
	int resistor;
	int max_mAh;
	int full_vol;
}battery_parameter_t;

extern int ns115_battery_set_config(battery_parameter_t *para);
extern int ns115_battery_get_config(battery_parameter_t *para);

#endif//#ifndef NS115_BATTER_PARAMETER_H

