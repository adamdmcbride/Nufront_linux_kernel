#ifndef __LINUX_POWER_POWER_SWITCH_H__
#define __LINUX_POWER_POWER_SWITCH_H__

struct power_switch_ops
{
	int (*power_on)(void * dev_id);
	int (*power_off)(void * dev_id);
	int (*get_state)(void * dev_id);
};

#endif
