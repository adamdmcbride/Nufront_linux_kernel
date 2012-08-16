#ifndef _MACH_WAKE_TIMER_H_
#define _MACH_WAKE_TIMER_H_
struct wake_timer_data
{
	const char * fw_name;	/*firmware for SARRAM*/
	int suspend_ms;		/*0 disable*/
	int wake_ms;		/*0 disable*/
};
#endif
