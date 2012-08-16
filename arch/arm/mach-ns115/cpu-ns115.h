#ifndef __CPU_NS115_H__
#define __CPU_NS115_H__

extern int ns115_auto_hotplug_init(struct mutex *cpu_lock, unsigned int top, unsigned int bottom);
extern void ns115_auto_hotplug_governor(unsigned int cpu_freq, bool suspend);
extern void ns115_auto_hotplug_exit(void);
extern void ns115_auto_hotplug_enable(bool flag);
extern void ns115_auto_hotplug_suspend(void);
extern void ns115_auto_hotplug_resume(void);

#endif
