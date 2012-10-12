#ifndef __NS115_CPU_FREQUENCY_H__
#define __NS115_CPU_FREQUENCY_H__

struct ns115_cpufreq_config {
	unsigned int max_frequency;
};

extern struct ns115_cpufreq_config*  ns115_cpufreq_cfg;

#endif
