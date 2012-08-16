#
# Makefile for the linux kernel.
#

obj-y					:= timer-dw.o core.o clock.o prcm.o scm.o get_bootargs.o common.o extend.o nusmart-dbg.o

obj-$(CONFIG_MACH_NS115_PAD_TEST)	+= ns115-pad-test.o pad_test_pinmux.o nusmart-pl330.o dma.o soc_power_ctrl.o 
obj-$(CONFIG_MACH_NS115_PAD_REF)	+= ns115-pad-ref.o pad_ref_pinmux.o nusmart-pl330.o dma.o soc_power_ctrl.o 
obj-$(CONFIG_SMP)			+= platsmp.o headsmp.o ns115-save.o suspend.o suspend-macro.o
obj-$(CONFIG_HOTPLUG_CPU)		+= hotplug.o
obj-$(CONFIG_LOCAL_TIMERS)		+= localtimer.o
obj-$(CONFIG_PM)			+= ns115-save.o suspend.o suspend-macro.o wake_timer.o
obj-$(CONFIG_CPU_IDLE)			+= cpuidle.o
obj-$(CONFIG_CPU_FREQ)			+= cpufreq.o
obj-$(CONFIG_AUTO_HOTPLUG)		+= cpu-ns115.o
obj-$(CONFIG_GENERIC_GPIO)		+= gpio.o
