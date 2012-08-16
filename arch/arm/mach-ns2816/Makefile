#
# Makefile for the linux kernel.
#

#GCOV_PROFILE := y
obj-y					:= timer-dw.o core.o clock.o prcm.o scm.o gpio.o get_bootargs.o common.o  extend.o gpio.o
obj-$(CONFIG_MACH_NS2816TB)		+= ns2816-tb.o
obj-$(CONFIG_MACH_NS2816_DEV_BOARD)	+= ns2816-dev-board.o

ifeq ($(CONFIG_NS2816_HUASHAN_MINIPC), y)
obj-$(CONFIG_MACH_NS2816_HUASHAN)	+= ns2816-minipc.o
else ifeq ($(CONFIG_NS2816_HUASHAN_SKYTONE), y)
obj-$(CONFIG_MACH_NS2816_HUASHAN)	+= ns2816-skytone.o
else ifeq ($(CONFIG_NS2816_HUASHAN_NETXEON), y)
obj-$(CONFIG_MACH_NS2816_HUASHAN)	+= ns2816-netxeon.o
else
obj-$(CONFIG_MACH_NS2816_HUASHAN)	+= ns2816-huashan.o
endif

obj-$(CONFIG_MACH_HUANGSHANS)		+= ns2816-huangshan.o soc_power_ctrl.o
obj-$(CONFIG_MACH_NS2816_YANGCHENG)	+= ns2816-yangcheng.o
obj-$(CONFIG_SMP)			+= platsmp.o headsmp.o ns2816-save.o suspend.o suspend-macro.o
obj-$(CONFIG_HOTPLUG_CPU)		+= hotplug.o
obj-$(CONFIG_LOCAL_TIMERS)		+= localtimer.o
#obj-$(CONFIG_PM)			+= ns2816-save.o suspend.o suspend-macro.o
obj-$(CONFIG_CPU_IDLE)			+= cpuidle.o
obj-$(CONFIG_GENERIC_GPIO)		+= gpio.o
