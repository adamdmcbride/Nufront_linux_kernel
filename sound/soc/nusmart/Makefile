# Nusmart Platform Support
EXTRA_CFLAGS += -Iarch/arm/mach-ns115/include/
#snd-soc-nusmart-plat-objs := nusmart-plat.o

#obj-$(CONFIG_SND_NUSMART_SOC_I2S) += snd-soc-nusmart-plat.o

obj-$(CONFIG_SND_NUSMART_SOC_I2S) += nusmart-plat.o
##obj-$(CONFIG_SND_NUSMART_SOC_EVA_I2S) += nusmart-pcm.o
obj-$(CONFIG_SND_NUSMART_SOC_EVA_I2S) += dma.o
obj-$(CONFIG_SND_NUSMART_SOC_EVA_I2S) += nusmart-eva-i2s-dai.o

# Nusmart Machine Support
#snd-soc-nusmart-mach-objs := nusmart-mach.o

#obj-$(CONFIG_SND_NUSMART_SOC_MACH) += snd-soc-nusmart-mach.o
#obj-$(CONFIG_SND_NUSMART_SOC_MACH) += nusmart-mach.o
obj-$(CONFIG_SND_SOC_ALC5631) += nusmart-mach.o
obj-$(CONFIG_SND_SOC_ALC5631) += ns115-jd.o
obj-$(CONFIG_SND_SOC_WM8960) += nusmart-mach-wm8960.o

obj-$(CONFIG_SND_NUSMART_HDMI_CH7033) += nusmart-mach-hdmi.o

