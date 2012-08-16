ifeq ($(CONFIG_NS115_HIGH_PHYS_OFFSET),y)
   zreladdr-y	:= 0x80008000
params_phys-y	:= 0x80000100
initrd_phys-y	:= 0x80800000
else
   zreladdr-y	:= 0x00008000
params_phys-y	:= 0x00000100
initrd_phys-y	:= 0x00800000
endif
