
ifeq ($(CONFIG_ARCH_MSM7201A),y)
ifeq ($(CONFIG_MSM_AMSS_SUPPORT_256MB_EBI1),y)
  zreladdr-y		:= 0x19208000
params_phys-y		:= 0x19200100
initrd_phys-y		:= 0x19A00000
else
	zreladdr-y		:= 0x10008000
params_phys-y		:= 0x10000100
initrd_phys-y		:= 0x10800000
endif
endif

ifeq ($(CONFIG_ARCH_MSM7200A),y)
zreladdr-y		:= 0x19208000
params_phys-y		:= 0x19200100
initrd_phys-y		:= 0x19A00000
endif

ifeq ($(CONFIG_ARCH_MSM7225),y)
zreladdr-y		:= 0x1B408000
params_phys-y		:= 0x1B400100
initrd_phys-y		:= 0x1BC00000
endif

ifeq ($(CONFIG_ARCH_MSM7501A),y)
zreladdr-y		:= 0x19208000
params_phys-y		:= 0x19200100
initrd_phys-y		:= 0x19A00000
endif
