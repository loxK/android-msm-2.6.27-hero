obj-y += io.o idle.o irq.o timer.o dma.o memory.o
obj-y += devices.o
obj-y += remote_spinlock.o
obj-y += proc_comm.o
obj-y += vreg.o
obj-y += acpuclock.o
obj-y += clock.o
obj-y += gpio.o generic_gpio.o
obj-y += nand_partitions.o
obj-y += drv_callback.o
obj-y += htc_port_list.o

obj-$(CONFIG_MSM_FIQ_SUPPORT) += fiq_glue.o
obj-$(CONFIG_MSM_SMD) += smd.o smd_tty.o smd_qmi.o
#obj-$(CONFIG_MSM_SMD) += smem_log.o
obj-$(CONFIG_MSM_ONCRPCROUTER) += smd_rpcrouter.o
obj-$(CONFIG_MSM_ONCRPCROUTER) += smd_rpcrouter_device.o
obj-$(CONFIG_MSM_ONCRPCROUTER) += smd_rpcrouter_servers.o
obj-$(CONFIG_MSM_RPCSERVERS) += rpc_server_dog_keepalive.o
obj-$(CONFIG_MSM_RPCSERVERS) += rpc_server_time_remote.o
obj-$(CONFIG_MSM_ADSP) += qdsp5/
obj-$(CONFIG_MSM_ADSP_COMP) += qdsp5_comp/
obj-$(CONFIG_MSM_HW3D) += hw3d.o
obj-$(CONFIG_PM) += pm.o
obj-$(CONFIG_MSM_CPU_FREQ) += cpufreq.o

obj-y += htc_wifi_nvs.o htc_wifi.o
obj-$(CONFIG_TOUCHSCREEN_TSSC_MANAGER) += htc_tp_cal.o
obj-$(CONFIG_SENSORS_AKM8976) += htc_akm_cal.o
obj-$(CONFIG_PERFLOCK) += perflock.o
obj-$(CONFIG_HTC_BATTCHG) += htc_battery.o
obj-$(CONFIG_HTC_PWRSINK) += htc_pwrsink.o
obj-$(CONFIG_HTC_HEADSET) += htc_headset.o
obj-$(CONFIG_HTC_AUDIOJACK) += htc_audio_jack.o
obj-$(CONFIG_HTC_ACOUSTIC) += htc_acoustic.o
obj-$(CONFIG_OLD_HTC_ACOUSTIC) += htc_old_acoustic.o

# MSM7201a/7200a boards
obj-$(CONFIG_MACH_TROUT) += board-trout-rfkill.o
obj-$(CONFIG_MACH_HALIBUT) += board-halibut.o board-halibut-keypad.o
obj-$(CONFIG_MACH_TROUT) += board-trout.o board-trout-gpio.o
obj-$(CONFIG_MACH_TROUT) += board-trout-keypad.o board-trout-panel.o
obj-$(CONFIG_MACH_TROUT) += board-trout-mmc.o
obj-$(CONFIG_TROUT_H2W) += board-trout-h2w.o
obj-$(CONFIG_MACH_SAPPHIRE) += board-sapphire.o board-sapphire-gpio.o
obj-$(CONFIG_MACH_SAPPHIRE) += board-sapphire-keypad.o board-sapphire-panel.o
obj-$(CONFIG_MACH_SAPPHIRE) += board-sapphire-mmc.o
obj-$(CONFIG_MACH_SAPPHIRE) += board-sapphire-rfkill.o
obj-$(CONFIG_MACH_HERO) += board-hero.o board-hero-keypad.o
obj-$(CONFIG_MACH_HERO) += board-hero-panel.o board-hero-mmc.o board-hero-camsensor.o
obj-$(CONFIG_MACH_HERO) += board-hero-rfkill.o
obj-$(CONFIG_MACH_MILAN) += board-milan.o board-milan-keypad.o
obj-$(CONFIG_MACH_MILAN) += board-milan-panel.o board-milan-mmc.o
obj-$(CONFIG_MACH_HERO_ESPRESSO) += board-espresso.o board-espresso-keypad.o
obj-$(CONFIG_MACH_HERO_ESPRESSO) += board-espresso-panel.o board-espresso-mmc.o
obj-$(CONFIG_MACH_HERO_ESPRESSO) += board-espresso-rfkill.o

# MSM7225 boards
obj-$(CONFIG_MACH_HTCJADE) += board-jade.o board-jade-panel.o board-jade-keypad.o
obj-$(CONFIG_MACH_HTCJADE) += board-jade-mmc.o
obj-$(CONFIG_MACH_MEMPHIS) += board-memphis-rfkill.o
obj-$(CONFIG_MACH_MEMPHIS) += board-memphis.o board-panel-eid.o board-memphis-keypad.o
obj-$(CONFIG_MACH_MEMPHIS) += board-memphis-mmc.o
obj-$(CONFIG_MACH_BAHAMAS) += board-bahamas.o board-panel-eid.o board-bahamas-keypad.o
obj-$(CONFIG_MACH_BAHAMAS) += board-bahamas-mmc.o
obj-$(CONFIG_MACH_BAHAMAS) += board-bahamas-rfkill.o

# MSM7501a boards
obj-$(CONFIG_MACH_NEONC) += board-neonc-rfkill.o
obj-$(CONFIG_MACH_NEONC) += board-neonc.o
obj-$(CONFIG_MACH_NEONC) += board-neonc-keypad.o board-neonc-panel.o
obj-$(CONFIG_MACH_HEROC) += board-heroc.o board-heroc-keypad.o
obj-$(CONFIG_MACH_HEROC) += board-heroc-panel.o board-heroc-mmc.o board-heroc-camsensor.o
obj-$(CONFIG_MACH_HEROC) += board-heroc-rfkill.o
obj-$(CONFIG_MACH_DESIREC) += board-desirec.o board-desirec-keypad.o
obj-$(CONFIG_MACH_DESIREC) += board-desirec-panel.o board-desirec-mmc.o board-desirec-camsensor.o
obj-$(CONFIG_MACH_DESIREC) += board-desirec-rfkill.o

