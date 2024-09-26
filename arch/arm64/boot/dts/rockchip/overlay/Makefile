# SPDX-License-Identifier: GPL-2.0
ifeq ($(CONFIG_ARCH_ROCKCHIP), y)

dtbo-$(CONFIG_CPU_RK3568) += \
	rk3568-hdmi0-overlay.dtbo \
	rk3568-hdmi1-overlay.dtbo \
	rk3568-vga-overlay.dtbo \
	rk3568-mipi-overlay.dtbo \
	rk3568-lvds-1280x800-overlay.dtbo \
	rk3568-lvds-800x1280-overlay.dtbo \
	rk3568-lvds-1024x600-overlay.dtbo

endif

targets += $(dtbo-y)

always-y		:= $(dtbo-y)
clean-files		:= *.dtbo
