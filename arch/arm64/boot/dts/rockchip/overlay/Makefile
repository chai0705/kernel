# SPDX-License-Identifier: GPL-2.0
ifeq ($(CONFIG_ARCH_ROCKCHIP), y)

dtbo-$(CONFIG_CPU_RK3588) += \
	rk3588-hdmi0-overlay.dtbo \
	rk3588-hdmi1-overlay.dtbo \
	rk3588-dp0-overlay.dtbo \
	rk3588-dsi0-mipi0-overlay.dtbo \
	rk3588-dsi1-mipi1-overlay.dtbo \
	rk3588-dsi0-lvds-1280x800-overlay.dtbo \
	rk3588-dsi0-lvds-800x1280-overlay.dtbo \
	rk3588-dsi0-lvds-1024x600-overlay.dtbo \
	rk3588-dsi1-lvds-1280x800-overlay.dtbo \
	rk3588-dsi1-lvds-800x1280-overlay.dtbo \
	rk3588-dsi1-lvds-1024x600-overlay.dtbo \
	rk3588-camera_j1_overlay.dtbo \
	rk3588-camera_j2_overlay.dtbo \
	rk3588-camera_j3_overlay.dtbo \
	rk3588-camera_j4_overlay.dtbo 

endif

dtbotxt-$(CONFIG_ARCH_ROCKCHIP) += \
	README.rockchip-overlays.md

targets += $(dtbo-y) $(dtbotxt-y)

always-y		:= $(dtbo-y) $(dtbotxt-y)
clean-files		:= *.dtbo
