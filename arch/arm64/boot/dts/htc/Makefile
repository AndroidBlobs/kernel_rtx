# SPDX-License-Identifier: GPL-2.0
ifeq ($(CONFIG_BUILD_ARM64_DT_OVERLAY),y)
	dtbo-$(CONFIG_ARCH_SM8150) += \
		sm8150-htc_evm-overlay.dtbo \
		sm8150-htc_evm-fusion-overlay.dtbo \
		sm8150-htc_rtx-xa-overlay.dtbo \
		sm8150-htc_rtx-xa02-overlay.dtbo \
		sm8150-htc_rtx-xb-overlay.dtbo \
		sm8150-htc_rtx-xc01-overlay.dtbo \
		sm8150-htc_rtx-xc02-overlay.dtbo \
		sm8150-htc_rtx-a-overlay.dtbo

sm8150-htc_evm-overlay.dtbo-base := ../qcom/sm8150.dtb ../qcom/sm8150-v2.dtb
sm8150-htc_evm-fusion-overlay.dtbo-base := ../qcom/sm8150.dtb ../qcom/sm8150-v2.dtb
sm8150-htc_rtx-xa-overlay.dtbo-base := ../qcom/sm8150.dtb ../qcom/sm8150-v2.dtb
sm8150-htc_rtx-xa02-overlay.dtbo-base := ../qcom/sm8150.dtb ../qcom/sm8150-v2.dtb
sm8150-htc_rtx-xb-overlay.dtbo-base := ../qcom/sm8150.dtb ../qcom/sm8150-v2.dtb
sm8150-htc_rtx-xc01-overlay.dtbo-base := ../qcom/sm8150.dtb ../qcom/sm8150-v2.dtb
sm8150-htc_rtx-xc02-overlay.dtbo-base := ../qcom/sm8150.dtb ../qcom/sm8150-v2.dtb
sm8150-htc_rtx-a-overlay.dtbo-base := ../qcom/sm8150.dtb ../qcom/sm8150-v2.dtb
else
dtb-$(CONFIG_ARCH_SM8150)	+= \
	sm8150-htc_evm.dtb
endif

ifeq ($(CONFIG_ARM64),y)
always		:= $(dtb-y)
subdir-y	:= $(dts-dirs)
else
targets += dtbs
targets += $(addprefix ../, $(dtb-y))

$(obj)/../%.dtb: $(src)/%.dts FORCE
	$(call if_changed_dep,dtc)

dtbs: $(addprefix $(obj)/../,$(dtb-y))
endif
clean-files	:= *.dtb *.dtbo
