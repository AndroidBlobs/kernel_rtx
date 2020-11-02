/* Copyright (c) 2018 The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#define pr_fmt(fmt) "htc_batt_smb5: " fmt

#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/power/htc_battery.h>
#include <linux/power/htc_battery_smb5.h>
#include <qcom/smb5-reg.h>
#include <qcom/smb5-lib.h>
#include <linux/workqueue.h>
#include <linux/htc_flags.h>
#include <linux/pmic-voter.h>
#include <linux/thermal.h>
#ifdef CONFIG_HTC_BATT_DCJACK
#include <linux/pinctrl/consumer.h>
#include <linux/irq.h>
#include <linux/mutex.h>
#endif

//TODO: Check if these should be moved to smb5-reg.h
//+++++ smb5 registers +++++
// BATTERY_CHARGER_STATUS_1_REG
#define ZERO_CHARGE_CURRENT	BIT(6)

#define JEITA_EN_FVCOMP_IN_CV_BIT   BIT(5)
#define JEITA_EN_HARDLIMIT_BIT      BIT(4)

#define JEITA_FVCOMP_CFG_REG        (CHGR_BASE + 0x91)
#define JEITA_FVCOMP_MASK       GENMASK(7, 0)

#define CHGR_SAFETY_TIMER_ENABLE_CFG_REG        (CHGR_BASE + 0xA0)
#define FAST_CHARGE_SAFETY_TIMER_EN_BIT     BIT(0)

#define USBIN_INPUT_STATUS_REG			(USBIN_BASE + 0x06)
#define USBIN_12V				BIT(5)
#define USBIN_9V_TO_12V				BIT(4)
#define USBIN_5V				BIT(0)

#define TYPE_C_CFG_REG				(USBIN_BASE + 0x58)

#define USBIN_ADAPTER_ALLOW_OVERRIDE_REG		(USBIN_BASE + 0x44)
#define ALLOW_CONTINUOUS			BIT(3)

#define HVDCP_PULSE_COUNT_MAX_QC3_MASK    GENMASK(5, 0)
#define HVDCP_PULSE_COUNT_QC2_SHIFT       6

// USBIN_ADAPTER_ALLOW_CFG
#define USBIN_ADAPTER_ALLOW_MASK        GENMASK(3, 0)

// USBIN_AICL_OPTIONS_CFG
#define SUSPEND_ON_COLLAPSE_USBIN_BIT		BIT(7)
#define USBIN_AICL_EN_BIT			BIT(2)

//CMD_HVDCP_2_REG
#define TRIGGER_AICL_BIT			BIT(6)

#define DCDC_OTG_STATUS_REG			(DCDC_BASE + 0x0D)

// DCDC_INT_RT_STS
#define SWITCHER_POWER_OK_RT_STS		BIT(7)

#define THERMREG_CONNECTOR_ADC_SRC_EN_BIT   BIT(4)
#define THERMREG_SKIN_ADC_SRC_EN_BIT        BIT(2)
#define THERMREG_DIE_ADC_SRC_EN_BIT     BIT(1)

// BATTERY_CHARGER_STATUS_2_REG
#define CHARGER_ERROR_STATUS_SFT_EXPIRE_BIT	BIT(2)

#define FG_ADC_RR_BATT_ID_STS			(0x425F)
//----- smb5 registers -----

#define INVALID_NEGATIVE_FORCE_ICL_UA		(-1)

#define HVDCP_PULSE_COUNT_QC2P0_9V  0x1 // limit vbus 9V for QC2
#define JEITA_FVCOMP_MINUS_P500    0x28 // minus 500mV

#define OTG_POLLING_TIME_MS (1000)
#define OTG_POLLING_TIME_INIT_MS (10000)

#ifdef CONFIG_HTC_BATT_DCJACK
enum htc_batt_smb5_detect_stage {
	DETECT_STAGE_DCJACK_NONE = 0,
	DETECT_STAGE_DCJACK_DISABLED,
	DETECT_STAGE_DCJACK,
	DETECT_STAGE_MAX,
};

enum usb_acok_status_t {
	USB_ACOK_NONE = 0,
	USB_ACOK_ONLY,
	USB_ACOK_OTG,
};
#endif

struct htc_batt_smb5_data {
	struct htc_battery_smb5_desc *desc;
	/* main charger data */
	struct delayed_work icl_set_work;
	struct delayed_work chk_usb_icl_work;
	int force_icl_ua;
#ifdef CONFIG_HTC_BATT_DCJACK
	bool is_dcjack_disabled;
	struct htc_batt_smb5_detect_data detect_data[DETECT_STAGE_MAX];
	struct delayed_work dcjack_check_work;
	struct delayed_work dcjack_reenable_work;
	struct delayed_work otg_plugin_chk_work;
	struct delayed_work restart_dcjack_disable_work;
	struct gpio_desc *acok_dcjack_gpiod;
	struct gpio_desc *acok_dcjack_ovp_gpiod;
	struct gpio_desc *acok_dcjack_switch_gpiod;
	struct gpio_desc *acok_usb_gpiod;
	u8              cc_status;
	int usb_acok_accu_times;
	bool force_disable_dcjack;
#endif
	/* smb1355 charger data */
};

#ifdef CONFIG_HTC_BATT_DCJACK
static enum usb_acok_status_t htc_battery_smb5_get_usb_acok_status(struct htc_batt_smb5_data *data);
static void htc_battery_smb5_disable_dcjack_in_period(struct htc_batt_smb5_data *data, unsigned int delay_time_ms);
static int htc_battery_smb5_disable_dcjack_acok(struct htc_batt_smb5_data *data, bool disable);
static bool htc_battery_smb5_is_dcjack_acok(struct htc_batt_smb5_data *data);
#endif

static struct htc_batt_smb5_data *htc_batt_smb5_dev[HTC_BATTERY_SMB5_DEVICE_MAX] = {0};

static inline struct htc_batt_smb5_data *
htc_battery_smb5_get_dev_data(const enum htc_battery_smb5_type type)
{
	struct htc_batt_smb5_data *data;

	if (type >= HTC_BATTERY_SMB5_DEVICE_MAX) {
		pr_err("Not supported type %d\n", type);
		return NULL;
	}

	data = htc_batt_smb5_dev[type];

	if (!data) {
		return NULL;
	}

	return data;
}

static inline int
htc_battery_smb5_set_dev_data(const enum htc_battery_smb5_type type,
			      struct htc_batt_smb5_data *data)
{
	if (type >= HTC_BATTERY_SMB5_DEVICE_MAX) {
		pr_err("Not supported type %d\n", type);
		return -EINVAL;
	}

	htc_batt_smb5_dev[type] = data;

	return 0;
}

static inline struct htc_battery_smb5_reg_ctrl *
htc_battery_smb5_get_ctrl_from_data(struct htc_batt_smb5_data *data)
{
	if (!data || !data->desc) {
		pr_err("No device description assigned\n");
		return ERR_PTR(-ENODEV);
	}

	if (!data->desc->ctrl) {
		pr_err("No ctrl for device data\n");
		return ERR_PTR(-ENXIO);
	}

	return data->desc->ctrl;
}

static inline struct htc_battery_smb5_reg_ctrl *
htc_battery_smb5_get_ctrl_by_type(const enum htc_battery_smb5_type type)
{
	struct htc_batt_smb5_data *data = htc_battery_smb5_get_dev_data(type);

	return htc_battery_smb5_get_ctrl_from_data(data);
}

static inline void *
htc_battery_smb5_get_func_from_data(struct htc_batt_smb5_data *data)
{
	if (!data || !data->desc) {
		pr_err("No device description assigned\n");
		return ERR_PTR(-ENODEV);
	}

	if (!data->desc->dev_func) {
		pr_err("No dev_func for device data\n");
		return ERR_PTR(-ENXIO);
	}

	return data->desc->dev_func;
}

static inline void *
htc_battery_smb5_get_func_by_type(const enum htc_battery_smb5_type type)
{
	struct htc_batt_smb5_data *data = htc_battery_smb5_get_dev_data(type);

	return htc_battery_smb5_get_func_from_data(data);
}

static struct smb_charger *
htc_battery_smb5_get_main_chg_from_data(struct htc_batt_smb5_data *data)
{
	if (!data || !data->desc || !data->desc->dev) {
		pr_err("No device assigned\n");
		return ERR_PTR(-ENODEV);
	}

	return container_of(data->desc->dev, struct smb_charger, dev);
}

static int htc_batt_smb5_masked_write(struct htc_batt_smb5_data *data, u16 reg, u8 mask, u8 val)
{
	struct htc_battery_smb5_reg_ctrl *ctrl = htc_battery_smb5_get_ctrl_from_data(data);

	if (IS_ERR(ctrl))
		return PTR_ERR(ctrl);

	if (!ctrl->dev_masked_write) {
		pr_err("No register masked write for dev ctrl\n");
		return -ENXIO;
	}

	return ctrl->dev_masked_write(data->desc->dev, reg, mask, val);
}

static int htc_batt_smb5_read(struct htc_batt_smb5_data *data, u16 reg, u8 *val)
{
	struct htc_battery_smb5_reg_ctrl *ctrl = htc_battery_smb5_get_ctrl_from_data(data);

	if (IS_ERR(ctrl))
		return PTR_ERR(ctrl);

	if (!val) {
		pr_err("Invalid value parameter for read\n");
		return -EINVAL;
	}

	if (!ctrl->dev_read) {
		pr_err("No register read for dev ctrl\n");
		return -ENXIO;
	}

	return ctrl->dev_read(data->desc->dev, reg, val);
}

static inline htc_batt_smb5_main_masked_write(u16 reg, u8 mask, u8 val)
{
	const enum htc_battery_smb5_type type = HTC_BATTERY_SMB5_MAIN;
	struct htc_batt_smb5_data *data = htc_battery_smb5_get_dev_data(type);

	return htc_batt_smb5_masked_write(data, reg, mask, val);
}

static int htc_batt_smb5_main_read(u16 reg, u8 *val)
{
	const enum htc_battery_smb5_type type = HTC_BATTERY_SMB5_MAIN;
	struct htc_batt_smb5_data *data = htc_battery_smb5_get_dev_data(type);

	if (!data) {
		pr_err("No dev data assigned\n");
		return -ENODEV;
	}

	return htc_batt_smb5_read(data, reg, val);
}


/*
 *
 * functions for htc_battery
 *
 */

static void htc_batt_smb5_enable_aicl(bool enable)
{
	u8 val = enable ? USBIN_AICL_EN_BIT : 0;

	htc_batt_smb5_main_masked_write(USBIN_AICL_OPTIONS_CFG_REG, USBIN_AICL_EN_BIT, val);
}

static void htc_batt_smb5_is_aicl_done(int *result)
{
	u8 stat = 0;

	htc_batt_smb5_main_read(AICL_STATUS_REG, &stat);
	*result = !!(stat & AICL_DONE_BIT);
}

static void htc_batt_smb5_get_aicl_config(int *stat)
{
	htc_batt_smb5_main_read(USBIN_AICL_OPTIONS_CFG_REG, (u8 *) stat);
}

static void htc_batt_smb5_rerun_aicl(void)
{
	htc_batt_smb5_main_masked_write(USBIN_AICL_OPTIONS_CFG_REG,
			SUSPEND_ON_COLLAPSE_USBIN_BIT, 0);
	msleep(50);
	htc_batt_smb5_main_masked_write(USBIN_AICL_OPTIONS_CFG_REG,
			SUSPEND_ON_COLLAPSE_USBIN_BIT, SUSPEND_ON_COLLAPSE_USBIN_BIT);
	// rerun AICL
	htc_batt_smb5_main_masked_write(CMD_HVDCP_2_REG, TRIGGER_AICL_BIT, TRIGGER_AICL_BIT);
}

#define RECOVERY_DCJACK_CHARGING_DELAY_MS	(50)
static void htc_batt_smb5_overload_recovery_check(void)
{
#ifdef CONFIG_HTC_BATT_DCJACK
	const enum htc_battery_smb5_type type = HTC_BATTERY_SMB5_MAIN;
	struct htc_batt_smb5_data *data = htc_battery_smb5_get_dev_data(type);
	struct smb_charger *chg = htc_battery_smb5_get_main_chg_from_data(data);
	int ret;
	u8 vbus, sw_pwr, stat;

	if (IS_ERR(chg)) {
		pr_err("No charger device assigned for type %d\n", type);
		return;
	}

	ret = htc_batt_smb5_read(data, USBIN_BASE + INT_RT_STS_OFFSET, &vbus);
	if (ret < 0) {
		pr_err("read USBIN_RT_STS failed, rc=%d\n", ret);
		return;
	}

	if (!!(vbus & USBIN_PLUGIN_RT_STS_BIT) && chg->is_dcjack_acok) {
		/* case 1: make sure ICL_OVERRIDE is set */
		ret = htc_batt_smb5_masked_write(data, CMD_ICL_OVERRIDE_REG,
							ICL_OVERRIDE_BIT, ICL_OVERRIDE_BIT);
		if (ret < 0) {
			pr_err("write ICL_OVERRIDE_BIT failed, rc=%d\n", ret);
			return;
		}

		/* case 2: disable/enable dcjack if INPUT status error */
		ret = htc_batt_smb5_read(data, DCDC_BASE + INT_RT_STS_OFFSET , &sw_pwr);
		if (ret < 0) {
			pr_err("read DCDC_INT_RT_STS failed, rc=%d\n", ret);
			return;
		}

		ret = htc_batt_smb5_read(data, USBIN_INPUT_STATUS_REG, &stat);
		if (ret < 0) {
			pr_err("read USBIN_INPUT_STATUS failed, rc=%d\n", ret);
			return;
		}

		if (!(sw_pwr & SWITCHER_POWER_OK_RT_STS) && !!(stat & USBIN_5V)) {
			pr_err("dcdc switch power not enabled, re-enable DCJACK\n");
			htc_battery_smb5_disable_dcjack_in_period(data, RECOVERY_DCJACK_CHARGING_DELAY_MS);
		}
	}
#endif
}

static void htc_batt_smb5_is_charging_terminate(int *result)
{
	u8 chg_sts = 0;

	htc_batt_smb5_main_read(BATTERY_CHARGER_STATUS_1_REG, &chg_sts);
	*result = (chg_sts & BATTERY_CHARGER_STATUS_MASK) == TERMINATE_CHARGE;
}

static void htc_batt_check_reset_safety_timer(bool is_safety_timeout)
{
	u8 reg;
	int ret;

	if (is_safety_timeout) {
		htc_batt_smb5_main_read(APSD_RESULT_STATUS_REG, &reg);
		if ((get_kernel_flag() & KERNEL_FLAG_KEEP_CHARG_ON)||
			(get_kernel_flag() & KERNEL_FLAG_DISABLE_SAFETY_TIMER)||
			(reg & SDP_CHARGER_BIT)) {
			pr_err("Reset safety timer.");
			/* Base on case 03753197, we reset safety timer by re-enable 0x1340 */
			ret = htc_batt_smb5_main_masked_write(USBIN_CMD_IL_REG, USBIN_SUSPEND_BIT, 1);
			if (ret < 0)
				pr_err("Fail to write reg USBIN_CMD_IL_REG for suspend 1, ret=%d\n", ret);
			msleep(50);
			ret = htc_batt_smb5_main_masked_write(USBIN_CMD_IL_REG, USBIN_SUSPEND_BIT, 0);
			if (ret < 0)
				pr_err("Fail to write reg USBIN_CMD_IL_REG for suspend 0, ret=%d\n", ret);
		}
	}
}

static void htc_batt_smb5_is_saftytimer_expired(int *result)
{
	u8 reg = 0;

	htc_batt_smb5_main_read(BATTERY_CHARGER_STATUS_2_REG, &reg);
	*result = !!(reg & CHARGER_ERROR_STATUS_SFT_EXPIRE_BIT);

	htc_batt_check_reset_safety_timer(!!(reg & CHARGER_ERROR_STATUS_SFT_EXPIRE_BIT));
}

#define WP_THERM_STR	"pm8150b-wp-therm"
#define WP_THERM_LIMIT_ENABLE_THRESHOLD		(85000)
#define WP_THERM_LIMIT_RECOVERY_THRESHOLD	(75000)
#define WP_THERM_LIMIT_MA	(1000)
static void htc_batt_smb5_get_charging_limit_ma(int *result)
{
	struct thermal_zone_device *tzd = NULL;
	int temp = 0, ret;
	static bool is_limit_on = false;

	if (!result)
		return;

	*result = -EINVAL;

	tzd = thermal_zone_get_zone_by_name(WP_THERM_STR);
	if (IS_ERR_OR_NULL(tzd) || !tzd->ops->get_temp) {
		pr_err("%s: sensor(%s) is not ready to read temp\n", __func__, WP_THERM_STR);
		return;
	}

	ret = tzd->ops->get_temp(tzd, &temp);
	if (ret != 0) {
		pr_err("%s: (%s) thermal_zone_get_temp() failed tzd->id=%d, ret=%d\n",
				__func__, tzd->type, tzd->id, ret);
		return;
	}

	if (temp > WP_THERM_LIMIT_ENABLE_THRESHOLD)
		is_limit_on = true;
	else if (temp < WP_THERM_LIMIT_RECOVERY_THRESHOLD)
		is_limit_on = false;

	if (is_limit_on)
		*result = WP_THERM_LIMIT_MA;
}

#ifdef CONFIG_HTC_BATT_DCJACK
static void htc_batt_smb5_log_regs(struct htc_batt_smb5_data *data, u32 start_addr, u32 len)
{
	u32 addr, end_addr;
	u8 reg[16], buf;
	int ret;

	end_addr = start_addr + (len & 0xFFF0) + 0xF;

	if (!data || end_addr < start_addr)
		return;

	for (addr = start_addr; addr <= end_addr; addr++) {
		ret = htc_batt_smb5_read(data, addr, &buf);
		if (!ret) {
			reg[addr%16] = buf;
		} else {
			pr_err("addr=%x read error", addr);
			break;
		}

		if (addr%16 == 15) {
			pr_err("Start addr %04x: [%02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x]\n",
					addr - 0xF, reg[0], reg[1], reg[2], reg[3], reg[4], reg[5], reg[6], reg[7],
					reg[8], reg[9], reg[10], reg[11], reg[12], reg[13], reg[14], reg[15]
			      );
		}
	}
}

static int htc_batt_smb5_handle_acok_type_mismatch(struct htc_batt_smb5_data *data)
{

	if (!data)
		return -EINVAL;

	pr_err("acok mismatch found!!! dump registers");
	htc_batt_smb5_log_regs(data, 0x1100, 0x5F);
	htc_batt_smb5_log_regs(data, 0x1310, 0xF);
	htc_batt_smb5_log_regs(data, 0xC100, 0x4F);
	htc_batt_smb5_log_regs(data, 0xC400, 0x4F);

	return 0;
}
#endif

#define ACOK_TYPE_MISMATCH_LOG_COUNT	(3)
#define ACOK_TYPE_MISMATCH_COUNT_MAX	(5)
static void htc_batt_smb5_charge_check(struct battery_info_reply *rep)
{
#ifdef CONFIG_HTC_BATT_DCJACK
	const enum htc_battery_smb5_type type = HTC_BATTERY_SMB5_MAIN;
	struct htc_batt_smb5_data *data = htc_battery_smb5_get_dev_data(type);
	struct smb_charger *chg = htc_battery_smb5_get_main_chg_from_data(data);
	u8 vbus;
	bool is_dcjack = htc_battery_smb5_is_dcjack_acok(data);
	bool is_usb = htc_battery_smb5_get_usb_acok_status(data) == USB_ACOK_ONLY;
	static unsigned int mismatch_count = 0;
	int rc;

	if (!rep)
		return;

	rc = htc_batt_smb5_read(data, USBIN_BASE + INT_RT_STS_OFFSET, &vbus);
	if (rc < 0) {
		pr_err("Couldn't read USBIN_RT_STS rc=%d\n", rc);
	}

	if ((is_dcjack ^ chg->is_dcjack_acok) ||
		(!is_dcjack && (is_usb ^ chg->is_usb_acok)) ||
		((bool) (vbus & USBIN_PLUGIN_RT_STS_BIT)) ^ (rep->chg_src > POWER_SUPPLY_TYPE_UNKNOWN)) {
		pr_err("is_dcjack_acok=%d(%d) is_usb_acok=%d(%d), vbus=0x%x",
				is_dcjack, chg->is_dcjack_acok,
				is_usb, chg->is_usb_acok, vbus);
		++mismatch_count;
	} else {
		mismatch_count = 0;
	}

	if (mismatch_count == ACOK_TYPE_MISMATCH_LOG_COUNT) {
		htc_batt_smb5_handle_acok_type_mismatch(data);
	} else if (mismatch_count >= ACOK_TYPE_MISMATCH_COUNT_MAX) {
		mismatch_count = 0;
	}
#endif
}

#define FG_BCL_LMH_STS1 0x4209
static void htc_batt_smb5_dump_reg2buf(struct htc_batt_dump_reg2buf_data *data)
{
	u8 reg, addr;
	char *buf = data->buf;
	int len = data->len;

	if (!buf)
		return;

	// Dump pm8150b register
	htc_batt_smb5_main_read(USBIN_CMD_IL_REG, &reg);			// 0x1340
	len += scnprintf(buf + len, PAGE_SIZE - len, "USB_CMD_IL_REG: 0x%02x;\n", reg);
	htc_batt_smb5_main_read(USBIN_CURRENT_LIMIT_CFG_REG, &reg);	// 0x1370
	len += scnprintf(buf + len, PAGE_SIZE - len, "USBIN_CURRENT_LIMIT_CFG: 0x%02x;\n", reg);
	htc_batt_smb5_main_read(USBIN_AICL_OPTIONS_CFG_REG, &reg);	// 0x1380
	len += scnprintf(buf + len, PAGE_SIZE - len, "USBIN_AICL_OPTIONS_CFG: 0x%02x;\n", reg);
	htc_batt_smb5_main_read(CHGR_FAST_CHARGE_CURRENT_CFG_REG, &reg);	// 0x1061
	len += scnprintf(buf + len, PAGE_SIZE - len, "FAST_CHARGE_CURRENT_CFG: 0x%02x;\n", reg);
	htc_batt_smb5_main_read(FG_BCL_LMH_STS1, &reg);
	len += scnprintf(buf + len, PAGE_SIZE - len, "FG_BCL_LMH_STS1: 0x%02x;\n", reg);//0x4209

	/* charger peripheral */
	len += scnprintf(buf + len, PAGE_SIZE - len, "CHGR_STS ------;\n");
	for (addr = 0x6; addr <= 0xE; addr++){
		htc_batt_smb5_main_read(CHGR_BASE + addr, &reg);	//"CHGR Status"
		len += scnprintf(buf + len, PAGE_SIZE - len,"0x%04x: 0x%02x;\n", CHGR_BASE + addr, reg);
	}
	/* battery interface peripheral */

	/* usb charge path peripheral */
	len += scnprintf(buf + len, PAGE_SIZE - len,"USB_STS ------;\n");
	for (addr = 0x6; addr <= 0x10; addr++){
		htc_batt_smb5_main_read(USBIN_BASE + addr, &reg);	//"USB Status"
		len += scnprintf(buf + len, PAGE_SIZE - len,"0x%04x: 0x%02x;\n", USBIN_BASE + addr, reg);
	}
	len += scnprintf(buf + len, PAGE_SIZE - len,"USB_CFG ------;\n");
	for (addr = 0x58; addr <= 0x66; addr++){
		htc_batt_smb5_main_read(USBIN_BASE + addr, &reg); //"USB Config"
		len += scnprintf(buf + len, PAGE_SIZE - len,"0x%04x: 0x%02x;\n", USBIN_BASE + addr, reg);
	}

	htc_batt_smb5_main_read(USBIN_BASE + 0x70, &reg); //"USB Config"
	len += scnprintf(buf + len, PAGE_SIZE - len,"0x%04x: 0x%02x;\n", USBIN_BASE + 0x70, reg);

	for (addr = 0x80; addr <= 0x84; addr++){
		htc_batt_smb5_main_read(USBIN_BASE + addr, &reg); //"USB Config"
		len += scnprintf(buf + len, PAGE_SIZE - len,"0x%04x: 0x%02x;\n", USBIN_BASE + addr, reg);
	}

	/* misc peripheral */
	len += scnprintf(buf + len, PAGE_SIZE - len,"MISC_STS ------;\n");
	for (addr = 0x6; addr <= 0x10; addr++){
		htc_batt_smb5_main_read(MISC_BASE + addr, &reg);	//"MISC Status"
		len += scnprintf(buf + len, PAGE_SIZE - len,"0x%04x: 0x%02x;\n", MISC_BASE + addr, reg);
	}
	data->len = len;
}

#define DUMP_ALL_PM8150B_REGS	0
static void htc_batt_smb5_dump_reg(void)
{
	const enum htc_battery_smb5_type type = HTC_BATTERY_SMB5_MAIN;
	struct htc_batt_smb5_data *data = htc_battery_smb5_get_dev_data(type);
#if DUMP_ALL_PM8150B_REGS
	u32 addr_l, addr_r, addr;
	u8 reg[16], buf;
	int ret;

	if (!data) {
		pr_err("%s:No dev data assigned\n", __func__);
		return;
	}

	for (addr_l = 0x10; addr_l <= 0x16; addr_l++) {
		for (addr_r = 0; addr_r <= 0xFF; addr_r++) {
			addr = (addr_l << 8) + addr_r;
			ret = htc_batt_smb5_read(data, addr, &buf);
			if (!ret) {
				reg[addr%16] = buf;
			} else {
				pr_err("addr=%x read error", addr);
				break;
			}

			if (addr%16 == 15) {
				pr_err("Start addr %04x: [%02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x]\n",
						addr - 0xF, reg[0], reg[1], reg[2], reg[3], reg[4], reg[5], reg[6], reg[7],
						reg[8], reg[9], reg[10], reg[11], reg[12], reg[13], reg[14], reg[15]
				      );
			}
		}
	}
#else
	u8 chg_sts[11], chg_42, chg_61, chg_70;
	u8 dcdc_09, dcdc_10, dcdc_50, bat_10;
	u8 usb_sts[11], usb_18, usb_40, usb_42, usb_44, usb_5x[5], usb_6x[9], usb_70, usb_8x[5];
	u8 typec_18;
	u8 misc_sts[11], misc_70;
	u8 fg_adcrr_5f;
	u16 idx;

	if (!data) {
		pr_err("%s:No dev data assigned\n", __func__);
		return;
	}

	/* Dump for CHGR part */
	for (idx = 0; idx < sizeof(chg_sts); idx++)
		htc_batt_smb5_read(data, BATTERY_CHARGER_STATUS_1_REG + idx, &chg_sts[idx]);
	htc_batt_smb5_read(data, CHARGING_ENABLE_CMD_REG, &chg_42);
	htc_batt_smb5_read(data, CHGR_FAST_CHARGE_CURRENT_CFG_REG, &chg_61);
	htc_batt_smb5_read(data, CHGR_FLOAT_VOLTAGE_CFG_REG, &chg_70);

	/* Dump for DCDC part */
	htc_batt_smb5_read(data, DCDC_OTG_STATUS_REG, &dcdc_09);
	htc_batt_smb5_read(data, (DCDC_BASE + INT_RT_STS_OFFSET), &dcdc_10);
	htc_batt_smb5_read(data, DCDC_FSW_SEL_REG, &dcdc_50);

	/* Dump for BATIF part */
	htc_batt_smb5_read(data, (BATIF_BASE + INT_RT_STS_OFFSET), &bat_10);

	/* Dump for USBIN part */
	for (idx = 0; idx < sizeof(usb_sts); idx++)
		htc_batt_smb5_read(data, USBIN_INPUT_STATUS_REG + idx, &usb_sts[idx]);

	htc_batt_smb5_read(data, USB_INT_LATCHED_STS_REG, &usb_18);
	htc_batt_smb5_read(data, USBIN_CMD_IL_REG, &usb_40);
	for (idx = 0; idx < sizeof(usb_5x); idx++)
		htc_batt_smb5_read(data, TYPE_C_CFG_REG + idx, &usb_5x[idx]);
	for (idx = 0; idx < sizeof(usb_6x); idx++)
		htc_batt_smb5_read(data, USBIN_ADAPTER_ALLOW_CFG_REG + idx, &usb_6x[idx]);
	htc_batt_smb5_read(data, USBIN_CURRENT_LIMIT_CFG_REG, &usb_70);
	for (idx = 0; idx < sizeof(usb_8x); idx++)
		htc_batt_smb5_read(data, USBIN_AICL_OPTIONS_CFG_REG + idx, &usb_8x[idx]);
	htc_batt_smb5_read(data, CMD_ICL_OVERRIDE_REG, &usb_42);
	htc_batt_smb5_read(data, USBIN_ADAPTER_ALLOW_OVERRIDE_REG, &usb_44);
	htc_batt_smb5_read(data, FG_ADC_RR_BATT_ID_STS, &fg_adcrr_5f);

	/* Dump for TYPEC part*/
	htc_batt_smb5_read(data, TYPEC_INT_LATCHED_STS_REG, &typec_18);

	/* Dump for MISC part */
	for (idx = 0; idx < sizeof(misc_sts); idx++)
		htc_batt_smb5_read(data, TEMP_RANGE_STATUS_REG + idx, &misc_sts[idx]);

	htc_batt_smb5_read(data, MISC_THERMREG_SRC_CFG_REG, &misc_70);

	BATT_DUMP("CHG[06:10]=[%02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x],CHG[42,61,70]=[%02x,%02x,%02x],FG_ADCRR[5F]=[%02x]\n",
				chg_sts[0], chg_sts[1], chg_sts[2], chg_sts[3], chg_sts[4], chg_sts[5],
				chg_sts[6],chg_sts[7], chg_sts[8], chg_sts[9], chg_sts[10],
				chg_42, chg_61, chg_70,
                                fg_adcrr_5f);

	BATT_DUMP("USB[06:10]=[%02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x],USB[18]=[%02x]\
					,USB[40,42,44]=[%02x,%02x,%02x],USB[58:5B]=[%02x,%02x,%02x,%02x,%02x]\n",
				usb_sts[0], usb_sts[1], usb_sts[2], usb_sts[3], usb_sts[4], usb_sts[5],
				usb_sts[6], usb_sts[7], usb_sts[8], usb_sts[9], usb_sts[10], usb_18,
				usb_40, usb_42, usb_44, usb_5x[0], usb_5x[1], usb_5x[2], usb_5x[3], usb_5x[4]);
	BATT_DUMP("USB[60:68]=[%02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x],USB[70]=[%02x],USB[80:84]=[%02x,%02x,%02x,%02x,%02x]\n",
				usb_6x[0], usb_6x[1], usb_6x[2], usb_6x[3], usb_6x[4], usb_6x[5],
				usb_6x[6], usb_6x[7], usb_6x[8],
				usb_70, usb_8x[0], usb_8x[1], usb_8x[2], usb_8x[3], usb_8x[4]);

	BATT_DUMP("TYPEC[18]=[%02x]\n", typec_18);

	BATT_DUMP("MISC[06:10]=[%02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x],MISC[70]=[%02x],DCDC[09,10,50]=[%02x,%02x.%02x],BAT[10]=[%02x]\n",
				misc_sts[0], misc_sts[1], misc_sts[2], misc_sts[3], misc_sts[4], misc_sts[5],
				misc_sts[6], misc_sts[7], misc_sts[8], misc_sts[9], misc_sts[10],misc_70,
				dcdc_09, dcdc_10, dcdc_50, bat_10);


#endif
}


#define DELAYED_OVERRIDE_ICL_MS	(1000)
static int htc_batt_smb5_set_input_current(struct htc_batt_set_icl_data *icl_val)
{
	const enum htc_battery_smb5_type type = HTC_BATTERY_SMB5_MAIN;
	struct htc_batt_smb5_data *data = htc_battery_smb5_get_dev_data(type);
	struct smb_charger *chg = htc_battery_smb5_get_main_chg_from_data(data);
	union power_supply_propval pval;

	if (IS_ERR(chg) || IS_ERR(chg->usb_psy)) {
		pr_err("No charger device assigned for type %d\n", type);
		return -ENODEV;
	}

	if (icl_val->val < 0) {
		pr_err("Invalid input current ul=%d\n", icl_val->val);
		return -EINVAL;
	}

	pval.intval = icl_val->val;
	power_supply_set_property(chg->usb_psy, POWER_SUPPLY_PROP_SDP_CURRENT_MAX, &pval);

	cancel_delayed_work_sync(&data->icl_set_work);
	if (icl_val->force) {
		// NOTE: Delay to override the late POWER_SUPPLY_PROP_SDP_CURRENT_MAX setting by USB in
		//       which will change the setting after htc_battery set it.
		data->force_icl_ua = icl_val->val;
		schedule_delayed_work(&data->icl_set_work, msecs_to_jiffies(DELAYED_OVERRIDE_ICL_MS));
	} else if (data->force_icl_ua >= 0) {
		data->force_icl_ua = INVALID_NEGATIVE_FORCE_ICL_UA;
	}

	return 0;
}

static int htc_batt_smb5_handle_prop(enum htc_batt_dev_ctrl_prop prop,
					    htc_batt_dev_ctrl_propval *val)
{
	int ret = 0;

	switch (prop) {
	case HTC_BATT_DEV_CTRL_ENABLE_AICL:
		if (!val)
			ret = -EINVAL;
		else
			htc_batt_smb5_enable_aicl(!!val->intval);
		break;
	case HTC_BATT_DEV_CTRL_IS_AICL_DONE:
		if (!val)
			ret = -EINVAL;
		else
			htc_batt_smb5_is_aicl_done(&val->intval);
		break;
	case HTC_BATT_DEV_CTRL_GET_AICL_CONFIG:
		if (!val)
			ret = -EINVAL;
		else
			htc_batt_smb5_get_aicl_config(&val->intval);
		break;
	case HTC_BATT_DEV_CTRL_RERUN_AICL:
		htc_batt_smb5_rerun_aicl();
		break;
	case HTC_BATT_DEV_CTRL_IS_CHARGING_TERMINATE:
		if (!val)
			ret = -EINVAL;
		else
			htc_batt_smb5_is_charging_terminate(&val->intval);
		break;
	case HTC_BATT_DEV_CTRL_IS_SAFTYTIMER_EXPIRED:
		if (!val)
			ret = -EINVAL;
		else
			htc_batt_smb5_is_saftytimer_expired(&val->intval);
		break;
	case HTC_BATT_DEV_CTRL_DUMP_REGISTER_TO_BUF:
		if (!val || !val->data)
			ret = -EINVAL;
		else
			htc_batt_smb5_dump_reg2buf((struct htc_batt_dump_reg2buf_data *) val->data);
		break;
	case HTC_BATT_DEV_CTRL_DUMP_REGISTER:
		htc_batt_smb5_dump_reg();
		break;
	case HTC_BATT_DEV_CTRL_SET_INPUT_CURRENT:
		if (!val)
			ret = -EINVAL;
		else
			ret = htc_batt_smb5_set_input_current((struct htc_batt_set_icl_data *) val->data);
		break;
	case HTC_BATT_DEV_CTRL_OVERLOAD_RECOVERY_CHECK:
		htc_batt_smb5_overload_recovery_check();
		break;
	case HTC_BATT_DEV_CTRL_GET_CHARGING_LIMIT_MA:
		if (!val)
			ret = -EINVAL;
		else
			htc_batt_smb5_get_charging_limit_ma(&val->intval);
		break;
	case HTC_BATT_DEV_CTRL_CHARGE_CHECK:
		if (!val)
			ret = -EINVAL;
		else
			htc_batt_smb5_charge_check((struct battery_info_reply *) val->data);
		break;
	default:
		pr_err("Not supported htc battery device control prop %d\n", prop);
		ret = -EINVAL;
	}

	return ret;
}

static enum htc_batt_dev_ctrl_prop htc_batt_smb5_props[] = {
	HTC_BATT_DEV_CTRL_ENABLE_AICL,
	HTC_BATT_DEV_CTRL_IS_AICL_DONE,
	HTC_BATT_DEV_CTRL_GET_AICL_CONFIG,
	HTC_BATT_DEV_CTRL_RERUN_AICL,
	HTC_BATT_DEV_CTRL_IS_CHARGING_TERMINATE,
	HTC_BATT_DEV_CTRL_IS_SAFTYTIMER_EXPIRED,
	HTC_BATT_DEV_CTRL_DUMP_REGISTER_TO_BUF,
	HTC_BATT_DEV_CTRL_DUMP_REGISTER,
	HTC_BATT_DEV_CTRL_SET_INPUT_CURRENT,
	HTC_BATT_DEV_CTRL_GET_CHARGING_LIMIT_MA,
};

#define PROP_NUM(props) (sizeof(props)/sizeof(props[0]))
static struct htc_batt_dev_ctrl_desc htc_batt_smb5_ctrl_desc[] = {
	[HTC_BATTERY_SMB5_MAIN] = {
		.dev_type = HTC_BATT_CHARGER_MASTER,
		.properties = htc_batt_smb5_props,
		.num_properties = PROP_NUM(htc_batt_smb5_props),
		.handle_prop = htc_batt_smb5_handle_prop
	},
	[HTC_BATTERY_SMB5_SMB1355] = {0},
};

static struct htc_batt_dev_ctrl_desc *htc_batt_smb5_ctrl_dev[] = {
	[HTC_BATTERY_SMB5_MAIN] = &htc_batt_smb5_ctrl_desc[HTC_BATTERY_SMB5_MAIN],
	[HTC_BATTERY_SMB5_SMB1355] = NULL,
};


/*
 *
 * functions for qpnp-smb5/smb5-lib
 *
 */

void htc_battery_smb5_info_update(enum power_supply_property prop, int intval)
{
	htc_battery_info_update(prop, intval);
}
EXPORT_SYMBOL(htc_battery_smb5_info_update);

int htc_battery_smb5_level_adjust(void)
{
	return htc_battery_level_adjust();
}
EXPORT_SYMBOL(htc_battery_smb5_level_adjust);

bool htc_battery_smb5_get_discharging_reason(void)
{
	return htc_battery_get_discharging_reason();
}
EXPORT_SYMBOL(htc_battery_smb5_get_discharging_reason);

void htc_battery_smb5_notify_unknown_charger(bool is_unknown)
{
	htc_battery_notify_unknown_charger(is_unknown);
}
EXPORT_SYMBOL(htc_battery_smb5_notify_unknown_charger);

int htc_battery_smb5_batt_temp_adjust(int temp)
{
	return htc_battery_adjust_batt_temp(temp);
}
EXPORT_SYMBOL(htc_battery_smb5_batt_temp_adjust);

// NOTE: get_rp_based_dcp_current is copied from smb5-lib.c
#define SDP_CURRENT_UA			500000
#define CDP_CURRENT_UA			1500000
#define DCP_CURRENT_UA			1500000
#define HVDCP_CURRENT_UA		3000000
#define TYPEC_DEFAULT_CURRENT_UA	900000
#define TYPEC_MEDIUM_CURRENT_UA		1500000
#define TYPEC_HIGH_CURRENT_UA		3000000
static int get_rp_based_dcp_current(struct smb_charger *chg, int typec_mode)
{
	int rp_ua;

	switch (typec_mode) {
	case POWER_SUPPLY_TYPEC_SOURCE_HIGH:
		rp_ua = TYPEC_HIGH_CURRENT_UA;
		break;
	case POWER_SUPPLY_TYPEC_SOURCE_MEDIUM:
	case POWER_SUPPLY_TYPEC_SOURCE_DEFAULT:
	/* fall through */
	default:
		rp_ua = DCP_CURRENT_UA;
	}

	return rp_ua;
}

#define ICL_CHECK_DELAY_MS		10000
void htc_battery_smb5_charger_check_after_apsd(
	const char * const name, const u8 bit, const enum power_supply_type pst)
{
	const enum htc_battery_smb5_type type = HTC_BATTERY_SMB5_MAIN;
	struct htc_batt_smb5_data *data = htc_battery_smb5_get_dev_data(type);
	struct smb_charger *chg = htc_battery_smb5_get_main_chg_from_data(data);
	int rp_current;

	if (IS_ERR(chg)) {
		pr_err("No charger device assigned for type %d\n", type);
		return;
	}

	switch (bit) {
	case SDP_CHARGER_BIT:
		break;
	case CDP_CHARGER_BIT:
		break;
	case FLOAT_CHARGER_BIT:
		vote(chg->usb_icl_votable, SW_ICL_MAX_VOTER, true, 500000);
		htc_battery_smb5_notify_unknown_charger(true);
		break;
	case OCP_CHARGER_BIT:
		break;
	case DCP_CHARGER_BIT:
#ifdef CONFIG_HTC_BATT_DCJACK
		if (chg->is_dcjack_acok)
			break;
#endif
		// DCP current
		if (!chg->pd_active){
			rp_current = get_rp_based_dcp_current(chg, chg->typec_mode);
			vote(chg->usb_icl_votable, USB_PSY_VOTER, true, rp_current);
			schedule_delayed_work(&data->chk_usb_icl_work,
					msecs_to_jiffies(ICL_CHECK_DELAY_MS));
		}
		break;
	default:
		break;
	}

	if ((pst == POWER_SUPPLY_TYPE_USB_HVDCP) || (pst == POWER_SUPPLY_TYPE_USB_HVDCP_3))
		cancel_delayed_work_sync(&data->chk_usb_icl_work);

}
EXPORT_SYMBOL(htc_battery_smb5_charger_check_after_apsd);

void htc_battery_smb5_handle_usb_removal(bool vbus_rising)
{
	const enum htc_battery_smb5_type type = HTC_BATTERY_SMB5_MAIN;
	struct htc_batt_smb5_data *data = htc_battery_smb5_get_dev_data(type);

	if (!data) {
		pr_err("%s: htc_batt device type %d not found\n", __func__, type);
		return;
	}

	if (!vbus_rising) {
		// 5V/2A check
		cancel_delayed_work_sync(&data->chk_usb_icl_work);
		// unknown charger check
		htc_battery_smb5_notify_unknown_charger(false);
		// set IUSB 500mA to prevent next time insert 1.5A peak.
		htc_batt_smb5_main_masked_write(USBIN_CURRENT_LIMIT_CFG_REG, 0xFF, 0xA);
	}
}
EXPORT_SYMBOL(htc_battery_smb5_handle_usb_removal);

#ifdef CONFIG_HTC_BATT_DCJACK
/*
 * This is added into smblib_set_opt_switcher_freq() in smb5-lib.c to adjust fsw_khz for DCJACK
 */
int htc_battery_smb5_adjust_fsw_freq(int fsw_khz)
{
	const enum htc_battery_smb5_type type = HTC_BATTERY_SMB5_MAIN;
	struct htc_batt_smb5_data *data = htc_battery_smb5_get_dev_data(type);
	struct smb_charger *chg = htc_battery_smb5_get_main_chg_from_data(data);

	if (IS_ERR(chg))
		return fsw_khz;

	if (chg->is_dcjack_acok && !data->is_dcjack_disabled) {
		return chg->chg_freq.freq_12V;
	}

	return fsw_khz;
}
EXPORT_SYMBOL(htc_battery_smb5_adjust_fsw_freq);

/*
 * This is added into smblib_set_adapter_allowance() in smb5-lib.c to adjust allowed_voltage for DCJACK
 */
#define ALLOWED_VOLTAGE_BITS_VALUE(v)	(v & 0xF)
u8 htc_battery_smb5_adjust_allowed_voltage(int allowed_voltage)
{
	const enum htc_battery_smb5_type type = HTC_BATTERY_SMB5_MAIN;
	struct htc_batt_smb5_data *data = htc_battery_smb5_get_dev_data(type);
	struct smb_charger *chg = htc_battery_smb5_get_main_chg_from_data(data);

	if (IS_ERR(chg))
		return allowed_voltage;

	if (chg->is_dcjack_acok && !data->is_dcjack_disabled) {
		return USBIN_ADAPTER_ALLOW_5V_OR_9V_TO_12V;
	}

	return (u8) ALLOWED_VOLTAGE_BITS_VALUE(allowed_voltage);
}
EXPORT_SYMBOL(htc_battery_smb5_adjust_allowed_voltage);

int htc_battery_smb5_check_typec_mode_change_ignored(void)
{
	const enum htc_battery_smb5_type type = HTC_BATTERY_SMB5_MAIN;
	struct htc_batt_smb5_data *data = htc_battery_smb5_get_dev_data(type);
	struct smb_charger *chg = htc_battery_smb5_get_main_chg_from_data(data);
	enum usb_acok_status_t usb_acok_status = htc_battery_smb5_get_usb_acok_status(data);

	if (IS_ERR(chg)) {
		pr_err("No charger data found\n");
		return -EINVAL;
	}

	pr_err("is->dcjack:%d, is_dcjack_disabled:%d, usb_acok_status_t:%d, pr_swap_in_progress:%d.\n",
					chg->is_dcjack_acok, data->is_dcjack_disabled, usb_acok_status, chg->pr_swap_in_progress);

	/*if dcjack plug in and the reenable work is finish, return 0 for ignoring the typec mode
	 * 0: not ignore, 1: ignore
	 */

	return (chg->is_dcjack_acok
				&& !data->is_dcjack_disabled
				&& usb_acok_status != USB_ACOK_NONE
				&& !chg->pr_swap_in_progress
				&& !(usb_acok_status == USB_ACOK_OTG && chg->sink_src_mode == UNATTACHED_MODE));

}
EXPORT_SYMBOL(htc_battery_smb5_check_typec_mode_change_ignored);

int htc_battery_smb5_notify(enum htc_batt_smb5_chg_event event, int val)
{
	const enum htc_battery_smb5_type type = HTC_BATTERY_SMB5_MAIN;
	struct htc_batt_smb5_data *data = htc_battery_smb5_get_dev_data(type);
	struct smb_charger *chg = htc_battery_smb5_get_main_chg_from_data(data);
	int reenable_dcjack_delay_ms = -1;
	enum htc_batt_smb5_detect_stage stage;

	if (IS_ERR(chg)) {
		pr_err("No charger data found\n");
		return -EINVAL;
	}

	mutex_lock(&chg->dcjack_lock);
	if (data->is_dcjack_disabled)
		stage = DETECT_STAGE_DCJACK_DISABLED;
	else if (!chg->is_dcjack_acok)
		stage = DETECT_STAGE_DCJACK_NONE;
	else
		stage = DETECT_STAGE_DCJACK;

	switch (event) {
	case HTC_BATT_SMB5_CHG_EVENT_TYPEC_MODE:
		data->detect_data[stage].typec_mode = val;
		break;
	case HTC_BATT_SMB5_CHG_EVENT_APSD:
		data->detect_data[stage].apsd_type = val;
		if (data->is_dcjack_disabled) {
			if (val == POWER_SUPPLY_TYPE_USB_DCP || val == POWER_SUPPLY_TYPE_USB_HVDCP)
				reenable_dcjack_delay_ms = 2000;
			else if (val == POWER_SUPPLY_TYPE_USB_FLOAT)
				reenable_dcjack_delay_ms = 2000;
			else
				reenable_dcjack_delay_ms = 2000;
		}
		// TODO: maybe notitify ok_to_pd here if noncompliance cable
		break;
	case HTC_BATT_SMB5_CHG_EVENT_OK_TO_PD:
		data->detect_data[stage].ok_to_pd = val;
		break;
	case HTC_BATT_SMB5_CHG_EVENT_PD_ACTIVE:
		data->detect_data[stage].pd_active = val;
		if (val == 1 && data->is_dcjack_disabled)
			reenable_dcjack_delay_ms = 1000;
		break;
	case HTC_BATT_SMB5_CHG_EVENT_PD_ICL:
		data->detect_data[stage].pd_icl = val;
		break;
	case HTC_BATT_SMB5_CHG_EVENT_OTG:
		data->detect_data[stage].is_otg = val;
		if (data->is_dcjack_disabled)
			reenable_dcjack_delay_ms = 100;
		break;
	case HTC_BATT_SMB5_CHG_EVENT_CC_ORIENTATION:
		data->detect_data[stage].cc_ori =val;
		break;
	default:
		break;
	}
	mutex_unlock(&chg->dcjack_lock);

	pr_err("notify charger event with (%d, %d, %d)\n", event, val, stage);
	if (reenable_dcjack_delay_ms > 0 && delayed_work_pending(&data->dcjack_reenable_work)) {
		cancel_delayed_work_sync(&data->dcjack_reenable_work);
		schedule_delayed_work(&data->dcjack_reenable_work, msecs_to_jiffies(reenable_dcjack_delay_ms));
	}
	return 0;
}
EXPORT_SYMBOL(htc_battery_smb5_notify);

enum power_supply_type htc_battery_smb5_map_apsd_type2ps_type(int apsd_type)
{
	return smblib_map_apsd_type2ps_type(apsd_type);
}
EXPORT_SYMBOL(htc_battery_smb5_map_apsd_type2ps_type);

int htc_battery_smb5_get_detect_data(struct htc_batt_smb5_detect_data *detect_data)
{
	const enum htc_battery_smb5_type type = HTC_BATTERY_SMB5_MAIN;
	struct htc_batt_smb5_data *data = htc_battery_smb5_get_dev_data(type);

	if (!detect_data || !data)
		return -EINVAL;

	memcpy(detect_data, &data->detect_data[DETECT_STAGE_DCJACK_DISABLED], sizeof(*detect_data));

	return 0;
}
EXPORT_SYMBOL(htc_battery_smb5_get_detect_data);

int htc_battery_smb5_get_dcjack_disabled(union power_supply_propval *pval)
{
	const enum htc_battery_smb5_type type = HTC_BATTERY_SMB5_MAIN;
	struct htc_batt_smb5_data *data = htc_battery_smb5_get_dev_data(type);

	if (!pval || !data)
		return -EINVAL;

	pval->intval = data->is_dcjack_disabled;

	return 0;
}
EXPORT_SYMBOL(htc_battery_smb5_get_dcjack_disabled);

int htc_battery_smb5_get_force_disable_dcjack(union power_supply_propval *pval)
{
	const enum htc_battery_smb5_type type = HTC_BATTERY_SMB5_MAIN;
	struct htc_batt_smb5_data *data = htc_battery_smb5_get_dev_data(type);

	if (!pval || !data)
		return -EINVAL;

	pr_err("is_dcjack_diable:%d, force_disable_dcjack:%d\n", data->is_dcjack_disabled, data->force_disable_dcjack);

	pval->intval = data->is_dcjack_disabled && data->force_disable_dcjack;
	data->force_disable_dcjack = pval->intval;

	 return 0;
}
EXPORT_SYMBOL(htc_battery_smb5_get_force_disable_dcjack);

int htc_battery_smb5_set_force_disable_dcjack(const union power_supply_propval *pval)
{
	const enum htc_battery_smb5_type type = HTC_BATTERY_SMB5_MAIN;
	struct htc_batt_smb5_data *data = htc_battery_smb5_get_dev_data(type);

	if (!pval || !data)
		return -EINVAL;

	data->force_disable_dcjack = pval->intval;
	htc_battery_smb5_disable_dcjack_acok(data, pval->intval);

	return 0;
}
EXPORT_SYMBOL(htc_battery_smb5_set_force_disable_dcjack);

/**
 *
 * static functions
 *
 */

static void htc_battery_smb5_clear_detect_data(struct htc_batt_smb5_data *data)
{
	int i;

	if (!data) {
		pr_err("%s: No dev data assigned\n", __func__);
		return;
	}

	for (i = 0; i < DETECT_STAGE_MAX; i++)
		memset(data->detect_data + i, 0, sizeof(data->detect_data[0]));
}

static bool htc_battery_smb5_is_dcjack_acok(struct htc_batt_smb5_data *data)
{
	if (!data) {
		pr_err("%s: No dev data assigned\n", __func__);
		return false;
	}

	// NOTE: It is also OK to use acok_dcjack since 12V would trigger both
	//       It is to use acok_dcjack_ovp due to no gpio change for different boards
	if (!data->acok_dcjack_ovp_gpiod) {
		pr_err("dcjack_ovp_acok is not supported! Do nothing!");
		return false;
	}

	return !gpiod_get_value(data->acok_dcjack_ovp_gpiod);
}

static bool htc_battery_smb5_is_usb_acok(struct htc_batt_smb5_data *data)
{
	if (!data) {
		pr_err("%s: No dev data assigned\n", __func__);
		return false;
	}

	if (!data->acok_usb_gpiod) {
		pr_err("usb_acok is not supported!!");
		return false;

	}

	return !gpiod_get_value(data->acok_usb_gpiod);
}

static enum usb_acok_status_t htc_battery_smb5_get_usb_acok_status(struct htc_batt_smb5_data *data)
{
	bool is_usb_acok;
	struct smb_charger *chg = htc_battery_smb5_get_main_chg_from_data(data);

	is_usb_acok = htc_battery_smb5_is_usb_acok(data);

	if (IS_ERR(chg))
		return is_usb_acok ? USB_ACOK_ONLY : USB_ACOK_NONE;

	if (is_usb_acok)
		return smblib_get_otg_vbus_gpio(chg) ? USB_ACOK_OTG : USB_ACOK_ONLY;

	return USB_ACOK_NONE;
}

static void htc_battery_smb5_dcjack_dump_debug(struct htc_batt_smb5_data *data)
{
	int rc;
	u8 stat = 0, legacy_status = 0, fsw = 0, vbus = 0;
	struct smb_charger *chg = htc_battery_smb5_get_main_chg_from_data(data);

	if (IS_ERR(chg))
		return;

	rc = htc_batt_smb5_read(data, TYPE_C_STATE_MACHINE_STATUS_REG, &stat);
	if (rc < 0) {
		pr_err("Couldn't read TYPE_C_STATE_MACHINE_STATUS_REG rc=%d\n",
				rc);
	}

	rc = htc_batt_smb5_read(data, LEGACY_CABLE_STATUS_REG, &legacy_status);
	if (rc < 0) {
		pr_err("Couldn't read TYPE_C_STATE_MACHINE_STATUS_REG rc=%d\n",
			rc);
	}

	rc = htc_batt_smb5_read(data, DCDC_FSW_SEL_REG, &fsw);
	if (rc < 0) {
		pr_err("Couldn't read TYPE_C_STATE_MACHINE_STATUS_REG rc=%d\n",
				rc);
	}

	rc = htc_batt_smb5_read(data, USBIN_BASE + INT_RT_STS_OFFSET, &vbus);
	if (rc < 0) {
		pr_err("Couldn't read USBIN_RT_STS rc=%d\n", rc);
	}

	pr_err("dcjack=%d(%d),usb=%d,vbus=%d,c_sm=0x%x,legacy=0x%x,fsw=%x,icl=%d(%s)\n",
		htc_battery_smb5_is_dcjack_acok(data),
		chg->is_dcjack_acok,
		htc_battery_smb5_get_usb_acok_status(data),
		!!(vbus  & USBIN_PLUGIN_RT_STS_BIT),
		stat,
		legacy_status,
		fsw,
		get_effective_result(chg->usb_icl_votable),
		get_effective_client(chg->usb_icl_votable)
	      );
	pr_err("dcjack_disable=%d,record[cm,ar,pd,pd_icl,pd_ok]=[(%d,%d,%d,%d,%d),(%d,%d,%d,%d,%d),(%d,%d,%d,%d,%d)]\n",
		data->is_dcjack_disabled,
		data->detect_data[0].typec_mode, data->detect_data[0].apsd_type, data->detect_data[0].pd_active, data->detect_data[0].pd_icl, data->detect_data[0].ok_to_pd,
		data->detect_data[1].typec_mode, data->detect_data[1].apsd_type, data->detect_data[1].pd_active, data->detect_data[1].pd_icl, data->detect_data[1].ok_to_pd,
		data->detect_data[2].typec_mode, data->detect_data[2].apsd_type, data->detect_data[2].pd_active, data->detect_data[2].pd_icl, data->detect_data[2].ok_to_pd
		);
}

// NOTE: basically, this function only handle which doese not depend on smb5-lib state
#define VALID_USBIN_ADAPTER_ALLOW_MASK (0xF)
static void htc_battery_smb5_adjust4dcjack_indep(struct htc_batt_smb5_data *data, bool enable)
{
	int ret;
	struct smb_charger *chg = htc_battery_smb5_get_main_chg_from_data(data);
	unsigned int adapter_allowance, fsw_freq, allow_override;

	if (IS_ERR(chg)) {
		pr_err("No charger device assigned\n");
		return;
	}

	pr_err("set dcjack adjust indep is %d\n", enable);
	adapter_allowance = USBIN_ADAPTER_ALLOW_5V_OR_9V_TO_12V;

	if (enable) {
		fsw_freq = chg->chg_freq.freq_12V;
		allow_override = ALLOW_CONTINUOUS;
	} else {
		fsw_freq = chg->chg_freq.freq_removal;
		allow_override = 0;
	}

	smblib_set_opt_switcher_freq(chg, fsw_freq);
	ret = htc_batt_smb5_masked_write(data, USBIN_ADAPTER_ALLOW_CFG_REG,
					VALID_USBIN_ADAPTER_ALLOW_MASK, adapter_allowance);

	ret = htc_batt_smb5_masked_write(data, USBIN_ADAPTER_ALLOW_OVERRIDE_REG,
					ALLOW_CONTINUOUS, allow_override);
}

#define DELAYED_CHECK_DCJACK_MS	(3000)
#define SDP_100_MA			100000
// TODO: 1. check voter status and parallel charging
static void htc_battery_smb5_adjust4dcjack(struct htc_batt_smb5_data *data, bool is_dcjack)
{
	int rc;
	u8 legacy_status = 0;
	struct smb_charger *chg = htc_battery_smb5_get_main_chg_from_data(data);

	if (IS_ERR(chg)) {
		pr_err("No charger device assigned\n");
		return;
	}

	rc = htc_batt_smb5_read(data, LEGACY_CABLE_STATUS_REG, &legacy_status);
	if (rc < 0) {
		pr_err("Couldn't read TYPE_C_STATE_MACHINE_STATUS_REG rc=%d\n",
			rc);
	}

	pr_err("set dcjack adjust icl is %d\n", is_dcjack);
	if (is_dcjack) {
		vote(chg->usb_icl_votable, PD_VOTER, false, 0);
		vote(chg->usb_icl_votable, DCP_VOTER, false, 0);
		vote(chg->usb_icl_votable, SW_QC3_VOTER, false, 0);
		vote(chg->usb_icl_votable, CTM_VOTER, false, 0);
		vote(chg->usb_icl_votable, SW_ICL_MAX_VOTER, false, 0);
	} else {
		// NOTE: If it is USB, ICL might be kept in high ICL
		//       Although HC mode is disabled, it is safer to set current to 100 first
		vote(chg->usb_icl_votable, USB_PSY_VOTER, true, SDP_100_MA);
		vote(chg->usb_icl_votable, USB_PSY_VOTER, false, 0);
	}
}

static int htc_battery_smb5_disable_dcjack_acok(
		struct htc_batt_smb5_data *data, bool disable)
{

	if (!data || !data->acok_dcjack_switch_gpiod) {
		pr_err("acok_dcjack_switch is not supported! Do nothing!");
		return -EINVAL;
	}

	gpiod_set_value(data->acok_dcjack_switch_gpiod, disable ? 1 : 0);
	data->is_dcjack_disabled = disable;
	pr_err("DCJACK switch disabled=%d\n", disable);

	return 0;
}

irqreturn_t htc_battery_smb5_acok_dcjack_ovp_irq_handler(int irq, void *irq_data)
{
	const enum htc_battery_smb5_type type = HTC_BATTERY_SMB5_MAIN;
	struct htc_batt_smb5_data *data = htc_battery_smb5_get_dev_data(type);
	struct smb_charger *chg = htc_battery_smb5_get_main_chg_from_data(data);
	bool is_dcjack_acok = htc_battery_smb5_is_dcjack_acok(data);
	enum usb_acok_status_t usb_acok_status = htc_battery_smb5_get_usb_acok_status(data);
	struct htc_battery_smb5_main_func *dev_func;
	bool reenable_dcjack = false, is_update_type = false;
	u8 stat = 0;
	int rc;
	int pd_icl;

	pr_err("IRQ: dcjack-plugin called\n");
	if (IS_ERR(chg)) {
		pr_err("No charger device assigned for type %d\n", type);
		return IRQ_HANDLED;
	}

	//NOTE: set this flags assert when the dcjack is deattached and usb works as source.
	chg->dcjack_removed_psy_changed_notifier_lock = (!is_dcjack_acok && usb_acok_status == USB_ACOK_ONLY);

	cancel_delayed_work_sync(&data->chk_usb_icl_work);
	cancel_delayed_work_sync(&data->dcjack_check_work);
	cancel_delayed_work_sync(&data->otg_plugin_chk_work);
	cancel_delayed_work_sync(&data->restart_dcjack_disable_work);
	data->usb_acok_accu_times = 0;

	rc = htc_batt_smb5_read(data, TYPE_C_CC_STATUS, &stat);
	if (rc < 0) {
		pr_err("Couldn't read TYPE_C_CC_STATUS_in %s rc=%d\n", __func__,rc);
	}

	mutex_lock(&chg->dcjack_lock);
	chg->is_dcjack_acok = is_dcjack_acok;
	data->cc_status = stat;
	// NOTE: if dcjack is disabled, someone will reenable it later
	if (!data->is_dcjack_disabled || usb_acok_status == USB_ACOK_NONE)
		reenable_dcjack = true;
	mutex_unlock(&chg->dcjack_lock);

	if (reenable_dcjack) {
		if (delayed_work_pending(&data->dcjack_reenable_work))
			cancel_delayed_work_sync(&data->dcjack_reenable_work);
		schedule_delayed_work(&data->dcjack_reenable_work, 0);
	}

	dev_func = (struct htc_battery_smb5_main_func *) htc_battery_smb5_get_func_from_data(data);
	if (IS_ERR_OR_NULL(dev_func)) {
		pr_err("No charger control function assigned\n");
		return IRQ_HANDLED;
	}

	if (is_dcjack_acok) {
		// NOTE: adjust4dcjack config is in reenable work

		if (usb_acok_status == USB_ACOK_NONE)
			schedule_delayed_work(&data->otg_plugin_chk_work, msecs_to_jiffies(OTG_POLLING_TIME_INIT_MS));
		is_update_type = true;
	} else {
		htc_battery_smb5_adjust4dcjack_indep(data, false);
		htc_battery_smb5_adjust4dcjack(data, false);

		schedule_delayed_work(&data->dcjack_check_work, msecs_to_jiffies(DELAYED_CHECK_DCJACK_MS));

		switch (usb_acok_status) {
		case USB_ACOK_NONE:
			// NOTE: typec_src_removal will update type so no update here
			if (dev_func->typec_src_removal)
				dev_func->typec_src_removal(chg);
			break;
		case USB_ACOK_OTG:
			is_update_type = true;
			// TODO: check if there are more should be disabled such as parallel charging
			break;
		case USB_ACOK_ONLY:
		default:
			vote(chg->usb_icl_votable, USB_PSY_VOTER, false, 0);
			pd_icl = data->detect_data[DETECT_STAGE_DCJACK_DISABLED].pd_icl;
			if (pd_icl <= 0 && data->detect_data[DETECT_STAGE_DCJACK].pd_icl > 0) {
				pd_icl = data->detect_data[DETECT_STAGE_DCJACK].pd_icl;
			}
			if (chg->pd_active) {
				is_update_type = true;
				if (pd_icl > 0) {
					pr_err("set stored pd current=%d\n", pd_icl);
					vote(chg->usb_icl_votable, PD_VOTER, true, pd_icl);
					vote(chg->usb_icl_votable, SW_ICL_MAX_VOTER, false, 0);
				}
			}
			break;
		}
	}

	if (is_update_type && dev_func->dev_update_usb_type)
		dev_func->dev_update_usb_type(chg);

	if (!IS_ERR(chg->usb_psy))
		power_supply_changed(chg->usb_psy);

	pr_err("IRQ: dcjack-plugin %s\n", is_dcjack_acok ? "attached" : "detached");

	return IRQ_HANDLED;
}

static void htc_battery_smb5_dcjack_check_worker(struct work_struct *work)
{
	struct htc_batt_smb5_data *data =
		container_of(work, struct htc_batt_smb5_data, dcjack_check_work.work);

	if (data->desc && data->desc->dev_type == HTC_BATTERY_SMB5_MAIN) {
		htc_battery_smb5_dcjack_dump_debug(data);
	}
}

#define DISABLE_DCJACK_DELAY_TIME_MS_LONG	(10000)
#define DISABLE_DCJACK_DELAY_TIME_MS_SHORT	(500)
static void htc_battery_smb5_disable_dcjack_in_period(struct htc_batt_smb5_data *data, unsigned int delay_time_ms)
{
	struct smb_charger *chg = htc_battery_smb5_get_main_chg_from_data(data);

	if (IS_ERR(chg)) {
		pr_err("No charger device assigned for dev data\n");
		return;
	}

	if (delayed_work_pending(&data->dcjack_reenable_work))
		cancel_delayed_work_sync(&data->dcjack_reenable_work);

	mutex_lock(&chg->dcjack_lock);
	pr_err("Disable DCJACK and reenable it after %dms", delay_time_ms);
	htc_battery_smb5_disable_dcjack_acok(data, true);
	mutex_unlock(&chg->dcjack_lock);

	schedule_delayed_work(&data->dcjack_reenable_work, msecs_to_jiffies(delay_time_ms));
}

#define USB_ACOK_FLOOD_CHECK_NS	(2000000000)
#define USB_ACOK_FLOOD_CHECK_TIMES	(30)
#define RESTART_DCJACK_DISABLE_DELAY_MS	(60000)
irqreturn_t htc_battery_smb5_acok_usb_irq_handler(int irq, void *irq_data)
{
	const enum htc_battery_smb5_type type = HTC_BATTERY_SMB5_MAIN;
	struct htc_batt_smb5_data *data = htc_battery_smb5_get_dev_data(type);
	struct smb_charger *chg = htc_battery_smb5_get_main_chg_from_data(data);
	enum usb_acok_status_t usb_acok_status = htc_battery_smb5_get_usb_acok_status(data);
	unsigned int delay_time_ms;
	bool reenable_otg_plugin_chk = false;
	struct timeval rtc_now;
	s64 usb_acok_now_time;
	static s64 usb_acok_last_time = 0;
	u8 usbin_latch_monitor, typec_latch_monitor;

	pr_err("IRQ: usb-acok-plugin called\n");
	if (IS_ERR(chg)) {
		pr_err("No charger device assigned for type %d\n", type);
		return IRQ_HANDLED;
	}

	htc_batt_smb5_read(data, USB_INT_LATCHED_STS_REG, &usbin_latch_monitor);
	htc_batt_smb5_read(data, TYPEC_INT_LATCHED_STS_REG, &typec_latch_monitor);

	if (usbin_latch_monitor){
		pr_err("USBIN latch:%02x\n", usbin_latch_monitor);
		htc_batt_smb5_masked_write(data, USB_INT_LATCHED_CLR_REG, 0xff, usbin_latch_monitor);
	}
	if (typec_latch_monitor){
		pr_err("Typec_latch:%02x\n", typec_latch_monitor);
		htc_batt_smb5_masked_write(data, TYPEC_INT_LATCHED_CLR_REG, 0xff, typec_latch_monitor);
	}

	if (usb_acok_status == USB_ACOK_NONE)
		htc_battery_smb5_clear_detect_data(data);

	//NOTE: unlock the notifier because the lock is only locked while the dcjack is deattached.
	chg->dcjack_removed_psy_changed_notifier_lock = false;
	chg->is_usb_acok = usb_acok_status == USB_ACOK_ONLY;

	cancel_delayed_work_sync(&data->otg_plugin_chk_work);

	if (usb_acok_status != USB_ACOK_OTG) {
		if (usb_acok_status == USB_ACOK_ONLY) {
			do_gettimeofday(&rtc_now);
			usb_acok_now_time = timeval_to_ns(&rtc_now);
			if (usb_acok_now_time - usb_acok_last_time <= USB_ACOK_FLOOD_CHECK_NS)
				++data->usb_acok_accu_times;
			else
				data->usb_acok_accu_times = 0;
			usb_acok_last_time = usb_acok_now_time;
		}
		if (chg->non_compliance_pd_hard_reset)
			data->usb_acok_accu_times = 0;

		if (data->usb_acok_accu_times < USB_ACOK_FLOOD_CHECK_TIMES) {
			/*Special case, if received hard reset, set disable delay time as LONG setting*/
			delay_time_ms = (usb_acok_status == USB_ACOK_ONLY || chg->pd_hard_reset) ?
					DISABLE_DCJACK_DELAY_TIME_MS_LONG : DISABLE_DCJACK_DELAY_TIME_MS_SHORT;
			htc_battery_smb5_disable_dcjack_in_period(data, delay_time_ms);
		} else if (!delayed_work_pending(&data->restart_dcjack_disable_work)) {
			pr_err("usb_acok flooding over %d times, stop dcjack disable for %dms\n",
					USB_ACOK_FLOOD_CHECK_TIMES, RESTART_DCJACK_DISABLE_DELAY_MS);
			schedule_delayed_work(&data->restart_dcjack_disable_work,
							msecs_to_jiffies(RESTART_DCJACK_DISABLE_DELAY_MS));
		}else{
			pr_err("accu_times over unknown activity\n");
		}
	} else if (delayed_work_pending(&data->dcjack_reenable_work)) {
		cancel_delayed_work_sync(&data->dcjack_reenable_work);
		schedule_delayed_work(&data->dcjack_reenable_work, msecs_to_jiffies(DISABLE_DCJACK_DELAY_TIME_MS_SHORT));
	} else{
		pr_err("OTG UNKNOWN ACTIVITY \n");
	}

	if (usb_acok_status == USB_ACOK_NONE && chg->is_dcjack_acok)
		reenable_otg_plugin_chk = true;

	if (reenable_otg_plugin_chk)
		schedule_delayed_work(&data->otg_plugin_chk_work, msecs_to_jiffies(OTG_POLLING_TIME_MS));

	pr_err("IRQ: usb-acok-plugin %s\n", usb_acok_status != USB_ACOK_NONE ? "attached" : "detached");

	return IRQ_HANDLED;
}

static void htc_battery_smb5_dcjack_reenable_worker(struct work_struct *work)
{
	struct htc_batt_smb5_data *data =
		container_of(work, struct htc_batt_smb5_data, dcjack_reenable_work.work);
	struct smb_charger *chg;

	if (data->desc && data->desc->dev_type == HTC_BATTERY_SMB5_MAIN) {
		chg = htc_battery_smb5_get_main_chg_from_data(data);
		if (IS_ERR(chg)) {
			pr_err("charger data is not found");
			return;
		}

		mutex_lock(&chg->dcjack_lock);
		htc_battery_smb5_disable_dcjack_acok(data, false);
		if (chg->is_dcjack_acok) {
			htc_battery_smb5_adjust4dcjack_indep(data, true);
			htc_battery_smb5_adjust4dcjack(data, true);
		}
		mutex_unlock(&chg->dcjack_lock);
	}
}

static void htc_battery_smb5_restart_dcjack_disable_worker(struct work_struct *work)
{
	struct htc_batt_smb5_data *data =
		container_of(work, struct htc_batt_smb5_data, restart_dcjack_disable_work.work);

	pr_err("Restart DCJACk disable\n");
	data->usb_acok_accu_times = 0;
}
#endif // CONFIG_HTC_BATT_DCJACK

#define ICL_RECHK_DELAY_MS	1000
#define RE_CHECK_MAX		3
static void chk_usb_icl_work(struct work_struct *work)
{
	struct htc_batt_smb5_data *data =
		container_of(work, struct htc_batt_smb5_data, chk_usb_icl_work.work);
	struct smb_charger *chg;
	int aicl_result;
	u8 reg = 0;
	static int chk_cnt = 0;

	if (data->desc && data->desc->dev_type == HTC_BATTERY_SMB5_MAIN) {
		chg = htc_battery_smb5_get_main_chg_from_data(data);
		if (IS_ERR(chg))
			return;

		if ((chg->typec_mode == POWER_SUPPLY_TYPEC_SOURCE_MEDIUM) ||
				(chg->typec_mode == POWER_SUPPLY_TYPEC_SOURCE_HIGH)) {
			if (chg->typec_legacy)
				smblib_run_aicl(chg, RERUN_AICL);
			pr_err("Rerun AICL for TypeC\n");
			return;
		}

		if (chg->real_charger_type != POWER_SUPPLY_TYPE_USB_DCP)
			return;

		smblib_read(chg, AICL_STATUS_REG, &reg);

		if ((reg & AICL_DONE_BIT) || (chk_cnt >= RE_CHECK_MAX)) {
			smblib_get_charge_param(chg, &chg->param.icl_stat, &aicl_result);
			if (aicl_result == 1500000) {
				// Start 5V/2A pre-check
				pr_err("AICL=%d, start pre-check\n", aicl_result/1000);
				htc_battery_5v2a_pre_chk();
			} else if (aicl_result < 1500000) {
				// Downgrade iusb to 1A
				vote(chg->usb_icl_votable, USB_PSY_VOTER, true, 1000000);
				pr_err("AICL=%d, downgrade 1A\n", aicl_result/1000);
			}
			chk_cnt = 0;
		} else {
			chk_cnt++;
			schedule_delayed_work(&data->chk_usb_icl_work,
					msecs_to_jiffies(ICL_RECHK_DELAY_MS));
		}
	}

	return;
}

static void htc_battery_smb5_icl_set_worker(struct work_struct *work)
{
	struct htc_batt_smb5_data *data =
		container_of(work, struct htc_batt_smb5_data, icl_set_work.work);
	struct smb_charger *chg;
	union power_supply_propval ret = {0, }, pval;

	if (data->desc && data->desc->dev_type == HTC_BATTERY_SMB5_MAIN) {
		chg = htc_battery_smb5_get_main_chg_from_data(data);
		if (IS_ERR(chg) || IS_ERR(chg->usb_psy))
			return;

		power_supply_get_property(chg->usb_psy, POWER_SUPPLY_PROP_PRESENT, &ret);
		if (!!ret.intval && data->force_icl_ua >= 0) {
			pval.intval = data->force_icl_ua;
			power_supply_set_property(chg->usb_psy, POWER_SUPPLY_PROP_SDP_CURRENT_MAX, &pval);
		}
	}
}


/*
 *
 * htc battery smb5 Init functions
 *
 */

int htc_battery_smb5_probe_done(enum htc_battery_smb5_type type)
{
	if (type == HTC_BATTERY_SMB5_MAIN) {
		htc_battery_probe_process(CHARGER_PROBE_DONE);
	}

	return 0;
}
EXPORT_SYMBOL(htc_battery_smb5_probe_done);

static int htc_battery_smb5_main_hw_init(struct smb_charger *chg)
{
	int ret;

	if (IS_ERR(chg)) {
		pr_err("No main charger assigned");
		return -EINVAL;
	}

	/* debug flag 6 4 set, disable hw soft/hard temp monitor */
	if (get_kernel_flag() & KERNEL_FLAG_KEEP_CHARG_ON) {
		ret = smblib_masked_write(chg, JEITA_EN_CFG_REG,
				JEITA_EN_FVCOMP_IN_CV_BIT | JEITA_EN_HARDLIMIT_BIT |
				JEITA_EN_HOT_SL_FCV_BIT | JEITA_EN_COLD_SL_FCV_BIT |
				JEITA_EN_HOT_SL_CCC_BIT | JEITA_EN_COLD_SL_CCC_BIT, 0);
	    if (ret < 0) {
			dev_err(chg->dev, "Couldn't disable hw soft/hard temp monitor = %d\n", ret);
			return ret;
	    }

		ret = smblib_masked_write(chg, MISC_THERMREG_SRC_CFG_REG,
				THERMREG_SKIN_ADC_SRC_EN_BIT | THERMREG_DIE_ADC_SRC_EN_BIT |
				THERMREG_CONNECTOR_ADC_SRC_EN_BIT, 0);
		if (ret < 0) {
			dev_err(chg->dev, "Couldn't disable SKIN/DIE thermreg = %d\n", ret);
			return ret;
		}
	} else {
		ret = smblib_masked_write(chg, JEITA_EN_CFG_REG,
				JEITA_EN_FVCOMP_IN_CV_BIT | JEITA_EN_HARDLIMIT_BIT |
				JEITA_EN_HOT_SL_FCV_BIT | JEITA_EN_COLD_SL_FCV_BIT |
				JEITA_EN_HOT_SL_CCC_BIT | JEITA_EN_COLD_SL_CCC_BIT,
				JEITA_EN_FVCOMP_IN_CV_BIT | JEITA_EN_HARDLIMIT_BIT | JEITA_EN_HOT_SL_FCV_BIT);
		if (ret < 0) {
			dev_err(chg->dev, "Couldn't disable hw soft/hard temp monitor = %d\n", ret);
			return ret;
		}

		ret = smblib_masked_write(chg, JEITA_FVCOMP_CFG_REG,
				JEITA_FVCOMP_MASK, JEITA_FVCOMP_MINUS_P500);
		if (ret < 0) {
			dev_err(chg->dev, "Couldn't disable JEITA FV COMP = %d\n", ret);
			return ret;
		}
	}

	/* debug flag 6 4 or 6 2000 set, disable safety timer */
	if ((get_kernel_flag() & KERNEL_FLAG_KEEP_CHARG_ON) ||
		(get_kernel_flag() & KERNEL_FLAG_DISABLE_SAFETY_TIMER)){
		ret = smblib_masked_write(chg, CHGR_SAFETY_TIMER_ENABLE_CFG_REG,
				FAST_CHARGE_SAFETY_TIMER_EN_BIT, 0);
		if (ret < 0) {
			dev_err(chg->dev, "Couldn't disable safety timer = %d\n", ret);
			return ret;
		}
	}

	/* Disable HVDCP 12V */
	ret = smblib_masked_write(chg, HVDCP_PULSE_COUNT_MAX_REG,
			HVDCP_PULSE_COUNT_MAX_QC2_MASK | HVDCP_PULSE_COUNT_MAX_QC3_MASK,
			(HVDCP_PULSE_COUNT_QC2P0_9V << HVDCP_PULSE_COUNT_QC2_SHIFT) | 10);
	if (ret < 0) {
		dev_err(chg->dev, "Couldn't configure hvdcp voltage limit ret=%d\n",
				ret);
		return ret;
	}

#if 0	// NOTE: No apply this since QCOM driver will reset it after cable out and DCJACK is 12V
	//       Reenable this with related modification if required
	ret = smblib_masked_write(chg, USBIN_ADAPTER_ALLOW_CFG_REG,
				USBIN_ADAPTER_ALLOW_MASK, USBIN_ADAPTER_ALLOW_5V_TO_9V);
	if (ret < 0) {
		dev_err(chg->dev, "Couldn't configure hvdcp voltage limit ret=%d\n",
			ret);
		return ret;
	}
#endif

	return 0;
}

#ifdef CONFIG_HTC_BATT_DCJACK
#define CC_STATUS_MASK1 0xFF
#define CC_STATUS_MASK2 0x00

static void dcjack_otg_plugin_chk_worker(struct work_struct *work)
{
	struct htc_batt_smb5_data *data = container_of(work, struct htc_batt_smb5_data, otg_plugin_chk_work.work);
	struct smb_charger *chg;
	struct htc_battery_smb5_main_func *dev_func;
	enum usb_acok_status_t usb_acok_status;
	int rc;
	int typec_mode;
	u8 stat;
	u8 cc_status;
	bool reschedule = false;

	if (!data->desc || data->desc->dev_type != HTC_BATTERY_SMB5_MAIN)
		return;

	chg = htc_battery_smb5_get_main_chg_from_data(data);
	if (IS_ERR(chg))
		return;

	usb_acok_status = htc_battery_smb5_get_usb_acok_status(data);

	dev_func = (struct htc_battery_smb5_main_func *) htc_battery_smb5_get_func_from_data(data);
	if (IS_ERR_OR_NULL(dev_func) || !dev_func->dev_get_prop_typec_mode)
		return;

	typec_mode = dev_func->dev_get_prop_typec_mode(chg);

	rc = smblib_read(chg, TYPE_C_CC_STATUS, &stat);
	if (rc < 0) {
		pr_err("Couldn't read TYPE_C_CC_STATUS_in %s rc=%d\n",
				__func__,rc);
		return;
	}

	cc_status = stat;

	if((cc_status == CC_STATUS_MASK1 || cc_status == CC_STATUS_MASK2 || data->cc_status == cc_status) && (chg->typec_mode == typec_mode)){
		if(data->cc_status != cc_status){
			mutex_lock(&chg->dcjack_lock);
			data->cc_status = cc_status;
			mutex_unlock(&chg->dcjack_lock);
		}
		reschedule = true;
	}
	else if((cc_status != CC_STATUS_MASK1 || cc_status != CC_STATUS_MASK2 ) && (chg->typec_mode == typec_mode)){
		if(data->cc_status != cc_status){
			pr_err("CC status change found (0x%x,0x%x)!! Check if OTG\n", cc_status, data->cc_status);
			htc_battery_smb5_disable_dcjack_in_period(data, DISABLE_DCJACK_DELAY_TIME_MS_LONG);
			mutex_lock(&chg->dcjack_lock);
			data->cc_status = cc_status;
			mutex_unlock(&chg->dcjack_lock);
		}
		//NOTE:no need to reschedule the work because the cc_stauts has changed.
		//reschedule = false;
	}else{
		data->cc_status = stat;
		chg->lpd_stage = LPD_STAGE_FLOAT_CANCEL;
		typec_mode = dev_func->dev_get_prop_typec_mode(chg);
		chg->typec_mode = typec_mode;

		if (!IS_ERR(chg->usb_psy))
			power_supply_changed(chg->usb_psy);
	}

	if (reschedule && chg->is_dcjack_acok && usb_acok_status == USB_ACOK_NONE)
		schedule_delayed_work(&data->otg_plugin_chk_work, msecs_to_jiffies(OTG_POLLING_TIME_MS));
}
#endif

static int htc_battery_smb5_main_data_init(struct htc_batt_smb5_data *data, struct smb_charger *chg)
{

	if (!data || IS_ERR(chg)) {
		pr_err("Invalid charger device\n");
		return -EINVAL;
	}

	INIT_DELAYED_WORK(&data->chk_usb_icl_work, chk_usb_icl_work);
	INIT_DELAYED_WORK(&data->icl_set_work, htc_battery_smb5_icl_set_worker);

#ifdef CONFIG_HTC_BATT_DCJACK
	INIT_DELAYED_WORK(&data->dcjack_check_work, htc_battery_smb5_dcjack_check_worker);
	INIT_DELAYED_WORK(&data->dcjack_reenable_work, htc_battery_smb5_dcjack_reenable_worker);
	INIT_DELAYED_WORK(&data->restart_dcjack_disable_work, htc_battery_smb5_restart_dcjack_disable_worker);
	INIT_DELAYED_WORK(&data->otg_plugin_chk_work, dcjack_otg_plugin_chk_worker);
#endif

	return 0;
}

#ifdef CONFIG_HTC_BATT_DCJACK
static int htc_battery_smb5_config_gpiod_and_irq(struct gpio_desc *gpiod)
{
	int irq, ret;

	irq = gpiod_to_irq(gpiod);
	if (irq < 0) {
		pr_err("Couldn't get irq for \n");
		return irq;
	}

	ret = irq_set_irq_type(irq, IRQ_TYPE_EDGE_BOTH);
	if (ret < 0) {
		pr_err("Couldn't set irq %d type\n", irq);
		return ret;
	}

	ret = irq_set_irq_wake(irq, 1);
	if (ret < 0) {
		pr_err("Couldn't set irq %d wake\n", irq);
		return ret;
	}

	return irq;
}

static int htc_battery_smb5_main_gpio_init(struct htc_batt_smb5_data *data, struct smb_charger *chg)
{
	int ret;
	int dcjack_irq, usb_irq;
	enum usb_acok_status_t usb_acok_status;

	data->acok_dcjack_gpiod = devm_gpiod_get_optional(chg->dev, "htc,acok-dcjack", GPIOD_IN);
	data->acok_dcjack_ovp_gpiod = devm_gpiod_get_optional(chg->dev, "htc,acok-dcjack-ovp", GPIOD_IN);
	data->acok_dcjack_switch_gpiod = devm_gpiod_get_optional(chg->dev, "htc,acok-dcjack-switch", GPIOD_OUT_LOW);
	data->acok_usb_gpiod = devm_gpiod_get_optional(chg->dev, "htc,acok-usb", GPIOD_IN);

	if (!data->acok_dcjack_ovp_gpiod) {
		pr_err("DCJACK is not supported!");
	} else {
		usb_irq = htc_battery_smb5_config_gpiod_and_irq(data->acok_usb_gpiod);
		if (usb_irq < 0) {
			pr_err("Couldn't get irq for USB\n");
			goto fail;
		}

		usb_acok_status = htc_battery_smb5_get_usb_acok_status(data);

		chg->is_usb_acok = usb_acok_status == USB_ACOK_ONLY;
		if (usb_acok_status != USB_ACOK_NONE) {
			htc_battery_smb5_disable_dcjack_in_period(data, DISABLE_DCJACK_DELAY_TIME_MS_LONG);
		}

		ret = devm_request_threaded_irq(chg->dev, usb_irq, NULL,
				htc_battery_smb5_acok_usb_irq_handler,
				IRQF_ONESHOT, "acok_usb_gpio", NULL);
		if (ret < 0) {
			pr_err("Couldn't request USB irq %d\n", usb_irq);
		}

		dcjack_irq = htc_battery_smb5_config_gpiod_and_irq(data->acok_dcjack_ovp_gpiod);
		if (dcjack_irq < 0) {
			pr_err("Couldn't get irq for DCJACK\n");
			goto fail;
		}

		chg->is_dcjack_acok = htc_battery_smb5_is_dcjack_acok(data);
		htc_battery_smb5_adjust4dcjack_indep(data, chg->is_dcjack_acok);
		if (chg->is_dcjack_acok) {
			chg->real_charger_type = POWER_SUPPLY_TYPE_DCJACK;
			if (usb_acok_status == USB_ACOK_NONE) {
				htc_battery_smb5_adjust4dcjack(data, true);
				schedule_delayed_work(&data->otg_plugin_chk_work, msecs_to_jiffies(OTG_POLLING_TIME_INIT_MS));
			}
		}

		chg->typec_mode_change_ignored = false;
		chg->dcjack_removed_psy_changed_notifier_lock = false;

		ret = devm_request_threaded_irq(chg->dev, dcjack_irq, NULL,
				htc_battery_smb5_acok_dcjack_ovp_irq_handler,
				IRQF_ONESHOT, "dcjack_gpio", NULL);
		if (ret < 0) {
			if (chg->is_dcjack_acok) {
				chg->is_dcjack_acok = false;
				chg->real_charger_type = POWER_SUPPLY_TYPE_UNKNOWN;
				htc_battery_smb5_adjust4dcjack_indep(data, false);
				htc_battery_smb5_adjust4dcjack(data, false);
			}
			pr_err("Couldn't request DCJACK irq %d\n", dcjack_irq);
			goto fail;
		}
	}
fail:

	return ret;
}
#endif

int htc_battery_smb5_init(struct htc_battery_smb5_desc *desc)
{
	struct htc_batt_smb5_data *data;
	struct smb_charger *chg;

	if (!desc || desc->dev_type >= HTC_BATTERY_SMB5_DEVICE_MAX || !desc->dev || !desc->ctrl)
		return -EINVAL;

	data = htc_battery_smb5_get_dev_data(desc->dev_type);
	if (data) {
		pr_err("Device type=%d is already assigned\n", desc->dev_type);
		return -EEXIST;
	}

	data = kzalloc(sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	pr_info("Device is assigned for type %d\n", desc->dev_type);

	data->desc = desc;
	data->force_icl_ua = INVALID_NEGATIVE_FORCE_ICL_UA;
#ifdef CONFIG_HTC_BATT_DCJACK
	data->is_dcjack_disabled = false;
	data->acok_dcjack_gpiod = NULL;
	data->acok_dcjack_ovp_gpiod = NULL;
	data->acok_dcjack_switch_gpiod = NULL;
	data->acok_usb_gpiod = NULL;
	data->cc_status = CC_STATUS_MASK1;
	data->force_disable_dcjack = false;
	htc_battery_smb5_clear_detect_data(data);
	data->usb_acok_accu_times = 0;
#endif

	switch (desc->dev_type) {
	case HTC_BATTERY_SMB5_MAIN:

		chg = container_of(desc->dev, struct smb_charger, dev);

		htc_battery_smb5_main_data_init(data, chg);
		htc_battery_smb5_main_hw_init(chg);
		break;
	case HTC_BATTERY_SMB5_SMB1355:
	default:
		break;
	}

	htc_battery_smb5_set_dev_data(desc->dev_type, data);

	switch (desc->dev_type) {
	case HTC_BATTERY_SMB5_MAIN:
#ifdef CONFIG_HTC_BATT_DCJACK
		htc_battery_smb5_main_gpio_init(data, chg);
#endif
		if (chg && chg->batt_psy && chg->dev)
			htc_battery_create_attrs(&chg->batt_psy->dev);
		break;
	case HTC_BATTERY_SMB5_SMB1355:
	default:
		break;
	}

	htc_batt_dev_ctrl_register(htc_batt_smb5_ctrl_dev[data->desc->dev_type]);

	return 0;
}
EXPORT_SYMBOL(htc_battery_smb5_init);

int htc_battery_smb5_post_init(struct smb_charger *chg)
{
	if (!chg)
		return -EINVAL;

	/* do nothing */

	return 0;
}
EXPORT_SYMBOL(htc_battery_smb5_post_init);

void htc_battery_smb5_deinit(struct htc_battery_smb5_desc *desc)
{
	struct htc_batt_smb5_data *data;

	if (!desc || desc->dev_type >= HTC_BATTERY_SMB5_DEVICE_MAX)
		return;

	data = htc_battery_smb5_get_dev_data(desc->dev_type);
	if (data && data->desc == desc) {
		htc_batt_dev_ctrl_unregister(htc_batt_smb5_ctrl_dev[desc->dev_type]);
		kfree(data);
		htc_battery_smb5_set_dev_data(desc->dev_type, NULL);
	}
}
EXPORT_SYMBOL(htc_battery_smb5_deinit);
