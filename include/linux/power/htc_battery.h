#ifndef __HTC_BATTERY_H__
#define __HTC_BATTERY_H__

// NOTE: Those functions are not defined in htc_battery.
//       Define value to 1 if function should be applied.
#ifndef APPLY_HTC_POWER_SUPPLY
#define APPLY_HTC_POWER_SUPPLY	0
#endif
#ifndef APPLY_HTC_USB_PD
#define APPLY_HTC_USB_PD	0
#endif
#ifndef APPLY_HTC_BOOTMODE
#define APPLY_HTC_BOOTMODE	0
#endif
#ifndef APPLY_HTC_KERNEL_FLAG
#define APPLY_HTC_KERNEL_FLAG	0
#endif
#ifndef APPLY_HTC_DISPLAY_FB
#define APPLY_HTC_DISPLAY_FB	0
#endif

#if APPLY_HTC_DISPLAY_FB && defined(CONFIG_FB)
#define ENABLE_HTC_FB	1
#else
#define ENABLE_HTC_FB	0
#endif

#include <linux/rtc.h>
#include <linux/alarmtimer.h>
#include <linux/pm_wakeup.h>
#include <linux/module.h>
#include <linux/power_supply.h>
#if APPLY_HTC_KERNEL_FLAG
#include <linux/htc_flags.h>
#endif

#include <linux/of_gpio.h>
#include <linux/gpio.h>

#define BATT_LOG(x...) do { \
	printk(KERN_INFO "[BATT] " x); \
} while (0)

#define BATT_DUMP(x...) do { \
	printk(KERN_ERR "[BATT][DUMP] " x); \
} while (0)

#define BATT_ERR(x...) do { \
	struct timespec ts; \
	struct rtc_time tm; \
	getnstimeofday(&ts); \
	rtc_time_to_tm(ts.tv_sec, &tm); \
	printk(KERN_ERR "[BATT] err:" x); \
	printk(" at %lld (%d-%02d-%02d %02d:%02d:%02d.%09lu UTC)\n", \
	ktime_to_ns(ktime_get()), tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday, \
	tm.tm_hour, tm.tm_min, tm.tm_sec, ts.tv_nsec); \
} while (0)

#define BATT_EMBEDDED(x...) do { \
	struct timespec ts; \
	struct rtc_time tm; \
	getnstimeofday(&ts); \
	rtc_time_to_tm(ts.tv_sec, &tm); \
	printk(KERN_ERR "[BATT] " x); \
	printk(" at %lld (%d-%02d-%02d %02d:%02d:%02d.%09lu UTC)\n", \
	ktime_to_ns(ktime_get()), tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday, \
	tm.tm_hour, tm.tm_min, tm.tm_sec, ts.tv_nsec); \
} while (0)

#define POWER_MONITOR_BATT_CAPACITY	77
#define POWER_MONITOR_BATT_TEMP	330
#define NO_PROTECTION_BATT_TEMP_UP_BOUND	580
#define BATTERY_CRITICAL_TEMP 600

/* stored consistent parameters */
#define STORE_MAGIC_NUM          0xDDAACC00
#define STORE_MAGIC_OFFSET       3104    /*0xC20*/
#define STORE_SOC_OFFSET         3108    /*0xC24*/
#define STORE_CURRTIME_OFFSET    3120    /*0xC30*/
#define STORE_TEMP_OFFSET		3140    /*0xC44*/

/* for batt cycle info */
#define HTC_BATT_TOTAL_LEVELRAW		3144
#define HTC_BATT_OVERHEAT_MSEC		3148
#define HTC_BATT_FIRST_USE_TIME		3152
#define HTC_BATT_CYCLE_CHECKSUM		3156

/* for htc_extension */
#define HTC_EXT_UNKNOWN_USB_CHARGER		(1<<0)
#define HTC_EXT_CHG_UNDER_RATING		(1<<1)
#define HTC_EXT_CHG_SAFTY_TIMEOUT		(1<<2)
#define HTC_EXT_CHG_FULL_EOC_STOP		(1<<3)
#define HTC_EXT_BAD_CABLE_USED			(1<<4)
#define HTC_EXT_QUICK_CHARGER_USED		(1<<5)
#define HTC_EXT_USB_OVERHEAT			(1<<6)
#define HTC_EXT_CHARGER_EXIST			(1<<7)
#define HTC_EXT_AI_CHARGING			(1<<8)

#define BATT_TIMER_UPDATE_TIME				(60)
#define BATT_SUSPEND_CHECK_TIME				(3600)
#define BATT_SUSPEND_HIGHFREQ_CHECK_TIME	(300)
#define BATT_TIMER_CHECK_TIME				(360)
#define CHECH_TIME_TOLERANCE_MS	(1000)

/* for suspend high frequency (5min) */
#define SUSPEND_HIGHFREQ_CHECK_BIT_TALK		(1)
#define SUSPEND_HIGHFREQ_CHECK_BIT_SEARCH	(1<<1)
#define SUSPEND_HIGHFREQ_CHECK_BIT_MUSIC	(1<<3)

#define UNKNOWN_BATTERY_ID 255

struct battery_info_reply {
	u32 batt_vol;
	u32 batt_id;
	s32 batt_temp;
	s32 batt_current;
	u32 charging_source;
	u32 level;
	u32 level_raw;
	u32 full_level;
	u32 status;
	u32 chg_src;
	u32 chg_en;
	u32 chg_batt_en;
	u32 full_level_dis_batt_chg;
	u32 overload;
	u32 over_vchg;
	u32 health;
	bool is_full;
};

struct battery_info_previous {
	s32 batt_temp;
	u32 charging_source;
	u32 level;
	u32 level_raw;
};

struct htc_battery_store {
	u32 batt_stored_magic_num;
	u32 batt_stored_soc;
	u32 batt_stored_temperature;
	unsigned long batt_stored_update_time;
	u32 consistent_flag;
};

struct htc_thermal_stage {
	int nextTemp;
	int recoverTemp;
};

struct htc_battery_info {
	struct battery_info_reply rep;
	struct battery_info_previous prev;
	struct htc_battery_store store;
	struct power_supply		*batt_psy;
	struct power_supply		*bms_psy;
	struct power_supply		*usb_psy;
	struct power_supply		*main_psy;
	int critical_low_voltage_mv;
	int smooth_chg_full_delay_min;
	int decreased_batt_level_check;
	int batt_full_voltage_mv;
	int batt_full_current_ma;
	int overload_curr_thr_ma;
	int batt_fcc_ma;
	struct wakeup_source *charger_exist_lock;
	struct wakeup_source *check_overheat_lock;
	struct delayed_work chg_full_check_work;
	struct delayed_work is_usb_overheat_work;
	struct delayed_work cable_impedance_work;
	struct delayed_work htc_usb_overheat_work;
	struct delayed_work iusb_5v_2a_ability_check_work;
	struct delayed_work quick_charger_check_work;
	struct delayed_work battery_age_detect_work;
	int state;
	int vbus;
	int k_debug_flag;
	int current_limit_reason;
	int chgr_stop_reason;
#if ENABLE_HTC_FB
	struct notifier_block fb_notif;
	struct workqueue_struct *batt_fb_wq;
	struct delayed_work work_fb;
#endif
	unsigned int htc_extension;	/* for htc in-house sw */
	int qc2_current_ua;
	int qc3_current_ua;
	int fastchg_current_ma;
	int health_level_max;
	int batt_health_good;
	int allow_power_off_voltage;
	int low_batt_check_soc_raw_threshold;
	int r_default_for_5v2a_pre_chk;
};

struct htc_battery_timer {
	unsigned long batt_system_jiffies;
	unsigned long total_time_ms;	/* since last do batt_work */
	unsigned long batt_suspend_ms;
	struct alarm batt_check_wakeup_alarm;
	struct work_struct batt_work;
	struct timer_list batt_timer;
	struct workqueue_struct *batt_wq;
	struct wakeup_source *battery_lock;
	unsigned int time_out;
};

#ifdef CONFIG_HTC_BATT_PCN0021
struct htc_charging_statistics {
        unsigned long begin_chg_time;
        unsigned long end_chg_time;
        int begin_chg_batt_level;
        int end_chg_batt_level;
};

struct htc_statistics_category {
        unsigned long chg_time_sum;
        unsigned long dischg_time_sum;
        int sample_count;
};
#endif //CONFIG_HTC_BATT_PCN0021

enum charger_control_flag {
	STOP_CHARGER = 0,
	ENABLE_CHARGER,
	ENABLE_LIMIT_CHARGER,
	DISABLE_LIMIT_CHARGER,
	DISABLE_PWRSRC,
	ENABLE_PWRSRC,
#ifdef CONFIG_FPC_HTC_DISABLE_CHARGING //HTC_BATT_WA_PCN0004
	DISABLE_PWRSRC_FINGERPRINT,
	ENABLE_PWRSRC_FINGERPRINT,
#endif //CONFIG_FPC_HTC_DISABLE_CHARGING //HTC_BATT_WA_PCN0004
	END_CHARGER
};

/*
 * MFG ftm mode charger control
 *
 * FTM_ENABLE_CHARGER: default, ftm control disabled
 * FTM_STOP_CHARGER: ftm control to disable charging
 * FTM_FAST_CHARGE: ftm control to force fast charge
 * FTM_SLOW_CHARGE: ftm control to force slow charge
 * FTM_END_CHARGER: do nothing, value for flag out of bound check
 */
enum ftm_charger_control_flag {
	FTM_ENABLE_CHARGER = 0,
	FTM_STOP_CHARGER,
	FTM_FAST_CHARGE,
	FTM_SLOW_CHARGE,
	FTM_END_CHARGER
};

enum user_set_input_current {
	SLOW_CHARGE_CURR = 500000,
	FAST_CHARGE_CURR = 900000,
	WALL_CHARGE_CURR = 1500000,
	HVDCP_CHARGE_CURR = 1600000,
	HVDCP_3_CHARGE_CURR = 2200000,
	DCP_5V2A_CHARGE_CURR = 2000000,
#ifdef CONFIG_HTC_BATT_DCJACK
	DCJACK_CHARGE_CURR = 2500000,
#endif
	OTHER_CHARG_CURR = 1500000,
};

enum htc_batt_probe {
	CHARGER_PROBE_DONE,
	GAUGE_PROBE_DONE,
	HTC_BATT_PROBE_DONE,
	BATT_PROBE_MAX,
};

enum htc_charger_request {
	CHARGER_ABILITY_DETECT_DONE,
	CHARGER_5V_2A_DETECT_DONE,
};

enum htc_chr_type_data {
	DATA_NO_CHARGING = 0,
	DATA_UNKNOWN_CHARGER,
	DATA_UNKNOWN_TYPE,
	DATA_USB,
	DATA_USB_CDP,
	DATA_AC,
	DATA_QC2,
	DATA_QC3,
	DATA_PD_5V,
	DATA_PD_9V,
	DATA_PD_12V,
	DATA_TYPE_C,
#ifdef CONFIG_HTC_BATT_DCJACK
	DATA_DCJACK,
#endif
};

enum htc_batt_cc_first_debounce {
	HTC_BATT_CC_DET_NONE = 0,
	HTC_BATT_CC_DET_DEFAULT,
	HTC_BATT_CC_DET_MEDIUM,
	HTC_BATT_CC_DET_HIGH,
};

/*
 * Platform related function to let htc_battery.c independent of chipset code.
 * This is used instead of extending power_supply property.
 * Also, this can be more customized for htc usage.
 * Please also check htc_batt_[dev_name].h such as htc_batt_smb5.h
 */
enum htc_batt_dev_type {
	HTC_BATT_CHARGER_MASTER = 0,
	HTC_BATT_CHARGER_SLAVE,
	HTC_BATT_DEVICE_MAX,
};

enum htc_batt_dev_ctrl_prop {
	HTC_BATT_DEV_CTRL_ENABLE_AICL = 0,
	HTC_BATT_DEV_CTRL_IS_AICL_DONE,
	HTC_BATT_DEV_CTRL_GET_AICL_CONFIG,
	HTC_BATT_DEV_CTRL_RERUN_AICL,
	HTC_BATT_DEV_CTRL_IS_CHARGING_TERMINATE,
	HTC_BATT_DEV_CTRL_IS_SAFTYTIMER_EXPIRED,
	HTC_BATT_DEV_CTRL_DUMP_REGISTER_TO_BUF,
	HTC_BATT_DEV_CTRL_DUMP_REGISTER,
	HTC_BATT_DEV_CTRL_SET_INPUT_CURRENT,
	HTC_BATT_DEV_CTRL_OVERLOAD_RECOVERY_CHECK,
	HTC_BATT_DEV_CTRL_GET_CHARGING_LIMIT_MA,
	HTC_BATT_DEV_CTRL_CHARGE_CHECK,
};

struct htc_batt_dump_reg2buf_data {
	char *buf;
	int len;
};

struct htc_batt_set_icl_data {
	bool force;
	int val;
};

typedef union {
	int intval;
	const char *strval;
	int64_t int64val;
	void *data;
} htc_batt_dev_ctrl_propval;

struct htc_batt_dev_ctrl_desc {
	enum htc_batt_dev_type dev_type;
	enum htc_batt_dev_ctrl_prop *properties;
	size_t num_properties;
	int (*handle_prop)(enum htc_batt_dev_ctrl_prop prop,
			   htc_batt_dev_ctrl_propval *val);
};

int htc_batt_dev_ctrl_register(struct htc_batt_dev_ctrl_desc *desc);
void htc_batt_dev_ctrl_unregister(struct htc_batt_dev_ctrl_desc *desc);

int htc_battery_create_attrs(struct device *dev);
void htc_battery_info_update(enum power_supply_property prop, int intval);
void htc_battery_probe_process(enum htc_batt_probe probe_type);
int htc_battery_level_adjust(void);
int htc_battery_adjust_batt_temp(int batt_temp);
#ifdef CONFIG_FPC_HTC_DISABLE_CHARGING //HTC_BATT_WA_PCN0004
int htc_battery_charger_switch_internal(int enable);
#endif
void htc_battery_5v2a_pre_chk(void);
void htc_battery_notify_unknown_charger(bool is_unknown);
bool htc_battery_get_discharging_reason(void);
void htc_ftm_disable_charger(bool disable);
#endif /* __HTC_BATTERY_H__ */

