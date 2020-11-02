#ifndef __HTC_BATTERY_SMB5_H__
#define __HTC_BATTERY_SMB5_H__

#ifndef USB_CONN_TEMP_READY
#define USB_CONN_TEMP_READY 0
#endif

#include <linux/device.h>
#include <linux/power_supply.h>

#if USB_CONN_TEMP_READY
#include <linux/qpnp/qpnp-adc.h>
static const struct qpnp_vadc_map_pt usb_conn_temp_adc_map[] = {
        {-200, 1668},
        {-190, 1659},
        {-180, 1651},
        {-170, 1641},
        {-160, 1632},
        {-150, 1622},
        {-140, 1611},
        {-130, 1600},
        {-120, 1589},
        {-110, 1577},
        {-100, 1565},
        {-90, 1552},
        {-80, 1539},
        {-70, 1525},
        {-60, 1511},
        {-50, 1496},
        {-40, 1481},
        {-30, 1466},
        {-20, 1449},
        {-10, 1433},
        {0, 1416},
        {10, 1398},
        {20, 1381},
        {30, 1362},
        {40, 1344},
        {50, 1325},
        {60, 1305},
        {70, 1286},
        {80, 1266},
        {90, 1245},
        {100, 1225},
        {110, 1204},
        {120, 1183},
        {130, 1161},
        {140, 1140},
        {150, 1118},
        {160, 1096},
        {170, 1075},
        {180, 1053},
        {190, 1031},
        {200, 1009},
        {210, 987},
        {220, 965},
        {230, 943},
        {240, 922},
        {250, 900},
        {260, 879},
        {270, 857},
        {280, 836},
        {290, 815},
        {300, 795},
        {310, 774},
        {320, 754},
        {330, 734},
        {340, 715},
        {350, 695},
        {360, 677},
        {370, 658},
        {380, 640},
        {390, 622},
        {400, 604},
        {410, 587},
        {420, 570},
        {430, 554},
        {440, 537},
        {450, 522},
        {460, 506},
        {470, 491},
        {480, 477},
        {490, 462},
        {500, 449},
        {510, 435},
        {520, 422},
        {530, 409},
        {540, 397},
        {550, 385},
        {560, 373},
        {570, 361},
        {580, 350},
        {590, 339},
        {600, 329},
        {610, 319},
        {620, 309},
        {630, 300},
        {640, 290},
        {650, 281},
        {660, 273},
        {670, 264},
        {680, 256},
        {690, 248},
        {700, 241},
        {710, 233},
        {720, 226},
        {730, 219},
        {740, 212},
        {750, 206},
        {760, 200},
        {770, 193},
        {780, 188},
        {790, 182},
        {800, 176},
        {810, 171},
        {820, 166},
        {830, 161},
        {840, 156},
        {850, 151},
        {860, 147},
        {870, 142},
        {880, 138},
        {890, 134},
        {900, 130},
        {910, 126},
        {920, 123},
        {930, 119},
        {940, 116},
        {950, 112},
        {960, 109},
        {970, 106},
        {980, 103},
        {990, 100},
        {1000, 97},
        {1010, 94},
        {1020, 91},
        {1030, 89},
        {1040, 86},
        {1050, 84},
        {1060, 82},
        {1070, 79},
        {1080, 77},
        {1090, 75},
        {1100, 73},
        {1110, 71},
        {1120, 69},
        {1130, 67},
        {1140, 65},
        {1150, 64},
        {1160, 62},
        {1170, 60},
        {1180, 59},
        {1190, 57},
        {1200, 56},
        {1210, 54},
        {1220, 53},
        {1230, 51},
        {1240, 50},
        {1250, 49}
};
#endif

// cc_first_debounce sync from htc_battery.h
enum cc_first_debounce {
	CC_DET_NONE = 0,
	CC_DET_DEFAULT,
	CC_DET_MEDIUM,
	CC_DET_HIGH,
};

enum htc_battery_smb5_type {
	HTC_BATTERY_SMB5_MAIN = 0,
	HTC_BATTERY_SMB5_SMB1355,
	HTC_BATTERY_SMB5_DEVICE_MAX,
};

struct htc_battery_smb5_desc {
	enum htc_battery_smb5_type dev_type;
	struct device **dev;
	struct htc_battery_smb5_reg_ctrl *ctrl;
	void *dev_func;
};

struct htc_battery_smb5_reg_ctrl {
	int (*dev_read)(struct device **dev, u16 addr, u8 *val);
	int (*dev_write)(struct device **dev, u16 addr, u8 val);
	int (*dev_masked_write)(struct device **dev, u16 addr, u8 mask, u8 val);
	int (*dev_bulk_read)(struct device **dev, u16 addr, u8 *val, uint len);
};

#ifdef CONFIG_HTC_BATT_DCJACK
enum htc_batt_smb5_chg_event {
	HTC_BATT_SMB5_CHG_EVENT_TYPEC_MODE,
	HTC_BATT_SMB5_CHG_EVENT_APSD,
	HTC_BATT_SMB5_CHG_EVENT_OK_TO_PD,
	HTC_BATT_SMB5_CHG_EVENT_PD_ACTIVE,
	HTC_BATT_SMB5_CHG_EVENT_PD_ICL,
	HTC_BATT_SMB5_CHG_EVENT_OTG,
	HTC_BATT_SMB5_CHG_EVENT_CC_ORIENTATION,
	HTC_BATT_SMB5_CHG_EVENT_MAX,
};

struct htc_batt_smb5_detect_data {
	int typec_mode;
	int apsd_type;
	bool ok_to_pd;
	bool pd_active;
	bool is_otg;
	int pd_icl;
	int cc_ori;
};

#endif

struct smb_charger;
struct apsd_result;
struct htc_battery_smb5_main_func {
	void (*dev_rerun_apsd)(struct smb_charger *chg);
	void (*typec_src_removal)(struct smb_charger *chg);
	const struct apsd_result *(*dev_update_usb_type)(struct smb_charger *chg);
	int (*dev_get_prop_typec_mode)(struct smb_charger *chg);
};

int htc_battery_smb5_init(struct htc_battery_smb5_desc *desc);
void htc_battery_smb5_deinit(struct htc_battery_smb5_desc *desc);
int htc_battery_smb5_post_init(struct smb_charger *chg);
int htc_battery_smb5_probe_done(enum htc_battery_smb5_type type);

void htc_battery_smb5_info_update(enum power_supply_property prop, int intval);
int htc_battery_smb5_level_adjust(void);
bool htc_battery_smb5_get_discharging_reason(void);
void htc_battery_smb5_notify_unknown_charger(bool is_unknown);
void htc_battery_smb5_charger_check_after_apsd(
	const char * const name, const u8 bit, const enum power_supply_type pst);
void htc_battery_smb5_handle_usb_removal(bool vbus_rising);
int htc_battery_smb5_batt_temp_adjust(int temp);
#ifdef CONFIG_HTC_BATT_DCJACK
enum power_supply_type;
int htc_battery_smb5_adjust_fsw_freq(int fsw_khz);
u8 htc_battery_smb5_adjust_allowed_voltage(int allowed_voltage);
int htc_battery_smb5_notify(enum htc_batt_smb5_chg_event event, int val);
int htc_battery_smb5_check_typec_mode_change_ignored(void);
int htc_battery_smb5_get_detect_data(struct htc_batt_smb5_detect_data *detect_data);
enum power_supply_type htc_battery_smb5_map_apsd_type2ps_type(int apsd_type);
int htc_battery_smb5_get_dcjack_disabled(union power_supply_propval *pval);
int htc_battery_smb5_set_force_disable_dcjack(const union power_supply_propval *pval);
int htc_battery_smb5_get_force_disable_dcjack(union power_supply_propval *pval);
#endif
#endif /* __HTC_BATTERY_SMB5_H__ */
