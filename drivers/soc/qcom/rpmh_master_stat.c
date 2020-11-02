/* Copyright (c) 2017-2018, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#define pr_fmt(fmt) "%s: " fmt, KBUILD_MODNAME

#include <linux/errno.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/types.h>
#include <linux/mm.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/uaccess.h>
#include <linux/soc/qcom/smem.h>
#include <asm/arch_timer.h>
#include "rpmh_master_stat.h"

#define UNIT_DIST 0x14
#define REG_VALID 0x0
#define REG_DATA_LO 0x4
#define REG_DATA_HI 0x8

#define GET_ADDR(REG, UNIT_NO) (REG + (UNIT_DIST * UNIT_NO))

enum master_smem_id {
	MPSS = 605,
	ADSP,
	CDSP,
	SLPI,
	GPU,
	DISPLAY,
};

enum master_pid {
	PID_APSS = 0,
	PID_MPSS = 1,
	PID_ADSP = 2,
	PID_SLPI = 3,
	PID_CDSP = 5,
	PID_GPU = PID_APSS,
	PID_DISPLAY = PID_APSS,
};

enum profile_data {
	POWER_DOWN_START,
	POWER_UP_END,
	POWER_DOWN_END,
	POWER_UP_START,
	NUM_UNIT,
};

struct msm_rpmh_master_data {
	char *master_name;
	enum master_smem_id smem_id;
	enum master_pid pid;
};

static const struct msm_rpmh_master_data rpmh_masters[] = {
	{"MPSS", MPSS, PID_MPSS},
	{"ADSP", ADSP, PID_ADSP},
	{"CDSP", CDSP, PID_CDSP},
	{"SLPI", SLPI, PID_SLPI},
	{"GPU", GPU, PID_GPU},
	{"DISPLAY", DISPLAY, PID_DISPLAY},
};

struct msm_rpmh_master_stats {
	uint32_t version_id;
	uint32_t counts;
	uint64_t last_entered;
	uint64_t last_exited;
	uint64_t accumulated_duration;
};

struct msm_rpmh_profile_unit {
	uint64_t value;
	uint64_t valid;
};

struct rpmh_master_stats_prv_data {
	struct kobj_attribute ka;
	struct kobject *kobj;
};

#ifdef CONFIG_HTC_POWER_DEBUG
/* 0:MPSS, 1:ADSP, 2:CDSP, 3: SLPI, 4:GPU, 5:DISPLAY, 6: APSS*/
/* The index of each SS should match the index in rpmh_masters array */
#define MASTER_NUM 7
#define MPSS_ACCU_TIME_INDEX 	0
#define ADSP_ACCU_TIME_INDEX 	1
#define CDSP_ACCU_TIME_INDEX 	2
#define SLPI_ACCU_TIME_INDEX 	3
#define GPU_ACCU_TIME_INDEX 	4
#define DISPLAY_ACCU_TIME_INDEX 5
#define APSS_ACCU_TIME_INDEX 	6

static uint64_t accu_sleep_time_at_boot_up[MASTER_NUM] = {0};
#endif

static struct msm_rpmh_master_stats apss_master_stats;
static void __iomem *rpmh_unit_base;

static DEFINE_MUTEX(rpmh_stats_mutex);

static ssize_t msm_rpmh_master_stats_print_data(char *prvbuf, ssize_t length,
				struct msm_rpmh_master_stats *record,
				const char *name)
{
	uint64_t accumulated_duration = record->accumulated_duration;
	/*
	 * If a master is in sleep when reading the sleep stats from SMEM
	 * adjust the accumulated sleep duration to show actual sleep time.
	 * This ensures that the displayed stats are real when used for
	 * the purpose of computing battery utilization.
	 */
	if (record->last_entered > record->last_exited)
		accumulated_duration +=
				(arch_counter_get_cntvct()
				- record->last_entered);

	return snprintf(prvbuf, length, "%s\n\tVersion:0x%x\n"
			"\tSleep Count:0x%x\n"
			"\tSleep Last Entered At:0x%llx\n"
			"\tSleep Last Exited At:0x%llx\n"
			"\tSleep Accumulated Duration:0x%llx\n\n",
			name, record->version_id, record->counts,
			record->last_entered, record->last_exited,
			accumulated_duration);
}

static ssize_t msm_rpmh_master_stats_show(struct kobject *kobj,
				struct kobj_attribute *attr, char *buf)
{
	ssize_t length;
	int i = 0;
	size_t size = 0;
	struct msm_rpmh_master_stats *record = NULL;

	mutex_lock(&rpmh_stats_mutex);

	/* First Read APSS master stats */

	length = msm_rpmh_master_stats_print_data(buf, PAGE_SIZE,
						&apss_master_stats, "APSS");

	/* Read SMEM data written by other masters */

	for (i = 0; i < ARRAY_SIZE(rpmh_masters); i++) {
		record = (struct msm_rpmh_master_stats *) qcom_smem_get(
					rpmh_masters[i].pid,
					rpmh_masters[i].smem_id, &size);
		if (!IS_ERR_OR_NULL(record) && (PAGE_SIZE - length > 0))
			length += msm_rpmh_master_stats_print_data(
					buf + length, PAGE_SIZE - length,
					record,
					rpmh_masters[i].master_name);
	}

	mutex_unlock(&rpmh_stats_mutex);

	return length;
}

#ifdef CONFIG_HTC_POWER_DEBUG
#define SLEEP_STATS_BUF_V2  1024
#define SLEEP_STATS_PIECE_V2  64
static DEFINE_MUTEX(rpmh_stats_mutex_v2);
int htc_ss_sleep_info_print(char *prvbuf, struct msm_rpmh_master_stats *record,
				const char *name, int ss_index)
{
	char piece_v2[SLEEP_STATS_PIECE_V2];
	static ssize_t length = 0;
	uint64_t accumulated_duration;

	memset(piece_v2, 0, SLEEP_STATS_PIECE_V2);

	if (!IS_ERR_OR_NULL(record)) {
		/* STEP1 :  Handle subsystem's accumulated sleep time */
		if(accu_sleep_time_at_boot_up[ss_index] == 0) // boot up case
			accu_sleep_time_at_boot_up[ss_index] = record->accumulated_duration;

		accumulated_duration = record->accumulated_duration - accu_sleep_time_at_boot_up[ss_index];
		/* If a master is in sleep when reading the sleep stats from SMEM
		 * adjust the accumulated sleep duration to show actual sleep time.
		 * This ensures that the displayed stats are real when used for
		 * the purpose of computing battery utilization.
		 */
		if (record->last_entered > record->last_exited)
			accumulated_duration +=
					(arch_counter_get_cntvct()
					- record->last_entered);

		/* STEP2 : write subsyste,s sleep info into piece_v2*/
		length = snprintf(piece_v2, SLEEP_STATS_PIECE_V2, "(%s, %x, %llx, %llx, %llx)",
				name, record->counts, record->last_entered, record->last_exited,
				accumulated_duration);

		/* STEP3 : combine each ss sleep info */
		if(length >= 0) {
			if(strlen(prvbuf) + strlen(piece_v2) < SLEEP_STATS_BUF_V2)
				strncat(prvbuf, piece_v2, SLEEP_STATS_BUF_V2 - strlen(prvbuf) - 1);
		} else {
			printk("[K] ss_sleep_stats_v2: get subsystem info, but string buffer is not enough!");
		}
	} else {
		length = snprintf(piece_v2, SLEEP_STATS_PIECE_V2, "(%s, -1, -1, -1, -1)", name);
		if(length >= 0) {
			if(strlen(prvbuf) + strlen(piece_v2) < SLEEP_STATS_BUF_V2)
				strncat(prvbuf, piece_v2, SLEEP_STATS_BUF_V2 - strlen(prvbuf) - 1);
		} else {
			printk("[K] ss_sleep_stats_v2: can not get subsystem info, string buffer is not enough!");
		}
	}
	return 0;
}

int htc_show_ss_sleep_info_v2(void)
{
	char buf_v2[SLEEP_STATS_BUF_V2];
	struct msm_rpmh_master_stats *record_v2 = NULL;
	int i;
	size_t size = 0;

	memset(buf_v2, 0, SLEEP_STATS_BUF_V2);

	mutex_lock(&rpmh_stats_mutex_v2);

	/* handle APSS first*/
	record_v2 = &apss_master_stats;
	htc_ss_sleep_info_print(buf_v2, record_v2, "APSS", APSS_ACCU_TIME_INDEX);

	/* Handle others Subsystems */
	for (i = 0; i < ARRAY_SIZE(rpmh_masters); i++) {
		/* read sleep information from smem, which store by masters */
		record_v2 = (struct msm_rpmh_master_stats *) qcom_smem_get(
								rpmh_masters[i].pid,
								rpmh_masters[i].smem_id, &size);

		htc_ss_sleep_info_print(buf_v2, record_v2, rpmh_masters[i].master_name, i);

	}

	mutex_unlock(&rpmh_stats_mutex_v2);

	buf_v2[strlen(buf_v2) - 1] = '\0';
	printk("[K] ss_sleep_stats_v2: %s\n", buf_v2);

	return 0;
}
#endif

static inline void msm_rpmh_apss_master_stats_update(
				struct msm_rpmh_profile_unit *profile_unit)
{
	apss_master_stats.counts++;
	apss_master_stats.last_entered = profile_unit[POWER_DOWN_END].value;
	apss_master_stats.last_exited = profile_unit[POWER_UP_START].value;
	apss_master_stats.accumulated_duration +=
					(apss_master_stats.last_exited
					- apss_master_stats.last_entered);
}

void msm_rpmh_master_stats_update(void)
{
	int i;
	struct msm_rpmh_profile_unit profile_unit[NUM_UNIT];

	if (!rpmh_unit_base)
		return;

	for (i = POWER_DOWN_END; i < NUM_UNIT; i++) {
		profile_unit[i].valid = readl_relaxed(rpmh_unit_base +
						GET_ADDR(REG_VALID, i));

		/*
		 * Do not update APSS stats if valid bit is not set.
		 * It means APSS did not execute cx-off sequence.
		 * This can be due to fall through at some point.
		 */

		if (!(profile_unit[i].valid & BIT(REG_VALID)))
			return;

		profile_unit[i].value = readl_relaxed(rpmh_unit_base +
						GET_ADDR(REG_DATA_LO, i));
		profile_unit[i].value |= ((uint64_t)
					readl_relaxed(rpmh_unit_base +
					GET_ADDR(REG_DATA_HI, i)) << 32);
	}
	msm_rpmh_apss_master_stats_update(profile_unit);
}
EXPORT_SYMBOL(msm_rpmh_master_stats_update);

static int msm_rpmh_master_stats_probe(struct platform_device *pdev)
{
	struct rpmh_master_stats_prv_data *prvdata = NULL;
	struct kobject *rpmh_master_stats_kobj = NULL;
	int ret = -ENOMEM;

	if (!pdev)
		return -EINVAL;

	prvdata = devm_kzalloc(&pdev->dev, sizeof(*prvdata), GFP_KERNEL);
	if (!prvdata)
		return ret;

	rpmh_master_stats_kobj = kobject_create_and_add(
					"rpmh_stats",
					power_kobj);
	if (!rpmh_master_stats_kobj)
		return ret;

	prvdata->kobj = rpmh_master_stats_kobj;

	sysfs_attr_init(&prvdata->ka.attr);
	prvdata->ka.attr.mode = 0444;
	prvdata->ka.attr.name = "master_stats";
	prvdata->ka.show = msm_rpmh_master_stats_show;
	prvdata->ka.store = NULL;

	ret = sysfs_create_file(prvdata->kobj, &prvdata->ka.attr);
	if (ret) {
		pr_err("sysfs_create_file failed\n");
		goto fail_sysfs;
	}

	rpmh_unit_base = of_iomap(pdev->dev.of_node, 0);
	if (!rpmh_unit_base) {
		pr_err("Failed to get rpmh_unit_base\n");
		ret = -ENOMEM;
		goto fail_iomap;
	}

	apss_master_stats.version_id = 0x1;
	platform_set_drvdata(pdev, prvdata);
	return ret;

fail_iomap:
	sysfs_remove_file(prvdata->kobj, &prvdata->ka.attr);
fail_sysfs:
	kobject_put(prvdata->kobj);
	return ret;
}

static int msm_rpmh_master_stats_remove(struct platform_device *pdev)
{
	struct rpmh_master_stats_prv_data *prvdata;

	if (!pdev)
		return -EINVAL;

	prvdata = (struct rpmh_master_stats_prv_data *)
				platform_get_drvdata(pdev);

	sysfs_remove_file(prvdata->kobj, &prvdata->ka.attr);
	kobject_put(prvdata->kobj);
	platform_set_drvdata(pdev, NULL);
	iounmap(rpmh_unit_base);
	rpmh_unit_base = NULL;

	return 0;
}

static const struct of_device_id rpmh_master_table[] = {
	{.compatible = "qcom,rpmh-master-stats-v1"},
	{},
};

static struct platform_driver msm_rpmh_master_stats_driver = {
	.probe	= msm_rpmh_master_stats_probe,
	.remove = msm_rpmh_master_stats_remove,
	.driver = {
		.name = "msm_rpmh_master_stats",
		.of_match_table = rpmh_master_table,
	},
};

module_platform_driver(msm_rpmh_master_stats_driver);
MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("MSM RPMH Master Statistics driver");
MODULE_ALIAS("platform:msm_rpmh_master_stat_log");
