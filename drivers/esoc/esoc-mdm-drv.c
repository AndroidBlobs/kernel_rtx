/* Copyright (c) 2013-2015, 2017-2018, The Linux Foundation. All rights reserved.
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

#include <linux/delay.h>
#include <linux/workqueue.h>
#include <linux/reboot.h>
#include <linux/of.h>
#include <linux/esoc_client.h>
#include "esoc.h"
#include "esoc-mdm.h"
#include "mdm-dbg.h"
//Modem_BSP++
#include <linux/htc_flags.h>
//Modem_BSP--

/* Default number of powerup trial requests per session */
//Modem_BSP++
#define ESOC_DEF_PON_REQ	10
//Modem_BSP--
static unsigned int n_pon_tries = ESOC_DEF_PON_REQ;
module_param(n_pon_tries, uint, 0644);
MODULE_PARM_DESC(n_pon_tries,
"Number of power-on retrials allowed upon boot failure");

enum esoc_boot_fail_action {
	BOOT_FAIL_ACTION_RETRY,
	BOOT_FAIL_ACTION_COLD_RESET,
	BOOT_FAIL_ACTION_SHUTDOWN,
	BOOT_FAIL_ACTION_PANIC,
	BOOT_FAIL_ACTION_NOP,
};

static unsigned int boot_fail_action = BOOT_FAIL_ACTION_PANIC;

//Modem_BSP++
int is_first_boot=1;
int boot_fail_times=0;
#define DEVICE_TREE_RADIO_PATH "/chosen/radio"

static void mdm_boot_fail_action(void)
{
	struct device_node *dnp;
	uint32_t val,val_2;

	dnp = of_find_node_by_path(DEVICE_TREE_RADIO_PATH);

	if(dnp) {
		of_property_read_u32(dnp, "htc_smem_radio_dbg_flag_ext1", &val);
		of_property_read_u32(dnp, "htc_smem_radio_dbg_flag", &val_2);

		if(val_2 & BIT(3)) {
			//8 0x08

			if(val & BIT(6)) {
				//A 0x40
				boot_fail_action = BOOT_FAIL_ACTION_NOP;
				esoc_mdm_log("mdm_boot_fail_action: BOOT_FAIL_ACTION_NOP\n");
			}
			else if(val & BIT(13)) {
				//A 0x2000
				boot_fail_action = BOOT_FAIL_ACTION_RETRY;
				esoc_mdm_log("mdm_boot_fail_action: BOOT_FAIL_ACTION_RETRY\n");
			}
			else if(val & BIT(14)) {
				//A 0x4000
				boot_fail_action = BOOT_FAIL_ACTION_COLD_RESET;
				esoc_mdm_log("mdm_boot_fail_action: BOOT_FAIL_ACTION_COLD_RESET\n");
			}
			else if(val & BIT(15)) {
				//A 0x8000
				boot_fail_action = BOOT_FAIL_ACTION_PANIC;
				esoc_mdm_log("mdm_boot_fail_action: BOOT_FAIL_ACTION_PANIC\n");
			}
			else{
				if(is_first_boot==1){
					boot_fail_action = BOOT_FAIL_ACTION_COLD_RESET;
					esoc_mdm_log("is_first_boot=1, mdm_boot_fail_action: BOOT_FAIL_ACTION_COLD_RESET\n");
					is_first_boot=0;
				}
				else if(is_first_boot==0){
					boot_fail_action = BOOT_FAIL_ACTION_PANIC;
					esoc_mdm_log("is_first_boot=0, mdm_boot_fail_action: BOOT_FAIL_ACTION_PANIC\n");
				}
			}
		}
		else{
			boot_fail_times++;
			if(boot_fail_times < ESOC_DEF_PON_REQ){
				boot_fail_action = BOOT_FAIL_ACTION_COLD_RESET;
				esoc_mdm_log("boot_fail_times=%d, mdm_boot_fail_action: BOOT_FAIL_ACTION_COLD_RESET\n",boot_fail_times);
			}
			else{
				boot_fail_action = BOOT_FAIL_ACTION_PANIC;
				esoc_mdm_log("boot_fail_times=%d, mdm_boot_fail_action: BOOT_FAIL_ACTION_PANIC\n",boot_fail_times);
			}
		}
	} else
		pr_err("[esoc]%s: cannot find path %s.\n", __func__, DEVICE_TREE_RADIO_PATH);
}
//Modem_BSP--

module_param(boot_fail_action, uint, 0644);
MODULE_PARM_DESC(boot_fail_action,
"Actions: 0:Retry PON; 1:Cold reset; 2:Power-down; 3:APQ Panic; 4:No action");

enum esoc_pon_state {
	PON_INIT,
	PON_SUCCESS,
	PON_RETRY,
	PON_FAIL
};

enum {
	 PWR_OFF = 0x1,
	 PWR_ON,
	 BOOT,
	 RUN,
	 CRASH,
	 IN_DEBUG,
	 SHUTDOWN,
	 RESET,
	 PEER_CRASH,
};

struct mdm_drv {
	unsigned int mode;
	struct esoc_eng cmd_eng;
	struct completion pon_done;
	struct completion req_eng_wait;
	struct esoc_clink *esoc_clink;
	enum esoc_pon_state pon_state;
	struct workqueue_struct *mdm_queue;
	struct work_struct ssr_work;
	struct notifier_block esoc_restart;
	struct mutex poff_lock;
};
#define to_mdm_drv(d)	container_of(d, struct mdm_drv, cmd_eng)

static void esoc_client_link_power_off(struct esoc_clink *esoc_clink,
							bool mdm_crashed);

static int esoc_msm_restart_handler(struct notifier_block *nb,
		unsigned long action, void *data)
{
	struct mdm_drv *mdm_drv = container_of(nb, struct mdm_drv,
					esoc_restart);
	struct esoc_clink *esoc_clink = mdm_drv->esoc_clink;
	const struct esoc_clink_ops *const clink_ops = esoc_clink->clink_ops;

	if (action == SYS_RESTART) {
		if (mdm_dbg_stall_notify(ESOC_PRIMARY_REBOOT))
			return NOTIFY_OK;
		mutex_lock(&mdm_drv->poff_lock);
		if (mdm_drv->mode == PWR_OFF) {
			esoc_mdm_log(
			"Reboot notifier: mdm already powered-off\n");
			mutex_unlock(&mdm_drv->poff_lock);
			return NOTIFY_OK;
		}
		esoc_client_link_power_off(esoc_clink, false);
		esoc_mdm_log(
			"Reboot notifier: Notifying esoc of cold reboot\n");
		dev_dbg(&esoc_clink->dev, "Notifying esoc of cold reboot\n");
		clink_ops->notify(ESOC_PRIMARY_REBOOT, esoc_clink);
		mdm_drv->mode = PWR_OFF;
		mutex_unlock(&mdm_drv->poff_lock);
	}
	return NOTIFY_OK;
}
static void mdm_handle_clink_evt(enum esoc_evt evt,
					struct esoc_eng *eng)
{
	struct mdm_drv *mdm_drv = to_mdm_drv(eng);
	bool unexpected_state = false;

	switch (evt) {
	case ESOC_INVALID_STATE:
		esoc_mdm_log(
		"ESOC_INVALID_STATE: Calling complete with state: PON_FAIL\n");
		mdm_drv->pon_state = PON_FAIL;
		complete(&mdm_drv->pon_done);
		break;
	case ESOC_RUN_STATE:
		esoc_mdm_log(
		"ESOC_RUN_STATE: Calling complete with state: PON_SUCCESS\n");
		mdm_drv->pon_state = PON_SUCCESS;
		mdm_drv->mode = RUN,
		complete(&mdm_drv->pon_done);
		//Modem_BSP++
		is_first_boot=0;
		boot_fail_times=0;
		//Modem_BSP--
		break;
	case ESOC_RETRY_PON_EVT:
		esoc_mdm_log(
		"ESOC_RETRY_PON_EVT: Calling complete with state: PON_RETRY\n");
		mdm_drv->pon_state = PON_RETRY;
		complete(&mdm_drv->pon_done);
		break;
	case ESOC_UNEXPECTED_RESET:
		esoc_mdm_log("evt_state: ESOC_UNEXPECTED_RESET\n");
		unexpected_state = true;
	case ESOC_ERR_FATAL:
		if (!unexpected_state)
			esoc_mdm_log("evt_state: ESOC_ERR_FATAL\n");

		/*
		 * Modem can crash while we are waiting for pon_done during
		 * a subsystem_get(). Setting mode to CRASH will prevent a
		 * subsequent subsystem_get() from entering poweron ops. Avoid
		 * this by seting mode to CRASH only if device was up and
		 * running.
		 */
		if (mdm_drv->mode == CRASH)
			esoc_mdm_log(
			"Modem in crash state already. Ignoring.\n");
		if (mdm_drv->mode != RUN)
			esoc_mdm_log("Modem not up. Ignoring.\n");
		if (mdm_drv->mode == CRASH || mdm_drv->mode != RUN)
			return;
		mdm_drv->mode = CRASH;
		queue_work(mdm_drv->mdm_queue, &mdm_drv->ssr_work);
		break;
	case ESOC_REQ_ENG_ON:
		esoc_mdm_log(
		"evt_state: ESOC_REQ_ENG_ON; Registered a req engine\n");
		complete(&mdm_drv->req_eng_wait);
		break;
	default:
		break;
	}
}

static void mdm_ssr_fn(struct work_struct *work)
{
	struct mdm_drv *mdm_drv = container_of(work, struct mdm_drv, ssr_work);
	struct mdm_ctrl *mdm = get_esoc_clink_data(mdm_drv->esoc_clink);

	mdm_wait_for_status_low(mdm, false);

	esoc_mdm_log("Starting SSR work\n");

	/*
	 * If restarting esoc fails, the SSR framework triggers a kernel panic
	 */
	esoc_clink_request_ssr(mdm_drv->esoc_clink);
}

static void esoc_client_link_power_on(struct esoc_clink *esoc_clink,
							bool mdm_crashed)
{
	int i;
	struct esoc_client_hook *client_hook;

	dev_dbg(&esoc_clink->dev, "Calling power_on hooks\n");
	esoc_mdm_log(
	"Calling power_on hooks with crash state: %d\n", mdm_crashed);

	for (i = 0; i < ESOC_MAX_HOOKS; i++) {
		client_hook = esoc_clink->client_hook[i];
		if (client_hook && client_hook->esoc_link_power_on)
			client_hook->esoc_link_power_on(client_hook->priv,
							mdm_crashed);
	}
}

static void esoc_client_link_power_off(struct esoc_clink *esoc_clink,
							bool mdm_crashed)
{
	int i;
	struct esoc_client_hook *client_hook;

	dev_dbg(&esoc_clink->dev, "Calling power_off hooks\n");
	esoc_mdm_log(
	"Calling power_off hooks with crash state: %d\n", mdm_crashed);

	for (i = 0; i < ESOC_MAX_HOOKS; i++) {
		client_hook = esoc_clink->client_hook[i];
		if (client_hook && client_hook->esoc_link_power_off) {
			client_hook->esoc_link_power_off(client_hook->priv,
							mdm_crashed);
		}
	}
}

static void mdm_crash_shutdown(const struct subsys_desc *mdm_subsys)
{
	struct esoc_clink *esoc_clink =
					container_of(mdm_subsys,
							struct esoc_clink,
								subsys);
	const struct esoc_clink_ops * const clink_ops = esoc_clink->clink_ops;

	esoc_mdm_log("MDM crashed notification from SSR\n");

	if (mdm_dbg_stall_notify(ESOC_PRIMARY_CRASH))
		return;
	clink_ops->notify(ESOC_PRIMARY_CRASH, esoc_clink);
}

static int mdm_subsys_shutdown(const struct subsys_desc *crashed_subsys,
							bool force_stop)
{
	int ret;
	struct esoc_clink *esoc_clink =
	 container_of(crashed_subsys, struct esoc_clink, subsys);
	struct mdm_drv *mdm_drv = esoc_get_drv_data(esoc_clink);
	const struct esoc_clink_ops * const clink_ops = esoc_clink->clink_ops;

	esoc_mdm_log("Shutdown request from SSR\n");

	if (mdm_drv->mode == CRASH || mdm_drv->mode == PEER_CRASH) {
		esoc_mdm_log("Shutdown in crash mode\n");
		if (mdm_dbg_stall_cmd(ESOC_PREPARE_DEBUG))
			/* We want to mask debug command.
			 * In this case return success
			 * to move to next stage
			 */
			return 0;

		esoc_clink_queue_request(ESOC_REQ_CRASH_SHUTDOWN, esoc_clink);
		esoc_client_link_power_off(esoc_clink, true);

		esoc_mdm_log("Executing the ESOC_PREPARE_DEBUG command\n");
		ret = clink_ops->cmd_exe(ESOC_PREPARE_DEBUG,
							esoc_clink);
		if (ret) {
			esoc_mdm_log("ESOC_PREPARE_DEBUG command failed\n");
			dev_err(&esoc_clink->dev, "failed to enter debug\n");
			return ret;
		}
		mdm_drv->mode = IN_DEBUG;
	} else if (!force_stop) {
		esoc_mdm_log("Graceful shutdown mode\n");
		mutex_lock(&mdm_drv->poff_lock);
		if (mdm_drv->mode == PWR_OFF) {
			mutex_unlock(&mdm_drv->poff_lock);
			esoc_mdm_log("mdm already powered-off\n");
			return 0;
		}
		if (esoc_clink->subsys.sysmon_shutdown_ret) {
			esoc_mdm_log(
			"Executing the ESOC_FORCE_PWR_OFF command\n");
			ret = clink_ops->cmd_exe(ESOC_FORCE_PWR_OFF,
							esoc_clink);
		} else {
			if (mdm_dbg_stall_cmd(ESOC_PWR_OFF)) {
				/* Since power off command is masked
				 * we return success, and leave the state
				 * of the command engine as is.
				 */
				mutex_unlock(&mdm_drv->poff_lock);
				return 0;
			}
			dev_dbg(&esoc_clink->dev, "Sending sysmon-shutdown\n");
			esoc_mdm_log("Executing the ESOC_PWR_OFF command\n");
			ret = clink_ops->cmd_exe(ESOC_PWR_OFF, esoc_clink);
		}
		if (ret) {
			esoc_mdm_log(
			"Executing the ESOC_PWR_OFF command failed\n");
			dev_err(&esoc_clink->dev, "failed to exe power off\n");
			mutex_unlock(&mdm_drv->poff_lock);
			return ret;
		}
		esoc_client_link_power_off(esoc_clink, false);
		/* Pull the reset line low to turn off the device */
		clink_ops->cmd_exe(ESOC_FORCE_PWR_OFF, esoc_clink);
		mdm_drv->mode = PWR_OFF;
		mutex_unlock(&mdm_drv->poff_lock);
	}
	esoc_mdm_log("Shutdown completed\n");
	return 0;
}

static void mdm_subsys_retry_powerup_cleanup(struct esoc_clink *esoc_clink)
{
	struct mdm_ctrl *mdm = get_esoc_clink_data(esoc_clink);
	struct mdm_drv *mdm_drv = esoc_get_drv_data(esoc_clink);

	esoc_mdm_log("Doing cleanup\n");

	esoc_client_link_power_off(esoc_clink, false);
	mdm_disable_irqs(mdm);
	mdm_drv->pon_state = PON_INIT;
	reinit_completion(&mdm_drv->pon_done);
	reinit_completion(&mdm_drv->req_eng_wait);
}

//Modem_BSP++
int is_boot_fail=0;
//Modem_BSP--

/* Returns 0 to proceed towards another retry, or an error code to quit */
static int mdm_handle_boot_fail(struct esoc_clink *esoc_clink, u8 *pon_trial)
{
	struct mdm_ctrl *mdm = get_esoc_clink_data(esoc_clink);

	//Modem BSP++
	mdm_boot_fail_action();
	//Modem BSP--

	switch (boot_fail_action) {
	case BOOT_FAIL_ACTION_RETRY:
		mdm_subsys_retry_powerup_cleanup(esoc_clink);
		esoc_mdm_log("Request to retry a warm reset\n");
		(*pon_trial)++;
		break;
	/*
	 * Issue a shutdown here and rerun the powerup again.
	 * This way it becomes a cold reset. Else, we end up
	 * issuing a cold reset & a warm reset back to back.
	 */
	case BOOT_FAIL_ACTION_COLD_RESET:
		mdm_subsys_retry_powerup_cleanup(esoc_clink);
		esoc_mdm_log("Doing cold reset by power-down and warm reset\n");
		(*pon_trial)++;
		//Modem_BSP++
		is_boot_fail=1;
		//Modem_BSP--
		mdm_power_down(mdm);
		//Modem_BSP++
		is_boot_fail=0;
		//Modem_BSP--
		break;
	case BOOT_FAIL_ACTION_PANIC:
		esoc_mdm_log("Calling panic!!\n");
		panic("Panic requested on external modem boot failure\n");
		break;
	case BOOT_FAIL_ACTION_NOP:
		esoc_mdm_log("Leaving the modem in its curent state\n");
		return -EIO;
	case BOOT_FAIL_ACTION_SHUTDOWN:
	default:
		mdm_subsys_retry_powerup_cleanup(esoc_clink);
		esoc_mdm_log("Shutdown the modem and quit\n");
		mdm_power_down(mdm);
		return -EIO;
	}

	return 0;
}

static int mdm_subsys_powerup(const struct subsys_desc *crashed_subsys)
{
	int ret;
	struct esoc_clink *esoc_clink =
				container_of(crashed_subsys, struct esoc_clink,
								subsys);
	struct mdm_drv *mdm_drv = esoc_get_drv_data(esoc_clink);
	const struct esoc_clink_ops * const clink_ops = esoc_clink->clink_ops;
	int timeout = INT_MAX;
	u8 pon_trial = 1;

	esoc_mdm_log("Powerup request from SSR\n");

	do {
		esoc_mdm_log("Boot trial: %d\n", pon_trial);
		if (!esoc_clink->auto_boot &&
			!esoc_req_eng_enabled(esoc_clink)) {
			esoc_mdm_log("Wait for req eng registration\n");
			dev_dbg(&esoc_clink->dev,
					"Wait for req eng registration\n");
			wait_for_completion(&mdm_drv->req_eng_wait);
		}
		esoc_mdm_log("Req eng available\n");
		if (mdm_drv->mode == PWR_OFF) {
			esoc_mdm_log("In normal power-on mode\n");
			if (mdm_dbg_stall_cmd(ESOC_PWR_ON))
				return -EBUSY;
			esoc_mdm_log("Executing the ESOC_PWR_ON command\n");
			ret = clink_ops->cmd_exe(ESOC_PWR_ON, esoc_clink);
			if (ret) {
				esoc_mdm_log("ESOC_PWR_ON command failed\n");
				dev_err(&esoc_clink->dev, "pwr on fail\n");
				return ret;
			}
			esoc_client_link_power_on(esoc_clink, false);
		} else if (mdm_drv->mode == IN_DEBUG) {
			esoc_mdm_log("In SSR power-on mode\n");
			esoc_mdm_log("Executing the ESOC_EXIT_DEBUG command\n");
			ret = clink_ops->cmd_exe(ESOC_EXIT_DEBUG, esoc_clink);
			if (ret) {
				esoc_mdm_log(
				"ESOC_EXIT_DEBUG command failed\n");
				dev_err(&esoc_clink->dev,
						"cannot exit debug mode\n");
				return ret;
			}
			mdm_drv->mode = PWR_OFF;
			esoc_mdm_log("Executing the ESOC_PWR_ON command\n");
			ret = clink_ops->cmd_exe(ESOC_PWR_ON, esoc_clink);
			if (ret) {
				dev_err(&esoc_clink->dev, "pwr on fail\n");
				return ret;
			}
			esoc_client_link_power_on(esoc_clink, true);
		}

		/*
		 * In autoboot case, it is possible that we can forever wait for
		 * boot completion, when esoc fails to boot. This is because
		 * there is no helper application which can alert esoc driver
		 * about boot failure. Prevent going to wait forever in such
		 * case.
		 */
		if (esoc_clink->auto_boot)
			timeout = 10 * HZ;
		esoc_mdm_log(
		"Modem turned-on. Waiting for pon_done notification..\n");
		ret = wait_for_completion_timeout(&mdm_drv->pon_done, timeout);
		if (mdm_drv->pon_state == PON_FAIL || ret <= 0) {
			dev_err(&esoc_clink->dev, "booting failed\n");
			esoc_mdm_log("booting failed\n");
			ret = mdm_handle_boot_fail(esoc_clink, &pon_trial);
			if (ret)
				return ret;
		} else if (mdm_drv->pon_state == PON_RETRY) {
			esoc_mdm_log(
			"Boot failed. Doing cleanup and attempting to retry\n");
			pon_trial++;
			mdm_subsys_retry_powerup_cleanup(esoc_clink);
		} else if (mdm_drv->pon_state == PON_SUCCESS) {
			break;
		}
	} while (pon_trial <= n_pon_tries);

	return 0;
}

static int mdm_subsys_ramdumps(int want_dumps,
				const struct subsys_desc *crashed_subsys)
{
	int ret;
	struct esoc_clink *esoc_clink =
				container_of(crashed_subsys, struct esoc_clink,
								subsys);
	const struct esoc_clink_ops * const clink_ops = esoc_clink->clink_ops;

	esoc_mdm_log("Ramdumps called from SSR\n");

	if (want_dumps) {
		esoc_mdm_log("Executing the ESOC_EXE_DEBUG command\n");
		ret = clink_ops->cmd_exe(ESOC_EXE_DEBUG, esoc_clink);
		if (ret) {
			esoc_mdm_log(
			"Failed executing the ESOC_EXE_DEBUG command\n");
			dev_err(&esoc_clink->dev, "debugging failed\n");
			return ret;
		}
	}
	return 0;
}

static int mdm_register_ssr(struct esoc_clink *esoc_clink)
{
	struct subsys_desc *subsys = &esoc_clink->subsys;

	subsys->shutdown = mdm_subsys_shutdown;
	subsys->ramdump = mdm_subsys_ramdumps;
	subsys->powerup = mdm_subsys_powerup;
	subsys->crash_shutdown = mdm_crash_shutdown;
	return esoc_clink_register_ssr(esoc_clink);
}

int esoc_ssr_probe(struct esoc_clink *esoc_clink, struct esoc_drv *drv)
{
	int ret;
	struct mdm_drv *mdm_drv;
	struct esoc_eng *esoc_eng;
	//Modem_BSP++
	struct mdm_ctrl *mdm = get_esoc_clink_data(esoc_clink);
	unsigned int abnrst_flag = get_abnrst_flag();
	//Modem_BSP--

	mdm_drv = devm_kzalloc(&esoc_clink->dev, sizeof(*mdm_drv), GFP_KERNEL);
	if (IS_ERR_OR_NULL(mdm_drv))
		return PTR_ERR(mdm_drv);
	esoc_eng = &mdm_drv->cmd_eng;
	esoc_eng->handle_clink_evt = mdm_handle_clink_evt;
	ret = esoc_clink_register_cmd_eng(esoc_clink, esoc_eng);
	if (ret) {
		dev_err(&esoc_clink->dev, "failed to register cmd engine\n");
		return ret;
	}
	mutex_init(&mdm_drv->poff_lock);
	ret = mdm_register_ssr(esoc_clink);
	if (ret)
		goto ssr_err;
	mdm_drv->mdm_queue = alloc_workqueue("mdm_drv_queue", 0, 0);
	if (!mdm_drv->mdm_queue) {
		dev_err(&esoc_clink->dev, "could not create mdm_queue\n");
		goto queue_err;
	}
	esoc_set_drv_data(esoc_clink, mdm_drv);
	init_completion(&mdm_drv->pon_done);
	init_completion(&mdm_drv->req_eng_wait);
	INIT_WORK(&mdm_drv->ssr_work, mdm_ssr_fn);
	mdm_drv->esoc_clink = esoc_clink;
	mdm_drv->mode = PWR_OFF;
	mdm_drv->pon_state = PON_INIT;
	mdm_drv->esoc_restart.notifier_call = esoc_msm_restart_handler;
	ret = register_reboot_notifier(&mdm_drv->esoc_restart);
	if (ret)
		dev_err(&esoc_clink->dev, "register for reboot failed\n");
	ret = mdm_dbg_eng_init(drv, esoc_clink);
	if (ret) {
		debug_init_done = false;
		dev_err(&esoc_clink->dev, "dbg engine failure\n");
	} else {
		dev_dbg(&esoc_clink->dev, "dbg engine initialized\n");
		debug_init_done = true;
	}

	//Modem_BSP++
	esoc_mdm_log("abnrst = %d\n" , abnrst_flag);
	if(abnrst_flag==0 && gpio_get_value(MDM_GPIO(mdm, AP2MDM_SOFT_RESET))==1)
	{
		gpio_direction_output(MDM_GPIO(mdm, AP2MDM_SOFT_RESET),0);
		esoc_mdm_log("Pull down AP2MDM_SOFT_RESET gpio\n");
	}
	//Modem_BSP--

	return 0;
queue_err:
	esoc_clink_unregister_ssr(esoc_clink);
ssr_err:
	esoc_clink_unregister_cmd_eng(esoc_clink, esoc_eng);
	return ret;
}

static struct esoc_compat compat_table[] = {
	{
		.name = "MDM9x55",
		.data = NULL,
	},
	{
		.name = "SDX50M",
		.data = NULL,
	},
	{
		.name = "SDXPRAIRIE",
		.data = NULL,
	},
};

static struct esoc_drv esoc_ssr_drv = {
	.owner = THIS_MODULE,
	.probe = esoc_ssr_probe,
	.compat_table = compat_table,
	.compat_entries = ARRAY_SIZE(compat_table),
	.driver = {
		.name = "mdm-4x",
	},
};

int __init esoc_ssr_init(void)
{
	return esoc_drv_register(&esoc_ssr_drv);
}
module_init(esoc_ssr_init);
MODULE_LICENSE("GPL v2");
