#include <linux/io.h>
#include <linux/rtc.h>
#include <linux/time.h>
#include <linux/tick.h>
#include <linux/sched.h>
#include <linux/sched/signal.h>
#include <linux/sched/cputime.h>
#include <linux/timer.h>
#include <linux/string.h>
#include <linux/module.h>
#include <linux/vmalloc.h>
#include <linux/pm_wakeup.h>
#include <linux/kernel_stat.h>
/* #include <mach/devices_dtb.h> Not implemented yet */
#include <soc/qcom/htc_util.h>
#include <soc/qcom/subsystem_restart.h>
#include <linux/irq.h>
#include <soc/qcom/pm.h>
#include <linux/jiffies.h>
#include <linux/htc_cpu_usage_stats.h>
//#include <linux/vmstat.h>
#include <linux/htc_flags.h>
#include <linux/thermal.h>

#define USE_STATISTICS_STRATEGY_CONTINUOUS_3    0
#define SEND_KOBJECT_UEVENT_ENV_ENABLED         0

#define NUM_BUSY_THREAD_CHECK                   5
#define HTC_KERNEL_TOP_CPU_USAGE_THRESHOLD      30
#define BUFFER_WARN_LEN                         64
#define BUFFER_TEMP_LEN                         32
#define CPU_MODE                                5

#define FORCE_CHARGE                            (1<<2)
#define Y_CABLE                                 (1<<26)

#define KTOP_ACCU_FD "/sys/power/ktop_accu"

/* For thermal 5G status detection.*/
#define MODEM_5G_UNKNOWN_STAT                   -1
#define MODEM_5G_OFFLINING                      0
#define MODEM_5G_OFFLINE                        1
#define MODEM_5G_ONLINE                         2
#define MODEM_5G_SOC_NAME                       "esoc0"

/*for cpu*/
typedef u64 __nocast cputime64_t;
#define jiffies64_to_cputime64(__jif) (__force cputime64_t)(__jif)
#define usecs_to_cputime64(__usec) \
    jiffies64_to_cputime64(nsecs_to_jiffies64((__usec) * 1000))

struct process_monitor_statistic {
    unsigned int pid;
    char *ppid_name;
    unsigned int cnt;
    unsigned char set_warn;
    unsigned char is_found;
#if SEND_KOBJECT_UEVENT_ENV_ENABLED
    unsigned char sent_uevent;
#endif /*SEND_KOBJECT_UEVENT_ENV_ENABLED */
};

// static int pm_monitor_enabled = 0;
/* Default Enable */
static int pm_monitor_enabled = 1;

static struct workqueue_struct *htc_pm_monitor_wq = NULL;
static struct workqueue_struct *htc_kernel_top_monitor_wq = NULL;

/* Previous process state */
#define MAX_PID                                 32768
#define MAX_PROC_NAME_LEN                       16
#define NUM_BUSY_THREAD_CHECK                   5

struct _htc_kernel_top {
    struct delayed_work dwork;

    u64 cpustat_time;
    u64 *cpu_time;
    u64 *prev_proc_stat;
    u64 *curr_proc_delta;

    struct kernel_cpustat curr_cpustat;
    struct kernel_cpustat prev_cpustat;

    int *curr_proc_pid;
    int top_loading_pid[NUM_BUSY_THREAD_CHECK];
    char *local_task_proc_name_array_start;

    spinlock_t lock;
};

struct st_htc_idle_statistic {
    u32 count;
    u32 time;
};

struct st_htc_idle_statistic htc_idle_stat[CONFIG_NR_CPUS][CPU_MODE];

static int msm_htc_util_delay_time = 10000;
module_param_named(kmonitor_delay, msm_htc_util_delay_time, int, S_IRUGO | S_IWUSR | S_IWGRP);
static int msm_htc_util_top_delay_time = 60000;
module_param_named(ktop_delay, msm_htc_util_top_delay_time, int, S_IRUGO | S_IWUSR | S_IWGRP);

enum {
    KERNEL_TOP,
    KERNEL_TOP_ACCU, /* Kernel Top Accumulation */
};

#if USE_STATISTICS_STRATEGY_CONTINUOUS_3
#define MAX_CONSECUTIVE_THRES_TIMES                     3
#define SIZE_OF_CURR_PID_FOUND_ARRAY                    ARRAY_SIZE(current_pid_found_array)
#define SIZE_OF_PROCESS_MONITOR_CONTINUOUS_3_ARRAY      ARRAY_SIZE(process_monitor_continuous_3_array)
struct current_pid_found {
    unsigned char pid_found;
    unsigned char need_to_add;
};

static struct current_pid_found current_pid_found_array[NUM_BUSY_THREAD_CHECK];
static struct process_monitor_statistic process_monitor_continuous_3_array[NUM_BUSY_THREAD_CHECK];

#else /* <Not> USE_STATISTICS_STRATEGY_CONTINUOUS_3 */
#define MAX_OVER_THRES_TIMES                             5
#define HTC_KERNEL_TOP_MONITOR_PERIOD                    10
#define MAX_PROCESS_MONITOR_ARRAY_FIELDS                 (HTC_KERNEL_TOP_MONITOR_PERIOD * NUM_BUSY_THREAD_CHECK)
#define PROCESS_MONITOR_ARRAY_5_IN_10_SIZE               MAX_PROCESS_MONITOR_ARRAY_FIELDS
#define BUFFER_WARN_5_IN_10_SIZE                         HTC_KERNEL_TOP_MONITOR_PERIOD
#define SIZE_OF_PROCESS_MONITOR_5_IN_10_ARRAY            ARRAY_SIZE(process_monitor_5_in_10_array)

static int statistic_monitor_period = 1;
static struct process_monitor_statistic process_monitor_5_in_10_array[PROCESS_MONITOR_ARRAY_5_IN_10_SIZE];
#endif /* USE_STATISTICS_STRATEGY_CONTINUOUS_3 */

inline char* get_proc_name_start(char* start, int idx) {
    return start + (idx * MAX_PROC_NAME_LEN);
}

int get_kernel_cluster_info(int *cluster_id, cpumask_t *cluster_cpus) {
    uint32_t _cpu, cluster_index, cluster_cnt;

    for (_cpu = 0, cluster_cnt = 0; _cpu < num_possible_cpus(); _cpu++) {
        if (topology_physical_package_id(_cpu) < 0) {
            pr_err("CPU%d topology not initialized.\n", _cpu);
            return -ENODEV;
        }
        /* Do not use the sibling cpumask from topology module.
        ** kernel topology module updates the sibling cpumask
        ** only when the cores are brought online for the first time.
        ** KTM figures out the sibling cpumask using the
        ** cluster and core ID mapping.
        */
        for (cluster_index = 0; cluster_index < num_possible_cpus();
            cluster_index++) {
            if (cluster_id[cluster_index] == -1) {
                cluster_id[cluster_index] =
                    topology_physical_package_id(_cpu);
                cpumask_clear(&cluster_cpus[cluster_index]);
                cpumask_set_cpu(_cpu,
                    &cluster_cpus[cluster_index]);
                cluster_cnt++;
                break;
            }
            if (cluster_id[cluster_index] == topology_physical_package_id(_cpu)) {
                cpumask_set_cpu(_cpu,
                    &cluster_cpus[cluster_index]);
                break;
            }
        }
    }

    return cluster_cnt;
}

bool is_commercial() {
    return get_radio_flag() == 0;
}

int check_top_malloc(struct _htc_kernel_top *ktop) {
    struct _htc_kernel_top *top = ktop;

    if(NULL == top) {
        pr_err("[K] ktop is null!\n");
        return 0;
    }

    if(top->local_task_proc_name_array_start && top->curr_proc_delta &&
            top->curr_proc_pid && top->prev_proc_stat && top->cpu_time) {
        return 1;
    }

    if(top->local_task_proc_name_array_start) {
        vfree(top->local_task_proc_name_array_start);
        top->local_task_proc_name_array_start = NULL;
    } else pr_err("[K] local_task_proc_name_array_start is null, vmalloc failed!\n");
    if(top->curr_proc_delta) {
        vfree(top->curr_proc_delta);
        top->curr_proc_delta = NULL;
    } else pr_err("[K] curr_proc_delta is null, vmalloc failed!\n");
    if(top->curr_proc_pid) {
        vfree(top->curr_proc_pid);
        top->curr_proc_pid = NULL;
    } else pr_err("[K] curr_proc_pid is null, vmalloc failed!\n");
    if(top->prev_proc_stat) {
        vfree(top->prev_proc_stat);
        top->prev_proc_stat = NULL;
    } else pr_err("[K] prev_proc_stat is null, vmalloc failed!\n");
    if(top->cpu_time) {
        vfree(top->cpu_time);
        top->cpu_time = NULL;
    } else pr_err("[K] cpu_time is null, vmalloc failed!\n");

    return 0;
}

static void clear_process_monitor_array(
        struct process_monitor_statistic *pArray, int array_size) {
    int j;

    for (j = 0; j < array_size; j++) {
        (pArray + j)->pid = 0;
        (pArray + j)->ppid_name = NULL;
        (pArray + j)->cnt = 0;
        (pArray + j)->set_warn = 0;
        (pArray + j)->is_found = 0;
#if SEND_KOBJECT_UEVENT_ENV_ENABLED
        (pArray + j)->sent_uevent = 0;
#endif /* SEND_KOBJECT_UEVENT_ENV_ENABLED */
    }
} /* clear_process_monitor_array() */

static void write_ktop_fd(struct _htc_kernel_top *ktop) {
    struct file *fp = NULL;
    char buf[BUFFER_TEMP_LEN * NUM_BUSY_THREAD_CHECK], temp[BUFFER_TEMP_LEN];
    uint8_t idx = 0, size;
    char name[MAX_PROC_NAME_LEN] = {'\0'};
    int tmp, pid, usage;
    u64 duration;

    loff_t pos = 0;

    if(!check_top_malloc(ktop)) {
        return;
    }

    fp = filp_open(KTOP_ACCU_FD, O_WRONLY, 0);
    if (IS_ERR(fp)) {
        pr_err("Unable to open KTOP_ACCU_FD\n");
        return;
    }

    duration = ktop->cpustat_time;
    if (duration) {
        buf[0] = '\0';
        size = sizeof(buf);

        while(idx < NUM_BUSY_THREAD_CHECK) {
            pid = ktop->top_loading_pid[idx];
            strncpy(name, get_proc_name_start(ktop->local_task_proc_name_array_start, pid), MAX_PROC_NAME_LEN);
            usage = (int)(ktop->curr_proc_delta[pid] * 100 / duration);
            pr_debug("pid: %d, name: %s, usage: %d\n", pid, name, usage);
            snprintf(temp, BUFFER_TEMP_LEN, "%d\t%d\t%s\n", usage, pid, name);
            strncat(buf, temp, BUFFER_TEMP_LEN);
            pr_debug("buf: %s, temp: %s\n", buf, temp);
            idx++;
        }

        tmp = kernel_write(fp, buf, size, &pos);
        if (tmp < 0)
            pr_err("Fail to write KTOP_ACCU_FD\n");
    }

    if (fp)
        filp_close(fp, NULL);
    return;
}

#if USE_STATISTICS_STRATEGY_CONTINUOUS_3
static void clear_current_pid_found_array(void) {
    int i;

    for (i = 0; i < SIZE_OF_CURR_PID_FOUND_ARRAY; i++) {
        current_pid_found_array[i].pid_found = 0;
        current_pid_found_array[i].need_to_add = 0;
    }
}

static int htc_kernel_top_statistics_continuous_3(struct _htc_kernel_top *ktop) {
    int rtn = 0, i, j, ok_to_send_uevent = 0, *ptr_top_loading = ktop->top_loading_pid;
    u64 cpu_usage = 0, delta_time = ktop->cpustat_time;
#if SEND_KOBJECT_UEVENT_ENV_ENABLED
    char buf_warn[SIZE_OF_PROCESS_MONITOR_CONTINUOUS_3_ARRAY * BUFFER_WARN_LEN],
         buf_temp[BUFFER_TEMP_LEN];
#endif /* SEND_KOBJECT_UEVENT_ENV_ENABLED */

    for (i = 0 ; i < SIZE_OF_CURR_PID_FOUND_ARRAY ; i++) {
        if (delta_time > 0)
            cpu_usage = (u64)(ktop->curr_proc_delta[*(ptr_top_loading + i)] * 100 / delta_time);
        /* Reach the threshold */
        if (cpu_usage >= HTC_KERNEL_TOP_CPU_USAGE_THRESHOLD) {
            /* Search in the array to check if we got any PID match. */
            for (j = 0; j < SIZE_OF_PROCESS_MONITOR_CONTINUOUS_3_ARRAY; j++) {
                /* Mate with the PID records. */
                if (process_monitor_continuous_3_array[j].pid == *(ptr_top_loading + i)) {
                    /* Found the PID record. */
                    process_monitor_continuous_3_array[j].cnt++;
                    process_monitor_continuous_3_array[j].is_found = 1;
                    /* Mark the PID was found. */
                    current_pid_found_array[i].pid_found = 1;
                    if ((process_monitor_continuous_3_array[j].cnt >= MAX_CONSECUTIVE_THRES_TIMES) &&
                            (!process_monitor_continuous_3_array[j].set_warn)) {
                        process_monitor_continuous_3_array[j].set_warn = 1;
                        pr_info("[K] CPU_Sniffer: PID=[%d], name=[%s], over-cpu-usage-threshold.\n",
                                process_monitor_continuous_3_array[j].pid, process_monitor_continuous_3_array[j].ppid_name);
                    }
                    break;
                }
            }
            if (!current_pid_found_array[i].pid_found) {
                current_pid_found_array[i].need_to_add = 1;
            }
        }
    }

#if SEND_KOBJECT_UEVENT_ENV_ENABLED
    /* Pack buffer for sending out kobject_uevent. */
    memset(buf_warn, 0x0, sizeof(buf_warn));
    strcpy(buf_warn, "");
    for (j = 0; j < SIZE_OF_PROCESS_MONITOR_CONTINUOUS_3_ARRAY; j++) {
        if ((process_monitor_continuous_3_array[j].set_warn == 1) &&
                (process_monitor_continuous_3_array[j].sent_uevent == 0)) {
            strcat(buf_warn, "PID=");
            sprintf(buf_temp, "%d", process_monitor_continuous_3_array[j].pid);
            strcat(buf_warn, buf_temp);
            strcat(buf_warn, ",0,0,0;");
            process_monitor_continuous_3_array[j].sent_uevent = 1;
            ok_to_send_uevent++;
        }
    }

    /* Need to send notification by kobject_uevent_env(). */
    if (ok_to_send_uevent) {
        /* End string. */
        strcat(buf_warn, "PID=0,0,0,0;");
        strcat(buf_warn, "#");
        send_cpu_usage_stats_kobject_uevent(&buf_warn[0]);
    }
#endif /* SEND_KOBJECT_UEVENT_ENV_ENABLED */

    /* Kick out the non-consecutive PID record. */
    for (j = 0; j < SIZE_OF_PROCESS_MONITOR_CONTINUOUS_3_ARRAY; j++) {
        if (!process_monitor_continuous_3_array[j].is_found) {
            /* Clear the record. */
            process_monitor_continuous_3_array[j].pid = 0;
            process_monitor_continuous_3_array[j].ppid_name = NULL;
            process_monitor_continuous_3_array[j].cnt = 0;
            process_monitor_continuous_3_array[j].set_warn = 0;
#if SEND_KOBJECT_UEVENT_ENV_ENABLED
            process_monitor_continuous_3_array[j].sent_uevent = 0;
#endif /* SEND_KOBJECT_UEVENT_ENV_ENABLED */
        }
        /* Clear the found flag of this round. */
        process_monitor_continuous_3_array[j].is_found = 0;
    }

    /* Add new record. */
    for (i = 0 ; i < SIZE_OF_CURR_PID_FOUND_ARRAY ; i++) {
        /* Store the newer to add into process monitor array. */
        for (j = 0; j < SIZE_OF_PROCESS_MONITOR_CONTINUOUS_3_ARRAY; j++) {
            if (current_pid_found_array[i].need_to_add && !process_monitor_continuous_3_array[j].pid) {
                process_monitor_continuous_3_array[j].pid = *(ptr_top_loading + i);
                process_monitor_continuous_3_array[j].ppid_name =
                    get_proc_name_start(ktop->local_task_proc_name_array_start, *(ptr_top_loading + i));
                process_monitor_continuous_3_array[j].cnt++;
                current_pid_found_array[i].need_to_add = 0;
                break;
            }
        }
    }
    clear_current_pid_found_array();

    return rtn;
}
#else /* <Not> USE_STATISTICS_STRATEGY_CONTINUOUS_3 */
static int htc_kernel_top_statistics_5_in_10(struct _htc_kernel_top *ktop)
{
    int i, j, rtn = 0, *ptr_top_loading = ktop->top_loading_pid;
    u64 cpu_usage = 0, delta_time = ktop->cpustat_time;
#if SEND_KOBJECT_UEVENT_ENV_ENABLED
    int ok_to_send_uevent = 0;
    char buf_warn[BUFFER_WARN_5_IN_10_SIZE * BUFFER_WARN_LEN], buf_temp[BUFFER_TEMP_LEN];
#endif /* SEND_KOBJECT_UEVENT_ENV_ENABLED */

    for (i = 0 ; i < NUM_BUSY_THREAD_CHECK ; i++) {
        if (delta_time > 0)
            cpu_usage = (u64)(ktop->curr_proc_delta[*(ptr_top_loading + i)] * 100 / delta_time);
        /* Reach the threshold */
        if (cpu_usage >= HTC_KERNEL_TOP_CPU_USAGE_THRESHOLD) {
            /* Search in the array to check if we got any PID match. */
            for (j = 0; j < SIZE_OF_PROCESS_MONITOR_5_IN_10_ARRAY; j++) {
                /* Mate with the PID records. */
                if (process_monitor_5_in_10_array[j].pid == *(ptr_top_loading + i)) {
                    /* Found the PID record. */
                    process_monitor_5_in_10_array[j].cnt++;
                    if ((process_monitor_5_in_10_array[j].cnt >= MAX_OVER_THRES_TIMES) &&
                            (process_monitor_5_in_10_array[j].set_warn == 0)) {
                        process_monitor_5_in_10_array[j].set_warn = 1;
                        process_monitor_5_in_10_array[j].ppid_name =
                            get_proc_name_start(ktop->local_task_proc_name_array_start, *(ptr_top_loading + i));
                    }
                    break;
                }
                /* Add as the new PID record. */
                else if (process_monitor_5_in_10_array[j].pid == 0) {
                    process_monitor_5_in_10_array[j].pid = *(ptr_top_loading + i);
                    process_monitor_5_in_10_array[j].cnt++;
                    break;
                }
            }
        }
    }

    if (statistic_monitor_period < HTC_KERNEL_TOP_MONITOR_PERIOD) {
        /* 1 ~ 9 */
        statistic_monitor_period++;
    } else {
        /* 10 -> 1 */
#if SEND_KOBJECT_UEVENT_ENV_ENABLED
        /* Pack buffer for sending out kobject_uevent. */
        memset(buf_warn, 0x0, sizeof(buf_warn));
        strcpy(buf_warn, "");
#endif /* SEND_KOBJECT_UEVENT_ENV_ENABLED */
        for (j = 0; j < SIZE_OF_PROCESS_MONITOR_5_IN_10_ARRAY; j++) {
            if (process_monitor_5_in_10_array[j].set_warn == 1) {
#if SEND_KOBJECT_UEVENT_ENV_ENABLED
                strcat(buf_warn, "PID=");
                sprintf(buf_temp, "%d", process_monitor_5_in_10_array[j].pid);
                strcat(buf_warn, buf_temp);
                strcat(buf_warn, ",0,0,0;");
#endif /* SEND_KOBJECT_UEVENT_ENV_ENABLED */
                pr_info("[K] CPU_Sniffer: PID=[%d], name=[%s], over-cpu-usage-threshold.\n",
                        process_monitor_5_in_10_array[j].pid, process_monitor_5_in_10_array[j].ppid_name);
#if SEND_KOBJECT_UEVENT_ENV_ENABLED
                process_monitor_5_in_10_array[j].sent_uevent = 1;
                ok_to_send_uevent++;
#endif /* SEND_KOBJECT_UEVENT_ENV_ENABLED */
                rtn = 1;
            }
        }

#if SEND_KOBJECT_UEVENT_ENV_ENABLED
        /* Need to send notification by kobject_uevent_env(). */
        if (ok_to_send_uevent) {
            /* End string. */
            strcat(buf_warn, "PID=0,0,0,0;");
            strcat(buf_warn, "#");
            send_cpu_usage_stats_kobject_uevent(&buf_warn[0]);
        }
#endif /* SEND_KOBJECT_UEVENT_ENV_ENABLED */

        if (pm_monitor_enabled)
            pr_debug("[K] [KTOP] Reach the number of statistics monitor period.\n");
        statistic_monitor_period = 1;
        clear_process_monitor_array(&process_monitor_5_in_10_array[0], SIZE_OF_PROCESS_MONITOR_5_IN_10_ARRAY);
    }

    return rtn;
}
#endif /* USE_STATISTICS_STRATEGY_CONTINUOUS_3 */

void htc_idle_stat_add(int sleep_mode, u32 time) {
    int cpu = smp_processor_id();
    if (cpu < CONFIG_NR_CPUS && sleep_mode < CPU_MODE) {
        htc_idle_stat[cpu][sleep_mode].count++;
        htc_idle_stat[cpu][sleep_mode].time += time;
    } else {
        pr_info("[Power_FDA][K] Miss logging idle stat: cpu=%d sleep_mode=%d time=%d\n", cpu, sleep_mode    , time);
    }
}

static void htc_idle_stat_clear(void) {
    memset(htc_idle_stat, 0, sizeof(htc_idle_stat));
}

static void htc_idle_stat_show(void) {
    int i = 0, cpu = 0, piece_size = 32,
        output_size = piece_size * (CONFIG_NR_CPUS + 1);
    char output[output_size], piece[piece_size];

    for (i = 0; i < CPU_MODE; i++) {
        memset(output, 0, sizeof(output));
        for (cpu = 0; cpu < CONFIG_NR_CPUS; cpu++) {
            if (htc_idle_stat[cpu][i].count > 0) {
                if (0 == strlen(output)) {
                    memset(piece, 0, sizeof(piece));
                    snprintf(piece, sizeof(piece), "C%d: ", i);
                    safe_strcat(output, piece);
                } else {
                    safe_strcat(output, ",");
                }
                memset(piece, 0, sizeof(piece));
                snprintf(piece, sizeof(piece), "(%d,%d,%dms)",cpu,
                        htc_idle_stat[cpu][i].count,
                        htc_idle_stat[cpu][i].time / 1000);
                safe_strcat(output, piece);
            }
        }
        if (strlen(output) > 0) {
            k_pr_embedded("[K] %s\n", output);
        }
    }
}

static void get_all_cpustat(struct _htc_kernel_top *ktop) {
    int cpu = 0; // only account cpu0 total time for cpustat
    struct kernel_cpustat *cpu_stat;

    if (NULL == ktop) {
        pr_err("[K] ktop should not be NULL!\n");
        return;
    }
    cpu_stat = &(ktop->curr_cpustat);
    memset(cpu_stat, 0, sizeof(struct kernel_cpustat));

    cpu_stat->cpustat[CPUTIME_USER] += kcpustat_cpu(cpu).cpustat[CPUTIME_USER];
    cpu_stat->cpustat[CPUTIME_NICE] += kcpustat_cpu(cpu).cpustat[CPUTIME_NICE];
    cpu_stat->cpustat[CPUTIME_SYSTEM] += kcpustat_cpu(cpu).cpustat[CPUTIME_SYSTEM];
    cpu_stat->cpustat[CPUTIME_SOFTIRQ] += kcpustat_cpu(cpu).cpustat[CPUTIME_SOFTIRQ];
    cpu_stat->cpustat[CPUTIME_IRQ] += kcpustat_cpu(cpu).cpustat[CPUTIME_IRQ];
    cpu_stat->cpustat[CPUTIME_IDLE] += kcpustat_cpu(cpu).cpustat[CPUTIME_IDLE];
    cpu_stat->cpustat[CPUTIME_IOWAIT] += kcpustat_cpu(cpu).cpustat[CPUTIME_IOWAIT];
    cpu_stat->cpustat[CPUTIME_STEAL] += kcpustat_cpu(cpu).cpustat[CPUTIME_STEAL];
    // CPUTIME_GUEST is already accounted in CPUTIME_USER
    // CPUTIME_GUEST_NICE is already accounted in CPUTIME_NICE
}

static void sort_cputime_by_pid(u64 *proc_delta,
        int *pid_pos, int pid_cnt, int *top_pid) {
    int p_idx, t_idx, tt_idx;
    u64 temp_max = 0;

    for(p_idx = 0; p_idx < pid_cnt; ++p_idx) {
        if(temp_max < proc_delta[pid_pos[p_idx]]) {
            temp_max = proc_delta[pid_pos[p_idx]];
            top_pid[0] = pid_pos[p_idx];
        }
    }

    for(t_idx = 1; t_idx < NUM_BUSY_THREAD_CHECK; ++t_idx) { //next 4 max
        temp_max = 0;
        for(p_idx = 0; p_idx < pid_cnt; ++p_idx) { // search all
            if(temp_max < proc_delta[pid_pos[p_idx]] && top_pid[t_idx-1] != pid_pos[p_idx]) {
                //find next max=proc_delta[top_pid[t_idx]]
                //proc_delta[top_pid[t_idx-1]] must >= proc_delta[pid_pos[p_idxi]]
                if(proc_delta[top_pid[t_idx-1]] >= proc_delta[pid_pos[p_idx]]) {
                    bool not_in_top_pid = true;
                    for(tt_idx = 0; tt_idx < t_idx; ++tt_idx) { //check this pid which is not in top
                        if(top_pid[tt_idx] == pid_pos[p_idx])
                            not_in_top_pid = false;
                        if(not_in_top_pid) {
                            temp_max = proc_delta[pid_pos[p_idx]];
                            top_pid[t_idx] = pid_pos[p_idx];
                        }
                    }  // for(tt_idx = 0; tt_idx < t_idx; ++tt_idx)
                }  // if(proc_delta[top_pid[t_idx-1]] >= proc_delta[pid_pos[i]])
            }  // if(temp_max < proc_delta[pid_pos[i]] && top_pid[t_idx-1] != pid_pos[i])
        }  // for(p_idx = 0; p_idx < pid_cnt; ++p_idx) {
    }  // for(t_idx = 1; t_idx < NUM_BUSY_THREAD_CHECK; ++j)
}

inline u64 cpustat_diff(int id, struct kernel_cpustat *curr_cpustat,
        struct kernel_cpustat *prev_cpustat) {
    return (curr_cpustat->cpustat[id] - prev_cpustat->cpustat[id]);
}

static u64 htc_calculate_cpustat_time(struct kernel_cpustat *curr_cpustat,
        struct kernel_cpustat *prev_cpustat) {
    u64 cpustat_time = 0;

    // CPUTIME_GUEST is already accounted in CPUTIME_USER
    cpustat_time += cpustat_diff(CPUTIME_USER, curr_cpustat, prev_cpustat);
    // CPUTIME_GUEST_NICE is already accounted in CPUTIME_NICE
    cpustat_time += cpustat_diff(CPUTIME_NICE, curr_cpustat, prev_cpustat);

    cpustat_time += cpustat_diff(CPUTIME_SYSTEM, curr_cpustat, prev_cpustat);

    cpustat_time += cpustat_diff(CPUTIME_SOFTIRQ, curr_cpustat, prev_cpustat);

    cpustat_time += cpustat_diff(CPUTIME_IRQ, curr_cpustat, prev_cpustat);

    cpustat_time += cpustat_diff(CPUTIME_IDLE, curr_cpustat, prev_cpustat);

    cpustat_time += cpustat_diff(CPUTIME_IOWAIT, curr_cpustat, prev_cpustat);

    cpustat_time += cpustat_diff(CPUTIME_STEAL, curr_cpustat, prev_cpustat);

    return cpustat_time;
}

static void htc_kernel_top_cal(struct _htc_kernel_top *ktop, int type) {
    int pid_cnt = 0, pid_num = 0;
    struct task_struct *process;

    if(!check_top_malloc(ktop)) {
        return;
    }

    memset(ktop->cpu_time, 0, sizeof(u64) * MAX_PID);
    /* Calculate cpu time of each process */
    rcu_read_lock();
    for_each_process(process) {
        if (process->pid < MAX_PID) {
            struct task_cputime cputime;

            thread_group_cputime(process, &cputime);
            pid_num = process->pid;
            ktop->cpu_time[pid_num] = cputime.utime + cputime.stime;
            strncpy(get_proc_name_start(ktop->local_task_proc_name_array_start, pid_num),
                    process->comm, MAX_PROC_NAME_LEN);

            // accmulate cpu time
            ktop->curr_proc_delta[pid_num] = ktop->cpu_time[pid_num] - ktop->prev_proc_stat[pid_num];
            ktop->prev_proc_stat[pid_num] = ktop->cpu_time[pid_num];

            if (ktop->curr_proc_delta[pid_num] > 0)
                ktop->curr_proc_pid[pid_cnt++] = pid_num;
        }
    }
    rcu_read_unlock();

    /* Calculate cpu time of cpus */
    get_all_cpustat(ktop);
    ktop->cpustat_time = htc_calculate_cpustat_time(&ktop->curr_cpustat, &ktop->prev_cpustat);

    sort_cputime_by_pid(ktop->curr_proc_delta, ktop->curr_proc_pid, pid_cnt, ktop->top_loading_pid);

    if (type == KERNEL_TOP_ACCU) {
#if USE_STATISTICS_STRATEGY_CONTINUOUS_3
        htc_kernel_top_statistics_continuous_3(ktop);
#else /* <Not> USE_STATISTICS_STRATEGY_CONTINUOUS_3 */
        htc_kernel_top_statistics_5_in_10(ktop);
#endif
    }

    memcpy(&ktop->prev_cpustat, &ktop->curr_cpustat, sizeof(struct kernel_cpustat));
}

static void htc_kernel_top_show(struct _htc_kernel_top *ktop, int type)
{
    int top_n_pid = 0, i;
    char ktag[] = "[KTOP]";

    if(type != KERNEL_TOP_ACCU)
        ktag[0]='\0';

    if(!check_top_malloc(ktop)) {
        return;
    }

    /* Print most time consuming processes */
    pr_info("[K]%s%9s%5s%15s%21s\n", ktag,
            "CPU Usage", "PID", "Name", "Time(ns)");
    for (i = 0; i < NUM_BUSY_THREAD_CHECK; i++) {
        if (ktop->cpustat_time > 0) {
            top_n_pid = ktop->top_loading_pid[i];
            pr_info("[K]%s%7llu%%%6d%20s%16llu", ktag,
                    (u64)(ktop->curr_proc_delta[top_n_pid] * 100 / ktop->cpustat_time),
                    top_n_pid,
                    get_proc_name_start(ktop->local_task_proc_name_array_start, top_n_pid),
                    ktop->curr_proc_delta[top_n_pid]);
        }

    }
    memset(ktop->curr_proc_delta, 0, sizeof(u64) * MAX_PID);
    memset(ktop->local_task_proc_name_array_start, '\0', sizeof(char) * MAX_PROC_NAME_LEN * MAX_PID);
    memset(ktop->curr_proc_pid, 0, sizeof(int) * MAX_PID);
}

static void htc_show_sensor_temp(void)
{
	int i;
	int ret = 0;
	int temp = 0;
	char *sensors[] = {"pm8150b-wp-therm", "wp-therm", "xo-therm",
			"skin-therm", "camera-flash-therm", "skin-msm-therm",
			"pa-therm1", "pa-therm2", "conn-therm",
			"battery", "bms", "11ad-therm",
			"wigig-bb-therm", "wigig-rf-therm", "modem1-pa0-usr",
			"modem1-pa1-usr", "modem1-pa2-usr"};
	struct thermal_zone_device *tzd = NULL;
	char piece[32];
	char output[512];

	static int modem_5g_status_prev = MODEM_5G_UNKNOWN_STAT;
	int modem_5g_status_cur;
	// Please refer to subsystem_restart.c : enum subsys_state.
	struct subsys_device *modem_5g_dev;

	modem_5g_dev = find_subsys_device(MODEM_5G_SOC_NAME);
	if (modem_5g_dev != NULL) {
		modem_5g_status_cur = subsys_get_state(modem_5g_dev);
		if (modem_5g_status_prev != modem_5g_status_cur) {
			modem_5g_status_prev = modem_5g_status_cur;
			pr_info("%s: 5G modem state change to %s \n", __func__,
				modem_5g_status_cur == MODEM_5G_ONLINE ? "ONLINE" : "OFFLINE");
		}
	} else {
		pr_info("%s: 5G modem device is not found. \n", __func__);
		modem_5g_status_prev = MODEM_5G_UNKNOWN_STAT;
		modem_5g_status_cur = MODEM_5G_UNKNOWN_STAT;
	}

	memset(output, 0, sizeof(output));
	for (i = 0; i < ARRAY_SIZE(sensors); i++) {
		if (!strncmp(sensors[i], "modem1", strlen("modem1")) &&
		    modem_5g_status_cur != MODEM_5G_ONLINE)
			continue;

		tzd = thermal_zone_get_zone_by_name(sensors[i]);
		if (IS_ERR_OR_NULL(tzd) || !tzd->ops->get_temp) {
			pr_info("%s: sensor(%s) is not ready to read temp\n", __func__, sensors[i]);
			continue;
		}

		ret = tzd->ops->get_temp(tzd, &temp);
		if (ret != 0) {
			pr_info("%s: (%s) thermal_zone_get_temp() failed tzd->id=%d, ret=%d\n",
				__func__, tzd->type, tzd->id, ret);
			continue;
		}

		memset(piece, 0, sizeof(piece));
		//log format: (60,emmc-therm-adc,28165)
		snprintf(piece, sizeof(piece), "%s(%d,%s,%d)",
			strlen(output)>0 ? "," : "", tzd->id, tzd->type, temp);
		safe_strcat(output, piece);
	}
	//log format:
	//[K] sensor_temp: (60,emmc-therm-adc,28165),(59,quiet-therm-adc,26880),...
	k_pr_embedded("[K] sensor_temp: %s\n", output);
}

extern void htc_print_pon_boot_reason(void);
extern void htc_lmh_stat_show(void);
extern void htc_lmh_stat_clear(void);

extern void dump_vm_events_counter(void);
extern void htc_show_system_stats(void);
extern void htc_show_ss_sleep_info_v2(void);
extern void htc_fan_stats(void);

static void htc_pm_monitor_work_func(struct work_struct *work) {
    struct _htc_kernel_top *ktop =
            container_of(work, struct _htc_kernel_top, dwork.work);
    struct timespec ts;
    struct rtc_time tm;

    if (!htc_pm_monitor_wq) {
        pr_info("[K] htc_pm_monitor_wq is unavaliable.\n");
        return;
    }
    getnstimeofday(&ts);
    rtc_time_to_tm(ts.tv_sec - (sys_tz.tz_minuteswest * 60), &tm);
    pr_info("[K][PM] hTC PM Statistic start (%02d-%02d %02d:%02d:%02d)\n",
            tm.tm_mon + 1, tm.tm_mday, tm.tm_hour, tm.tm_min, tm.tm_sec);
    /*Show the boot reason*/
    htc_print_pon_boot_reason();

    /* Show interesting sensor temperature */
    htc_show_sensor_temp();

    /* Show lmh status*/
    htc_lmh_stat_show();
    htc_lmh_stat_clear();

    /* Show fan stats*/
    htc_fan_stats();

    /* Show interrupt status */
    htc_show_interrupts();

    /* Show idle stats */
    htc_idle_stat_clear();
    htc_idle_stat_show();

    htc_show_system_stats();
    htc_show_ss_sleep_info_v2();

    /* Show wakeup source */
    htc_print_active_wakeup_sources();

    queue_delayed_work(htc_pm_monitor_wq, &ktop->dwork,
            msecs_to_jiffies(msm_htc_util_delay_time));
    htc_kernel_top_cal(ktop, KERNEL_TOP);

    htc_kernel_top_show(ktop, KERNEL_TOP);
    dump_vm_events_counter();
    pr_info("[K][PM] hTC PM Statistic done\n");
}

static void htc_kernel_top_accumulation_monitor_work_func(struct work_struct *work) {
    struct _htc_kernel_top *ktop =
            container_of(work, struct _htc_kernel_top, dwork.work);
    struct timespec ts;
    struct rtc_time tm;

    if (htc_kernel_top_monitor_wq == NULL){
        if (pm_monitor_enabled)
            printk( "[K] hTc Kernel Top statistic is NULL.\n");
        return;
    }

    getnstimeofday(&ts);
    rtc_time_to_tm(ts.tv_sec - (sys_tz.tz_minuteswest * 60), &tm);
    if (pm_monitor_enabled)
        printk("[K][KTOP] hTC Kernel Top Statistic start (%02d-%02d %02d:%02d:%02d) \n",
                tm.tm_mon + 1, tm.tm_mday, tm.tm_hour, tm.tm_min, tm.tm_sec);

    queue_delayed_work(htc_kernel_top_monitor_wq, &ktop->dwork,
            msecs_to_jiffies(msm_htc_util_top_delay_time));
    htc_kernel_top_cal(ktop, KERNEL_TOP_ACCU);
    write_ktop_fd(ktop);
    htc_kernel_top_show(ktop, KERNEL_TOP_ACCU);

    if (pm_monitor_enabled)
        printk("[K][KTOP] hTC Kernel Top Statistic done\n");
}

void htc_local_init_top_array_members(struct _htc_kernel_top *top) {
    if(!top) {
        pr_err("[K] top is NULL, please alloc it before alloc members.\n");
        return;
    }

    memset(top->prev_proc_stat, 0, sizeof(u64) * MAX_PID);
    memset(top->curr_proc_delta, 0, sizeof(u64) * MAX_PID);
    memset(top->local_task_proc_name_array_start, '\0', sizeof(char) * MAX_PROC_NAME_LEN * MAX_PID);
    memset(top->curr_proc_pid, 0, sizeof(int) * MAX_PID);
    memset(top->cpu_time, 0, sizeof(long) * MAX_PID);
}

void htc_local_vmalloc_top_array_members(struct _htc_kernel_top *top) {
    if(!top) {
        pr_err("[K] top is NULL, please alloc it before alloc members.\n");
        return;
    }
    top->prev_proc_stat = vmalloc(sizeof(u64) * MAX_PID);
    top->curr_proc_delta = vmalloc(sizeof(u64) * MAX_PID);
    top->local_task_proc_name_array_start = vmalloc(sizeof(char) * MAX_PROC_NAME_LEN * MAX_PID);
    top->curr_proc_pid = vmalloc(sizeof(int) * MAX_PID);
    top->cpu_time = vmalloc(sizeof(u64) * MAX_PID);
}

void htc_monitor_init(void) {
    struct _htc_kernel_top *htc_kernel_top, *htc_kernel_top_accu;

    if (!strcmp(htc_get_bootmode(), "offmode_charging"))
        return;

    if (pm_monitor_enabled) {
        if (htc_pm_monitor_wq == NULL)
            /* Create private workqueue */
            htc_pm_monitor_wq = create_workqueue("htc_pm_monitor_wq");

        if (!htc_pm_monitor_wq)
            return;

        pr_info("[K] Success to create htc_pm_monitor_wq (0x%p).\n",
                htc_pm_monitor_wq);
        htc_kernel_top = vmalloc(sizeof(struct _htc_kernel_top));

        if(!htc_kernel_top) {
            flush_workqueue(htc_pm_monitor_wq);
            destroy_workqueue(htc_pm_monitor_wq);
        } else {
            spin_lock_init(&htc_kernel_top->lock);

            htc_local_vmalloc_top_array_members(htc_kernel_top);

            if(check_top_malloc(htc_kernel_top)) {
                htc_local_init_top_array_members(htc_kernel_top);

                get_all_cpustat(htc_kernel_top);

                INIT_DELAYED_WORK(&htc_kernel_top->dwork, htc_pm_monitor_work_func);
                queue_delayed_work(htc_pm_monitor_wq, &htc_kernel_top->dwork,
                        msecs_to_jiffies(msm_htc_util_delay_time));
            }
        }
    }

    if (htc_kernel_top_monitor_wq == NULL) {
        /* Create private workqueue... */
        htc_kernel_top_monitor_wq = create_workqueue("htc_kernel_top_monitor_wq");
        printk( "[K][KTOP] Create HTC private workqueue(0x%p)...\n",
                htc_kernel_top_monitor_wq);
    }

    if (!htc_kernel_top_monitor_wq)
        return;

    printk( "[K][KTOP] Success to create htc_kernel_top_monitor_wq (0x%p).\n",
            htc_kernel_top_monitor_wq);
#if USE_STATISTICS_STRATEGY_CONTINUOUS_3
    clear_current_pid_found_array();
    clear_process_monitor_array(&process_monitor_continuous_3_array[0],
            SIZE_OF_PROCESS_MONITOR_CONTINUOUS_3_ARRAY);
#else /* <Not> USE_STATISTICS_STRATEGY_CONTINUOUS_3 */
    clear_process_monitor_array(&process_monitor_5_in_10_array[0],
            SIZE_OF_PROCESS_MONITOR_5_IN_10_ARRAY);
#endif /* USE_STATISTICS_STRATEGY_CONTINUOUS_3 */

    htc_kernel_top_accu = vmalloc(sizeof(struct _htc_kernel_top));

    if(!htc_kernel_top_accu) {
        flush_workqueue(htc_kernel_top_monitor_wq);
        destroy_workqueue(htc_kernel_top_monitor_wq);
    } else {
        spin_lock_init(&htc_kernel_top_accu->lock);

        htc_local_vmalloc_top_array_members(htc_kernel_top_accu);

        if(check_top_malloc(htc_kernel_top_accu)) {
            htc_local_init_top_array_members(htc_kernel_top_accu);

            get_all_cpustat(htc_kernel_top_accu);

            INIT_DELAYED_WORK(&htc_kernel_top_accu->dwork,
                    htc_kernel_top_accumulation_monitor_work_func);
            queue_delayed_work(htc_kernel_top_monitor_wq, &htc_kernel_top_accu->dwork,
                    msecs_to_jiffies(msm_htc_util_top_delay_time));
        }
    }
}

static int __init htc_cpu_monitor_init(void) {
    htc_monitor_init();
    return 0;
}
late_initcall(htc_cpu_monitor_init);
