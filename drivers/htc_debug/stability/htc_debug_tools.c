/* Copyright (c) 2013, HTC Corporation. All rights reserved.
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

#include <linux/htc_debug_tools.h>
#include <linux/kernel_stat.h>
#include <linux/sched/clock.h>
#include <linux/sched/debug.h>
#include <linux/nmi.h>
#include <linux/delay.h>

#if defined(CONFIG_HTC_DEBUG_WATCHDOG)

#define PET_CHECK_THRESHOLD    12
#define NSEC_PER_THRESHOLD     PET_CHECK_THRESHOLD * NSEC_PER_SEC
static unsigned long long pet_check_counter = 0;
static unsigned long long last_pet_check;
static unsigned long long htc_debug_watchdog_last_pet;


static cpumask_t backtrace_mask;
static DEFINE_RAW_SPINLOCK(backtrace_lock);

/* "in progress" flag of arch_trigger_all_cpu_backtrace */
static unsigned long backtrace_flag;

/*
 * ipi_cpu_backtrace - handle IPI from smp_send_all_cpu_backtrace()
 */
static void ipi_cpu_backtrace(void *dummy)
{
	unsigned int cpu = smp_processor_id();

	if (cpumask_test_cpu(cpu, &backtrace_mask)) {

		raw_spin_lock(&backtrace_lock);

		pr_warn("\nIPI backtrace for cpu %d\n", cpu);
		dump_stack();

		raw_spin_unlock(&backtrace_lock);

		cpumask_clear_cpu(cpu, &backtrace_mask);
	}
}

static void smp_send_all_cpu_backtrace(unsigned int backtrace_timeout)
{
	unsigned int this_cpu = smp_processor_id();
	int i;

	if (test_and_set_bit(0, &backtrace_flag))
		/*
		 * If there is already a trigger_all_cpu_backtrace() in progress
		 * (backtrace_flag == 1), don't output double cpu dump infos.
		 */
		return;

	cpumask_copy(&backtrace_mask, cpu_online_mask);
	cpumask_clear_cpu(this_cpu, &backtrace_mask);

	pr_info("\nBacktrace for cpu %d (current):\n", this_cpu);
	dump_stack();

	pr_info("\nsending IPI to all other CPUs:\n");
	if (!cpumask_empty(&backtrace_mask)) {
		//smp_cross_call(&backtrace_mask, IPI_CPU_BACKTRACE);
		smp_call_function( ipi_cpu_backtrace, NULL, 0);
	}

	/* Wait for up to 10 seconds for all other CPUs to do the backtrace */
	for (i = 0; i < backtrace_timeout * 1000; i++)
	{
		if (cpumask_empty(&backtrace_mask))
			break;
		mdelay(1);
	}

	if(i == backtrace_timeout)
		pr_info( " dump cpu backtrace timeout \n");

	clear_bit(0, &backtrace_flag);
	smp_mb__after_atomic();
}

void arch_trigger_different_cpu_backtrace_dump_timeout(unsigned int time_out)
{
	pr_info(" dump cpu backtrace with timeout %u sec \n", time_out);
	smp_send_all_cpu_backtrace(time_out);
}

void htc_debug_watchdog_check_pet(unsigned long long timestamp)
{
	if (!htc_debug_watchdog_enabled() || !htc_debug_watchdog_last_pet)
		return;

	if (timestamp - htc_debug_watchdog_last_pet > (unsigned long long)NSEC_PER_THRESHOLD) {
		if (timestamp - last_pet_check > (unsigned long long)NSEC_PER_SEC*2) {
			last_pet_check = timestamp;
			pet_check_counter = timestamp - htc_debug_watchdog_last_pet;
			do_div(pet_check_counter, NSEC_PER_SEC);

			pr_info("\n%s: MSM watchdog was blocked for more than %d seconds!\n", __func__,(unsigned int)pet_check_counter);
			pr_info("%s: Prepare to dump stack...\n", __func__);
			arch_trigger_different_cpu_backtrace_dump_timeout(2);
#if defined(CONFIG_HTC_DEBUG_WORKQUEUE)
			pr_info("%s: Prepare to dump pending works on global workqueue...\n", __func__);
			workqueue_show_pending_work();
#endif /* CONFIG_HTC_DEBUG_WORKQUEUE */
			pr_info("\n ### Show Blocked State ###\n");
			show_state_filter(TASK_UNINTERRUPTIBLE);
		}
	}
}

void htc_debug_watchdog_update_last_pet(unsigned long long last_pet)
{
	htc_debug_watchdog_last_pet = last_pet;
}

/* TODO: support this funciton with CONFIG_SPARSE_IRQ */
#if !defined(CONFIG_SPARSE_IRQ)
static unsigned int last_irqs[NR_IRQS];
void htc_debug_watchdog_dump_irqs(unsigned int dump)
{
	int n;
	if (dump) {
		pr_debug("\nWatchdog dump irqs:\n");
		pr_debug("irqnr       total  since-last   status  name\n");
	}
	for (n = 1; n < NR_IRQS; n++) {
		struct irqaction *act = irq_desc[n].action;
		if (!act && !kstat_irqs(n))
			continue;
		if (dump) {
			pr_debug("%5d: %10u %11u %8x  %s\n", n, kstat_irqs(n), kstat_irqs(n) - last_irqs[n], irq_desc[n].status_use_accessors, (act && act->name) ? act->name : "???");
		}
		last_irqs[n] = kstat_irqs(n);
	}
}
#endif /* !defined(CONFIG_SPARSE_IRQ) */

#endif /* CONFIG_HTC_DEBUG_WATCHDOG */

/* n.b.:
 * 1. sched_clock is not irq safe
 * 2. 32 bit: overflows every 4,294,967,296 msecs
 */
unsigned long htc_debug_get_sched_clock_ms(void)
{
	unsigned long long timestamp;
	timestamp = sched_clock();
	do_div(timestamp, NSEC_PER_MSEC);
	return ((unsigned long) timestamp);
}
