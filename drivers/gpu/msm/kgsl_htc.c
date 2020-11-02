#include "adreno.h"

/* use schedule_work to print each process memory usage
 * to prevent dead lock by process_mutex
 */
static void kgsl_printmem_fn(struct work_struct *work)
{
	struct kgsl_process_private *private = NULL;

	pr_info("kgsl: kgsl_driver.stats page_alloc = %ld, vmalloc = %ld\n",
		atomic_long_read(&kgsl_driver.stats.page_alloc), atomic_long_read(&kgsl_driver.stats.vmalloc));

	mutex_lock(&kgsl_driver.process_mutex);
	list_for_each_entry(private, &kgsl_driver.process_list, list) {
		if (!private)
			continue;

		pr_info("kgsl: proc %5d alloc: %8ld bytes / peek: %8ld bytes\n", private->pid,
			private->stats[KGSL_MEM_ENTRY_KERNEL].cur, private->stats[KGSL_MEM_ENTRY_KERNEL].max);
	}
	mutex_unlock(&kgsl_driver.process_mutex);
}

static DECLARE_WORK(kgsl_printmem_work, kgsl_printmem_fn);

/* API for meminfo to query total page size allocaed by kgsl
 */
unsigned int kgsl_get_alloc_size()
{
	static DEFINE_RATELIMIT_STATE(_rs, 30 * HZ, 1);

	if (__ratelimit(&_rs)) {
		schedule_work(&kgsl_printmem_work);
	}

	return atomic_long_read(&kgsl_driver.stats.page_alloc);
}
