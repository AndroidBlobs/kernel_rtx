#ifndef _MSM_KGSL_H
#define _MSM_KGSL_H

#include <uapi/linux/msm_kgsl.h>

/* Limits mitigations APIs */
void *kgsl_pwr_limits_add(enum kgsl_deviceid id);
void kgsl_pwr_limits_del(void *limit);
int kgsl_pwr_limits_set_freq(void *limit, unsigned int freq);
void kgsl_pwr_limits_set_default(void *limit);
unsigned int kgsl_pwr_limits_get_freq(enum kgsl_deviceid id);

/*
 * kgsl_get_alloc_size - acquire memory size allocated in kernel space used by kgsl
 *
 * Returns allocated memory size and prints kgsl processes meminfo in kernel log
 */
#ifdef CONFIG_QCOM_KGSL
unsigned int kgsl_get_alloc_size(void);
#else
static inline unsigned int kgsl_get_alloc_size(void)
{
	return 0;
}
#endif

#endif /* _MSM_KGSL_H */
