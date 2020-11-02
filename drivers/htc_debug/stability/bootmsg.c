/*
 * bootmsg.c - keep the first bootmsg_size bytes of kernel log to memory
 *
 * Copyright (C) HTC Corporation.
 */

#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/dma-mapping.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/console.h>
#include <linux/slab.h>
#include <linux/workqueue.h>

#define RELEASE_RETRY_DELAY (msecs_to_jiffies(60 * MSEC_PER_SEC))

static uint64_t bootmsg_buffer_phys = 0x0;
static uint64_t bootmsg_size = 0;

static char*    bootmsg_buffer = NULL;
static int      bootmsg_len    = 0;

static bool should_release_console = false;

static void
bootmsg_console_write(struct console *console, const char *s, unsigned int count)
{
	int len = min(count, (unsigned int) (bootmsg_size - bootmsg_len));
	if (len) {
		memcpy(bootmsg_buffer + bootmsg_len, s, len);
		bootmsg_len += len;
	}

	if (bootmsg_len >= bootmsg_size) {
		console->flags &= ~CON_ENABLED;
		should_release_console = true;
	}
}

static struct console bootmsg_console = {
	.name	= "bootmsg",
	.write	= bootmsg_console_write,
	.flags	= CON_PRINTBUFFER | CON_ENABLED | CON_ANYTIME,
	.index	= -1,
};

static void release_console_work_fn(struct work_struct *work)
{
	if (should_release_console) {
		volatile void __iomem * bootmsg_dst = NULL;

		unregister_console(&bootmsg_console);

		pr_info("bootmsg is disabled (bootmsg_buffer_phys=%08lx bootmsg_size:%08lx)\n",
				bootmsg_buffer_phys, bootmsg_size);

		if (NULL != (bootmsg_dst = ioremap(bootmsg_buffer_phys, bootmsg_size))) {
			memcpy_toio(bootmsg_dst, bootmsg_buffer, bootmsg_size);
			iounmap(bootmsg_dst);
			kfree(bootmsg_buffer);

			pr_info("bootmsg is stored (bootmsg_buffer_phys=%08lx bootmsg_size:%08lx)\n",
				bootmsg_buffer_phys, bootmsg_size);
		}
	} else
		schedule_delayed_work(container_of(work, struct delayed_work, work),
				RELEASE_RETRY_DELAY);
}
static DECLARE_DELAYED_WORK(release_console_work, release_console_work_fn);

static int __init bootmsg_release_console_work_init(void)
{
	schedule_delayed_work(&release_console_work, RELEASE_RETRY_DELAY);
	return 0;
}
late_initcall(bootmsg_release_console_work_init);

static int __init bootmsg_console_init(void)
{
	char *bootmsg_node_name = "htc_bootmsg";
	char *bootmsg_res_name = "bootmsg_res";
	struct device_node *dt_node = NULL;
	int i = 0, ret = 0;
	struct resource r = {0};

	dt_node = of_find_node_by_name(NULL, bootmsg_node_name);
	if(dt_node == NULL) {
		pr_err("%s: bootmsg node <%s> not found in DTB!!\n", __func__,
				bootmsg_node_name);
		return -EINVAL;
	}

	for(i = 0 ; (ret = of_address_to_resource(dt_node, i, &r)) == 0 ; i++) {
		if(!strcmp(bootmsg_res_name, r.name))
			break;
	}

	if (ret) {
		pr_err("%s: couldn't found resource \"%s\" in node %s\n", __func__,
				bootmsg_res_name, bootmsg_node_name);
		goto out;
	}

	bootmsg_size = resource_size(&r);
	bootmsg_buffer_phys = r.start;

	pr_info("bootmsg: phys: 0x%08llx size: 0x%08llx\n",
			bootmsg_buffer_phys, bootmsg_size);

	if ( NULL == (bootmsg_buffer = kzalloc( bootmsg_size, GFP_KERNEL) )) {
		pr_err("%s: memory allocate error!\n", __func__);
		ret = -ENOMEM;
		goto out;
	} else {
		pr_info("bootmsg: initialization ok!\n");
		register_console(&bootmsg_console);
		ret = 0;
	}

out:
	if (dt_node) {
		of_node_put(dt_node);
	}

	return ret;
}

core_initcall(bootmsg_console_init);
