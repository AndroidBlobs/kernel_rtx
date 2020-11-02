#ifndef _FSA3030_H
#define _FSA3030_H

#define USB_SWITCH_LOGD(fmt, args...) pr_debug("[fsa3030_USB_SWITCH] "fmt, ##args)
#define USB_SWITCH_LOGI(fmt, args...) pr_info("[fsa3030_USB_SWITCH] "fmt, ##args)
#define USB_SWITCH_LOGE(fmt, args...) pr_err("[fsa3030_USB_SWITCH][ERR] "fmt, ##args)

#include <linux/of.h>
#include <linux/notifier.h>

enum fsa3030_function {
	FSA3030_USBC_ORIENTATION_CC1,
	FSA3030_USBC_ORIENTATION_CC2,
	FSA3030_USBC_DEFAULT,
	FSA3030_EVENT_MAX,
};

int fsa3030_switch_event(struct device_node *node, enum fsa3030_function event);
int fsa3030_reg_notifier(struct notifier_block *nb, struct device_node *node);
int fsa3030_unreg_notifier(struct notifier_block *nb, struct device_node *node);


#endif
