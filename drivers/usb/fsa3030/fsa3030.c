/*
*
* Copyright (C) 2008-2009 HTC Corporation.
*
* This software is licensed under the terms of the GNU General Public
* License version 2, as published by the Free Software Foundation, and
* may be copied, distributed, and modified under those terms.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
*/

#include <linux/init.h>
#include <linux/fs.h>
#include <linux/platform_device.h>
#include <linux/module.h>
#include <linux/mod_devicetable.h>
#include <linux/kernel.h>

#include <linux/of_platform.h>
#include <linux/of.h>
#include <linux/pinctrl/pinctrl.h>

#include <linux/soc/qcom/fsa3030.h>
#include <linux/htc_flags.h>

enum fsa3030_prv_function {
	FSA3030_USBC_PRV_ORIENTATION_CC1,
	FSA3030_USBC_PRV_ORIENTATION_CC2,
	FSA3030_USBC_PRV_UART,
	FSA3030_USBC_PRV_HI_Z,
	FSA3030_PRV_EVENT_MAX,
};

struct fsa3030_platform_data{
	struct pinctrl *pinctrl;
	struct pinctrl_state *pinctrl_Hi_Z;
	struct pinctrl_state *pinctrl_ori_cc1;
	struct pinctrl_state *pinctrl_ori_cc2;
	struct pinctrl_state *pinctrl_uart;
	const char *name;
	int uart_enabled;
};

BLOCKING_NOTIFIER_HEAD(fsa3030_notifier_list);

int fsa3030_switch_event(struct device_node *node, enum fsa3030_function event)
{
	int ret = -1;
	struct platform_device *ppdev = NULL;
	struct device *pdev = NULL;
	struct fsa3030_platform_data *pdata = NULL;
	enum fsa3030_function switch_event = event;
	enum fsa3030_prv_function switch_prv_event = FSA3030_PRV_EVENT_MAX;

	USB_SWITCH_LOGI("%s\n", __func__);


	ppdev = of_find_device_by_node(node);
	if(!ppdev)
		return -ENOMEM;

	pdev = &ppdev->dev;
	if(!pdev){
		USB_SWITCH_LOGE("no pdev\n");
		return -ENOMEM;
	}

	pdata = dev_get_platdata(pdev);
	if(!pdata){
		USB_SWITCH_LOGE("no pdata\n");
		return -ENOMEM;
	}
	/*Check uart is enabled or not*/
	if (switch_event == FSA3030_USBC_DEFAULT){
		switch_prv_event = (pdata->uart_enabled) ? FSA3030_USBC_PRV_UART : FSA3030_USBC_PRV_HI_Z;
		USB_SWITCH_LOGI("uart_enable:%d, switch_event:%d\n", pdata->uart_enabled, switch_event);
	}
	else if(switch_event < FSA3030_USBC_DEFAULT)
		switch_prv_event = (enum fsa3030_prv_function)switch_event;

	switch (switch_prv_event) {
		case FSA3030_USBC_PRV_UART:
			USB_SWITCH_LOGI("fsa3030 switch to FSA3030_USBC_UART mode");
			ret = pinctrl_select_state(pdata->pinctrl, pdata->pinctrl_uart);
			if (ret < 0){
				USB_SWITCH_LOGE("set fsa3030_uart fail\n");
				break;
			}

			break;

		case FSA3030_USBC_PRV_ORIENTATION_CC1:
			USB_SWITCH_LOGI("fsa3030 switch to FSA3030_USBC_ORIENTATION_CC1 mode");
			ret = pinctrl_select_state(pdata->pinctrl, pdata->pinctrl_ori_cc1);
			if (ret < 0){
				USB_SWITCH_LOGE("set fsa3030_cc1 fail\n");
				break;
			}

			break;

		case FSA3030_USBC_PRV_ORIENTATION_CC2:
			USB_SWITCH_LOGI("fsa3030 switch to FSA3030_USBC_ORIENTATION_CC2 mode");
			ret = pinctrl_select_state(pdata->pinctrl, pdata->pinctrl_ori_cc2);
			if (ret < 0){
				USB_SWITCH_LOGE("set fsa3030_cc2 fail\n");
				break;
			}

			break;

		case FSA3030_USBC_PRV_HI_Z:
			USB_SWITCH_LOGI("fsa3030 switch to FSA3030_USBC_Hi_Z mode");
			ret = pinctrl_select_state(pdata->pinctrl, pdata->pinctrl_Hi_Z);
			if (ret < 0){
				USB_SWITCH_LOGE("set fsa3030_Hi_Z fail\n");
				break;
			}

			break;

		default:
			USB_SWITCH_LOGE("incorrect mode switch\n");
			break;
	}
	USB_SWITCH_LOGI("notifier publisher\n");
	//blocking_notifier_call_chain(&fsa3030_notifier_list, (unsigned long)event, &ret);

	return ret;
}
EXPORT_SYMBOL(fsa3030_switch_event);

int fsa3030_reg_notifier(struct notifier_block *nb, struct device_node *node)
{
	int ret = -1;
	USB_SWITCH_LOGI("%s\n", __func__);

	ret = blocking_notifier_chain_register(&fsa3030_notifier_list, nb);
	if(ret)
		USB_SWITCH_LOGE("register fsa3030_notifier_list fail\n");

	return ret;
}
EXPORT_SYMBOL(fsa3030_reg_notifier);

int fsa3030_unreg_notifier(struct notifier_block *nb, struct device_node *node)
{
	int ret = -1;
	USB_SWITCH_LOGI("%s\n", __func__);

	ret = blocking_notifier_chain_unregister(&fsa3030_notifier_list, nb);

	if(ret)
		USB_SWITCH_LOGE("UNregister fsa3030_notifier_list fail\n");

	return ret;
}
EXPORT_SYMBOL(fsa3030_unreg_notifier);

/* Add attribute for switch event  */

static ssize_t fsa3030_switch_event_show(struct device *dev, struct device_attribute *attr,
		char *buf)
{
	struct fsa3030_platform_data *pdata = dev_get_platdata(dev);

	if(!pdata)
			return -ENOMEM;

	return snprintf(buf, PAGE_SIZE, "%d\n", pdata->uart_enabled);
}

static ssize_t fsa3030_switch_event_store(struct device *dev, struct device_attribute *attr,
				const char *buf, size_t size)
{
	struct fsa3030_platform_data *pdata = dev_get_platdata(dev);
	int val = 0;

	if (!pdata)
		return -ENOMEM;

	if (sscanf(buf, "%d\n", &val) != 1)
		return -EINVAL;

	pdata->uart_enabled = !!val;

	USB_SWITCH_LOGI("uart_enable:%d\n", pdata->uart_enabled);
	return size;
}

static DEVICE_ATTR_RW(fsa3030_switch_event);

/*
 * Translate OpenFirmware node properties into platform_data
 */

static int fsa3030_get_devtree_pdata(struct device *dev)
{
	struct device_node *node;
	struct fsa3030_platform_data *pdata;
	struct pinctrl *pinctrl;
	struct pinctrl_state *pinctrl_Hi_Z;
	struct pinctrl_state *pinctrl_ori_cc1;
	struct pinctrl_state *pinctrl_ori_cc2;
	struct pinctrl_state *pinctrl_uart;
	int ret = -1;
	int uart_enabled = 0;

	node = dev->of_node;
	if (!node){
		ret = -ENODEV;
		goto nodev;
	}

	pdata = devm_kzalloc(dev, sizeof(*pdata), GFP_KERNEL);
	if (!pdata){
		ret = -ENOMEM;
		goto nomem;
	}

	dev->platform_data = (void*)pdata;

	ret = device_create_file(dev, &dev_attr_fsa3030_switch_event);
	if (ret){
		USB_SWITCH_LOGE("create attr file fail\n");
		goto put_pinctrl;
	}

	of_property_read_string(node, "label", &pdata->name);

	/*get pinctrl struct*/
	pinctrl = devm_pinctrl_get(dev);
	if (IS_ERR(pinctrl)) {
		USB_SWITCH_LOGE("fail to get pinctrl");
		goto put_pinctrl;
	}

	/*look up fsa3030 pinctrl status and set sel0:1 and sel1:1 in Hi_Z status*/
	pinctrl_Hi_Z = pinctrl_lookup_state(pinctrl, "usb_hph_fsa3030_Hi_Z");
	if (IS_ERR(pinctrl_Hi_Z)) {
		USB_SWITCH_LOGE("fail to get pin usb_hph_fsa3030_Hi_Z pinctrl, err=%ld\n", PTR_ERR(pinctrl_Hi_Z));
		goto put_pinctrl;
	}

	/*look up fsa3030 pinctrl status and set sel0:1 and sel1:0 in orientation cc1 status*/
	pinctrl_ori_cc1 = pinctrl_lookup_state(pinctrl, "usb_hph_fsa3030_ori_cc1");
	if (IS_ERR(pinctrl_ori_cc1)) {
		USB_SWITCH_LOGE("fail to get pin usb_hph_fsa3030_ori_cc1 pinctrl, err=%ld\n", PTR_ERR(pinctrl_ori_cc1));
		goto put_pinctrl;
	}

	/*look up fsa3030 pinctrl status and set sel0:0 and sel1:1 in orientation cc2 status*/
	pinctrl_ori_cc2 = pinctrl_lookup_state(pinctrl, "usb_hph_fsa3030_ori_cc2");
	if (IS_ERR(pinctrl_ori_cc2)) {
		USB_SWITCH_LOGE("fail to get pin usb_hph_fsa3030_sel1_high pinctrl, err=%ld\n", PTR_ERR(pinctrl_ori_cc2));
		goto put_pinctrl;
	}

	/*look up fsa3030 pinctrl status and set sel0:0 and sel1:0 in uart mode*/
	pinctrl_uart = pinctrl_lookup_state(pinctrl, "usb_hph_fsa3030_uart");
	if (IS_ERR(pinctrl_uart)) {
		USB_SWITCH_LOGE("fail to get pin usb_hph_fsa3030_dbg_uart pinctrl, err=%ld\n", PTR_ERR(pinctrl_uart));
		goto put_pinctrl;
	}

	/*If the kernel flag is set 6 2(enable uart debug, always keep the swtich status in debug uart)*/
	uart_enabled = !!(get_kernel_flag() & KERNEL_FLAG_SERIAL_HSL_ENABLE);

	if (uart_enabled){
		/*set in dbg uart as default.*/
		ret = pinctrl_select_state(pinctrl, pinctrl_uart);
		if (ret < 0){
			USB_SWITCH_LOGE("fail to init fsa3030 to uart mode\n");
			goto put_pinctrl;
		} else
			USB_SWITCH_LOGI("init fsa3030 to uart successful\n");
	}else{
		/*set in Hi-Z as default.*/
		ret = pinctrl_select_state(pinctrl, pinctrl_Hi_Z);
		if (ret < 0){
			USB_SWITCH_LOGE("fail to init fsa3030 to Hi_Z mode\n");
			goto put_pinctrl;
		} else
			USB_SWITCH_LOGI("init fsa3030 to Hi_Z successful\n");
	}

	pdata->pinctrl = pinctrl;
	pdata->pinctrl_Hi_Z = pinctrl_Hi_Z;
	pdata->pinctrl_ori_cc1 = pinctrl_ori_cc1;
	pdata->pinctrl_ori_cc2 = pinctrl_ori_cc2;
	pdata->pinctrl_uart = pinctrl_uart;
	pdata->uart_enabled = uart_enabled;

	return 0;

put_pinctrl:
	devm_kfree(dev, pdata);
nomem:
nodev:

	return ret;
}

static int fsa3030_probe(struct platform_device *pdev)
{
	struct device *dev = NULL;
	struct fsa3030_platform_data *pdata = NULL;
	int res;
	USB_SWITCH_LOGI("%s +++\n", __func__);

	dev = &pdev->dev;
	pdata = dev_get_platdata(dev);

	if (!pdata){
		res = fsa3030_get_devtree_pdata(dev);

		if (res){
			USB_SWITCH_LOGE("pdata fail\n");
			return -ENOMEM;
		}
	}

	USB_SWITCH_LOGI("%s ---\n", __func__);

	return 0;
}

//add for compatible
static struct of_device_id fsa3030_of_match[] = {
		{ .compatible = "fsa3030", },
		{ },
};
MODULE_DEVICE_TABLE(of, fsa3030_of_match);

static struct platform_driver fsa3030_device_driver = {
		.probe	  = fsa3030_probe,
//	  .remove	 = fsa3030_remove,
		.driver	 = {
			.name   = "fsa3030",
//		  .pm = &fsa3030_pm_ops,
			.of_match_table = of_match_ptr(fsa3030_of_match),
		},
};

static int __init fsa3030_init(void)
{
	return platform_driver_register(&fsa3030_device_driver);
}

static void __exit fsa3030_exit(void)
{
	platform_driver_unregister(&fsa3030_device_driver);
}

late_initcall(fsa3030_init);
module_exit(fsa3030_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("USB Switch FSA3030");

