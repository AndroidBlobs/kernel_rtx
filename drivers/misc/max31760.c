/*
 * Maxim Integrated MAX31760 Precision Fan-Speed Controller driver
 * Copyright (C) 2017 HTC Corporation.
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/debugfs.h>
#include <linux/seq_file.h>
#include <linux/regulator/consumer.h>
#include <linux/string.h>
#include <linux/of_gpio.h>
#include <linux/pinctrl/pinctrl.h>

#define I2C_WRITE_RETRY_TIMES       7
#define FAN_I2C_WRITE_BLOCK_SIZE    80

#define CR1_REGISTER   0x00    /* Control Register 1 */
#define CR2_REGISTER   0x01    /* Control Register 2 */
#define 	DFC_BIT   0x01     /* Direct Fan Control */
#define 	STBY_BIT   0x80     /* Standby Mode Enable */
#define CR3_REGISTER   0x02    /* Control Register 3 */
#define FFDC_REGISTER   0x03    /* Fan Fault Duty Cycle */
#define AMR_REGISTER   0x04    /* Alert Mask Register */
#define RHSH_REGISTER   0x06    /* Remote High Set-point MSB */
#define RHSL_REGISTER   0x07    /* Remote High Set-point LSB */
#define LOTSH_REGISTER   0x08    /* Local Overtemperature Set-point MSB */
#define LOTSL_REGISTER   0x09    /* Local Overtemperature Set-point LSB */
#define ROTSH_REGISTER   0x0a    /* Remote Overtemperature Set-point MSB */
#define ROTSL_REGISTER   0x0b    /* Remote Overtemperature Set-point LSB */
#define LHSH_REGISTER   0x0c    /* Local High Set-point MSB */
#define LHSL_REGISTER   0x0d    /* Local High Set-point LSB */
#define TCTH_REGISTER   0x0E    /* TACH Count Threshold Register, MSB */
#define TCTL_REGISTER   0x0F    /* TACH Count Threshold Register, LSB */
#define LUT_REGISTER   0x20    /* 48-Byte Lookup Table (LUT) */
#define PWMR_REGISTER   0x50    /* Direct Duty-Cycle Control Register */
#define PWMV_REGISTER   0x51    /* Current PWM Duty-Cycle Register */
#define TC1H_REGISTER   0x52    /* TACH1 Count Register, MSB */
#define TC1L_REGISTER   0x53    /* TACH1 Count Register, LSB */

#define RTH_REGISTER   0x56    /* Remote Temperature Reading Register, MSB */
#define RTL_REGISTER   0x57    /* Remote Temperature Reading Register, LSB */
#define LTH_REGISTER   0x58    /* Local Temperature Reading Register, MSB */
#define LTL_REGISTER   0x59    /* Local Temperature Reading Register, LSB */
#define EEX_REGISTER   0x5b /* Load EEPROM to RAM; Write RAM to EEPROM */

#define power_enable	1
#define power_disable	0

/* PWM values of Look-up Table */
/* Temperatures range from <=16, +2C for every increment for 48 entries.
          16  18  20  22  24  26  28  30  32  34  36  38  40  42  44  46
          48  50  52  54  56  58  60  62  64  66  68  70  72  74  76  78
          80  82  84  86  88  90  92  94  96  98 100 102 104 106 108 110+ */

/* PWM duty cycle = (0xREG VALUE)*0.39% */
/* ex. 40 %: 0x66 = 102, 102*0.39 = 40%  */

static uint8_t pwm_lut[48] = {	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
				0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
				0x00, 0x66, 0x66, 0x66, 0x66, 0x66, 0x80, 0x80,
				0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80,
				0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80,
				0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80 };

static uint8_t modem_status = 0;
static uint8_t sound_trigger = 0;
static uint8_t noise_limit = 128;
static uint8_t current_pwm = 0;
static void set_noise_limit(struct work_struct *work);
static DECLARE_DELAYED_WORK(work, set_noise_limit);

struct max31760_i2c_platform_data {
    int pwr_ena;
	struct pinctrl *pinctrl;
	struct pinctrl_state *gpio_state_active;
	struct pinctrl_state *gpio_state_sleep;
	struct mutex lock;
};

struct max31760_chip {
	struct max31760_i2c_platform_data *pdata;
	struct i2c_client	*client;
};

static struct max31760_i2c_platform_data *private_max31760_pdata = NULL;
static struct i2c_client *private_max31760_client = NULL;

static int max31760_update_fan_states(bool enable);

static int i2c_write_block(struct i2c_client *client, uint8_t addr,
		uint8_t *data, int length)
{
	int retry;
	uint8_t buf[FAN_I2C_WRITE_BLOCK_SIZE];
	int i;

	struct i2c_msg msg[] = {
		{
			.addr = client->addr,
			.flags = 0,
			.len = length + 1,
			.buf = buf,
		}
	};

	if (length + 1 > FAN_I2C_WRITE_BLOCK_SIZE) {
		dev_err(&client->dev, "[FAN] i2c_write_block length too long\n");
		return -E2BIG;
	}

	buf[0] = addr;
	for (i = 0; i < length; i++)
		buf[i+1] = data[i];

	for (retry = 0; retry < I2C_WRITE_RETRY_TIMES; retry++) {
		if (i2c_transfer(client->adapter, msg, 1) == 1)
			break;
//		dev_err(&client->dev, "[FAN]%s error %d time",__func__,retry+1);
	}
	if (retry >= I2C_WRITE_RETRY_TIMES) {
		dev_err(&client->dev, "[FAN] i2c_write_block retry over %d times\n",
				I2C_WRITE_RETRY_TIMES);
		return -EIO;
	}

	return 0;
}


static int I2C_RxData_2(char *rxData, int length)
{
	uint8_t loop_i;

	struct i2c_msg msgs[] = {
		{
			.addr = private_max31760_client->addr,
			.flags = 0,
			.len = 1,
			.buf = rxData,
		},
		{
			.addr = private_max31760_client->addr,
			.flags = I2C_M_RD,
			.len = length,
			.buf = rxData,
		},
	};

	for (loop_i = 0; loop_i < I2C_WRITE_RETRY_TIMES; loop_i++) {
		if (i2c_transfer(private_max31760_client->adapter, msgs, 2) > 0)
			break;
		//msleep(10);
	}

	if (loop_i >= I2C_WRITE_RETRY_TIMES) {
		printk(KERN_ERR "[FAN] %s retry over %d times\n",
				__func__, I2C_WRITE_RETRY_TIMES);
		return -EIO;
	}

	return 0;
}

static int i2c_read_block(struct i2c_client *client,
		uint8_t cmd, uint8_t *pdata, int length)
{
	char buffer[3] = {0};
	int ret = 0, i;

	if (pdata == NULL)
		return -EFAULT;

	if (length > 2) {
		pr_err("[FAN]%s: length %d> 2: \n", __func__, length);
		return ret;
	}
	buffer[0] = cmd;
	ret = I2C_RxData_2(buffer, length);
	if (ret < 0) {
		pr_err("[FAN]%s: I2C_RxData 0x%X fail \n", __func__, cmd);
		return ret;
	}

	for (i = 0; i < length; i++) {
		*(pdata+i) = buffer[i];
	}
	return ret;
}

static int max31760_registers_update(struct i2c_client *client)
{
	int ret = 0;
	uint8_t data = 0x00;

	data = 0x1B; // 1. PWM Frequency = 25k Hz for 4-wire fan  2. Set Temperature index source to MTI
	ret |= i2c_write_block(client, CR1_REGISTER, &data, 1);

	data = 0x01; // 1. Set ALERTS, FF Mode bit to 0.  2. Direct control mode selected
	ret |= i2c_write_block(client, CR2_REGISTER, &data, 1);

	data = 0x31; // 1. RAMP rate change to Fast (Immediate) 2. Disable TACH2E since only one FAN
	ret |= i2c_write_block(client, CR3_REGISTER, &data, 1);

	data = 0x4C;	// FFDC to PWM 30%
	ret |= i2c_write_block(client, FFDC_REGISTER, &data, 1);

	data = 0x00;	// Direct Duty-Cycle Control Register to PWM 0%
	ret |= i2c_write_block(client, PWMR_REGISTER, &data, 1);

	data = 0x0B;	// Set TACH count Threshold to 1000 RPM
	ret |= i2c_write_block(client, TCTH_REGISTER, &data, 1);
	data = 0xB8;
	ret |= i2c_write_block(client, TCTL_REGISTER, &data, 1);

	/* Set Temperature related index. */
	data = 0x55;
	ret |= i2c_write_block(client, ROTSH_REGISTER, &data, 1);
	data = 0x00;
	ret |= i2c_write_block(client, ROTSL_REGISTER, &data, 1);
	data = 0x55;
	ret |= i2c_write_block(client, LOTSH_REGISTER, &data, 1);
	data = 0x00;
	ret |= i2c_write_block(client, LOTSL_REGISTER, &data, 1);

	data = 0xC2;	// Does not cause FF/FS assert of TACH2
	ret |= i2c_write_block(client, AMR_REGISTER, &data, 1);

	return ret;

}

static int max31760_pinctrl_init(struct i2c_client *client, struct max31760_i2c_platform_data *pdata)
{
	int retval;

	pdata->pinctrl = devm_pinctrl_get(&client->dev);
	if( IS_ERR_OR_NULL(pdata->pinctrl) ){
		dev_err(&client->dev, "%s: Target does not use pinctrl",__func__);
		retval = PTR_ERR(pdata->pinctrl);
		pdata->pinctrl = NULL;
		return retval;
	}

	pdata->gpio_state_active = pinctrl_lookup_state(pdata->pinctrl,"max31760_active");
	if( IS_ERR_OR_NULL(pdata->gpio_state_active)){
		dev_err(&client->dev, "%s: Cannot get pinctrl active state",__func__);
		retval = PTR_ERR(pdata->gpio_state_active);
		pdata->gpio_state_active = NULL;
		return retval;
	}

	pdata->gpio_state_sleep = pinctrl_lookup_state(pdata->pinctrl,"max31760_sleep");
	if( IS_ERR_OR_NULL(pdata->gpio_state_sleep)){
		dev_err(&client->dev, "%s: Cannot get pinctrl sleep state",__func__);
		retval = PTR_ERR(pdata->gpio_state_sleep);
		pdata->gpio_state_sleep = NULL;
		return retval;
	}

	return 0;
}
static int max31760_write_lut(struct i2c_client *client)
{
	int i;
	int err;

	for(i = 0; i < 48; i++){
		err = i2c_write_block(client, (LUT_REGISTER+i), &pwm_lut[i], 1);
		if(err<0)
			return err;
	}

	return 0;
}

static int read_pwm(void)
{
	struct i2c_client *client = private_max31760_client;
	struct max31760_i2c_platform_data *pdata = private_max31760_pdata;
	int ret;
	uint8_t pwm_data = 0;

	if(!client) {
		dev_err(&client->dev, "[FAN] %s cannot identify max31760\n", __func__);
		return -1;
	}
	if(!pdata->pwr_ena){
		dev_err(&client->dev, "[FAN] %s power was turned off\n", __func__);
		return -1;
	}
	ret = i2c_read_block(client, PWMV_REGISTER, &pwm_data, 1);

	return pwm_data;
}

static long read_rpm(void)
{
	struct i2c_client *client = private_max31760_client;
	struct max31760_i2c_platform_data *pdata = private_max31760_pdata;
	int ret;
	u8 msb, lsb;
	u16 word;
	u32 rpm = 0;

	//TACH is measured only when PWM is on, thus ONLY when fan speed is 0, use the value from read_pwm instead
	if (read_pwm() != 0) {
		if (!client) {
			dev_err(&client->dev, "[FAN] %s cannot identify max31760\n", __func__);
			return -1;
		}
		if (!pdata->pwr_ena) {
			dev_err(&client->dev, "[FAN] %s power was turned off\n", __func__);
			return -1;
		}
		ret = i2c_read_block(client, TC1H_REGISTER, &msb, 1);
		if (ret)
			return ret;

		ret = i2c_read_block(client, TC1L_REGISTER, &lsb, 1);
		if (ret)
			return ret;

		word = (((u16)msb << 8) & 0xFF00) | (lsb & 0xFF);
		rpm = 60 * 100000 / (u32)word / 2;
	}
	dev_info(&client->dev, "[FAN] RPM now: %d\n", rpm);
	return rpm;
}

static void set_noise_limit(struct work_struct *dummy)
{
	struct i2c_client *client = private_max31760_client;

	dev_info(&client->dev, "[FAN] %s: Set PWM = %d\n",__func__, noise_limit);
	i2c_write_block(client, PWMR_REGISTER, &noise_limit, 1);
}

static ssize_t get_control_mode(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = private_max31760_client;
	int ret;
	uint8_t current_data = 0;
	uint8_t data = 0;

	struct max31760_i2c_platform_data *pdata = private_max31760_pdata;
	if(!pdata->pwr_ena)
		return scnprintf(buf, PAGE_SIZE, "Max31760 is powered OFF\n");

	if(!client)
		return -EINVAL;
	ret = i2c_read_block(client, CR2_REGISTER, &current_data, 1);
	data  = current_data%2; //DFC: Bit 0

	if (data>0)
		return scnprintf(buf, PAGE_SIZE, "Direct Fan control\n");
	else
		return scnprintf(buf, PAGE_SIZE, "Auto Fan control\n");
}

static ssize_t set_control_mode(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct i2c_client *client = private_max31760_client;
	int ret;
	bool bl_data = 0;
	uint8_t data = 0;
	uint8_t current_data = 0;

	ret = kstrtobool(buf, &bl_data);
	if (ret) {
		dev_err(dev, "%s: Input string is out of range: %d\n", __func__, ret);
		return ret;
	}

	ret = i2c_read_block(client, CR2_REGISTER, &current_data, 1);

	if (bl_data){
		data = current_data | DFC_BIT;
		ret = i2c_write_block(client, CR2_REGISTER, &data, 1);
		if (ret) {
			return ret;
		}
	}
	else {
		data = current_data & ~DFC_BIT;
		ret = i2c_write_block(client, CR2_REGISTER, &data, 1);
		if (ret) {
			return ret;
		}
	}

	return count;
}

static ssize_t get_pwm_control(struct device *dev, struct device_attribute *attr, char *buf)
{
	return scnprintf(buf, PAGE_SIZE, "%d \n", read_pwm());
}

static ssize_t set_pwm_control(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct i2c_client *client = private_max31760_client;
	struct max31760_i2c_platform_data *pdata = private_max31760_pdata;
	int ret;
	uint8_t data = 0;

	mutex_lock(&pdata->lock);
	ret = kstrtou8(buf, 16, &data);
	if (ret) {
		dev_err(dev, "%s: Input string is out of range: %d\n", __func__, ret);
		mutex_unlock(&pdata->lock);
		return ret;
	}

	current_pwm = data;

	if (delayed_work_pending(&work)) {
		dev_info(dev, "[FAN] %s: New PWM is set, cancel previous delayed work\n", __func__);
		cancel_delayed_work(&work);
	}

	if (sound_trigger) {
		dev_info(dev, "[FAN] %s: sound_trigger = %d\n",__func__, sound_trigger);
		if (data > noise_limit) {
			dev_info(dev, "[FAN] %s: Reserved PWM = %d\n", __func__, current_pwm);
			data = noise_limit;
		}
	}

	dev_info(&client->dev, "[FAN] %s: Set PWM = %d\n", __func__, data);

	ret = i2c_write_block(client, PWMR_REGISTER, &data, 1);
	if (ret) {
		mutex_unlock(&pdata->lock);
		return ret;
	}
	mutex_unlock(&pdata->lock);

	return count;
}

static ssize_t get_fan_rpm(struct device *dev, struct device_attribute *attr, char *buf)
{
	long rpm;
	struct max31760_i2c_platform_data *pdata = private_max31760_pdata;
	if(!pdata->pwr_ena)
		return scnprintf(buf, PAGE_SIZE, "Max31760 is powered OFF\n");
	rpm = read_rpm();
	return scnprintf(buf, PAGE_SIZE, "%ld\n", rpm);
}

static ssize_t get_internal_temp(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = private_max31760_client;
	struct max31760_i2c_platform_data *pdata = private_max31760_pdata;
	int ret;
	u8 msb, lsb;
	u16 word;
	int32_t val;

	if(!client)
		return -EINVAL;
	if(!pdata->pwr_ena)
		return scnprintf(buf, PAGE_SIZE, "Max31760 is powered OFF\n");
	ret = i2c_read_block(client, LTH_REGISTER, &msb, 1);
	ret = i2c_read_block(client, LTL_REGISTER, &lsb, 1);

	word = (((u16)msb << 8) & 0xFF00) | (lsb & 0xFF);
	val = (u32)(word & 0xFFE0) * 1000 >> 8;

	return scnprintf(buf, PAGE_SIZE, "%d\n", val);
}

static ssize_t get_remote_temp(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = private_max31760_client;
	struct max31760_i2c_platform_data *pdata = private_max31760_pdata;
	int ret;
	u8 msb, lsb;
	u16 word;
	int32_t val;

	if(!client)
		return -EINVAL;
	if(!pdata->pwr_ena)
		return scnprintf(buf, PAGE_SIZE, "Max31760 is powered OFF\n");

	ret = i2c_read_block(client, RTH_REGISTER, &msb, 1);
	ret = i2c_read_block(client, RTL_REGISTER, &lsb, 1);

	word = (((u16)msb << 8) & 0xFF00) | (lsb & 0xFF);
	val = (u32)(word & 0xFFE0) * 1000 >> 8;

	return scnprintf(buf, PAGE_SIZE, "%d\n", val);
}

static ssize_t get_power_control( struct device *dev, struct device_attribute *attr, char *buf){
	struct max31760_i2c_platform_data *pdata = private_max31760_pdata;
	return scnprintf(buf, PAGE_SIZE, "%d \n", pdata->pwr_ena );

}

static ssize_t set_power_control( struct device *dev ,struct device_attribute *attr,
				const char *buf, size_t count){
	int ret;
	bool bl_data = 0;
	uint8_t data = 0x00;
	struct i2c_client *client = private_max31760_client;
	struct max31760_i2c_platform_data *pdata = private_max31760_pdata;

	mutex_lock(&pdata->lock);
	ret = kstrtobool(buf, &bl_data);
	if (ret) {
		dev_err(dev, "%s: Input string is out of range: %d\n", __func__, ret);
		mutex_unlock(&pdata->lock);
		return ret;
	}

	if (bl_data){
		dev_info(dev, "[FAN] %s power turn on\n",__func__);
		ret = max31760_update_fan_states(power_enable);
		if(ret){
			dev_err(dev, "%s: cannot update fan states ",__func__);
			mutex_unlock(&pdata->lock);
			return ret;
		}

		dev_info(dev, "[FAN] max31760 restart, load blocks from EEPROM\n",__func__);
		data = 0x9F;    // Load blocks from EEPROM
		ret = i2c_write_block(client, EEX_REGISTER, &data, 1);
		if (ret) {
			dev_err(dev, "[FAN] Restart failed to load from EEPROM: %d\n", ret);
			mutex_unlock(&pdata->lock);
			return ret;
		}
	}
	else {
		dev_info(dev, "[FAN] %s power turn off\n",__func__);
		ret = max31760_update_fan_states(power_disable);
		if(ret){
			dev_err(dev, "%s: cannot update fan states ",__func__);
			mutex_unlock(&pdata->lock);
			return ret;
		}
		current_pwm = 0;
	}
	mutex_unlock(&pdata->lock);
	return count;
}

static ssize_t get_modem_status(struct device *dev, struct device_attribute *attr, char *buf)
{
	dev_info(dev, "[FAN] %s: %d", __func__, modem_status);
	return scnprintf(buf, PAGE_SIZE, "%d \n", modem_status);
}

static ssize_t set_modem_status(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int ret;
	bool bl_data = 0;

	ret = kstrtobool(buf, &bl_data);
	if (ret) {
		dev_err(dev, "%s: Input string is out of range: %d\n", __func__, ret);
		return ret;
	}

	if (bl_data)
		modem_status = 1;
	else
		modem_status = 0;

	dev_info(dev, "[FAN] %s: %d", __func__, modem_status);

	return count;
}

static ssize_t get_sound_trigger(struct device *dev, struct device_attribute *attr, char *buf)
{
	dev_info(dev, "[FAN] %s: %d", __func__, sound_trigger);
	return scnprintf(buf, PAGE_SIZE, "%d \n", sound_trigger);
}

static ssize_t set_sound_trigger(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct i2c_client *client = private_max31760_client;
	struct max31760_i2c_platform_data *pdata = private_max31760_pdata;
	int ret;
	bool bl_data = 0;
	uint8_t data = 0x00;
	uint16_t decelerate_time = 1 * HZ;

	mutex_lock(&pdata->lock);
	ret = kstrtobool(buf, &bl_data);
	if (ret) {
		dev_err(dev, "%s: Input string is out of range: %d\n", __func__, ret);
		mutex_unlock(&pdata->lock);
		return ret;
	}

	if (!client) {
		dev_err(&client->dev, "[FAN] %s cannot identify max31760\n", __func__);
		mutex_unlock(&pdata->lock);
		return -EINVAL;
	}

	if (!pdata->pwr_ena) {
		dev_err(&client->dev, "[FAN] %s: Max31760 is powered OFF\n", __func__);
		mutex_unlock(&pdata->lock);
		return -EINVAL;
	}

	if (bl_data) {
		sound_trigger = 1;

		dev_info(dev, "[FAN] %s: %d, current PWM = %d\n", __func__, sound_trigger, current_pwm);

		if (current_pwm > noise_limit) {
			if (delayed_work_pending(&work)) {
				dev_info(dev, "[FAN] %s: New work is set, cancel previous delayed work\n", __func__);
				cancel_delayed_work(&work);
			}

			dev_info(dev, "[FAN] %s: Decelerating to PWM = %d\n", __func__, data);
			ret = i2c_write_block(client, PWMR_REGISTER, &data, 1);
			if (ret) {
				mutex_unlock(&pdata->lock);
				return ret;
			}
			schedule_delayed_work(&work, decelerate_time);	// Wait 1s for fan to decelerate
		}
	}
	else {
		sound_trigger = 0;

		if (delayed_work_pending(&work)) {
			dev_info(dev, "[FAN] %s: New work is set, cancel previous delayed work\n", __func__);
			cancel_delayed_work(&work);
		}

		dev_info(dev, "[FAN] %s: %d, Set PWM = %d\n", __func__, sound_trigger, current_pwm);
		ret = i2c_write_block(client, PWMR_REGISTER, &current_pwm, 1);
		if (ret) {
			mutex_unlock(&pdata->lock);
			return ret;
		}
	}
	mutex_unlock(&pdata->lock);
	return count;
}

#ifdef CONFIG_HTC_POWER_DEBUG
void htc_fan_stats(void)
{
	struct i2c_client *client = private_max31760_client;
	struct max31760_i2c_platform_data *pdata = private_max31760_pdata;
	int ret;
	u8 msb, lsb;
	u16 word;
	u32 rpm = 0;
	int32_t internal_temp, external_temp;

	if (!client) {
		pr_err("[K] can't identify max31760\n");
		return;
	}

	// Check fan pinctrl state
	if (!pdata->pwr_ena) {
		dev_info(&client->dev, "[K] fan: max31760 is powered OFF\n");
		return;
	}

	// RPM
	if (read_pwm() != 0) {
		ret = i2c_read_block(client, TC1H_REGISTER, &msb, 1);
		ret |= i2c_read_block(client, TC1L_REGISTER, &lsb, 1);
		if (!ret) {
			word = (((u16)msb << 8) & 0xFF00) | (lsb & 0xFF);
			rpm = 60 * 100000 / (u32)word / 2;
		}
	}

	// Internal Temperature
	ret = i2c_read_block(client, LTH_REGISTER, &msb, 1);
	ret |= i2c_read_block(client, LTL_REGISTER, &lsb, 1);
	if (!ret) {
		word = (((u16)msb << 8) & 0xFF00) | (lsb & 0xFF);
		internal_temp = (u32)(word & 0xFFE0) * 1000 >> 8;
	} else {
		pr_err("[K] fan: Local temperature i2c_read_block failed\n");
		internal_temp = 0x7FFFFFFF;
	}

	// External Temperature
	ret = i2c_read_block(client, RTH_REGISTER, &msb, 1);
	ret |= i2c_read_block(client, RTL_REGISTER, &lsb, 1);
	if (!ret) {
		word = (((u16)msb << 8) & 0xFF00) | (lsb & 0xFF);
		external_temp = (u32)(word & 0xFFE0) * 1000 >> 8;
	} else {
		pr_err("[K] fan: Remote temperature i2c_read_block failed\n");
		external_temp = 0x7FFFFFFF;
	}

	dev_info(&client->dev, "[K] fan: (RPM,%d), (Int_temp,%d), (Ext_temp,%d)\n",
		   rpm, internal_temp, external_temp);
}
#endif

static struct class *htc_fan_class;
static struct device_attribute htc_fan_attributes[] = {
	__ATTR(control_mode, 0644, get_control_mode, set_control_mode),
	__ATTR(pwm_control, 0664, get_pwm_control, set_pwm_control),
	__ATTR(fan_rpm, 0440, get_fan_rpm, NULL),
	__ATTR(internal_temp, 0440, get_internal_temp, NULL),
	__ATTR(remote_temp, 0440, get_remote_temp, NULL),
	__ATTR(power_control, 0664, get_power_control, set_power_control),
	__ATTR(thermal_flag, 0644, get_modem_status, set_modem_status),
	__ATTR(sound_trigger_flag, 0664, get_sound_trigger, set_sound_trigger),
};

static int max31760_set_attr_files(struct device *dev)
{
	int ret = 0;
	int i;

	htc_fan_class = class_create(THIS_MODULE, "htc_fan");
	if (IS_ERR(htc_fan_class)) {
		ret = PTR_ERR(htc_fan_class);
		pr_err("%s: could not allocate htc_fan_class, ret = %d\n", __func__, ret);
	}

	dev = device_create(htc_fan_class, NULL, 0, "%s", "max31760");
	if (IS_ERR(dev)) {
		ret = PTR_ERR(dev);
		pr_err("%s: could not allocate htc_fan_dev, ret = %d\n", __func__, ret);
	}

	for (i = 0; i < ARRAY_SIZE(htc_fan_attributes); i++) {
		ret = device_create_file(dev, htc_fan_attributes + i);
		if (ret) {
			pr_err("%s: could not allocate htc_fan_attributes, i=%d, ret=%d\n", __func__, i, ret);
		}
	}

	return ret;
}

static int max31760_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct device *dev = &client->dev;
	struct max31760_i2c_platform_data *pdata;
	int ret = 0;
	uint8_t data = 0x00;

	dev_info(dev, "[FAN][PROBE] FAN driver probe +++\n");

	pdata = kzalloc(sizeof(*pdata), GFP_KERNEL);
	if (!pdata) {
		ret = -ENOMEM;
		goto err_exit;
	}
	mutex_init(&pdata->lock);

	dev_dbg(dev, "[FAN][PROBE] max31760_pinctrl_init\n");
	ret = max31760_pinctrl_init(client, pdata);
	if (ret) {
		dev_err(dev, "[FAN][PROBE] failed of max31760_pinctrl_init: %d\n", ret);
		return ret;
	}

	ret = pinctrl_select_state(pdata->pinctrl , pdata->gpio_state_active);
	if(ret){
		dev_err(dev, "[FAN][PROBE] %s: Cannot select pinctrl state gpio active\n", __func__);
		return ret;
	}

	pdata->pwr_ena = power_enable;

	dev_dbg(dev, "[FAN][PROBE] max31760_registers_update\n");
	ret = max31760_registers_update(client);
	if (ret) {
		dev_err(dev, "[FAN][PROBE] failed of max31760_registers_update: %d\n", ret);
		return ret;
	}

	dev_dbg(dev, "[FAN][PROBE] max31760_write_lut\n");
	ret = max31760_write_lut(client);
	if (ret) {
		dev_err(dev, "[FAN][PROBE] failed of max31760_write_lut: %d\n", ret);
		return ret;
	}

	dev_dbg(dev, "[FAN][PROBE] max31760 write blocks to EEPROM\n");
	data = 0x1F;    // Write blocks to EEPROM
	ret = i2c_write_block(client, EEX_REGISTER, &data, 1);
	if (ret) {
		dev_err(dev, "[FAN][PROBE] failed to write EEPROM: %d\n", ret);
		return ret;
	}

	mdelay(550);

	private_max31760_pdata = pdata;
	private_max31760_client = client;

	dev_dbg(dev, "[FAN][PROBE] max31760 set_attr_file\n");
	ret = max31760_set_attr_files(dev);
	if (ret) {
		dev_err(dev, "[FAN][PROBE] failed of max31760_set_attr_files: %d\n", ret);
		return ret;
	}

	ret = pinctrl_select_state(pdata->pinctrl , pdata->gpio_state_sleep);
	if(ret){
		dev_err(dev, "[FAN][PROBE] %s: Cannot select pinctrl state gpio sleep\n", __func__);
		return ret;
	}

	pdata->pwr_ena = power_disable;

	dev_info(dev, "[FAN][PROBE] FAN driver probe ---\n");
	return ret;

	err_exit:
		dev_err(dev, "[FAN][PROBE] FAN driver probe error exit\n");
		return ret;
}

/* Toggle the fan GPIOs and regulators to match enable state. */
static int max31760_update_fan_states(bool enable)
{
	struct max31760_i2c_platform_data *pdata = private_max31760_pdata;
	struct i2c_client *client = private_max31760_client;
	int ret = 0;

	dev_dbg(&client->dev, "[FAN] %s set_state:%d",__func__,enable);

	if ( enable ){
		dev_dbg(&client->dev, "[FAN] %s set_active",__func__);
		ret = pinctrl_select_state(pdata->pinctrl , pdata->gpio_state_active);
		if(ret){
			dev_err(&client->dev, "%s: cannot slect pinctrl active state ",__func__);
			return ret;
		}
		pdata->pwr_ena = power_enable;
	}
	else {
		dev_dbg(&client->dev, "[FAN] %s set_sleep",__func__);
		ret = pinctrl_select_state(pdata->pinctrl , pdata->gpio_state_sleep);
		if(ret){
			dev_err(&client->dev, "%s: cannot select pinctrl sleep state ",__func__);
			return ret;
		}
		pdata->pwr_ena = power_disable;
	}

	return 0;

}

static ssize_t auto_mode(struct device *dev)
{
	struct i2c_client *client = private_max31760_client;
	int ret;
	uint8_t current_data = 0;
	uint8_t data = 0;

	ret = i2c_read_block(client, CR2_REGISTER, &current_data, 1);

	data = current_data & ~DFC_BIT;
	ret = i2c_write_block(client, CR2_REGISTER, &data, 1);
	if (ret) {
		dev_err(dev, "[FAN] %s: Fail to write Control Register 2: %d", __func__, ret);
		return ret;
	}

	return 0;
}

static int max31760_suspend(struct device *dev)
{
	int ret;
	// Check Wi-Fi and BT modem status, if on, then change to auto mode (DFC bit = 0)
	if (modem_status) {
		dev_info(dev, "[FAN] %s - auto mode on\n", __func__);
		ret = max31760_update_fan_states(power_enable);
		if (ret) {
			dev_err(dev, "[FAN] %s: Cannot update fan states: %d\n", __func__, ret);
			return ret;
		}
		ret = auto_mode(dev);
		if (ret) {
			dev_err(dev, "[FAN] %s: Cannot select auto mode: %d", __func__, ret);
			max31760_update_fan_states(power_disable);
			return ret;
		}
	}
	else {
		dev_info(dev, "[FAN] %s - fan power off\n", __func__);
		ret = max31760_update_fan_states(power_disable);
		if (ret) {
			dev_err(dev, "[FAN] %s: Cannot update fan states ", __func__);
			return ret;
		}
	}
	return 0;
}

static int max31760_resume(struct device *dev)
{
	struct i2c_client *client = private_max31760_client;
	int ret;
	uint8_t current_data = 0;
	uint8_t data = 0;

	htc_fan_stats();

	dev_info(dev, "[FAN] %s - power on\n", __func__);
	ret = max31760_update_fan_states(power_enable);
	if (ret) {
		dev_err(dev, "[FAN] %s: cannot update fan states ",__func__);
		return ret;
	}

	ret = i2c_read_block(client, CR2_REGISTER, &current_data, 1);
	if (ret) {
		dev_err(dev, "[FAN] %s: Fail to read CR2_REGISTER: %d", __func__, ret);
		return ret;
	}

	data = current_data | DFC_BIT;
	ret = i2c_write_block(client, CR2_REGISTER, &data, 1);
	if (ret) {
		dev_err(dev, "[FAN] %s: Fail to write CR2_REGISTER: %d", __func__, ret);
		return ret;
	}

	dev_info(dev, "[FAN] %s - Set PWM = %d\n", __func__, current_pwm);
	ret = i2c_write_block(client, PWMR_REGISTER, &current_pwm, 1);
	if (ret) {
		dev_err(dev, "[FAN] %s: Set PWM failed: %d",__func__, ret);
		return ret;
	}
	return 0;
}

static SIMPLE_DEV_PM_OPS(max31760_dev_pm_ops, max31760_suspend,
			 max31760_resume);

static const struct i2c_device_id max31760_id[] = {
	{ "max31760", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, max31760_id);

static const struct of_device_id max31760_mttable[] = {
	{ .compatible = "max31760"},
	{ },
};

static struct i2c_driver max31760_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name	= "max31760",
		.pm	= &max31760_dev_pm_ops,
		.of_match_table = max31760_mttable,
	},
	.probe		= max31760_probe,
	//.remove		= max31760_remove,
	.id_table	= max31760_id,
};

module_i2c_driver(max31760_driver);

MODULE_DESCRIPTION("Maxim Integration MAX31760");
MODULE_AUTHOR("York_Wang@htc.com");
