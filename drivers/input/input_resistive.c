/*
 *         Y+ (up)
 *  --------------------
 *  |                  |
 *  |                  |
 *  |                  |
 *  |                  |
 *  |                  | x+ (right)
 *  |                  |
 *  ^ y                |
 *  ||                 |
 *  ||                 |
 *  ====>---------------
 *      x
 *
 *     <connectors>
 */

#define DT_DRV_COMPAT input_resistive

#include <zephyr/drivers/gpio.h>
#include <zephyr/input/input.h>
#include <zephyr/input/input_touch.h>
#include <zephyr/pm/device.h>
#include <zephyr/pm/device_runtime.h>
#include <zephyr/sys/util.h>
#include <zephyr/drivers/adc.h>

#include <zephyr/logging/log.h>
#include <hardware/address_mapped.h>
#include <hardware/structs/pads_bank0.h>
#include <hardware/gpio.h>
LOG_MODULE_REGISTER(resistive, CONFIG_INPUT_LOG_LEVEL);

#define ADC_NODE DT_NODELABEL(adc)
#define ADC_CHANNEL_NODE DT_CHILD(ADC_NODE, channel_0)
#define ADC_RESOLUTION DT_PROP(ADC_CHANNEL_NODE, zephyr_resolution)

#define ADC_MAX_VALUE  ((2 << (ADC_RESOLUTION - 1)) - 1)
#define ADC_MAX_X      3500
#define ADC_MIN_X      750
#define ADC_MAX_Y      3500
#define ADC_MIN_Y      750



/** resistive configuration (DT). */
struct resistive_config {
	struct input_touchscreen_common_config common;
	struct gpio_dt_spec xm_gpio;
	struct gpio_dt_spec xp_gpio;
	struct gpio_dt_spec ym_gpio;
	struct gpio_dt_spec yp_gpio;
};

/** resistive data. */
struct resistive_data {
	/** Device pointer. */
	const struct device *dev;
	/** Work queue (for deferred read). */
	struct k_work work;
	/** Timer (polling mode). */
	struct k_timer timer;
	/** Last pressed state. */
	bool pressed_old;
};

enum {
	YP = 0,
	XM,
};

/* Data of ADC device specified in devicetree. */
static const struct device *adc = DEVICE_DT_GET(DT_NODELABEL(adc));
static const struct adc_channel_cfg adc_cfg_dt[] = {
	[YP] = ADC_CHANNEL_CFG_DT(DT_CHILD(DT_NODELABEL(adc), channel_0)),
	[XM] = ADC_CHANNEL_CFG_DT(DT_CHILD(DT_NODELABEL(adc), channel_1))};

#define CHANNELS (sizeof(adc_cfg_dt) / sizeof(struct adc_channel_cfg))

static uint16_t channel_reading[CHANNELS]; // [samples][channels]

/* Configure the sampling sequence to be made. */
static struct adc_sequence sequence = {
	.buffer = channel_reading,
	/* buffer size in bytes, not number of samples */
	.buffer_size = sizeof(channel_reading),
	.resolution = ADC_RESOLUTION,
	.options = NULL,
};

static uint32_t thresholds_x[2] = {UINT32_MAX, 0};
static uint32_t thresholds_y[2] = {UINT32_MAX, 0};

INPUT_TOUCH_STRUCT_CHECK(struct resistive_config);

static uint16_t read_x_position(const struct resistive_config *config)
{
	gpio_pin_configure_dt(&config->xp_gpio, GPIO_OUTPUT_INACTIVE);
	gpio_pin_configure_dt(&config->xm_gpio, GPIO_OUTPUT_ACTIVE);

	// Set input enable off, output disable on
	gpio_set_function(config->yp_gpio.pin, GPIO_FUNC_NULL);
	hw_clear_bits(&pads_bank0_hw->io[config->yp_gpio.pin], PADS_BANK0_GPIO0_IE_BITS);
	hw_write_masked(&pads_bank0_hw->io[config->yp_gpio.pin], PADS_BANK0_GPIO0_OD_BITS,
			PADS_BANK0_GPIO0_OD_BITS);

	gpio_pin_configure_dt(&config->ym_gpio, GPIO_INPUT | GPIO_PULL_DOWN);

	int err = adc_read(adc, &sequence);
	if (err < 0) {
		printf("Could not read YP (%d)\n", err);
	}
	uint16_t x = channel_reading[YP];
	return ADC_MAX_VALUE - x;
}

static uint16_t read_y_position(const struct resistive_config *config)
{
	gpio_pin_configure_dt(&config->yp_gpio, GPIO_OUTPUT_ACTIVE);
	gpio_pin_configure_dt(&config->ym_gpio, GPIO_OUTPUT_INACTIVE);

	// Set input enable off, output disable on
	gpio_set_function(config->xm_gpio.pin, GPIO_FUNC_NULL);
	hw_clear_bits(&pads_bank0_hw->io[config->xm_gpio.pin], PADS_BANK0_GPIO0_IE_BITS);
	hw_write_masked(&pads_bank0_hw->io[config->xm_gpio.pin], PADS_BANK0_GPIO0_OD_BITS,
			PADS_BANK0_GPIO0_OD_BITS);

	gpio_pin_configure_dt(&config->xp_gpio, GPIO_INPUT | GPIO_PULL_DOWN);

	int err = adc_read(adc, &sequence);
	if (err < 0) {
		printf("Could not read XM (%d)\n", err);
	}
	uint16_t y = channel_reading[XM];
	return ADC_MAX_VALUE - y;
}

static int read_pressure(const struct resistive_config *config, uint16_t x)
{
	// Set X+ to ground
	gpio_pin_configure_dt(&config->xp_gpio, GPIO_OUTPUT_INACTIVE);

	// Set Y- to VCC
	gpio_pin_set_dt(&config->ym_gpio, 1);

	// Hi-Z X- and Y+
	gpio_set_function(config->yp_gpio.pin, GPIO_FUNC_NULL);
	hw_clear_bits(&pads_bank0_hw->io[config->yp_gpio.pin], PADS_BANK0_GPIO0_IE_BITS);
	hw_write_masked(&pads_bank0_hw->io[config->yp_gpio.pin], PADS_BANK0_GPIO0_OD_BITS,
			PADS_BANK0_GPIO0_OD_BITS);

	int err = adc_read(adc, &sequence);
	if (err < 0) {
		printf("Could not read XM (%d)\n", err);
	}
	int z1 = channel_reading[XM];
	int z2 = channel_reading[YP];

	int32_t z = 0;
	if (z2 < ADC_MAX_VALUE * 0.95 && z1 > ADC_MAX_VALUE * 0.05) {
		// now read the x
		float rtouch = (float)z2;
		rtouch /= (float)z1;
		rtouch -= 1;
		rtouch *= (float)x;
		rtouch *= CONFIG_INPUT_RESISTANCE_PLATE_RESISTANCE;
		rtouch /= (ADC_MAX_VALUE + 1);

		z = (int32_t)rtouch;
	}
	return z;
}

static void restore_gpios(const struct resistive_config *config)
{
	gpio_pin_configure_dt(&config->xp_gpio, GPIO_OUTPUT_ACTIVE);
	gpio_pin_configure_dt(&config->xm_gpio, GPIO_OUTPUT_ACTIVE);
	gpio_pin_configure_dt(&config->yp_gpio, GPIO_OUTPUT_ACTIVE);
	gpio_pin_configure_dt(&config->ym_gpio, GPIO_OUTPUT_ACTIVE);
}

static void update_thresholds(uint32_t new_value, uint32_t *thresholds)
{
	if (new_value < thresholds[0]) {
		thresholds[0] = new_value;
	}
	if (new_value > thresholds[1]) {
		thresholds[1] = new_value;
	}
}

static void resistive_process(const struct device *dev)
{
	const struct resistive_config *config = dev->config;
	struct resistive_data *data = dev->data;

	uint16_t x = read_x_position(config);
	uint16_t y = read_y_position(config);
	int32_t z = read_pressure(config, x);

	bool pressed = z > CONFIG_INPUT_RESISTANCE_TOUCH_DETECTION_THRESHOLD;
	// if (pressed && !data->pressed_old) {
	if (pressed) {

		update_thresholds(x, thresholds_x);
		update_thresholds(y, thresholds_y);

		// these values need to be determined. touch the edges of the display and see
		// what adc values are reported this are the limits.
		x = MIN(ADC_MAX_X, MAX(ADC_MIN_X, x));
		y = MIN(ADC_MAX_Y, MAX(ADC_MIN_Y, y));

		// We assume linear behaviour
		int32_t q_x, q_y;
		q_x = (int32_t)config->common.screen_height * ADC_MIN_X / (ADC_MAX_X - ADC_MIN_X);
		q_y = (int32_t)config->common.screen_width * ADC_MIN_Y / (ADC_MAX_Y - ADC_MIN_Y);

		uint32_t disp_x = config->common.screen_height * x / (ADC_MAX_X - ADC_MIN_X) - q_x;
		uint32_t disp_y = config->common.screen_width * y / (ADC_MAX_Y - ADC_MIN_Y) - q_y;

		input_touchscreen_report_pos(dev, disp_x, disp_y, K_FOREVER);
		input_report_key(dev, INPUT_BTN_TOUCH, 1, true, K_FOREVER);
		LOG_WRN("Pressed at (%4u,%4u) with intensity %i (th: %u,%u,%u,%u)", disp_x, disp_y, z, thresholds_x[0], thresholds_x[1], thresholds_y[0], thresholds_y[1]);
	}
	if (data->pressed_old && !pressed) {
		input_report_key(dev, INPUT_BTN_TOUCH, 0, true, K_FOREVER);
		LOG_ERR("Released!");
	}
	data->pressed_old = pressed;

	restore_gpios(config);
}

#ifdef CONFIG_INPUT_RESISTIVE_MANUAL
void input_resistive_trigger_measurement(const void *dev)
{
	resistive_process(dev);
}
#else
static void resistive_work_handler(struct k_work *work)
{
	struct resistive_data *data = CONTAINER_OF(work, struct resistive_data, work);

	resistive_process(data->dev);
}

static void resistive_timer_handler(struct k_timer *timer)
{
	struct resistive_data *data = CONTAINER_OF(timer, struct resistive_data, timer);

	k_work_submit(&data->work);
}
#endif

static int resistive_init(const struct device *dev)
{
	LOG_ERR("resistive init!");

	const struct resistive_config *config = dev->config;
	struct resistive_data *data = dev->data;

	data->dev = dev;

	if (!gpio_is_ready_dt(&config->xm_gpio)) {
		LOG_ERR("Xm GPIO controller device not ready");
		return -ENODEV;
	}

	if (!gpio_is_ready_dt(&config->xp_gpio)) {
		LOG_ERR("Xp GPIO controller device not ready");
		return -ENODEV;
	}

	if (!gpio_is_ready_dt(&config->ym_gpio)) {
		LOG_ERR("Ym GPIO controller device not ready");
		return -ENODEV;
	}

	if (!gpio_is_ready_dt(&config->yp_gpio)) {
		LOG_ERR("Yp GPIO controller device not ready");
		return -ENODEV;
	}

	if (!device_is_ready(adc)) {
		printf("ADC controller device %s not ready\n", adc->name);
		return 0;
	}

	int err;
	/* Configure channels individually prior to sampling. */
	for (size_t i = 0U; i < CHANNELS; i++) {
		sequence.channels |= BIT(adc_cfg_dt[i].channel_id);
		err = adc_channel_setup(adc, &adc_cfg_dt[i]);
		if (err < 0) {
			printf("Could not setup channel #%d (%d)\n", i, err);
			return 0;
		}
	}

#ifndef CONFIG_INPUT_RESISTIVE_MANUAL
	k_work_init(&data->work, resistive_work_handler);
	k_timer_init(&data->timer, resistive_timer_handler, NULL);
	k_timer_start(&data->timer, K_MSEC(CONFIG_INPUT_RESISTIVE_PERIOD),
		      K_MSEC(CONFIG_INPUT_RESISTIVE_PERIOD));
#endif

	int r = pm_device_runtime_enable(dev);
	if (r < 0 && r != -ENOTSUP) {
		LOG_ERR("Failed to enable runtime power management");
		return r;
	}

	return 0;
}

#ifdef CONFIG_PM_DEVICE
static int resistive_pm_action(const struct device *dev, enum pm_device_action action)
{
	const struct resistive_config *config = dev->config;
	struct resistive_data *data = dev->data;
	int ret;

	if (config->reset_gpio.port == NULL) {
		return -ENOTSUP;
	}

	switch (action) {
	case PM_DEVICE_ACTION_SUSPEND:
		k_timer_stop(&data->timer);
		break;
	case PM_DEVICE_ACTION_RESUME:
		k_timer_start(&data->timer, K_MSEC(CONFIG_INPUT_RESISTIVE_PERIOD),
			      K_MSEC(CONFIG_INPUT_RESISTIVE_PERIOD));
		break;
	default:
		return -ENOTSUP;
	}

	return 0;
}
#endif

#define RESISTIVE_INIT(index)                                                                      \
	PM_DEVICE_DT_INST_DEFINE(n, resistive_pm_action);                                          \
	static const struct resistive_config resistive_config_##index = {                          \
		.common = INPUT_TOUCH_DT_INST_COMMON_CONFIG_INIT(index),                           \
		.xm_gpio = GPIO_DT_SPEC_INST_GET_OR(index, xm_gpios, {0}),                         \
		.xp_gpio = GPIO_DT_SPEC_INST_GET_OR(index, xp_gpios, {0}),                         \
		.ym_gpio = GPIO_DT_SPEC_INST_GET_OR(index, ym_gpios, {0}),                         \
		.yp_gpio = GPIO_DT_SPEC_INST_GET_OR(index, yp_gpios, {0}),                         \
	};                                                                                         \
	static struct resistive_data resistive_data_##index;                                       \
	DEVICE_DT_INST_DEFINE(index, resistive_init, PM_DEVICE_DT_INST_GET(n),                     \
			      &resistive_data_##index, &resistive_config_##index, POST_KERNEL,     \
			      CONFIG_INPUT_INIT_PRIORITY, NULL);

DT_INST_FOREACH_STATUS_OKAY(RESISTIVE_INIT)
