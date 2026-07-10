#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/input/input.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(trackpoint_i2c, CONFIG_INPUT_LOG_LEVEL);

#define DT_DRV_COMPAT trackpoint_i2c

struct trackpoint_i2c_config {
	struct i2c_dt_spec i2c;
	struct gpio_dt_spec dr;
};

struct trackpoint_i2c_data {
	const struct device *dev;
	struct gpio_callback gpio_cb;
	struct k_work work;
};

static void trackpoint_i2c_work_cb(struct k_work *work)
{
	struct trackpoint_i2c_data *data =
		CONTAINER_OF(work, struct trackpoint_i2c_data, work);
	const struct trackpoint_i2c_config *cfg = data->dev->config;
	uint8_t buf[2];
	int ret;

	ret = i2c_read_dt(&cfg->i2c, buf, sizeof(buf));
	if (ret != 0) {
		LOG_WRN("I2C read failed: %d", ret);
		return;
	}

	int8_t x = (int8_t)buf[0];
	int8_t y = (int8_t)buf[1];

	LOG_DBG("X=%d Y=%d", x, y);

	if (x != 0 || y != 0) {
		input_report_rel(data->dev, INPUT_REL_X, x, false, K_FOREVER);
		input_report_rel(data->dev, INPUT_REL_Y, y, true, K_FOREVER);
	}
}

static void trackpoint_i2c_gpio_cb(const struct device *port,
				    struct gpio_callback *cb, uint32_t pins)
{
	struct trackpoint_i2c_data *data =
		CONTAINER_OF(cb, struct trackpoint_i2c_data, gpio_cb);

	k_work_submit(&data->work);
}

static int trackpoint_i2c_init(const struct device *dev)
{
	const struct trackpoint_i2c_config *cfg = dev->config;
	struct trackpoint_i2c_data *data = dev->data;
	int ret;

	if (!device_is_ready(cfg->i2c.bus)) {
		LOG_ERR("I2C bus not ready");
		return -ENODEV;
	}

	data->dev = dev;

	k_work_init(&data->work, trackpoint_i2c_work_cb);

	if (!device_is_ready(cfg->dr.port)) {
		LOG_ERR("DR GPIO port not ready");
		return -ENODEV;
	}

	ret = gpio_pin_configure_dt(&cfg->dr, GPIO_INPUT);
	if (ret < 0) {
		LOG_ERR("Failed to configure DR pin: %d", ret);
		return ret;
	}

	ret = gpio_pin_interrupt_configure_dt(&cfg->dr,
					      GPIO_INT_EDGE_FALLING);
	if (ret < 0) {
		LOG_ERR("Failed to configure DR interrupt: %d", ret);
		return ret;
	}

	gpio_init_callback(&data->gpio_cb, trackpoint_i2c_gpio_cb,
			   BIT(cfg->dr.pin));
	ret = gpio_add_callback(cfg->dr.port, &data->gpio_cb);
	if (ret < 0) {
		LOG_ERR("Failed to add DR callback: %d", ret);
		return ret;
	}

	if (gpio_pin_get_dt(&cfg->dr) == 0) {
		LOG_DBG("DR already asserted, submitting initial work");
		k_work_submit(&data->work);
	}

	LOG_INF("TrackPoint I2C initialized at addr 0x%02x",
		cfg->i2c.addr);
	return 0;
}

#define TRACKPOINT_I2C_DEFINE(inst)                                          \
	static struct trackpoint_i2c_data trackpoint_i2c_data_##inst;        \
	static const struct trackpoint_i2c_config                             \
		trackpoint_i2c_config_##inst = {                               \
			.i2c = I2C_DT_SPEC_INST_GET(inst),                    \
			.dr = GPIO_DT_SPEC_INST_GET(inst, dr_gpios),          \
	};                                                                     \
	DEVICE_DT_INST_DEFINE(inst, trackpoint_i2c_init, NULL,                \
			      &trackpoint_i2c_data_##inst,                     \
			      &trackpoint_i2c_config_##inst,                   \
			      POST_KERNEL, CONFIG_INPUT_INIT_PRIORITY, NULL);

DT_INST_FOREACH_STATUS_OKAY(TRACKPOINT_I2C_DEFINE)
