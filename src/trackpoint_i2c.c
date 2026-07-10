#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/input/input.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(trackpoint_i2c, CONFIG_INPUT_LOG_LEVEL);

#define DT_DRV_COMPAT trackpoint_i2c

struct trackpoint_i2c_config {
	const struct device *i2c_bus;
	uint16_t i2c_addr;
	uint32_t poll_ms;
	int16_t probe_addr;
};

struct trackpoint_i2c_data {
	struct k_work_delayable poll_work;
	const struct device *dev;
};

static int trackpoint_i2c_probe(const struct device *i2c_bus, uint16_t addr)
{
	uint8_t dummy;
	return i2c_write_read(i2c_bus, addr, NULL, 0, &dummy, 0);
}

static void trackpoint_i2c_poll_handler(struct k_work *work)
{
	struct k_work_delayable *dwork = k_work_delayable_from_work(work);
	struct trackpoint_i2c_data *data =
		CONTAINER_OF(dwork, struct trackpoint_i2c_data, poll_work);
	const struct trackpoint_i2c_config *cfg = data->dev->config;
	uint8_t buf[2];
	int ret;

	ret = i2c_read(cfg->i2c_bus, buf, sizeof(buf), cfg->i2c_addr);
	if (ret != 0) {
		LOG_WRN("I2C read failed: %d", ret);
	} else {
		int8_t x = (int8_t)buf[0];
		int8_t y = (int8_t)buf[1];

		LOG_DBG("X=%d Y=%d", x, y);

		if (x != 0 || y != 0) {
			input_report_rel(data->dev, INPUT_REL_X, x,
					 true, K_NO_WAIT);
			input_report_rel(data->dev, INPUT_REL_Y, y,
					 true, K_NO_WAIT);
		}
	}

	k_work_schedule(dwork, K_MSEC(cfg->poll_ms));
}

static int trackpoint_i2c_init(const struct device *dev)
{
	const struct trackpoint_i2c_config *cfg = dev->config;
	struct trackpoint_i2c_data *data = dev->data;
	int ret;
	uint16_t addrs[2];
	int num_addrs = 1;

	if (!device_is_ready(cfg->i2c_bus)) {
		LOG_ERR("I2C bus not ready");
		return -ENODEV;
	}

	data->dev = dev;

	addrs[0] = cfg->i2c_addr;
	if (cfg->probe_addr > 0 && cfg->probe_addr != cfg->i2c_addr) {
		addrs[1] = (uint16_t)cfg->probe_addr;
		num_addrs = 2;
	}

	ret = -EIO;
	for (int i = 0; i < num_addrs; i++) {
		ret = trackpoint_i2c_probe(cfg->i2c_bus, addrs[i]);
		if (ret == 0) {
			LOG_INF("TrackPoint found at addr 0x%02x (probe addr 0x%02x)",
				addrs[i], cfg->i2c_addr);
			break;
		}
		LOG_WRN("No ACK at 0x%02x (probe addr 0x%02x): %d",
			addrs[i], cfg->i2c_addr, ret);
	}

	if (ret != 0) {
		LOG_ERR("TrackPoint not found at configured addr 0x%02x or probe addr",
			cfg->i2c_addr);
	}

	k_work_init_delayable(&data->poll_work,
			      trackpoint_i2c_poll_handler);
	k_work_schedule(&data->poll_work, K_MSEC(cfg->poll_ms));

	LOG_INF("TrackPoint I2C initialized at addr 0x%x, poll %d ms",
		cfg->i2c_addr, cfg->poll_ms);
	return 0;
}

#define TRACKPOINT_I2C_DEFINE(inst)                                          \
	static const struct trackpoint_i2c_config                            \
		trackpoint_i2c_config_##inst = {                              \
			.i2c_bus = DEVICE_DT_GET(DT_INST_BUS(inst)),         \
			.i2c_addr = DT_INST_REG_ADDR(inst),                  \
			.poll_ms = DT_INST_PROP_OR(inst, poll_rate_ms, 10),  \
			.probe_addr = DT_INST_PROP_OR(inst, probe_addr, -1), \
	};                                                                    \
	static struct trackpoint_i2c_data trackpoint_i2c_data_##inst;         \
	DEVICE_DT_INST_DEFINE(inst, trackpoint_i2c_init, NULL,                \
			      &trackpoint_i2c_data_##inst,                     \
			      &trackpoint_i2c_config_##inst,                   \
			      POST_KERNEL, CONFIG_INPUT_INIT_PRIORITY, NULL);

DT_INST_FOREACH_STATUS_OKAY(TRACKPOINT_I2C_DEFINE)
