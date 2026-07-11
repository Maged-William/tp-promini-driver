#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/input/input.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>

LOG_MODULE_REGISTER(trackpoint_spi, CONFIG_INPUT_LOG_LEVEL);

#define DT_DRV_COMPAT trackpoint_spi

struct trackpoint_spi_config {
	struct spi_dt_spec bus;
	uint16_t report_ms;
	bool swap_xy;
	bool inv_x;
	bool inv_y;
};

struct trackpoint_spi_data {
	const struct device *dev;
	int32_t dx;
	int32_t dy;
	struct k_work_delayable poll_work;
};

static int trackpoint_spi_read_xy(const struct device *dev,
				  int8_t *x, int8_t *y)
{
	const struct trackpoint_spi_config *cfg = dev->config;
	uint8_t tx[3] = { 0, 0, 0 };
	uint8_t rx[3];
	int ret;

	const struct spi_buf tx_buf = { .buf = tx, .len = sizeof(tx) };
	const struct spi_buf_set tx_set = { .buffers = &tx_buf, .count = 1 };
	struct spi_buf rx_buf = { .buf = rx, .len = sizeof(rx) };
	const struct spi_buf_set rx_set = { .buffers = &rx_buf, .count = 1 };

	ret = spi_transceive_dt(&cfg->bus, &tx_set, &rx_set);
	if (ret != 0) {
		return ret;
	}

	*x = (int8_t)rx[1];
	*y = (int8_t)rx[2];
	return 0;
}

static void trackpoint_spi_poll_handler(struct k_work *work)
{
	struct k_work_delayable *dwork = k_work_delayable_from_work(work);
	struct trackpoint_spi_data *data =
		CONTAINER_OF(dwork, struct trackpoint_spi_data, poll_work);
	const struct trackpoint_spi_config *cfg = data->dev->config;
	int8_t x_raw, y_raw, x, y;

	if (trackpoint_spi_read_xy(data->dev, &x_raw, &y_raw) != 0) {
		LOG_WRN("SPI read failed");
		return;
	}

	if (cfg->swap_xy) {
		x = y_raw;
		y = x_raw;
	} else {
		x = x_raw;
		y = y_raw;
	}

	if (cfg->inv_x) x = -x;
	if (cfg->inv_y) y = -y;

	LOG_DBG("raw: x=%d y=%d  mapped: x=%d y=%d", x_raw, y_raw, x, y);

	if (x != 0 || y != 0) {
		data->dx += x;
		data->dy += y;

		int16_t rx = CLAMP(data->dx, INT16_MIN, INT16_MAX);
		int16_t ry = CLAMP(data->dy, INT16_MIN, INT16_MAX);

		data->dx = 0;
		data->dy = 0;

		input_report_rel(data->dev, INPUT_REL_X, rx, false, K_NO_WAIT);
		input_report_rel(data->dev, INPUT_REL_Y, ry, true, K_NO_WAIT);
	}

	k_work_schedule(dwork, K_MSEC(cfg->report_ms));
}

static int trackpoint_spi_init(const struct device *dev)
{
	const struct trackpoint_spi_config *cfg = dev->config;
	struct trackpoint_spi_data *data = dev->data;

	if (!spi_is_ready_dt(&cfg->bus)) {
		LOG_ERR("SPI bus not ready");
		return -ENODEV;
	}

	data->dev = dev;
	data->dx = 0;
	data->dy = 0;

	k_work_init_delayable(&data->poll_work, trackpoint_spi_poll_handler);
	k_work_schedule(&data->poll_work, K_MSEC(cfg->report_ms));

	LOG_INF("TrackPoint SPI initialized (report=%dms swap=%d inv_x=%d inv_y=%d)",
		cfg->report_ms, cfg->swap_xy, cfg->inv_x, cfg->inv_y);
	return 0;
}

#define SPI_OP (SPI_OP_MODE_MASTER | SPI_WORD_SET(8) | SPI_TRANSFER_MSB)

#define TRACKPOINT_SPI_DEFINE(inst)                                            \
	static const struct trackpoint_spi_config                             \
		trackpoint_spi_config_##inst = {                                \
			.bus = SPI_DT_SPEC_INST_GET(inst, SPI_OP, 0),          \
			.report_ms = DT_INST_PROP_OR(inst, report_rate_ms, 10),\
			.swap_xy = DT_INST_PROP(inst, swap_xy),                 \
			.inv_x = DT_INST_PROP(inst, invert_x),                   \
			.inv_y = DT_INST_PROP(inst, invert_y),                   \
	};                                                                      \
	static struct trackpoint_spi_data trackpoint_spi_data_##inst;           \
	DEVICE_DT_INST_DEFINE(inst, trackpoint_spi_init, NULL,                  \
			      &trackpoint_spi_data_##inst,                      \
			      &trackpoint_spi_config_##inst,                    \
			      POST_KERNEL, CONFIG_INPUT_INIT_PRIORITY, NULL);

DT_INST_FOREACH_STATUS_OKAY(TRACKPOINT_SPI_DEFINE)
