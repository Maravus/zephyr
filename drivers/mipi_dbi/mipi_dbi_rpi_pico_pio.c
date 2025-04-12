/*
 * MIPI DBI Type A and B driver using PIO
 *
 * Copyright 2024 Stefan Gloor
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT raspberrypi_pico_mipi_dbi_pio

#include <zephyr/drivers/mipi_dbi.h>
#include <zephyr/drivers/gpio.h>

#include <zephyr/drivers/misc/pio_rpi_pico/pio_rpi_pico.h>

#include <hardware/pio.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(mipi_dbi_pico_pio, CONFIG_MIPI_DBI_LOG_LEVEL);

/* The MIPI DBI spec allows 8, 9, and 16 bits */
#define MIPI_DBI_MAX_DATA_BUS_WIDTH 16

/* Compile in a data bus LUT for improved performance if at least one instance uses an 8-bit bus */
#define _8_BIT_MODE_PRESENT(n) (DT_INST_PROP_LEN(n, data_gpios) == 8) |
#define MIPI_DBI_8_BIT_MODE    DT_INST_FOREACH_STATUS_OKAY(_8_BIT_MODE_PRESENT) 0

struct mipi_dbi_pico_pio_config {
	const struct device *piodev;
	/* Parallel 8080/6800 data GPIOs */
	const struct gpio_dt_spec data[MIPI_DBI_MAX_DATA_BUS_WIDTH];
	const uint8_t data_bus_width;

	/* Read (type B) GPIO */
	const struct gpio_dt_spec rd;

	/* Write (type B) or Read/!Write (type A) GPIO */
	const struct gpio_dt_spec wr;

	/* Enable/strobe GPIO (type A) */
	const struct gpio_dt_spec e;

	/* Chip-select GPIO */
	const struct gpio_dt_spec cs;

	/* Command/Data GPIO */
	const struct gpio_dt_spec cmd_data;

	/* Reset GPIO */
	const struct gpio_dt_spec reset;

#if MIPI_DBI_8_BIT_MODE
	/* Data GPIO remap look-up table. Valid if mipi_dbi_pico_pio_data.single_port is set */
	const uint32_t data_lut[256];

	/* Mask of all data pins. Valid if mipi_dbi_pico_pio_data.single_port is set */
	const uint32_t data_mask;
#endif
};

struct mipi_dbi_pico_pio_data {
	PIO pio;
	size_t pio_sm_firsttwobits;
	size_t pio_sm_thirdbit;
	size_t pio_sm_otherbits;
	struct k_mutex lock;

#if MIPI_DBI_8_BIT_MODE
	/* Indicates whether all data GPIO pins are on the same port and the data LUT is used. */
	bool single_port;

	/* Data GPIO port device. Valid if mipi_dbi_pico_pio_data.single_port is set */
	const struct device *data_port;
#endif
};

#if 0
RPI_PICO_PIO_DEFINE_PROGRAM(firsttwobits, 0, 3,
	//     .wrap_target
	0x98a0, //  0: pull   block           side 1
	0x2044, //  1: wait   0 irq, 4
	0xb042, //  2: nop                    side 0
	0x6102, //  3: out    pins, 2                [1]
	    //     .wrap
);

RPI_PICO_PIO_DEFINE_PROGRAM(thirdbit, 0, 3,
	//     .wrap_target
	0x80a0, //  0: pull   block
	0x2044, //  1: wait   0 irq, 4
	0x6062, //  2: out    null, 2
	0x6101, //  3: out    pins, 1                [1]
	    //     .wrap
);

RPI_PICO_PIO_DEFINE_PROGRAM(otherbits, 0, 4,
	//     .wrap_target
	0x80a0, //  0: pull   block
	0xc044, //  1: irq    clear 4
	0x6063, //  2: out    null, 3
	0x6005, //  3: out    pins, 5
	0xc004, //  4: irq    nowait 4
	    //     .wrap
);
#else
RPI_PICO_PIO_DEFINE_PROGRAM(parallel_8bit, 9, 13,
	0x20c4, //  0: wait   1 irq, 4
	0x81a0, //  1: pull   block                  [1]
	0x7002, //  2: out    pins, 2         side 0
	0x1800, //  3: jmp    0               side 1
	0x20c4, //  4: wait   1 irq, 4
	0x80a0, //  5: pull   block
	0x6062, //  6: out    null, 2
	0x6001, //  7: out    pins, 1
	0x0004, //  8: jmp    4
	    //     .wrap_target
	0x80a0, //  9: pull   block
	0xc104, // 10: irq    nowait 4               [1]
	0x6063, // 11: out    null, 3
	0x6005, // 12: out    pins, 5
	0xc044, // 13: irq    clear 4
	    //     .wrap
);
#endif

static int mipi_dbi_pio_configure(const struct mipi_dbi_pico_pio_config *dev_cfg,
				  struct mipi_dbi_pico_pio_data *data,
				  uint16_t clock_div)
{
	data->pio = pio_rpi_pico_get_pio(dev_cfg->piodev);
	int rc = pio_rpi_pico_allocate_sm(dev_cfg->piodev, &data->pio_sm_firsttwobits);
	if (rc < 0) {
		return rc;
	}

	// Load the PIO program. Starting by default with 16 bits mode
	// since it's at the begining of the PIO program.
	uint32_t offset = pio_add_program(data->pio, RPI_PICO_PIO_GET_PROGRAM(parallel_8bit));
	pio_sm_config sm_config = pio_get_default_sm_config();
	sm_config_set_sideset(&sm_config, 2, true, false);
	sm_config_set_sideset_pins(&sm_config, dev_cfg->wr.pin);
	sm_config_set_out_pins(&sm_config, dev_cfg->data[0].pin, 2);
	// Set clock divider. Value of 1 for max speed.
	sm_config_set_clkdiv_int_frac(&sm_config, clock_div, 0);
	sm_config_set_fifo_join(&sm_config, PIO_FIFO_JOIN_TX);
	// sm_config_set_wrap(&sm_config,
	// 		   offset + RPI_PICO_PIO_GET_WRAP_TARGET(parallel_8bit),
	// 		   offset + RPI_PICO_PIO_GET_WRAP(parallel_8bit));
	// The OSR register shifts to the right, sending the MSB byte
	// first, in a double bytes transfers.
	sm_config_set_out_shift(&sm_config, true, false, 0);
	pio_gpio_init(data->pio, dev_cfg->wr.pin);
	pio_gpio_init(data->pio, dev_cfg->data[0].pin);
	pio_gpio_init(data->pio, dev_cfg->data[1].pin);
	pio_sm_set_consecutive_pindirs(data->pio, data->pio_sm_firsttwobits, dev_cfg->wr.pin, 1, true);
	pio_sm_set_consecutive_pindirs(data->pio, data->pio_sm_firsttwobits, dev_cfg->data[0].pin, 2, true);
	pio_sm_init(data->pio, data->pio_sm_firsttwobits, offset, &sm_config);


	rc = pio_rpi_pico_allocate_sm(dev_cfg->piodev, &data->pio_sm_thirdbit);
	if (rc < 0) {
		return rc;
	}

	// Load the PIO program. Starting by default with 16 bits mode
	// since it's at the begining of the PIO program.
	// offset = pio_add_program(data->pio, RPI_PICO_PIO_GET_PROGRAM(parallel_8bit));
	sm_config = pio_get_default_sm_config();
	sm_config_set_out_pins(&sm_config, dev_cfg->data[2].pin, 1);
	// Set clock divider. Value of 1 for max speed.
	sm_config_set_clkdiv_int_frac(&sm_config, clock_div, 0);
	sm_config_set_fifo_join(&sm_config, PIO_FIFO_JOIN_TX);
	// sm_config_set_wrap(&sm_config,
	// 		   offset + 4 + RPI_PICO_PIO_GET_WRAP_TARGET(parallel_8bit),
	// 		   offset + 4 + RPI_PICO_PIO_GET_WRAP(parallel_8bit));
	// The OSR register shifts to the right, sending the MSB byte
	// first, in a double bytes transfers.
	sm_config_set_out_shift(&sm_config, true, false, 0);
	pio_gpio_init(data->pio, dev_cfg->data[2].pin);
	pio_sm_set_consecutive_pindirs(data->pio, data->pio_sm_thirdbit, dev_cfg->data[2].pin, 1, true);
	pio_sm_init(data->pio, data->pio_sm_thirdbit, offset + 4, &sm_config);


	rc = pio_rpi_pico_allocate_sm(dev_cfg->piodev, &data->pio_sm_otherbits);
	if (rc < 0) {
		return rc;
	}

	// Load the PIO program. Starting by default with 16 bits mode
	// since it's at the begining of the PIO program.
	// offset = pio_add_program(data->pio, RPI_PICO_PIO_GET_PROGRAM(parallel_8bit));
	sm_config = pio_get_default_sm_config();
	sm_config_set_out_pins(&sm_config, dev_cfg->data[3].pin, 5);
	// Set clock divider. Value of 1 for max speed.
	sm_config_set_clkdiv_int_frac(&sm_config, clock_div, 0);
	sm_config_set_fifo_join(&sm_config, PIO_FIFO_JOIN_TX);
	sm_config_set_wrap(&sm_config,
			   offset + RPI_PICO_PIO_GET_WRAP_TARGET(parallel_8bit),
			   offset + RPI_PICO_PIO_GET_WRAP(parallel_8bit));
	// The OSR register shifts to the right, sending the MSB byte
	// first, in a double bytes transfers.
	sm_config_set_out_shift(&sm_config, true, false, 0);
	pio_gpio_init(data->pio, dev_cfg->data[3].pin);
	pio_gpio_init(data->pio, dev_cfg->data[4].pin);
	pio_gpio_init(data->pio, dev_cfg->data[5].pin);
	pio_gpio_init(data->pio, dev_cfg->data[6].pin);
	pio_gpio_init(data->pio, dev_cfg->data[7].pin);
	pio_sm_set_consecutive_pindirs(data->pio, data->pio_sm_otherbits, dev_cfg->data[3].pin, 5, true);
	pio_sm_init(data->pio, data->pio_sm_otherbits, offset + 9, &sm_config);

	// enable all state machines in sync
	uint32_t mask = BIT(data->pio_sm_firsttwobits) | BIT(data->pio_sm_thirdbit) | BIT(data->pio_sm_otherbits);
	pio_enable_sm_mask_in_sync(data->pio, mask);

	return 0;
}

static inline void mipi_dbi_pico_pio_set_data_gpios(const struct mipi_dbi_pico_pio_config *config,
						   struct mipi_dbi_pico_pio_data *data,
						   uint32_t value)
{
#if MIPI_DBI_8_BIT_MODE
	if (data->single_port) {
		gpio_port_set_masked(data->data_port, config->data_mask, config->data_lut[value]);
	} else {
#endif
		for (int i = 0; i < config->data_bus_width; i++) {
			gpio_pin_set_dt(&config->data[i], (value & (1 << i)) != 0);
		}
#if MIPI_DBI_8_BIT_MODE
	}
#endif
}

static int mipi_dbi_pico_pio_write_helper(const struct device *dev,
					 const struct mipi_dbi_config *dbi_config, bool cmd_present,
					 uint8_t cmd, const uint8_t *data_buf, size_t len)
{
	const struct mipi_dbi_pico_pio_config *config = dev->config;
	struct mipi_dbi_pico_pio_data *data = dev->data;
	int ret = 0;
	uint8_t value;

	ret = k_mutex_lock(&data->lock, K_FOREVER);
	if (ret < 0) {
		return ret;
	}

	switch (dbi_config->mode) {
	case MIPI_DBI_MODE_8080_BUS_8_BIT:
	case MIPI_DBI_MODE_8080_BUS_9_BIT:
	case MIPI_DBI_MODE_8080_BUS_16_BIT:
		gpio_pin_set_dt(&config->cs, 1);
		if (cmd_present) {
			// gpio_pin_set_dt(&config->wr, 0);
			gpio_pin_set_dt(&config->cmd_data, 0);
			// mipi_dbi_pico_pio_set_data_gpios(config, data, cmd);
			// gpio_pin_set_dt(&config->wr, 1);
			pio_sm_put_blocking(data->pio,data->pio_sm_firsttwobits, (uint32_t)cmd);
			pio_sm_put_blocking(data->pio,data->pio_sm_thirdbit, (uint32_t)cmd);
			pio_sm_put_blocking(data->pio,data->pio_sm_otherbits, (uint32_t)cmd);
		}
		if (len > 0) {
			// setup dma to transfer data, lock with a message queue or simple mutex
			// interrupt routine listens on end of dma.. usually this would be the case
			// but as we have shared data pins with touch controller we have to listen
			// to end of fifo queue.. or we introduce a blocking while loop until fifo is
			// empty may be good enough?
			gpio_pin_set_dt(&config->cmd_data, 1);

			while (len > 0) {
				value = *(data_buf++);
				// value = value == 0b11001001 ? 0x0 : 0b11001001;
				// value = value == 0xff ? 0x0 : 0xff;
				pio_sm_put_blocking(data->pio,data->pio_sm_firsttwobits, (uint32_t)value);
				pio_sm_put_blocking(data->pio,data->pio_sm_thirdbit, (uint32_t)value);
				pio_sm_put_blocking(data->pio,data->pio_sm_otherbits, (uint32_t)value);
				len--;
			}
		}
		gpio_pin_set_dt(&config->cs, 0);
		break;

	/* Clocked E mode */
	case MIPI_DBI_MODE_6800_BUS_8_BIT:
	case MIPI_DBI_MODE_6800_BUS_9_BIT:
	case MIPI_DBI_MODE_6800_BUS_16_BIT:
		gpio_pin_set_dt(&config->cs, 1);
		gpio_pin_set_dt(&config->wr, 0);
		if (cmd_present) {
			gpio_pin_set_dt(&config->e, 1);
			gpio_pin_set_dt(&config->cmd_data, 0);
			mipi_dbi_pico_pio_set_data_gpios(config, data, cmd);
			gpio_pin_set_dt(&config->e, 0);
		}
		if (len > 0) {
			gpio_pin_set_dt(&config->cmd_data, 1);
			while (len > 0) {
				value = *(data_buf++);
				gpio_pin_set_dt(&config->e, 1);
				mipi_dbi_pico_pio_set_data_gpios(config, data, value);
				gpio_pin_set_dt(&config->e, 0);
				len--;
			}
		}
		gpio_pin_set_dt(&config->cs, 0);
		break;

	default:
		LOG_ERR("MIPI DBI mode %u is not supported.", dbi_config->mode);
		ret = -ENOTSUP;
	}

	k_mutex_unlock(&data->lock);
	return ret;
}

static int mipi_dbi_pico_pio_command_write(const struct device *dev,
					  const struct mipi_dbi_config *dbi_config, uint8_t cmd,
					  const uint8_t *data_buf, size_t len)
{
	return mipi_dbi_pico_pio_write_helper(dev, dbi_config, true, cmd, data_buf, len);
}

static int mipi_dbi_pico_pio_write_display(const struct device *dev,
					  const struct mipi_dbi_config *dbi_config,
					  const uint8_t *framebuf,
					  struct display_buffer_descriptor *desc,
					  enum display_pixel_format pixfmt)
{
	ARG_UNUSED(pixfmt);

	return mipi_dbi_pico_pio_write_helper(dev, dbi_config, false, 0x0, framebuf, desc->buf_size);
}

static int mipi_dbi_pico_pio_reset(const struct device *dev, k_timeout_t delay)
{
	const struct mipi_dbi_pico_pio_config *config = dev->config;
	int ret;

	LOG_DBG("Performing hw reset.");

	ret = gpio_pin_set_dt(&config->reset, 1);
	if (ret < 0) {
		return ret;
	}
	k_sleep(delay);
	return gpio_pin_set_dt(&config->reset, 0);
}

static int mipi_dbi_pico_pio_init(const struct device *dev)
{
	const struct mipi_dbi_pico_pio_config *config = dev->config;
	const char *failed_pin = NULL;
	int ret = 0;
#if MIPI_DBI_8_BIT_MODE
	struct mipi_dbi_pico_pio_data *data = dev->data;
#endif

	if (gpio_is_ready_dt(&config->cmd_data)) {
		ret = gpio_pin_configure_dt(&config->cmd_data, GPIO_OUTPUT_ACTIVE);
		if (ret < 0) {
			failed_pin = "cmd_data";
			goto fail;
		}
		gpio_pin_set_dt(&config->cmd_data, 0);
	}
	if (gpio_is_ready_dt(&config->rd)) {
		gpio_pin_configure_dt(&config->rd, GPIO_OUTPUT_ACTIVE);
		/* Don't emit an error because this pin is unused in type A */
		gpio_pin_set_dt(&config->rd, 1);
	}
	if (gpio_is_ready_dt(&config->wr)) {
		ret = gpio_pin_configure_dt(&config->wr, GPIO_OUTPUT_ACTIVE);
		if (ret < 0) {
			failed_pin = "wr";
			goto fail;
		}
		gpio_pin_set_dt(&config->wr, 1);
	}
	if (gpio_is_ready_dt(&config->e)) {
		gpio_pin_configure_dt(&config->e, GPIO_OUTPUT_ACTIVE);
		/* Don't emit an error because this pin is unused in type B */
		gpio_pin_set_dt(&config->e, 0);
	}
	if (gpio_is_ready_dt(&config->cs)) {
		ret = gpio_pin_configure_dt(&config->cs, GPIO_OUTPUT_ACTIVE);
		if (ret < 0) {
			failed_pin = "cs";
			goto fail;
		}
		gpio_pin_set_dt(&config->cs, 0);
	}
	if (gpio_is_ready_dt(&config->reset)) {
		ret = gpio_pin_configure_dt(&config->reset, GPIO_OUTPUT_ACTIVE);
		if (ret < 0) {
			failed_pin = "reset";
			goto fail;
		}
		gpio_pin_set_dt(&config->reset, 0);
	}
	for (int i = 0; i < config->data_bus_width; i++) {
		if (gpio_is_ready_dt(&config->data[i])) {
			ret = gpio_pin_configure_dt(&config->data[i], GPIO_OUTPUT_ACTIVE);
			if (ret < 0) {
				failed_pin = "data";
				goto fail;
			}
			gpio_pin_set_dt(&config->data[i], 0);
		}
	}

#if MIPI_DBI_8_BIT_MODE
	/* To optimize performance, we test whether all the data pins are
	 * on the same port. If they are, we can set the whole port in one go
	 * instead of setting each pin individually.
	 * For 8-bit mode only because LUT size grows exponentially.
	 */
	if (config->data_bus_width == 8) {
		data->single_port = true;
		data->data_port = config->data[0].port;
		for (int i = 1; i < config->data_bus_width; i++) {
			if (data->data_port != config->data[i].port) {
				data->single_port = false;
			}
		}
	}
	if (data->single_port) {
		LOG_DBG("LUT optimization enabled. data_mask=0x%x", config->data_mask);
	}
#endif

	mipi_dbi_pio_configure(config, data, 80);

	return ret;
fail:
	LOG_ERR("Failed to configure %s GPIO pin.", failed_pin);
	return ret;
}

static const struct mipi_dbi_driver_api mipi_dbi_pico_pio_driver_api = {
	.reset = mipi_dbi_pico_pio_reset,
	.command_write = mipi_dbi_pico_pio_command_write,
	.write_display = mipi_dbi_pico_pio_write_display
};

/* This macro is repeatedly called by LISTIFY() at compile-time to generate the data bus LUT */
#define LUT_GEN(i, n) (((i & (1 << 0)) ? (1 << DT_INST_GPIO_PIN_BY_IDX(n, data_gpios, 0)) : 0) |   \
		       ((i & (1 << 1)) ? (1 << DT_INST_GPIO_PIN_BY_IDX(n, data_gpios, 1)) : 0) |   \
		       ((i & (1 << 2)) ? (1 << DT_INST_GPIO_PIN_BY_IDX(n, data_gpios, 2)) : 0) |   \
		       ((i & (1 << 3)) ? (1 << DT_INST_GPIO_PIN_BY_IDX(n, data_gpios, 3)) : 0) |   \
		       ((i & (1 << 4)) ? (1 << DT_INST_GPIO_PIN_BY_IDX(n, data_gpios, 4)) : 0) |   \
		       ((i & (1 << 5)) ? (1 << DT_INST_GPIO_PIN_BY_IDX(n, data_gpios, 5)) : 0) |   \
		       ((i & (1 << 6)) ? (1 << DT_INST_GPIO_PIN_BY_IDX(n, data_gpios, 6)) : 0) |   \
		       ((i & (1 << 7)) ? (1 << DT_INST_GPIO_PIN_BY_IDX(n, data_gpios, 7)) : 0))

/* If at least one instance has an 8-bit bus, add a data look-up table to the read-only config.
 * Whether or not it is valid and actually used for a particular instance is decided at runtime
 * and stored in the instance's mipi_dbi_pico_pio_data.single_port.
 */
#if MIPI_DBI_8_BIT_MODE
#define DATA_LUT_OPTIMIZATION(n)                                                                   \
		.data_lut = { LISTIFY(256, LUT_GEN, (,), n) },                                     \
		.data_mask = ((1 << DT_INST_GPIO_PIN_BY_IDX(n, data_gpios, 0)) |                   \
			      (1 << DT_INST_GPIO_PIN_BY_IDX(n, data_gpios, 1)) |                   \
			      (1 << DT_INST_GPIO_PIN_BY_IDX(n, data_gpios, 2)) |                   \
			      (1 << DT_INST_GPIO_PIN_BY_IDX(n, data_gpios, 3)) |                   \
			      (1 << DT_INST_GPIO_PIN_BY_IDX(n, data_gpios, 4)) |                   \
			      (1 << DT_INST_GPIO_PIN_BY_IDX(n, data_gpios, 5)) |                   \
			      (1 << DT_INST_GPIO_PIN_BY_IDX(n, data_gpios, 6)) |                   \
			      (1 << DT_INST_GPIO_PIN_BY_IDX(n, data_gpios, 7)))
#else
#define DATA_LUT_OPTIMIZATION(n)
#endif

#define PIO_MIPI_DBI_INIT(n)                                                                   \
	static const struct mipi_dbi_pico_pio_config mipi_dbi_pico_pio_config_##n = {                \
                .piodev = DEVICE_DT_GET(DT_INST_PARENT(n)),                                     \
		.data = {GPIO_DT_SPEC_INST_GET_BY_IDX_OR(n, data_gpios, 0, {0}),                   \
			 GPIO_DT_SPEC_INST_GET_BY_IDX_OR(n, data_gpios, 1, {0}),                   \
			 GPIO_DT_SPEC_INST_GET_BY_IDX_OR(n, data_gpios, 2, {0}),                   \
			 GPIO_DT_SPEC_INST_GET_BY_IDX_OR(n, data_gpios, 3, {0}),                   \
			 GPIO_DT_SPEC_INST_GET_BY_IDX_OR(n, data_gpios, 4, {0}),                   \
			 GPIO_DT_SPEC_INST_GET_BY_IDX_OR(n, data_gpios, 5, {0}),                   \
			 GPIO_DT_SPEC_INST_GET_BY_IDX_OR(n, data_gpios, 6, {0}),                   \
			 GPIO_DT_SPEC_INST_GET_BY_IDX_OR(n, data_gpios, 7, {0}),                   \
			 GPIO_DT_SPEC_INST_GET_BY_IDX_OR(n, data_gpios, 8, {0}),                   \
			 GPIO_DT_SPEC_INST_GET_BY_IDX_OR(n, data_gpios, 9, {0}),                   \
			 GPIO_DT_SPEC_INST_GET_BY_IDX_OR(n, data_gpios, 10, {0}),                  \
			 GPIO_DT_SPEC_INST_GET_BY_IDX_OR(n, data_gpios, 11, {0}),                  \
			 GPIO_DT_SPEC_INST_GET_BY_IDX_OR(n, data_gpios, 12, {0}),                  \
			 GPIO_DT_SPEC_INST_GET_BY_IDX_OR(n, data_gpios, 13, {0}),                  \
			 GPIO_DT_SPEC_INST_GET_BY_IDX_OR(n, data_gpios, 14, {0}),                  \
			 GPIO_DT_SPEC_INST_GET_BY_IDX_OR(n, data_gpios, 15, {0})},                 \
		.data_bus_width = DT_INST_PROP_LEN(n, data_gpios),                                 \
		.rd = GPIO_DT_SPEC_INST_GET_OR(n, rd_gpios, {}),                                   \
		.wr = GPIO_DT_SPEC_INST_GET_OR(n, wr_gpios, {}),                                   \
		.e = GPIO_DT_SPEC_INST_GET_OR(n, e_gpios, {}),                                     \
		.cs = GPIO_DT_SPEC_INST_GET_OR(n, cs_gpios, {}),                                   \
		.cmd_data = GPIO_DT_SPEC_INST_GET_OR(n, dc_gpios, {}),                             \
		.reset = GPIO_DT_SPEC_INST_GET_OR(n, reset_gpios, {}),                             \
		DATA_LUT_OPTIMIZATION(n)                                                           \
	};                                                                                         \
	BUILD_ASSERT(DT_INST_PROP_LEN(n, data_gpios) < MIPI_DBI_MAX_DATA_BUS_WIDTH,                \
		     "Number of data GPIOs in DT exceeds MIPI_DBI_MAX_DATA_BUS_WIDTH");            \
	static struct mipi_dbi_pico_pio_data mipi_dbi_pico_pio_data_##n;                             \
	DEVICE_DT_INST_DEFINE(n, mipi_dbi_pico_pio_init, NULL, &mipi_dbi_pico_pio_data_##n,          \
			      &mipi_dbi_pico_pio_config_##n, POST_KERNEL,                           \
			      CONFIG_MIPI_DBI_INIT_PRIORITY, &mipi_dbi_pico_pio_driver_api);

DT_INST_FOREACH_STATUS_OKAY(PIO_MIPI_DBI_INIT)
