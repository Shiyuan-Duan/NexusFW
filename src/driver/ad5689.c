/*
 * Zephyr driver for dsy,ad5689 / dsy,ad5687 dual nanoDAC+
 *
 * Provides its *own* syscall layer (see ad5689.h) and does *not* depend on
 * Zephyr's generic DAC subsystem.
 */

#define DT_DRV_COMPAT dsy_ad5689

#include <zephyr/device.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <driver/ad5689.h>

LOG_MODULE_REGISTER(ad5689, CONFIG_LOG_DEFAULT_LEVEL);

/* ---------- build‑time tunables ---------------------------------------- */
#ifndef CONFIG_AD5689_INIT_PRIORITY
/* fallback if user didn’t set it in prj.conf */
#define CONFIG_AD5689_INIT_PRIORITY 90
#endif

#define AD5689_INIT_LEVEL     POST_KERNEL
#define AD5689_INIT_PRIORITY  CONFIG_AD5689_INIT_PRIORITY

/* ---------- SPI operation: 8‑bit words, MSB‑first, CPHA = 1 (mode‑1) ---- */
#define AD5689_SPI_OP (SPI_WORD_SET(8) | SPI_TRANSFER_MSB | SPI_MODE_CPHA)

/* ---------- Address bits (A3‑A0) --------------------------------------- */
#define AD5689_ADDR_DAC_A     0x1
#define AD5689_ADDR_DAC_B     0x8
#define AD5689_ADDR_DAC_BOTH  0x9

/* ---------- 24‑bit frame helper ---------------------------------------- */
#define AD5689_FRAME(cmd, addr, data16) \
	(((uint32_t)(cmd)  & 0xF) << 20 | \
	 ((uint32_t)(addr) & 0xF) << 16 | \
	 ((uint32_t)(data16) & 0xFFFF))

/* ---------- per‑instance static config --------------------------------- */
struct ad5689_config {
	struct spi_dt_spec spi;
	struct gpio_dt_spec ldac;
	struct gpio_dt_spec clr;
	struct gpio_dt_spec reset;
};

/* ---------- runtime data ----------------------------------------------- */
struct ad5689_data {
	struct k_mutex lock;
	uint16_t cached[2];
};

/* ---------- low‑level 24‑bit SPI write --------------------------------- */
static int ad5689_spi_write24(const struct device *dev, uint32_t frame)
{
	const struct ad5689_config *cfg = dev->config;
	uint8_t tx[3] = { frame >> 16, frame >> 8, frame };

	struct spi_buf buf = { .buf = tx, .len = 3 };
	struct spi_buf_set txs = { .buffers = &buf, .count = 1 };

	return spi_write_dt(&cfg->spi, &txs);
}

/* ---------- core write backend (syscall) -------------------------------- */
static int ad5689_write_backend(const struct device *dev,
				uint8_t channel, uint16_t code)
{
	if (channel > 1) {
		return -EINVAL;
	}
	const uint8_t addr = channel ? AD5689_ADDR_DAC_B : AD5689_ADDR_DAC_A;

	struct ad5689_data *data = dev->data;
	k_mutex_lock(&data->lock, K_FOREVER);
	data->cached[channel] = code;
	int ret = ad5689_spi_write24(dev,
			AD5689_FRAME(AD5689_CMD_WRITE_UPDATE_N, addr, code));
	k_mutex_unlock(&data->lock);
	return ret;
}

/* ---------- software reset & power‑down helpers ------------------------- */
static int ad5689_soft_reset(const struct device *dev)
{
	return ad5689_spi_write24(dev, AD5689_FRAME(AD5689_CMD_SW_RESET, 0, 0));
}

static int ad5689_power_mode(const struct device *dev, uint8_t mode)
{
	/* mode encodes PD1:PD0 for both channels – caller validates */
	return ad5689_spi_write24(dev,
		AD5689_FRAME(AD5689_CMD_POWERDOWN_DAC, 0, mode & 0xFFFF));
}

/* ---------- driver vtable ---------------------------------------------- */
static const struct ad5689_driver_api ad5689_api = {
	.write        = ad5689_write_backend,
	.reset        = ad5689_soft_reset,
	.ldac         = NULL,
	.set_pd_mode  = ad5689_power_mode,
	.internal_ref = NULL,
	.read_dac     = NULL,
};

/* ---------- init -------------------------------------------------------- */
static int ad5689_init(const struct device *dev)
{
	const struct ad5689_config *cfg = dev->config;
	struct ad5689_data *data = dev->data;

	if (!device_is_ready(cfg->spi.bus)) {
		LOG_ERR("SPI bus not ready");
		return -ENODEV;
	}

#define OPT_GPIO_READY(gpio) (!(gpio).port || device_is_ready((gpio).port))
	if (!OPT_GPIO_READY(cfg->ldac) ||
	    !OPT_GPIO_READY(cfg->clr)  ||
	    !OPT_GPIO_READY(cfg->reset)) {
		return -ENODEV;
	}
#undef OPT_GPIO_READY

	/* optional GPIOs as outputs, default inactive */
	if (cfg->ldac.port)  gpio_pin_configure_dt(&cfg->ldac,  GPIO_OUTPUT_INACTIVE);
	if (cfg->clr.port)   gpio_pin_configure_dt(&cfg->clr,   GPIO_OUTPUT_INACTIVE);
	if (cfg->reset.port) gpio_pin_configure_dt(&cfg->reset, GPIO_OUTPUT_INACTIVE);

	k_mutex_init(&data->lock);

	// /* hardware reset pulse if pin provided */
	// if (cfg->reset.port) {
	// 	gpio_pin_set_dt(&cfg->reset, 0);
	// 	k_sleep(K_MSEC(1));
	// 	gpio_pin_set_dt(&cfg->reset, 1);
	// 	k_sleep(K_MSEC(1));
	// }
    printk("Init DAC\n");
	return ad5689_soft_reset(dev);
}

/* ---------- DT‑driven instantiation helper ----------------------------- */
#define AD5689_CONFIG_SPI(inst)                                       \
	{                                                                 \
		.spi   = SPI_DT_SPEC_INST_GET(inst, AD5689_SPI_OP, 0),    \
		.ldac  = GPIO_DT_SPEC_INST_GET_OR(inst, ldac_gpios,  {0}),\
		.clr   = GPIO_DT_SPEC_INST_GET_OR(inst, clr_gpios,   {0}),\
		.reset = GPIO_DT_SPEC_INST_GET_OR(inst, reset_gpios, {0}),\
	}

#define AD5689_DEFINE(inst)                                            \
	static struct ad5689_data ad5689_data_##inst;                  \
	static const struct ad5689_config ad5689_cfg_##inst =          \
		AD5689_CONFIG_SPI(inst);                               \
	DEVICE_DT_INST_DEFINE(inst, ad5689_init, NULL,                 \
			      &ad5689_data_##inst, &ad5689_cfg_##inst,  \
			      AD5689_INIT_LEVEL, AD5689_INIT_PRIORITY, \
			      &ad5689_api);

DT_INST_FOREACH_STATUS_OKAY(AD5689_DEFINE)
