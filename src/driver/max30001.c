/*
 * MAX30001 Zephyr driver (custom API, SPI 8+24 framing)
 *
 * Notes:
 *  - SPI frame: 8-bit command + 24-bit data.
 *  - For FIFO streaming, we use NORMAL reads (1 word / transaction) so we can stop
 *    precisely at EOF/EMPTY and avoid over-reading.
 */

#define DT_DRV_COMPAT dsy_max30001

#include <zephyr/device.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>
#include <errno.h>

#include <driver/max30001.h>

LOG_MODULE_REGISTER(max30001, LOG_LEVEL_INF);

/* Mode 0, MSB first, 8-bit words */
#define MAX30001_SPI_OP (SPI_WORD_SET(8) | SPI_TRANSFER_MSB)

static inline uint8_t max30001_cmd(uint8_t reg, bool rd)
{
	/* Command byte is: A6..A0 then R/W bit (LSB) */
	return (uint8_t)((reg << 1) | (rd ? 1u : 0u));
}

struct max30001_config {
	struct spi_dt_spec spi;
	struct gpio_dt_spec int_gpio;
	struct gpio_dt_spec int2_gpio;

	/* devicetree defaults (raw codes) */
	uint8_t fmstr;
	uint8_t ecg_rate;
	uint8_t bioz_rate;
	uint8_t bioz_cgmag;
	uint8_t bioz_fcgen;
	bool enable_rbias_ecg;
};

struct max30001_data {
	struct k_mutex lock;
};

static int spi_xfer(const struct device *dev, const uint8_t *tx, uint8_t *rx, size_t len)
{
	const struct max30001_config *cfg = dev->config;

	struct spi_buf txb = { .buf = (void *)tx, .len = len };
	struct spi_buf rxb = { .buf = rx, .len = len };
	struct spi_buf_set txs = { .buffers = &txb, .count = 1 };
	struct spi_buf_set rxs = { .buffers = &rxb, .count = 1 };

	return spi_transceive_dt(&cfg->spi, &txs, &rxs);
}

static int write_reg_locked(const struct device *dev, uint8_t reg, uint32_t val24)
{
	uint8_t tx[4];
	uint8_t rx[4];

	val24 &= 0xFFFFFFu;

	tx[0] = max30001_cmd(reg, false);
	tx[1] = (uint8_t)(val24 >> 16);
	tx[2] = (uint8_t)(val24 >> 8);
	tx[3] = (uint8_t)(val24);

	return spi_xfer(dev, tx, rx, sizeof(tx));
}

static int read_reg_locked(const struct device *dev, uint8_t reg, uint32_t *val24)
{
	uint8_t tx[4] = { max30001_cmd(reg, true), 0, 0, 0 };
	uint8_t rx[4] = { 0 };

	int ret = spi_xfer(dev, tx, rx, sizeof(tx));
	if (ret == 0 && val24) {
		/* rx[0] is don't-care/Z during command byte, data is rx[1..3] */
		*val24 = ((uint32_t)rx[1] << 16) | ((uint32_t)rx[2] << 8) | ((uint32_t)rx[3]);
	}
	return ret;
}

/* ────────── backends ────────── */

static int write_reg_backend(const struct device *dev, uint8_t reg, uint32_t val24)
{
	struct max30001_data *d = dev->data;
	k_mutex_lock(&d->lock, K_FOREVER);
	int ret = write_reg_locked(dev, reg, val24);
	k_mutex_unlock(&d->lock);
	return ret;
}

static int read_reg_backend(const struct device *dev, uint8_t reg, uint32_t *val24)
{
	struct max30001_data *d = dev->data;
	if (!val24) return -EINVAL;
	k_mutex_lock(&d->lock, K_FOREVER);
	int ret = read_reg_locked(dev, reg, val24);
	k_mutex_unlock(&d->lock);
	return ret;
}

static int sw_reset_backend(const struct device *dev)
{
	struct max30001_data *d = dev->data;
	k_mutex_lock(&d->lock, K_FOREVER);
	int ret = write_reg_locked(dev, MAX30001_REG_SW_RST, 0);
	k_mutex_unlock(&d->lock);

	/* give the digital some time */
	k_sleep(K_MSEC(10));
	return ret;
}

static int synch_backend(const struct device *dev)
{
	struct max30001_data *d = dev->data;
	k_mutex_lock(&d->lock, K_FOREVER);
	int ret = write_reg_locked(dev, MAX30001_REG_SYNCH, 0);
	k_mutex_unlock(&d->lock);
	return ret;
}

static int fifo_reset_backend(const struct device *dev)
{
	struct max30001_data *d = dev->data;
	k_mutex_lock(&d->lock, K_FOREVER);
	int ret = write_reg_locked(dev, MAX30001_REG_FIFO_RST, 0);
	k_mutex_unlock(&d->lock);
	return ret;
}

/* helper: build default register words from devicetree */
static void build_defaults(const struct device *dev,
			   bool en_ecg, bool en_bioz,
			   uint32_t *cnfg_gen_no_rbias,
			   uint32_t *cnfg_gen_with_rbias,
			   uint32_t *cnfg_ecg,
			   uint32_t *cnfg_bioz,
			   uint32_t *cnfg_emux_close,
			   uint32_t *cnfg_emux_open,
			   uint32_t *cnfg_bmux_close,
			   uint32_t *cnfg_bmux_open)
{
	const struct max30001_config *cfg = dev->config;

	/* CNFG_GEN: only what we need for streaming (FMSTR + EN bits + optional RBIAS) */
	uint32_t gen = ((uint32_t)(cfg->fmstr & 0x3u) << 20);
	if (en_ecg)  gen |= BIT(19);
	if (en_bioz) gen |= BIT(18);

	*cnfg_gen_no_rbias = gen;

	/* enable rbias on ECG if requested (EN_RBIAS=01, RBIASV=01, RBIASP/RBIASN=1) */
	if (cfg->enable_rbias_ecg && en_ecg) {
		gen |= (1u << 4); /* EN_RBIAS=01 */
		gen |= (1u << 2); /* RBIASV=01 (100M) */
		gen |= BIT(1) | BIT(0);
	}
	*cnfg_gen_with_rbias = gen;

	/* CNFG_ECG: ECG_RATE + defaults (GAIN=00, DHPF=1, DLPF=01) */
	*cnfg_ecg = ((uint32_t)(cfg->ecg_rate & 0x3u) << 22) |
		    BIT(14) | (1u << 12);

	/* CNFG_BIOZ: defaults + user cgmag/fcgen */
	/* BIOZ_AHPF default 010 (800Hz), DLPF default 01 (4Hz), others 0. */
	*cnfg_bioz = ((uint32_t)(cfg->bioz_rate & 0x1u) << 23) |
		     ((uint32_t)0x2u << 20) |
		     (1u << 12) |
		     ((uint32_t)(cfg->bioz_fcgen & 0xFu) << 8) |
		     ((uint32_t)(cfg->bioz_cgmag & 0x7u) << 4);

	/* EMUX: close ECG switches => write 0, open => set OPENP/OPENN bits (D21/D20) */
	*cnfg_emux_close = 0x000000;
	*cnfg_emux_open  = BIT(21) | BIT(20);

	/* BMUX: keep RMOD default (100 at D[6:4]) => 0x40. close => OPEN bits 0, open => set D21/D20. */
	*cnfg_bmux_close = 0x000040;
	*cnfg_bmux_open  = (BIT(21) | BIT(20) | 0x000040);
}

static int stream_on_backend(const struct device *dev, bool enable_ecg, bool enable_bioz)
{
	struct max30001_data *d = dev->data;

	uint32_t gen0, gen1, ecg, bioz, emux_close, emux_open, bmux_close, bmux_open;
	build_defaults(dev, enable_ecg, enable_bioz, &gen0, &gen1, &ecg, &bioz,
		       &emux_close, &emux_open, &bmux_close, &bmux_open);

	k_mutex_lock(&d->lock, K_FOREVER);

	/* clear FIFOs first */
	int ret = write_reg_locked(dev, MAX30001_REG_FIFO_RST, 0);
	if (ret) goto out;

	/* enable blocks first (without rbias) */
	ret = write_reg_locked(dev, MAX30001_REG_CNFG_GEN, gen0);
	if (ret) goto out;

	/* close input switches only if that channel enabled */
	ret = write_reg_locked(dev, MAX30001_REG_CNFG_EMUX, enable_ecg ? emux_close : emux_open);
	if (ret) goto out;

	ret = write_reg_locked(dev, MAX30001_REG_CNFG_BMUX, enable_bioz ? bmux_close : bmux_open);
	if (ret) goto out;

	/* program channel configs */
	if (enable_ecg) {
		ret = write_reg_locked(dev, MAX30001_REG_CNFG_ECG, ecg);
		if (ret) goto out;
	}
	if (enable_bioz) {
		ret = write_reg_locked(dev, MAX30001_REG_CNFG_BIOZ, bioz);
		if (ret) goto out;
	}

	/* now apply rbias (if enabled in DT) */
	ret = write_reg_locked(dev, MAX30001_REG_CNFG_GEN, gen1);
	if (ret) goto out;

	/* resync after configuration changes */
	(void)write_reg_locked(dev, MAX30001_REG_SYNCH, 0);

	/* start clean */
	(void)write_reg_locked(dev, MAX30001_REG_FIFO_RST, 0);

out:
	k_mutex_unlock(&d->lock);
	return ret;
}

static int stream_off_backend(const struct device *dev)
{
	struct max30001_data *d = dev->data;
	const struct max30001_config *cfg = dev->config;

	/* CNFG_GEN: keep FMSTR, disable channels */
	uint32_t gen_off = ((uint32_t)(cfg->fmstr & 0x3u) << 20);

	/* open switches to isolate pins when off */
	uint32_t emux_open = BIT(21) | BIT(20);
	uint32_t bmux_open = BIT(21) | BIT(20) | 0x000040;

	/* also force BioZ current generator magnitude to OFF (safe) */
	uint32_t bioz_off = ((uint32_t)(cfg->bioz_rate & 0x1u) << 23) |
			    ((uint32_t)0x2u << 20) |
			    (1u << 12) |
			    ((uint32_t)(cfg->bioz_fcgen & 0xFu) << 8) |
			    (0u << 4);

	k_mutex_lock(&d->lock, K_FOREVER);

	int ret = write_reg_locked(dev, MAX30001_REG_CNFG_GEN, gen_off);
	if (ret) goto out;

	(void)write_reg_locked(dev, MAX30001_REG_CNFG_EMUX, emux_open);
	(void)write_reg_locked(dev, MAX30001_REG_CNFG_BMUX, bmux_open);
	(void)write_reg_locked(dev, MAX30001_REG_CNFG_BIOZ, bioz_off);

	(void)write_reg_locked(dev, MAX30001_REG_FIFO_RST, 0);

out:
	k_mutex_unlock(&d->lock);
	return ret;
}

static int read_ecg_word_backend(const struct device *dev, uint32_t *w24)
{
	struct max30001_data *d = dev->data;
	if (!w24) return -EINVAL;

	k_mutex_lock(&d->lock, K_FOREVER);
	int ret = read_reg_locked(dev, MAX30001_REG_ECG_FIFO, w24);
	k_mutex_unlock(&d->lock);
	return ret;
}

static int read_bioz_word_backend(const struct device *dev, uint32_t *w24)
{
	struct max30001_data *d = dev->data;
	if (!w24) return -EINVAL;

	k_mutex_lock(&d->lock, K_FOREVER);
	int ret = read_reg_locked(dev, MAX30001_REG_BIOZ_FIFO, w24);
	k_mutex_unlock(&d->lock);
	return ret;
}

static const struct max30001_driver_api api = {
	.write_reg      = write_reg_backend,
	.read_reg       = read_reg_backend,
	.sw_reset       = sw_reset_backend,
	.synch          = synch_backend,
	.fifo_reset     = fifo_reset_backend,
	.stream_on      = stream_on_backend,
	.stream_off     = stream_off_backend,
	.read_ecg_word  = read_ecg_word_backend,
	.read_bioz_word = read_bioz_word_backend,
};

static int max30001_init(const struct device *dev)
{
	const struct max30001_config *cfg = dev->config;
	struct max30001_data *d = dev->data;

	if (!device_is_ready(cfg->spi.bus)) {
		LOG_ERR("SPI bus not ready");
		return -ENODEV;
	}

	if (cfg->int_gpio.port && !device_is_ready(cfg->int_gpio.port)) {
		LOG_ERR("INT GPIO port not ready");
		return -ENODEV;
	}
	if (cfg->int2_gpio.port && !device_is_ready(cfg->int2_gpio.port)) {
		LOG_ERR("INT2 GPIO port not ready");
		return -ENODEV;
	}

	/* optional: configure interrupt pins as inputs with pull-up */
	if (cfg->int_gpio.port) {
		(void)gpio_pin_configure_dt(&cfg->int_gpio, GPIO_INPUT | GPIO_PULL_UP);
	}
	if (cfg->int2_gpio.port) {
		(void)gpio_pin_configure_dt(&cfg->int2_gpio, GPIO_INPUT | GPIO_PULL_UP);
	}

	k_mutex_init(&d->lock);

	/* default safe state */
	(void)stream_off_backend(dev);

	/* read INFO for sanity */
	uint32_t info;
	if (read_reg_backend(dev, MAX30001_REG_INFO, &info) == 0) {
		LOG_INF("MAX30001 INFO=0x%06x", (unsigned)info);
	}

	LOG_INF("MAX30001 init ok");
	return 0;
}

#define MAX30001_DEFINE(inst)                                                     \
	static struct max30001_data data_##inst;                                  \
	static const struct max30001_config cfg_##inst = {                        \
		.spi = SPI_DT_SPEC_INST_GET(inst, MAX30001_SPI_OP, 0),             \
		.int_gpio = GPIO_DT_SPEC_INST_GET_OR(inst, int_gpios, {0}),        \
		.int2_gpio = GPIO_DT_SPEC_INST_GET_OR(inst, int2_gpios, {0}),      \
		.fmstr = (uint8_t)DT_INST_PROP(inst, fmstr),                       \
		.ecg_rate = (uint8_t)DT_INST_PROP(inst, ecg_rate),                 \
		.bioz_rate = (uint8_t)DT_INST_PROP(inst, bioz_rate),               \
		.bioz_cgmag = (uint8_t)DT_INST_PROP(inst, bioz_cgmag),             \
		.bioz_fcgen = (uint8_t)DT_INST_PROP(inst, bioz_fcgen),             \
		.enable_rbias_ecg = DT_INST_PROP_OR(inst, enable_rbias_ecg, 0),    \
	};                                                                        \
	DEVICE_DT_INST_DEFINE(inst, max30001_init, NULL,                          \
			      &data_##inst, &cfg_##inst, POST_KERNEL, 90, &api);

DT_INST_FOREACH_STATUS_OKAY(MAX30001_DEFINE)