/*
 * AD5778R Zephyr driver (32-bit aligned, per datasheet SDO pipeline)
 *
 * Key datasheet facts:
 *  - SDO is driven by a 32-bit shift register and output is delayed by 32 SCK rising edges.
 *  - Recommended: use 32-bit transactions: 8 "don't care" bits + 24-bit command word.
 *  - SDO data you observe corresponds to the PREVIOUS transaction (pipeline).
 */

#define DT_DRV_COMPAT dsy_ad5778r

#include <zephyr/device.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <errno.h>

#include <driver/ad5778r.h>

LOG_MODULE_REGISTER(ad5778r, LOG_LEVEL_INF);

/* Mode 0: CPOL=0, CPHA=0; MSB-first; 8-bit words */
#define AD5778R_SPI_OP (SPI_WORD_SET(8) | SPI_TRANSFER_MSB)

static inline uint8_t dac_addr(uint8_t ch)
{
	return (ch == 0) ? AD5778R_ADDR_DAC0 : AD5778R_ADDR_DAC1;
}

struct ad5778r_config {
	struct spi_dt_spec spi;
};

struct ad5778r_data {
	struct k_mutex lock;
	uint8_t last_fr;
	bool fr_valid;
};

/* 32-bit transaction:
 *   TX[0] = don't care
 *   TX[1] = (cmd<<4) | addr
 *   TX[2] = data[15:8]
 *   TX[3] = data[7:0]
 *
 * RX[0..3] corresponds to the previous transaction due to the 32-bit pipeline. :contentReference[oaicite:3]{index=3}
 */
static int spi_xfer32(const struct device *dev, const uint8_t tx[4], uint8_t rx[4])
{
	const struct ad5778r_config *cfg = dev->config;

	struct spi_buf txb = { .buf = (void *)tx, .len = 4 };
	struct spi_buf rxb = { .buf = rx, .len = 4 };
	struct spi_buf_set txs = { .buffers = &txb, .count = 1 };
	struct spi_buf_set rxs = { .buffers = &rxb, .count = 1 };

	return spi_transceive_dt(&cfg->spi, &txs, &rxs);
}

static void build_tx32(uint8_t cmd, uint8_t addr, uint16_t data, uint8_t tx[4])
{
	tx[0] = 0x00; /* don't care */
	tx[1] = (uint8_t)(((cmd & 0xF) << 4) | (addr & 0xF));
	tx[2] = (uint8_t)(data >> 8);
	tx[3] = (uint8_t)(data & 0xFF);
}

/* Execute one command, return:
 *  - ret code
 *  - optionally provides the "previous transaction" RX word (pipeline).
 */
static int exec32(const struct device *dev, uint8_t cmd, uint8_t addr, uint16_t data,
		  uint8_t prev_rx[4])
{
	struct ad5778r_data *d = dev->data;

	uint8_t tx[4], rx[4] = {0};
	build_tx32(cmd, addr, data, tx);

	int ret = spi_xfer32(dev, tx, rx);
	if (ret == 0) {
		/* rx is previous transaction's response */
		if (prev_rx) {
			prev_rx[0] = rx[0];
			prev_rx[1] = rx[1];
			prev_rx[2] = rx[2];
			prev_rx[3] = rx[3];
		}
		d->last_fr = rx[0];
		d->fr_valid = true;
	}
	return ret;
}

/* Read FR reliably:
 * Because of the 32-bit pipeline, you need two NOPs and take the 2nd RX.
 */
static int read_fr_pipelined(const struct device *dev, uint8_t *fr_out)
{
	uint8_t rx1[4], rx2[4];
	int ret;

	/* First NOP clocks out previous (unknown) */
	ret = exec32(dev, AD5778R_CMD_NOP, 0, 0, rx1);
	if (ret) return ret;

	/* Second NOP clocks out the response to the first NOP */
	ret = exec32(dev, AD5778R_CMD_NOP, 0, 0, rx2);
	if (ret) return ret;

	if (fr_out) {
		*fr_out = rx2[0];
	}
	LOG_INF("FR(pipelined)=0x%02x (FR4=%d FR3=%d FR2=%d FR1=%d FR0=%d) rx=[%02x %02x %02x %02x]",
		rx2[0],
		!!(rx2[0] & BIT(4)),
		!!(rx2[0] & BIT(3)),
		!!(rx2[0] & BIT(2)),
		!!(rx2[0] & BIT(1)),
		!!(rx2[0] & BIT(0)),
		rx2[0], rx2[1], rx2[2], rx2[3]);

	return 0;
}

/* ======= API backends ======= */

static int write_backend(const struct device *dev, uint8_t ch, uint16_t code)
{
	if (ch > 1) return -EINVAL;

	/* WRITE_CODE_UPDATE_N: cmd=0x3, addr=DACn */
	return exec32(dev, AD5778R_CMD_WRITE_CODE_UPDATE_N, dac_addr(ch), code, NULL);
}

static int write_input_backend(const struct device *dev, uint8_t ch, uint16_t code)
{
	if (ch > 1) return -EINVAL;
	return exec32(dev, AD5778R_CMD_WRITE_CODE_N, dac_addr(ch), code, NULL);
}

static int update_backend(const struct device *dev, uint8_t ch)
{
	if (ch > 1) return -EINVAL;
	return exec32(dev, AD5778R_CMD_UPDATE_N, dac_addr(ch), 0, NULL);
}

static int update_all_backend(const struct device *dev)
{
	return exec32(dev, AD5778R_CMD_UPDATE_ALL, 0, 0, NULL);
}

static int set_span_backend(const struct device *dev, uint8_t ch, uint8_t span)
{
	if (ch > 1) return -EINVAL;

	/* WRITE_SPAN_N then UPDATE_N to apply span (datasheet) */
	int ret = exec32(dev, AD5778R_CMD_WRITE_SPAN_N, dac_addr(ch), (span & 0x0F), NULL);
	if (ret) return ret;
	return exec32(dev, AD5778R_CMD_UPDATE_N, dac_addr(ch), 0, NULL);
}

static int set_span_input_backend(const struct device *dev, uint8_t ch, uint8_t span)
{
	if (ch > 1) return -EINVAL;
	return exec32(dev, AD5778R_CMD_WRITE_SPAN_N, dac_addr(ch), (span & 0x0F), NULL);
}

static int power_down_backend(const struct device *dev, uint8_t ch)
{
	if (ch > 1) return -EINVAL;
	return exec32(dev, AD5778R_CMD_POWER_DOWN_N, dac_addr(ch), 0, NULL);
}

static int power_down_chip_backend(const struct device *dev)
{
	return exec32(dev, AD5778R_CMD_POWER_DOWN_CHIP, 0, 0, NULL);
}

static int config_backend(const struct device *dev, uint8_t cfg_bits)
{
	return exec32(dev, AD5778R_CMD_CONFIG, 0, (cfg_bits & 0x0F), NULL);
}

static int monitor_mux_backend(const struct device *dev, uint8_t mux_code)
{
	return exec32(dev, AD5778R_CMD_MONITOR_MUX, 0, (mux_code & 0x1F), NULL);
}

static int read_fault_backend(const struct device *dev, uint8_t *fault)
{
	return read_fr_pipelined(dev, fault);
}

static int get_last_fault_backend(const struct device *dev, uint8_t *fault)
{
	struct ad5778r_data *d = dev->data;
	if (!fault) return -EINVAL;
	if (!d->fr_valid) return -EAGAIN;
	*fault = d->last_fr;
	return 0;
}

static int ldac_backend(const struct device *dev) { ARG_UNUSED(dev); return -ENOTSUP; }
static int clr_backend(const struct device *dev)  { ARG_UNUSED(dev); return -ENOTSUP; }

static const struct ad5778r_driver_api api = {
	.write          = write_backend,
	.write_input    = write_input_backend,
	.update         = update_backend,
	.update_all     = update_all_backend,
	.set_span       = set_span_backend,
	.set_span_input = set_span_input_backend,
	.power_down     = power_down_backend,
	.power_down_chip= power_down_chip_backend,
	.config         = config_backend,
	.monitor_mux    = monitor_mux_backend,
	.read_fault     = read_fault_backend,
	.get_last_fault = get_last_fault_backend,
	.ldac           = ldac_backend,
	.clr            = clr_backend,
};

static int ad5778r_init(const struct device *dev)
{
	const struct ad5778r_config *cfg = dev->config;
	struct ad5778r_data *d = dev->data;

	if (!device_is_ready(cfg->spi.bus)) {
		LOG_ERR("SPI bus not ready");
		return -ENODEV;
	}

	k_mutex_init(&d->lock);
	d->fr_valid = false;
	d->last_fr = 0;

	LOG_INF("AD5778R init (32-bit aligned)");

	/* Read FR once (pipelined) so we know SDO path is meaningful */
	uint8_t fr;
	(void)read_fr_pipelined(dev, &fr);

	return 0;
}

#define AD5778R_DEFINE(inst)                                                   \
	static struct ad5778r_data data_##inst;                                \
	static const struct ad5778r_config cfg_##inst = {                      \
		.spi = SPI_DT_SPEC_INST_GET(inst, AD5778R_SPI_OP, 0),           \
	};                                                                      \
	DEVICE_DT_INST_DEFINE(inst, ad5778r_init, NULL,                        \
			      &data_##inst, &cfg_##inst, POST_KERNEL, 90, &api);

DT_INST_FOREACH_STATUS_OKAY(AD5778R_DEFINE)
