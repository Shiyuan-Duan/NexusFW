/*
 * mlx90393.c
 * Zephyr driver for dsy,mlx90393 Triaxis® magnetometer (I2C mode)
 *
 * Provides its *own* syscall layer (see mlx90393.h) and does *not* depend
 * on Zephyr's generic sensor subsystem.
 *
 * Assumes MLX90393 is wired in I2C mode:
 *  - MS/CS pin pulled high to VDD (I2C mode)
 *  - SDA/SCL wired to the same I2C bus
 */

#define DT_DRV_COMPAT dsy_mlx90393

#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include <driver/mlx90393.h>

LOG_MODULE_REGISTER(mlx90393, CONFIG_LOG_DEFAULT_LEVEL);

/* ---------- build-time tunables ---------------------------------------- */
#ifndef CONFIG_MLX90393_INIT_PRIORITY
#define CONFIG_MLX90393_INIT_PRIORITY 90
#endif

#define MLX90393_INIT_LEVEL     POST_KERNEL
#define MLX90393_INIT_PRIORITY  CONFIG_MLX90393_INIT_PRIORITY

/* ---------- MLX90393 register / bitfield helpers (根据你给的 bit map) -- */

#define MLX90393_REG0            0x00u
#define MLX90393_REG1            0x01u
#define MLX90393_REG2            0x02u

/* GAIN_SEL[2:0] in REG0 bits [6:4] — 现在先不改，只做掩码备用 */
#define MLX90393_GAIN_SEL_SHIFT  4u
#define MLX90393_GAIN_SEL_MASK   (0x7u << MLX90393_GAIN_SEL_SHIFT)

/* TCMP_EN in REG1 bit 10 */
#define MLX90393_TCMP_EN_SHIFT   10u
#define MLX90393_TCMP_EN_MASK    (1u << MLX90393_TCMP_EN_SHIFT)

/* OSR[1:0] in REG2 bits [1:0] */
#define MLX90393_OSR_SHIFT       0u
#define MLX90393_OSR_MASK        (0x3u << MLX90393_OSR_SHIFT)

/* DIG_FILT[2:0] in REG2 bits [4:2] */
#define MLX90393_DIG_FILT_SHIFT  2u
#define MLX90393_DIG_FILT_MASK   (0x7u << MLX90393_DIG_FILT_SHIFT)

/* RES_XYZ[5:0] in REG2 bits [10:5] — 保持默认，不去动 */
#define MLX90393_RES_XYZ_SHIFT   5u
#define MLX90393_RES_XYZ_MASK    (0x3Fu << MLX90393_RES_XYZ_SHIFT)

/* OSR2[1:0] in REG2 bits [12:11] — 温度 ADC oversampling，保持默认 */
#define MLX90393_OSR2_SHIFT      11u
#define MLX90393_OSR2_MASK       (0x3u << MLX90393_OSR2_SHIFT)

/* OSR / DIG_FILT 的枚举值，便于看代码 */
#define MLX90393_OSR_0           0u
#define MLX90393_OSR_1           1u
#define MLX90393_OSR_2           2u
#define MLX90393_OSR_3           3u

#define MLX90393_DIG_FILT_0      0u
#define MLX90393_DIG_FILT_1      1u
#define MLX90393_DIG_FILT_2      2u
#define MLX90393_DIG_FILT_3      3u
#define MLX90393_DIG_FILT_4      4u
#define MLX90393_DIG_FILT_5      5u
#define MLX90393_DIG_FILT_6      6u
#define MLX90393_DIG_FILT_7      7u

/* ---------- per-instance static config --------------------------------- */

struct mlx90393_config {
	struct i2c_dt_spec i2c;
	struct gpio_dt_spec int_gpio;   /* optional */
	struct gpio_dt_spec reset_gpio; /* optional, usually unused */
};

/* ---------- runtime data ----------------------------------------------- */

struct mlx90393_data {
	struct k_mutex lock;
	uint8_t last_status;
};

/* ---------- small I2C helper ------------------------------------------- */

/*
 * Commands that only return a status byte (SM/SB/SW/EX/HR/HS) — I2C timing
 * per datasheet:
 *
 *   S, addr(W), CMD, Sr, addr(R), Status, NACK, P
 *
 * In Zephyr: i2c_write_read_dt.
 */
static int mlx90393_i2c_cmd_status(const struct device *dev,
				   uint8_t cmd, uint8_t *status_out)
{
	const struct mlx90393_config *cfg = dev->config;
	struct mlx90393_data *data = dev->data;
	uint8_t status = 0;
	int ret;

	k_mutex_lock(&data->lock, K_FOREVER);

	ret = i2c_write_read_dt(&cfg->i2c, &cmd, 1, &status, 1);

	k_mutex_unlock(&data->lock);

	if (ret == 0) {
		data->last_status = status;
		if (status_out) {
			*status_out = status;
		}
	}

	return ret;
}

/* ---------- vtable: SM / RM / RR / WR / EX / RESET --------------------- */

static int mlx90393_sm_backend(const struct device *dev,
			       uint8_t zyxt, uint8_t *status_out)
{
	uint8_t cmd = MLX90393_CMD_SM(zyxt & 0x0F);

	return mlx90393_i2c_cmd_status(dev, cmd, status_out);
}

/*
 * Read measurement (RM) — datasheet: Status-TXYZ in I2C mode
 *
 * I2C timing（简化）:
 *   S, addr(W), CMD_RM(zyxt), Sr, addr(R),
 *      Status, T_hi, T_lo, X_hi, X_lo, ...
 */
static int mlx90393_rm_backend(const struct device *dev, uint8_t zyxt,
			       int16_t *x, int16_t *y,
			       int16_t *z, int16_t *t,
			       uint8_t *status_out)
{
	const struct mlx90393_config *cfg = dev->config;
	struct mlx90393_data *data = dev->data;
	uint8_t mask = zyxt & 0x0F;
	uint8_t n_comp = 0;
	int ret;
	uint8_t idx;

	for (int i = 0; i < 4; i++) {
		if (mask & BIT(i)) {
			n_comp++;
		}
	}
	if (n_comp == 0) {
		return -EINVAL;
	}

	const size_t rx_len = 1U + 2U * n_comp; /* Status + N*2 bytes */
	uint8_t cmd = MLX90393_CMD_RM(mask);
	uint8_t rx[1 + 2 * 4] = {0};

	k_mutex_lock(&data->lock, K_FOREVER);

	ret = i2c_write_read_dt(&cfg->i2c, &cmd, 1, rx, rx_len);
	if (ret == 0) {
		uint8_t status = rx[0];
		data->last_status = status;
		if (status_out) {
			*status_out = status;
		}

		idx = 1;

		if (mask & MLX90393_ZYXT_T) {
			if (t) {
				*t = (int16_t)((rx[idx] << 8) | rx[idx + 1]);
			}
			idx += 2;
		}
		if (mask & MLX90393_ZYXT_X) {
			if (x) {
				*x = (int16_t)((rx[idx] << 8) | rx[idx + 1]);
			}
			idx += 2;
		}
		if (mask & MLX90393_ZYXT_Y) {
			if (y) {
				*y = (int16_t)((rx[idx] << 8) | rx[idx + 1]);
			}
			idx += 2;
		}
		if (mask & MLX90393_ZYXT_Z) {
			if (z) {
				*z = (int16_t)((rx[idx] << 8) | rx[idx + 1]);
			}
			/* idx += 2; */
		}
	}

	k_mutex_unlock(&data->lock);

	return ret;
}

/*
 * RR (Read Register) — I2C timing：
 *
 *   S, addr(W), CMD_RR, (reg<<2),
 *   Sr, addr(R), Status, Data[15:8], Data[7:0], NACK, P
 *
 * We fetch: Status + Data_hi + Data_lo.
 */
static int mlx90393_rr_backend(const struct device *dev,
			       uint8_t reg, uint16_t *val,
			       uint8_t *status_out)
{
	const struct mlx90393_config *cfg = dev->config;
	struct mlx90393_data *data = dev->data;
	uint8_t tx[2];
	uint8_t rx[3];
	int ret;

	if (!val) {
		return -EINVAL;
	}

	tx[0] = MLX90393_CMD_RR;
	tx[1] = (uint8_t)(reg << 2);

	k_mutex_lock(&data->lock, K_FOREVER);

	ret = i2c_write_read_dt(&cfg->i2c, tx, sizeof(tx), rx, sizeof(rx));
	if (ret == 0) {
		uint8_t status = rx[0];

		data->last_status = status;
		if (status_out) {
			*status_out = status;
		}
		*val = (uint16_t)((rx[1] << 8) | rx[2]);
	}

	k_mutex_unlock(&data->lock);

	return ret;
}

/*
 * WR (Write Register) — 常见 I2C 用法：
 *
 *   写:   [CMD_WR, Data_hi, Data_lo, reg<<2]
 *   读:   [Status]
 */
static int mlx90393_wr_backend(const struct device *dev,
			       uint8_t reg, uint16_t val,
			       uint8_t *status_out)
{
	const struct mlx90393_config *cfg = dev->config;
	struct mlx90393_data *data = dev->data;
	uint8_t tx[4];
	uint8_t status = 0;
	int ret;

	tx[0] = MLX90393_CMD_WR;
	tx[1] = (uint8_t)(val >> 8);      /* AH */
	tx[2] = (uint8_t)(val & 0xFF);    /* AL */
	tx[3] = (uint8_t)(reg << 2);      /* register address << 2 */

	k_mutex_lock(&data->lock, K_FOREVER);

	ret = i2c_write_dt(&cfg->i2c, tx, sizeof(tx));
	if (ret == 0) {
		ret = i2c_read_dt(&cfg->i2c, &status, 1);
	}

	k_mutex_unlock(&data->lock);

	if (ret == 0) {
		data->last_status = status;
		if (status_out) {
			*status_out = status;
		}
	}

	return ret;
}

static int mlx90393_ex_backend(const struct device *dev, uint8_t *status_out)
{
	return mlx90393_i2c_cmd_status(dev, MLX90393_CMD_EX, status_out);
}

/*
 * RT: Reset command —— datasheet 里 RT 本身只写命令即可，
 * 真正的 reset 信息通过下一条命令的 Status 里的 RS bit 体现。
 */
static int mlx90393_reset_backend(const struct device *dev,
				  uint8_t *status_out)
{
	ARG_UNUSED(status_out);

	const struct mlx90393_config *cfg = dev->config;
	struct mlx90393_data *data = dev->data;
	int ret;
	uint8_t cmd = MLX90393_CMD_RT;

	/* optional: EX before RT */
	uint8_t tmp;
	(void)mlx90393_ex_backend(dev, &tmp);
	k_sleep(K_MSEC(1));

	k_mutex_lock(&data->lock, K_FOREVER);

	ret = i2c_write_dt(&cfg->i2c, &cmd, 1);

	k_mutex_unlock(&data->lock);

	if (ret < 0) {
		return ret;
	}

	k_sleep(K_MSEC(2));
	data->last_status = 0;

	return 0;
}

/* ---------- driver vtable ---------------------------------------------- */

static const struct mlx90393_driver_api mlx90393_api = {
	.sm    = mlx90393_sm_backend,
	.rm    = mlx90393_rm_backend,
	.rr    = mlx90393_rr_backend,
	.wr    = mlx90393_wr_backend,
	.ex    = mlx90393_ex_backend,
	.reset = mlx90393_reset_backend,
};

/*
 * 可选 quiet mode：
 *  - 打开 TCMP_EN（REG1 bit10）
 *  - 把 OSR / DIG_FILT 调到一个中等偏安静的档位（OSR=2, DIG_FILT=3）
 *
 * 为了安全：
 *  - 只改 TCMP_EN / OSR / DIG_FILT
 *  - 不动 GAIN_SEL / RES_XYZ / OSR2
 */
// static int mlx90393_config_quiet_mode(const struct device *dev)
// {
// 	uint8_t status;
// 	uint16_t reg1, reg2;
// 	int ret;

// 	/* REG1: 打开 TCMP_EN(bit10)，其它 bit 保持原值 */
// 	ret = mlx90393_rr_backend(dev, MLX90393_REG1, &reg1, &status);
// 	if (ret < 0) {
// 		printk("MLX90393: RR(0x01) failed: %d\n", ret);
// 		return ret;
// 	}

// 	reg1 |= MLX90393_TCMP_EN_MASK;

// 	ret = mlx90393_wr_backend(dev, MLX90393_REG1, reg1, &status);
// 	if (ret < 0) {
// 		printk("MLX90393: WR(0x01) failed: %d\n", ret);
// 		return ret;
// 	}

// 	/* REG2: 只改 OSR / DIG_FILT，RES_XYZ / OSR2 不动 */
// 	ret = mlx90393_rr_backend(dev, MLX90393_REG2, &reg2, &status);
// 	if (ret < 0) {
// 		printk("MLX90393: RR(0x02) failed: %d\n", ret);
// 		return ret;
// 	}

// 	/* 清掉 OSR / DIG_FILT 原有值 */
// 	reg2 &= ~MLX90393_OSR_MASK;
// 	reg2 &= ~MLX90393_DIG_FILT_MASK;

// 	/* 设置一个相对保守、不会踩坑的组合：OSR=2, DIG_FILT=3
// 	 * 你如果有 datasheet 上明确推荐的组合，可以把这两行改掉。
// 	 */
// 	reg2 |= (MLX90393_OSR_2      << MLX90393_OSR_SHIFT);
// 	reg2 |= (MLX90393_DIG_FILT_3 << MLX90393_DIG_FILT_SHIFT);

// 	ret = mlx90393_wr_backend(dev, MLX90393_REG2, reg2, &status);
// 	if (ret < 0) {
// 		printk("MLX90393: WR(0x02) failed: %d\n", ret);
// 		return ret;
// 	}

// 	printk("MLX90393: quiet mode OK, REG1=0x%04x REG2=0x%04x\n",
// 	       reg1, reg2);
// 	return 0;
// }

/* ----------- Simple tests (可选) -------------------- */

static void mlx90393_reg_echo_test(const struct device *dev)
{
	uint8_t status;
	uint16_t before = 0, after = 0;
	int ret;

	ret = mlx90393_rr(dev, 0x0A, &before, &status);
	printk("RR(0x0A) before: ret=%d val=0x%04x status=0x%02x\n",
	       ret, before, status);

	ret = mlx90393_wr(dev, 0x0A, 0x5AA5, &status);
	printk("WR(0x0A) write 0x5AA5: ret=%d status=0x%02x\n",
	       ret, status);

	ret = mlx90393_rr(dev, 0x0A, &after, &status);
	printk("RR(0x0A) after:  ret=%d val=0x%04x status=0x%02x\n",
	       ret, after, status);
}

static void mlx90393_measure_once(const struct device *dev)
{
	uint8_t status = 0;
	int16_t x = 0, y = 0, z = 0, t = 0;
	const uint8_t zyxt =
		MLX90393_ZYXT_T |
		MLX90393_ZYXT_X |
		MLX90393_ZYXT_Y |
		MLX90393_ZYXT_Z;

	int ret = mlx90393_sm(dev, zyxt, &status);
	printk("SM ret=%d status=0x%02x\n", ret, status);
	k_sleep(K_MSEC(5));

	ret = mlx90393_rm(dev, zyxt, &x, &y, &z, &t, &status);
	printk("RM ret=%d status=0x%02x, X=%d Y=%d Z=%d T=%d\n",
	       ret, status, x, y, z, t);
}

/* ---------- init -------------------------------------------------------- */

static int mlx90393_init(const struct device *dev)
{
	const struct mlx90393_config *cfg = dev->config;
	struct mlx90393_data *data = dev->data;
	int ret;

	printk("MLX90393(I2C): init on bus=%s addr=0x%02x\n",
	       cfg->i2c.bus->name, cfg->i2c.addr);

	if (!device_is_ready(cfg->i2c.bus)) {
#if IS_ENABLED(CONFIG_LOG)
		LOG_ERR("I2C bus not ready");
#endif
		return -ENODEV;
	}

#define OPT_GPIO_READY(gpio) (!(gpio).port || device_is_ready((gpio).port))
	if (!OPT_GPIO_READY(cfg->int_gpio) ||
	    !OPT_GPIO_READY(cfg->reset_gpio)) {
#if IS_ENABLED(CONFIG_LOG)
		LOG_ERR("One or more GPIO controllers not ready");
#endif
		return -ENODEV;
	}
#undef OPT_GPIO_READY

	if (cfg->int_gpio.port) {
		ret = gpio_pin_configure_dt(&cfg->int_gpio, GPIO_INPUT);
		if (ret < 0) {
#if IS_ENABLED(CONFIG_LOG)
			LOG_ERR("Failed to config INT gpio: %d", ret);
#endif
			return ret;
		}
	}

	if (cfg->reset_gpio.port) {
		ret = gpio_pin_configure_dt(&cfg->reset_gpio,
					    GPIO_OUTPUT_INACTIVE);
		if (ret < 0) {
#if IS_ENABLED(CONFIG_LOG)
			LOG_ERR("Failed to config RESET gpio: %d", ret);
#endif
			return ret;
		}
	}

	k_mutex_init(&data->lock);
	data->last_status = 0;

	if (cfg->reset_gpio.port) {
		printk("MLX90393(I2C): pulsing RESET pin\n");
		gpio_pin_set_dt(&cfg->reset_gpio, 1);
		k_sleep(K_MSEC(1));
		gpio_pin_set_dt(&cfg->reset_gpio, 0);
		k_sleep(K_MSEC(5));
	}

	/* Quick comms sanity: EX + RR(0x00) */
	uint8_t status = 0;
	ret = mlx90393_ex_backend(dev, &status);
	printk("MLX90393(I2C): EX ret=%d status=0x%02x\n", ret, status);
	if (ret < 0) {
#if IS_ENABLED(CONFIG_LOG)
		LOG_ERR("MLX90393 EX failed: %d", ret);
#endif
		return ret;
	}

	uint16_t reg0 = 0;
	ret = mlx90393_rr_backend(dev, 0x00, &reg0, &status);
	printk("MLX90393(I2C): RR(0x00) ret=%d reg0=0x%04x status=0x%02x\n",
	       ret, reg0, status);
	if (ret < 0) {
#if IS_ENABLED(CONFIG_LOG)
		LOG_ERR("MLX90393 RR(0x00) failed: %d", ret);
#endif
		return ret;
	}

	/* debugging helpers — 调通后可以删掉 */
	mlx90393_reg_echo_test(dev);
	mlx90393_measure_once(dev);

	/* 可选：在 init 阶段打开 quiet mode
	 * 先保持 #if 0，确认基础功能都正常后，你可以改成 #if 1 测试噪声下降情况。
	 */
// #if 0
// 	ret = mlx90393_config_quiet_mode(dev);
// 	if (ret < 0) {
// 		printk("MLX90393: quiet mode config failed: %d\n", ret);
// 	}
// #endif

#if IS_ENABLED(CONFIG_LOG)
	LOG_INF("MLX90393(I2C) initialized OK");
#endif
	return 0;
}

/* ---------- DT-driven instantiation helper ----------------------------- */

#define MLX90393_CONFIG_I2C(inst)                                       \
	{                                                               \
		.i2c        = I2C_DT_SPEC_INST_GET(inst),               \
		.int_gpio   = GPIO_DT_SPEC_INST_GET_OR(inst, int_gpios, {0}), \
		.reset_gpio = GPIO_DT_SPEC_INST_GET_OR(inst, reset_gpios, {0}), \
	}

#define MLX90393_DEFINE(inst)                                          \
	static struct mlx90393_data mlx90393_data_##inst;              \
	static const struct mlx90393_config mlx90393_cfg_##inst =      \
		MLX90393_CONFIG_I2C(inst);                             \
	DEVICE_DT_INST_DEFINE(inst, mlx90393_init, NULL,               \
			      &mlx90393_data_##inst,                \
			      &mlx90393_cfg_##inst,                 \
			      MLX90393_INIT_LEVEL,                  \
			      MLX90393_INIT_PRIORITY,               \
			      &mlx90393_api);

DT_INST_FOREACH_STATUS_OKAY(MLX90393_DEFINE)
