#include <zephyr/kernel.h>
#include <zephyr/drivers/i2c.h>

#include <errno.h>
#include <string.h>

#include <driver/mlx90397.h>

static int mlx90397_read_regs(const struct i2c_dt_spec *i2c,
			      uint8_t start_reg, uint8_t *buf, size_t len)
{
	/* Addressed read requires repeated START; i2c_write_read_dt provides it. */
	return i2c_write_read_dt(i2c, &start_reg, 1, buf, len);
}

int mlx90397_read_u8(const struct i2c_dt_spec *i2c, uint8_t reg, uint8_t *val)
{
	if (!i2c || !val) {
		return -EINVAL;
	}
	return mlx90397_read_regs(i2c, reg, val, 1);
}

int mlx90397_write_u8(const struct i2c_dt_spec *i2c, uint8_t reg, uint8_t val)
{
	if (!i2c) {
		return -EINVAL;
	}
	uint8_t tx[2] = { reg, val };
	return i2c_write_dt(i2c, tx, sizeof(tx));
}

int mlx90397_read_ids(const struct i2c_dt_spec *i2c, uint8_t *cid, uint8_t *did)
{
	int ret;
	uint8_t v;

	if (!i2c) {
		return -EINVAL;
	}

	if (cid) {
		ret = mlx90397_read_u8(i2c, MLX90397_REG_CID, &v);
		if (ret < 0) {
			return ret;
		}
		*cid = v;
	}

	if (did) {
		ret = mlx90397_read_u8(i2c, MLX90397_REG_DID, &v);
		if (ret < 0) {
			return ret;
		}
		*did = v;
	}

	return 0;
}

int mlx90397_set_mode(const struct i2c_dt_spec *i2c,
		      enum mlx90397_mode mode,
		      bool x_en, bool y_en, bool z_en)
{
	if (!i2c) {
		return -EINVAL;
	}

	uint8_t axes = 0;
	if (x_en) axes |= MLX90397_CTRL_X_EN;
	if (y_en) axes |= MLX90397_CTRL_Y_EN;
	if (z_en) axes |= MLX90397_CTRL_Z_EN;

	/* Datasheet note: mode switch should pass through POWERDOWN. */
	int ret = mlx90397_write_u8(i2c, MLX90397_REG_CTRL,
				    (uint8_t)(axes | (MLX90397_MODE_POWERDOWN & 0x0F)));
	if (ret < 0) {
		return ret;
	}

	k_sleep(K_MSEC(2));

	return mlx90397_write_u8(i2c, MLX90397_REG_CTRL,
				 (uint8_t)(axes | (mode & 0x0F)));
}

static int mlx90397_wait_drdy(const struct i2c_dt_spec *i2c,
			      int32_t timeout_ms,
			      uint8_t *stat1_out)
{
	int64_t deadline = (timeout_ms < 0) ? INT64_MAX : (k_uptime_get() + timeout_ms);
	uint8_t st1 = 0;

	while (true) {
		int ret = mlx90397_read_u8(i2c, MLX90397_REG_STAT1, &st1);
		if (ret < 0) {
			return ret;
		}

		if (stat1_out) {
			*stat1_out = st1;
		}

		if (st1 & MLX90397_STAT1_DRDY) {
			return 0;
		}

		if (k_uptime_get() >= deadline) {
			return -ETIMEDOUT;
		}

		k_sleep(K_MSEC(1));
	}
}

int mlx90397_read_sample(const struct i2c_dt_spec *i2c,
			 struct mlx90397_sample *out,
			 int32_t timeout_ms)
{
	if (!i2c || !out) {
		return -EINVAL;
	}

	uint8_t st1 = 0;
	int ret = mlx90397_wait_drdy(i2c, timeout_ms, &st1);
	if (ret < 0) {
		return ret;
	}

	/* Burst read: 0x01..0x07 => XYZ (6 bytes) + STAT2 (1 byte) */
	uint8_t buf[7] = {0};
	ret = mlx90397_read_regs(i2c, MLX90397_REG_X_L, buf, sizeof(buf));
	if (ret < 0) {
		return ret;
	}

	out->t_ms  = (uint32_t)k_uptime_get_32();
	out->stat1 = st1;
	out->x     = (int16_t)((buf[1] << 8) | buf[0]);
	out->y     = (int16_t)((buf[3] << 8) | buf[2]);
	out->z     = (int16_t)((buf[5] << 8) | buf[4]);
	out->stat2 = buf[6];

	return 0;
}
