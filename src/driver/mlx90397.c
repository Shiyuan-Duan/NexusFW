/*
 * MLX90397 minimal I2C driver + "default most sensitive" configuration
 * Zephyr RTOS
 */

#include <zephyr/kernel.h>
#include <zephyr/drivers/i2c.h>

#include <errno.h>
#include <string.h>

#include <driver/mlx90397.h>

/* =========================
 * Register map / bitfields
 * =========================
 * If your mlx90397.h already defines these, remove duplicates and keep yours.
 */

/* Core registers used by this driver */
#ifndef MLX90397_REG_CTRL
#define MLX90397_REG_CTRL        0x0E
#endif

#ifndef MLX90397_REG_X_L
#define MLX90397_REG_X_L         0x01
#endif

#ifndef MLX90397_REG_STAT1
#define MLX90397_REG_STAT1       0x00
#endif

#ifndef MLX90397_REG_CID
#define MLX90397_REG_CID         0x0A
#endif

#ifndef MLX90397_REG_DID
#define MLX90397_REG_DID         0x0B
#endif

/* Config registers that control "sensitivity" */
#ifndef MLX90397_REG_CUST_CTRL2
#define MLX90397_REG_CUST_CTRL2  0x0F
#endif

#ifndef MLX90397_REG_OSR_DIG_FILT
#define MLX90397_REG_OSR_DIG_FILT 0x14
#endif

#ifndef MLX90397_REG_CUST_CTRL
#define MLX90397_REG_CUST_CTRL   0x15
#endif

/* STAT1 bits */
#ifndef MLX90397_STAT1_DRDY
#define MLX90397_STAT1_DRDY      0x01
#endif

/* CTRL axis enables (bit positions depend on your header; these are typical) */
#ifndef MLX90397_CTRL_X_EN
#define MLX90397_CTRL_X_EN       0x10
#endif
#ifndef MLX90397_CTRL_Y_EN
#define MLX90397_CTRL_Y_EN       0x20
#endif
#ifndef MLX90397_CTRL_Z_EN
#define MLX90397_CTRL_Z_EN       0x40
#endif

/* Mode values (low nibble) */
#ifndef MLX90397_MODE_POWERDOWN
#define MLX90397_MODE_POWERDOWN  0x00
#endif
#ifndef MLX90397_MODE_SINGLE
#define MLX90397_MODE_SINGLE     0x01
#endif
#ifndef MLX90397_MODE_CONTINUOUS
#define MLX90397_MODE_CONTINUOUS 0x02
#endif

/* CUST_CTRL2 (0x0F): RANGE_SEL[2:0] */
#ifndef MLX90397_CUST_CTRL2_RANGE_SEL_SHIFT
#define MLX90397_CUST_CTRL2_RANGE_SEL_SHIFT  0
#endif
#ifndef MLX90397_CUST_CTRL2_RANGE_SEL_MASK
#define MLX90397_CUST_CTRL2_RANGE_SEL_MASK   (0x7u << MLX90397_CUST_CTRL2_RANGE_SEL_SHIFT)
#endif

/* OSR_DIG_FILT (0x14):
 * - DIG_FILT_HALL_XY[2:0]
 * - OSR_HALL (0=32, 1=64)
 */
#ifndef MLX90397_OSR_DIG_FILT_DIG_FILT_XY_SHIFT
#define MLX90397_OSR_DIG_FILT_DIG_FILT_XY_SHIFT  3
#endif
#ifndef MLX90397_OSR_DIG_FILT_DIG_FILT_XY_MASK
#define MLX90397_OSR_DIG_FILT_DIG_FILT_XY_MASK   (0x7u << MLX90397_OSR_DIG_FILT_DIG_FILT_XY_SHIFT)
#endif
#ifndef MLX90397_OSR_DIG_FILT_OSR_HALL_SHIFT
#define MLX90397_OSR_DIG_FILT_OSR_HALL_SHIFT     7
#endif
#ifndef MLX90397_OSR_DIG_FILT_OSR_HALL_MASK
#define MLX90397_OSR_DIG_FILT_OSR_HALL_MASK      (0x1u << MLX90397_OSR_DIG_FILT_OSR_HALL_SHIFT)
#endif

/* CUST_CTRL (0x15): DIG_FILT_HALL_Z[2:0] */
#ifndef MLX90397_CUST_CTRL_DIG_FILT_Z_SHIFT
#define MLX90397_CUST_CTRL_DIG_FILT_Z_SHIFT      0
#endif
#ifndef MLX90397_CUST_CTRL_DIG_FILT_Z_MASK
#define MLX90397_CUST_CTRL_DIG_FILT_Z_MASK       (0x7u << MLX90397_CUST_CTRL_DIG_FILT_Z_SHIFT)
#endif


/* =========================
 * Low-level I2C helpers
 * ========================= */

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

/* Read-modify-write helper */
static int mlx90397_update_u8(const struct i2c_dt_spec *i2c, uint8_t reg,
			      uint8_t mask, uint8_t value_shifted)
{
	uint8_t v;
	int ret = mlx90397_read_u8(i2c, reg, &v);
	if (ret < 0) {
		return ret;
	}
	v = (uint8_t)((v & ~mask) | (value_shifted & mask));
	return mlx90397_write_u8(i2c, reg, v);
}

/* =========================
 * Public APIs
 * ========================= */

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

/* =========================
 * "Default most sensitive" configuration
 * =========================
 *
 * This sets:
 *  - RANGE_SEL = 0  => XY=25mT, Z=25mT
 *  - OSR_HALL  = 1  => OSR=64
 *  - DIG_FILT_XY = 7
 *  - DIG_FILT_Z  = 7
 *
 * WARNING: conversion time increases; you may need to lower streaming rate.
 */
int mlx90397_apply_default_sensitive(const struct i2c_dt_spec *i2c,
				    bool x_en, bool y_en, bool z_en,
				    enum mlx90397_mode final_mode)
{
	if (!i2c) {
		return -EINVAL;
	}

	int ret;

	/* 1) Go powerdown first */
	ret = mlx90397_set_mode(i2c, MLX90397_MODE_POWERDOWN, x_en, y_en, z_en);
	if (ret < 0) return ret;

	k_sleep(K_MSEC(2));

	/* 2) RANGE_SEL = 0 (most sensitive range) */
	ret = mlx90397_update_u8(i2c,
				 MLX90397_REG_CUST_CTRL2,
				 (uint8_t)MLX90397_CUST_CTRL2_RANGE_SEL_MASK,
				 (uint8_t)(0u << MLX90397_CUST_CTRL2_RANGE_SEL_SHIFT));
	if (ret < 0) return ret;

	/* 3) OSR_HALL = 1 (64), DIG_FILT_XY = 7 */
	ret = mlx90397_update_u8(i2c,
				 MLX90397_REG_OSR_DIG_FILT,
				 (uint8_t)MLX90397_OSR_DIG_FILT_OSR_HALL_MASK,
				 (uint8_t)(1u << MLX90397_OSR_DIG_FILT_OSR_HALL_SHIFT));
	if (ret < 0) return ret;

	ret = mlx90397_update_u8(i2c,
				 MLX90397_REG_OSR_DIG_FILT,
				 (uint8_t)MLX90397_OSR_DIG_FILT_DIG_FILT_XY_MASK,
				 (uint8_t)(7u << MLX90397_OSR_DIG_FILT_DIG_FILT_XY_SHIFT));
	if (ret < 0) return ret;

	/* 4) DIG_FILT_Z = 7 */
	ret = mlx90397_update_u8(i2c,
				 MLX90397_REG_CUST_CTRL,
				 (uint8_t)MLX90397_CUST_CTRL_DIG_FILT_Z_MASK,
				 (uint8_t)(7u << MLX90397_CUST_CTRL_DIG_FILT_Z_SHIFT));
	if (ret < 0) return ret;

	/* 5) Enter final mode */
	ret = mlx90397_set_mode(i2c, final_mode, x_en, y_en, z_en);
	if (ret < 0) return ret;

	return 0;
}

/* Optional debug helper: read back config registers */
int mlx90397_dump_cfg(const struct i2c_dt_spec *i2c, uint8_t *cust_ctrl2,
		      uint8_t *osr_dig_filt, uint8_t *cust_ctrl)
{
	if (!i2c) return -EINVAL;

	int ret;
	uint8_t v;

	ret = mlx90397_read_u8(i2c, MLX90397_REG_CUST_CTRL2, &v);
	if (ret < 0) return ret;
	if (cust_ctrl2) *cust_ctrl2 = v;

	ret = mlx90397_read_u8(i2c, MLX90397_REG_OSR_DIG_FILT, &v);
	if (ret < 0) return ret;
	if (osr_dig_filt) *osr_dig_filt = v;

	ret = mlx90397_read_u8(i2c, MLX90397_REG_CUST_CTRL, &v);
	if (ret < 0) return ret;
	if (cust_ctrl) *cust_ctrl = v;

	return 0;
}
