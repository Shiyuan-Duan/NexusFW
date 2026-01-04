/*
 * mlx90397.h - minimal MLX90397 I2C helper (app-local "driver" module)
 *
 * This is NOT a Zephyr sensor-subsystem driver.
 * It is a tiny, explicit API you can call from your app threads.
 *
 * Datasheet normal read sequence (continuous mode):
 *   1) poll STAT1.DRDY
 *   2) read XYZ registers (reading any XYZ drops DRDY)
 *   3) read STAT2 for consistency (clears DOR when sequence is followed)
 */

#pragma once

#include <zephyr/drivers/i2c.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/util.h>

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Register map */
#define MLX90397_REG_STAT1       0x00
#define MLX90397_REG_X_L         0x01
#define MLX90397_REG_X_H         0x02
#define MLX90397_REG_Y_L         0x03
#define MLX90397_REG_Y_H         0x04
#define MLX90397_REG_Z_L         0x05
#define MLX90397_REG_Z_H         0x06
#define MLX90397_REG_STAT2       0x07
#define MLX90397_REG_T_L         0x08
#define MLX90397_REG_T_H         0x09
#define MLX90397_REG_CID         0x0A
#define MLX90397_REG_DID         0x0B
#define MLX90397_REG_CTRL        0x0E
#define MLX90397_REG_CUST_CTRL2  0x0F

/* STAT1 bits */
#define MLX90397_STAT1_DRDY      BIT(0)

/* STAT2 bits */
#define MLX90397_STAT2_HOVF_X    BIT(0)
#define MLX90397_STAT2_HOVF_Y    BIT(1)
#define MLX90397_STAT2_HOVF_Z    BIT(2)
#define MLX90397_STAT2_DOR       BIT(3)

/* CTRL bits */
#define MLX90397_CTRL_X_EN       BIT(4)
#define MLX90397_CTRL_Y_EN       BIT(5)
#define MLX90397_CTRL_Z_EN       BIT(6)

/* CTRL.MODE[3:0] values */
enum mlx90397_mode {
	MLX90397_MODE_POWERDOWN   = 0,
	MLX90397_MODE_SINGLE      = 1,

	MLX90397_MODE_CONT_10HZ   = 2,
	MLX90397_MODE_CONT_20HZ   = 3,
	MLX90397_MODE_CONT_50HZ   = 4,
	MLX90397_MODE_CONT_100HZ  = 5,
	MLX90397_MODE_CONT_125HZ  = 6,

	MLX90397_MODE_CONT_200HZ  = 10,
	MLX90397_MODE_CONT_500HZ  = 11,
	MLX90397_MODE_CONT_700HZ  = 12,
	MLX90397_MODE_CONT_1KHZ   = 13,
	MLX90397_MODE_CONT_14KHZ  = 14,
};

struct mlx90397_sample {
	uint32_t t_ms;   /* k_uptime_get_32() snapshot */
	int16_t  x;
	int16_t  y;
	int16_t  z;
	uint8_t  stat1;
	uint8_t  stat2;
};

/* Basic 8-bit read/write */
int mlx90397_read_u8(const struct i2c_dt_spec *i2c, uint8_t reg, uint8_t *val);
int mlx90397_write_u8(const struct i2c_dt_spec *i2c, uint8_t reg, uint8_t val);

/* Convenience: read CID/DID */
int mlx90397_read_ids(const struct i2c_dt_spec *i2c, uint8_t *cid, uint8_t *did);

/* Set CTRL:
 * Datasheet note: to change mode, go to POWERDOWN first.
 */
int mlx90397_set_mode(const struct i2c_dt_spec *i2c,
		      enum mlx90397_mode mode,
		      bool x_en, bool y_en, bool z_en);

/* Read one XYZ sample + STAT1/STAT2.
 * timeout_ms: max time to wait for DRDY (poll STAT1).
 */
int mlx90397_read_sample(const struct i2c_dt_spec *i2c,
			 struct mlx90397_sample *out,
			 int32_t timeout_ms);

#ifdef __cplusplus
}
#endif
