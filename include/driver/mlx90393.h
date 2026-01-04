/*
 * mlx90393.h
 * MLX90393 Triaxis® magnetometer — public header
 *
 * Thin user-space API, independent of Zephyr's generic sensor subsystem.
 * Each helper below is a true syscall, marshalled via <syscalls/mlx90393.h>.
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_MLX90393_H_
#define ZEPHYR_INCLUDE_DRIVERS_MLX90393_H_

#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/__assert.h>
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* bind this driver to devicetree nodes with compatible = "dsy,mlx90393" */
#define DT_DRV_COMPAT dsy_mlx90393

/* ────────── Datasheet command bytes (Table 11, I2C/SPI) ────────── */
#define MLX90393_CMD_SB(zyxt)    (0x10 | ((zyxt) & 0x0F))  /* Start Burst     */
#define MLX90393_CMD_SW(zyxt)    (0x20 | ((zyxt) & 0x0F))  /* Start WOC       */
#define MLX90393_CMD_SM(zyxt)    (0x30 | ((zyxt) & 0x0F))  /* Start Single    */
#define MLX90393_CMD_RM(zyxt)    (0x40 | ((zyxt) & 0x0F))  /* Read Meas.      */

#define MLX90393_CMD_RR          0x50  /* Read register      */
#define MLX90393_CMD_WR          0x60  /* Write register     */
#define MLX90393_CMD_EX          0x80  /* Exit mode          */
#define MLX90393_CMD_HR          0xD0  /* Memory recall      */
#define MLX90393_CMD_HS          0xE0  /* Memory store       */
#define MLX90393_CMD_RT          0xF0  /* Soft reset (RT)    */

/* zyxt bit helpers (argument nibble) */
#define MLX90393_ZYXT_T          0x1
#define MLX90393_ZYXT_X          0x2
#define MLX90393_ZYXT_Y          0x4
#define MLX90393_ZYXT_Z          0x8

/* ────────── Driver-side vtable ────── */
__subsystem struct mlx90393_driver_api {
	int (*sm)(const struct device *dev,
		  uint8_t zyxt, uint8_t *status_out);

	int (*rm)(const struct device *dev,
		  uint8_t zyxt,
		  int16_t *x, int16_t *y, int16_t *z, int16_t *t,
		  uint8_t *status_out);

	int (*rr)(const struct device *dev,
		  uint8_t reg, uint16_t *val, uint8_t *status_out);

	int (*wr)(const struct device *dev,
		  uint8_t reg, uint16_t val, uint8_t *status_out);

	int (*ex)(const struct device *dev, uint8_t *status_out);

	int (*reset)(const struct device *dev, uint8_t *status_out);
};

/* ────────── User-visible syscalls ────────── */

__syscall int mlx90393_sm(const struct device *dev,
			  uint8_t zyxt, uint8_t *status_out);

static inline int z_impl_mlx90393_sm(const struct device *dev,
				     uint8_t zyxt, uint8_t *status_out)
{
	const struct mlx90393_driver_api *api = dev->api;

	__ASSERT(api && api->sm, "mlx90393_sm not implemented");
	return api->sm(dev, zyxt, status_out);
}

__syscall int mlx90393_rm(const struct device *dev, uint8_t zyxt,
			  int16_t *x, int16_t *y, int16_t *z, int16_t *t,
			  uint8_t *status_out);

static inline int z_impl_mlx90393_rm(const struct device *dev, uint8_t zyxt,
				     int16_t *x, int16_t *y,
				     int16_t *z, int16_t *t,
				     uint8_t *status_out)
{
	const struct mlx90393_driver_api *api = dev->api;

	__ASSERT(api && api->rm, "mlx90393_rm not implemented");
	return api->rm(dev, zyxt, x, y, z, t, status_out);
}

__syscall int mlx90393_rr(const struct device *dev,
			  uint8_t reg, uint16_t *val, uint8_t *status_out);

static inline int z_impl_mlx90393_rr(const struct device *dev,
				     uint8_t reg, uint16_t *val,
				     uint8_t *status_out)
{
	const struct mlx90393_driver_api *api = dev->api;

	__ASSERT(api && api->rr, "mlx90393_rr not implemented");
	return api->rr(dev, reg, val, status_out);
}

__syscall int mlx90393_wr(const struct device *dev,
			  uint8_t reg, uint16_t val, uint8_t *status_out);

static inline int z_impl_mlx90393_wr(const struct device *dev,
				     uint8_t reg, uint16_t val,
				     uint8_t *status_out)
{
	const struct mlx90393_driver_api *api = dev->api;

	__ASSERT(api && api->wr, "mlx90393_wr not implemented");
	return api->wr(dev, reg, val, status_out);
}

__syscall int mlx90393_ex(const struct device *dev, uint8_t *status_out);

static inline int z_impl_mlx90393_ex(const struct device *dev,
				     uint8_t *status_out)
{
	const struct mlx90393_driver_api *api = dev->api;

	__ASSERT(api && api->ex, "mlx90393_ex not implemented");
	return api->ex(dev, status_out);
}

__syscall int mlx90393_reset(const struct device *dev, uint8_t *status_out);

static inline int z_impl_mlx90393_reset(const struct device *dev,
					uint8_t *status_out)
{
	const struct mlx90393_driver_api *api = dev->api;

	__ASSERT(api && api->reset, "mlx90393_reset not implemented");
	return api->reset(dev, status_out);
}

/* ────────── AUTO-GENERATED WRAPPERS — *must* be last line ────────── */
#include <syscalls/mlx90393.h>

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_INCLUDE_DRIVERS_MLX90393_H_ */
