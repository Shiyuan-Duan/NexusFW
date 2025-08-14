/*
 * AD5689 / AD5687 nanoDAC+ ― public header
 *
 * Provides a thin user‑space API that is **independent of Zephyr’s generic
 * DAC subsystem**.  Each helper below is a real system‑call, marshalled by
 * the generated <syscalls/ad5689.h> wrapper.
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_AD5689_H_
#define ZEPHYR_INCLUDE_DRIVERS_AD5689_H_

#include <zephyr/device.h>
#include <zephyr/kernel.h>          /* brings in bool, __ASSERT()    */
#include <zephyr/sys/__assert.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* bind this driver to devicetree nodes with compatible = "dsy,ad5689" */
#define DT_DRV_COMPAT dsy_ad5689

/* ────────── Datasheet command nibbles (C3‑C0, Table 8) ────────── */
#define AD5689_CMD_NOP                0x0
#define AD5689_CMD_WRITE_INPUT_N      0x1
#define AD5689_CMD_UPDATE_DAC_N       0x2
#define AD5689_CMD_WRITE_UPDATE_N     0x3
#define AD5689_CMD_POWERDOWN_DAC      0x4
#define AD5689_CMD_LDAC_MASK          0x5
#define AD5689_CMD_SW_RESET           0x6
#define AD5689_CMD_INTERNAL_REF_CFG   0x8
#define AD5689_CMD_READBACK_ENABLE    0x9

/* ────────── Driver‑side vtable — must be first member of dev struct ────── */
__subsystem struct ad5689_driver_api {
	/* core write primitive (channel 0 = A, 1 = B) */
	int (*write)(const struct device *dev,
		     uint8_t channel, uint16_t code);

	/* optional extras */
	int (*reset)(const struct device *dev);
	int (*ldac)(const struct device *dev);               /* pulse LDAC */
	int (*set_pd_mode)(const struct device *dev, uint8_t mode);
	int (*internal_ref)(const struct device *dev, bool enable);
	int (*read_dac)(const struct device *dev,
			uint8_t channel, uint16_t *code);
};

/* ────────── User‑visible syscalls ────────── */

__syscall int ad5689_write(const struct device *dev,
			   uint8_t channel, uint16_t code);
static inline int z_impl_ad5689_write(const struct device *dev,
				      uint8_t channel, uint16_t code)
{
	const struct ad5689_driver_api *api = dev->api;
	__ASSERT(api && api->write, "ad5689_write not implemented");
	return api->write(dev, channel, code);
}

__syscall int ad5689_reset(const struct device *dev);
static inline int z_impl_ad5689_reset(const struct device *dev)
{
	const struct ad5689_driver_api *api = dev->api;
	__ASSERT(api && api->reset, "ad5689_reset not implemented");
	return api->reset(dev);
}

__syscall int ad5689_ldac(const struct device *dev);
static inline int z_impl_ad5689_ldac(const struct device *dev)
{
	const struct ad5689_driver_api *api = dev->api;
	__ASSERT(api && api->ldac, "ad5689_ldac not implemented");
	return api->ldac(dev);
}

__syscall int ad5689_set_pd_mode(const struct device *dev, uint8_t mode);
static inline int z_impl_ad5689_set_pd_mode(const struct device *dev,
					    uint8_t mode)
{
	const struct ad5689_driver_api *api = dev->api;
	__ASSERT(api && api->set_pd_mode,
		 "ad5689_set_pd_mode not implemented");
	return api->set_pd_mode(dev, mode);
}

__syscall int ad5689_internal_ref(const struct device *dev, bool enable);
static inline int z_impl_ad5689_internal_ref(const struct device *dev,
					     bool enable)
{
	const struct ad5689_driver_api *api = dev->api;
	__ASSERT(api && api->internal_ref,
		 "ad5689_internal_ref not implemented");
	return api->internal_ref(dev, enable);
}

__syscall int ad5689_read_dac(const struct device *dev,
			      uint8_t channel, uint16_t *code);
static inline int z_impl_ad5689_read_dac(const struct device *dev,
					 uint8_t channel, uint16_t *code)
{
	const struct ad5689_driver_api *api = dev->api;
	__ASSERT(api && api->read_dac, "ad5689_read_dac not implemented");
	return api->read_dac(dev, channel, code);
}

/* ────────── AUTO‑GENERATED WRAPPERS — *must* be last line ────────── */
#include <syscalls/ad5689.h>

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_INCLUDE_DRIVERS_AD5689_H_ */
