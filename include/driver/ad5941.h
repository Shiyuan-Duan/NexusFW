/*
 * AD5941 AFE — minimal public API
 ad5941.h
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_AD5941_H_
#define ZEPHYR_INCLUDE_DRIVERS_AD5941_H_

#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/__assert.h>
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Bind to devicetree nodes with compatible = "dsy,ad5941" */
#define DT_DRV_COMPAT dsy_ad5941

/* Optional driver API vtable (kept simple for now) */
__subsystem struct ad5941_driver_api {
    int (*reset)(const struct device *dev);
    int (*reg_write)(const struct device *dev, uint16_t reg, uint32_t val);
    int (*reg_read)(const struct device *dev, uint16_t reg, uint32_t *val);
    int (*read_current_nA)(const struct device *dev, int32_t *nA);
    int (*set_vgs_mV)(const struct device *dev, int32_t mv); /* LPDAC set */
    int (*lptia_config)(const struct device *dev, int32_t vzero_mV, uint32_t rtia_ohm);
};

/* Syscalls */
__syscall int ad5941_reset(const struct device *dev);
static inline int z_impl_ad5941_reset(const struct device *dev)
{
    const struct ad5941_driver_api *api = dev->api;
    __ASSERT(api && api->reset, "ad5941_reset not implemented");
    return api->reset(dev);
}

__syscall int ad5941_reg_write(const struct device *dev, uint16_t reg, uint32_t val);
static inline int z_impl_ad5941_reg_write(const struct device *dev, uint16_t reg, uint32_t val)
{
    const struct ad5941_driver_api *api = dev->api;
    __ASSERT(api && api->reg_write, "ad5941_reg_write not implemented");
    return api->reg_write(dev, reg, val);
}

__syscall int ad5941_reg_read(const struct device *dev, uint16_t reg, uint32_t *val);
static inline int z_impl_ad5941_reg_read(const struct device *dev, uint16_t reg, uint32_t *val)
{
    const struct ad5941_driver_api *api = dev->api;
    __ASSERT(api && api->reg_read, "ad5941_reg_read not implemented");
    return api->reg_read(dev, reg, val);
}

__syscall int ad5941_read_current_nA(const struct device *dev, int32_t *nA);
static inline int z_impl_ad5941_read_current_nA(const struct device *dev, int32_t *nA)
{
    const struct ad5941_driver_api *api = dev->api;
    __ASSERT(api && api->read_current_nA, "ad5941_read_current_nA not implemented");
    return api->read_current_nA(dev, nA);
}

__syscall int ad5941_set_vgs_mV(const struct device *dev, int32_t mv);
static inline int z_impl_ad5941_set_vgs_mV(const struct device *dev, int32_t mv)
{
    const struct ad5941_driver_api *api = dev->api;
    __ASSERT(api && api->set_vgs_mV, "ad5941_set_vgs_mV not implemented");
    return api->set_vgs_mV(dev, mv);
}

__syscall int ad5941_lptia_config(const struct device *dev, int32_t vzero_mV, uint32_t rtia_ohm);
static inline int z_impl_ad5941_lptia_config(const struct device *dev, int32_t vzero_mV, uint32_t rtia_ohm)
{
    const struct ad5941_driver_api *api = dev->api;
    __ASSERT(api && api->lptia_config, "ad5941_lptia_config not implemented");
    return api->lptia_config(dev, vzero_mV, rtia_ohm);
}

/* Auto-generated syscall wrappers must be included to provide
 * the exported symbols for the __syscall declarations above. */
#include <syscalls/ad5941.h>

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_INCLUDE_DRIVERS_AD5941_H_ */
