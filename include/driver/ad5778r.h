/*
 * AD5778R ― public header (custom Zephyr driver API)
 *
 * Like your AD5689 style:
 *  - independent of Zephyr generic DAC subsystem
 *  - exposes its own syscall layer
 *  ad5789r.h
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_AD5778R_H_
#define ZEPHYR_INCLUDE_DRIVERS_AD5778R_H_

#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/__assert.h>
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* bind to devicetree nodes with compatible = "dsy,ad5778r" */
#define DT_DRV_COMPAT dsy_ad5778r

/* ────────── Datasheet command nibbles (Table 7) ────────── */
#define AD5778R_CMD_WRITE_CODE_N              0x0
#define AD5778R_CMD_WRITE_CODE_ALL            0x8
#define AD5778R_CMD_WRITE_SPAN_N              0x6
#define AD5778R_CMD_UPDATE_N                  0x1
#define AD5778R_CMD_UPDATE_ALL                0x9
#define AD5778R_CMD_WRITE_CODE_UPDATE_N       0x3
#define AD5778R_CMD_WRITE_CODE_UPDATE_ALL     0x2
#define AD5778R_CMD_WRITE_CODE_ALL_UPDATE_ALL 0xA
#define AD5778R_CMD_POWER_DOWN_N              0x4
#define AD5778R_CMD_POWER_DOWN_CHIP           0x5
#define AD5778R_CMD_MONITOR_MUX               0xB
#define AD5778R_CMD_TOGGLE_SELECT             0xC
#define AD5778R_CMD_GLOBAL_TOGGLE             0xD
#define AD5778R_CMD_CONFIG                    0x7
#define AD5778R_CMD_NOP                       0xF

/* “write span to all” 在文字里给了 (1110b) */
#define AD5778R_CMD_WRITE_SPAN_ALL            0xE  /* (0110b to n, 1110b to all) */
                                                   /* see Table 9 section text */

/* ────────── DAC address codes (Table 8) ────────── */
#define AD5778R_ADDR_DAC0                     0x0
#define AD5778R_ADDR_DAC1                     0x3

/* ────────── Span codes (Table 9) ────────── */
enum ad5778r_span_code {
	AD5778R_SPAN_HIGH_Z        = 0x0, /* High-Z */
	AD5778R_SPAN_3P125_MA      = 0x1,
	AD5778R_SPAN_6P25_MA       = 0x2,
	AD5778R_SPAN_12P5_MA       = 0x3,
	AD5778R_SPAN_25_MA         = 0x4,
	AD5778R_SPAN_50_MA         = 0x5,
	AD5778R_SPAN_100_MA        = 0x6,
	AD5778R_SPAN_200_MA        = 0x7,
	AD5778R_SPAN_SWITCH_TO_VM  = 0x8, /* “Switch to V–” */
	AD5778R_SPAN_300_MA        = 0xF,
};

/* ────────── Monitor MUX control codes (Table 10) ────────── */
enum ad5778r_mux_code {
	AD5778R_MUX_DISABLED          = 0x00,
	AD5778R_MUX_OUT0_CURRENT      = 0x01,
	AD5778R_MUX_OUT1_CURRENT      = 0x04,
	AD5778R_MUX_VCC               = 0x06,
	AD5778R_MUX_VREF              = 0x08,
	AD5778R_MUX_VREFLO            = 0x09,
	AD5778R_MUX_DIE_TEMP          = 0x0A,
	AD5778R_MUX_VDD0              = 0x10,
	AD5778R_MUX_VDD1              = 0x13,
	AD5778R_MUX_VPLUS             = 0x15,
	AD5778R_MUX_VMINUS            = 0x16,
	AD5778R_MUX_GND               = 0x17,
	AD5778R_MUX_OUT0_PIN_VOLTAGE  = 0x18,
	AD5778R_MUX_OUT1_PIN_VOLTAGE  = 0x1B,
};

/* ────────── CONFIG command bits (Figure 30 text) ────────── */
#define AD5778R_CFG_OC_DISABLE   (1u << 3)
#define AD5778R_CFG_PL_DISABLE   (1u << 2)
#define AD5778R_CFG_TS_DISABLE   (1u << 1)
/* RD=1 selects external-reference operation (REFCOMP must be grounded in HW) */
#define AD5778R_CFG_RD_EXTERNAL  (1u << 0)

/* ────────── Fault Register bits (Table 11) ────────── */
#define AD5778R_FR0_OUT0_OC         (1u << 0)
#define AD5778R_FR1_OUT1_OC         (1u << 1)
#define AD5778R_FR2_OVERTEMP        (1u << 2)
#define AD5778R_FR3_POWER_LIMIT     (1u << 3)
#define AD5778R_FR4_INVALID_LEN     (1u << 4)

/* ────────── Driver vtable ────────── */
__subsystem struct ad5778r_driver_api {
	/* core ops */
	int (*write)(const struct device *dev, uint8_t channel, uint16_t code);
	int (*write_input)(const struct device *dev, uint8_t channel, uint16_t code);
	int (*update)(const struct device *dev, uint8_t channel);
	int (*update_all)(const struct device *dev);

	/* span */
	int (*set_span)(const struct device *dev, uint8_t channel, uint8_t span_code);
	int (*set_span_input)(const struct device *dev, uint8_t channel, uint8_t span_code);

	/* power */
	int (*power_down)(const struct device *dev, uint8_t channel);
	int (*power_down_chip)(const struct device *dev);

	/* misc */
	int (*config)(const struct device *dev, uint8_t cfg_bits);
	int (*monitor_mux)(const struct device *dev, uint8_t mux_code);

	/* fault readback */
	int (*read_fault)(const struct device *dev, uint8_t *fault);
	int (*get_last_fault)(const struct device *dev, uint8_t *fault);

	/* optional GPIO helpers */
	int (*ldac)(const struct device *dev); /* pulse LDAC (active-low recommended) */
	int (*clr)(const struct device *dev);  /* pulse CLR  (active-low recommended) */
};

/* ────────── User-visible syscalls ────────── */

__syscall int ad5778r_write(const struct device *dev, uint8_t channel, uint16_t code);
static inline int z_impl_ad5778r_write(const struct device *dev, uint8_t channel, uint16_t code)
{
	const struct ad5778r_driver_api *api = dev->api;
	__ASSERT(api && api->write, "ad5778r_write not implemented");
	return api->write(dev, channel, code);
}

__syscall int ad5778r_write_input(const struct device *dev, uint8_t channel, uint16_t code);
static inline int z_impl_ad5778r_write_input(const struct device *dev, uint8_t channel, uint16_t code)
{
	const struct ad5778r_driver_api *api = dev->api;
	__ASSERT(api && api->write_input, "ad5778r_write_input not implemented");
	return api->write_input(dev, channel, code);
}

__syscall int ad5778r_update(const struct device *dev, uint8_t channel);
static inline int z_impl_ad5778r_update(const struct device *dev, uint8_t channel)
{
	const struct ad5778r_driver_api *api = dev->api;
	__ASSERT(api && api->update, "ad5778r_update not implemented");
	return api->update(dev, channel);
}

__syscall int ad5778r_update_all(const struct device *dev);
static inline int z_impl_ad5778r_update_all(const struct device *dev)
{
	const struct ad5778r_driver_api *api = dev->api;
	__ASSERT(api && api->update_all, "ad5778r_update_all not implemented");
	return api->update_all(dev);
}

__syscall int ad5778r_set_span(const struct device *dev, uint8_t channel, uint8_t span_code);
static inline int z_impl_ad5778r_set_span(const struct device *dev, uint8_t channel, uint8_t span_code)
{
	const struct ad5778r_driver_api *api = dev->api;
	__ASSERT(api && api->set_span, "ad5778r_set_span not implemented");
	return api->set_span(dev, channel, span_code);
}

__syscall int ad5778r_set_span_input(const struct device *dev, uint8_t channel, uint8_t span_code);
static inline int z_impl_ad5778r_set_span_input(const struct device *dev, uint8_t channel, uint8_t span_code)
{
	const struct ad5778r_driver_api *api = dev->api;
	__ASSERT(api && api->set_span_input, "ad5778r_set_span_input not implemented");
	return api->set_span_input(dev, channel, span_code);
}

__syscall int ad5778r_power_down(const struct device *dev, uint8_t channel);
static inline int z_impl_ad5778r_power_down(const struct device *dev, uint8_t channel)
{
	const struct ad5778r_driver_api *api = dev->api;
	__ASSERT(api && api->power_down, "ad5778r_power_down not implemented");
	return api->power_down(dev, channel);
}

__syscall int ad5778r_power_down_chip(const struct device *dev);
static inline int z_impl_ad5778r_power_down_chip(const struct device *dev)
{
	const struct ad5778r_driver_api *api = dev->api;
	__ASSERT(api && api->power_down_chip, "ad5778r_power_down_chip not implemented");
	return api->power_down_chip(dev);
}

__syscall int ad5778r_config(const struct device *dev, uint8_t cfg_bits);
static inline int z_impl_ad5778r_config(const struct device *dev, uint8_t cfg_bits)
{
	const struct ad5778r_driver_api *api = dev->api;
	__ASSERT(api && api->config, "ad5778r_config not implemented");
	return api->config(dev, cfg_bits);
}

__syscall int ad5778r_monitor_mux(const struct device *dev, uint8_t mux_code);
static inline int z_impl_ad5778r_monitor_mux(const struct device *dev, uint8_t mux_code)
{
	const struct ad5778r_driver_api *api = dev->api;
	__ASSERT(api && api->monitor_mux, "ad5778r_monitor_mux not implemented");
	return api->monitor_mux(dev, mux_code);
}

__syscall int ad5778r_read_fault(const struct device *dev, uint8_t *fault);
static inline int z_impl_ad5778r_read_fault(const struct device *dev, uint8_t *fault)
{
	const struct ad5778r_driver_api *api = dev->api;
	__ASSERT(api && api->read_fault, "ad5778r_read_fault not implemented");
	return api->read_fault(dev, fault);
}

__syscall int ad5778r_get_last_fault(const struct device *dev, uint8_t *fault);
static inline int z_impl_ad5778r_get_last_fault(const struct device *dev, uint8_t *fault)
{
	const struct ad5778r_driver_api *api = dev->api;
	__ASSERT(api && api->get_last_fault, "ad5778r_get_last_fault not implemented");
	return api->get_last_fault(dev, fault);
}

__syscall int ad5778r_ldac(const struct device *dev);
static inline int z_impl_ad5778r_ldac(const struct device *dev)
{
	const struct ad5778r_driver_api *api = dev->api;
	__ASSERT(api && api->ldac, "ad5778r_ldac not implemented");
	return api->ldac(dev);
}

__syscall int ad5778r_clr(const struct device *dev);
static inline int z_impl_ad5778r_clr(const struct device *dev)
{
	const struct ad5778r_driver_api *api = dev->api;
	__ASSERT(api && api->clr, "ad5778r_clr not implemented");
	return api->clr(dev);
}

/* ────────── AUTO-GENERATED WRAPPERS — must be last line ────────── */
#include <syscalls/ad5778r.h>

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_INCLUDE_DRIVERS_AD5778R_H_ */
