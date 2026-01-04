/*
 * MAX30001 — public header (custom Zephyr driver API)
 *
 *  - independent of Zephyr sensor subsystem
 *  - exposes its own syscall layer
 *
 *  include/driver/max30001.h
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_MAX30001_H_
#define ZEPHYR_INCLUDE_DRIVERS_MAX30001_H_

#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/__assert.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* bind to devicetree nodes with compatible = "dsy,max30001" */
#define DT_DRV_COMPAT dsy_max30001

/* ────────── Register addresses (REG[6:0]) ────────── */
#define MAX30001_REG_NO_OP            0x00
#define MAX30001_REG_STATUS           0x01
#define MAX30001_REG_EN_INT           0x02
#define MAX30001_REG_EN_INT2          0x03
#define MAX30001_REG_MNGR_INT         0x04
#define MAX30001_REG_MNGR_DYN         0x05

#define MAX30001_REG_SW_RST           0x08 /* W, data=0 */
#define MAX30001_REG_SYNCH            0x09 /* W, data=0 */
#define MAX30001_REG_FIFO_RST         0x0A /* W, data=0 */

#define MAX30001_REG_INFO             0x0F
#define MAX30001_REG_CNFG_GEN         0x10
#define MAX30001_REG_CNFG_CAL         0x12
#define MAX30001_REG_CNFG_EMUX        0x14
#define MAX30001_REG_CNFG_ECG         0x15
#define MAX30001_REG_CNFG_BMUX        0x17
#define MAX30001_REG_CNFG_BIOZ        0x18

#define MAX30001_REG_ECG_FIFO_BURST   0x20 /* R+ */
#define MAX30001_REG_ECG_FIFO         0x21 /* R */
#define MAX30001_REG_BIOZ_FIFO_BURST  0x22 /* R+ */
#define MAX30001_REG_BIOZ_FIFO        0x23 /* R */
#define MAX30001_REG_RTOR             0x25 /* R */

/* ────────── FIFO tag codes ────────── */
/* ECG: ETAG = D[5:3] (we treat 0x2/0x3 as EOF, 0x6 empty, 0x7 overflow) */
#define MAX30001_ECG_ETAG_VALID           0x0
#define MAX30001_ECG_ETAG_OVER_UNDER      0x1
#define MAX30001_ECG_ETAG_VALID_EOF       0x2
#define MAX30001_ECG_ETAG_OU_EOF          0x3
#define MAX30001_ECG_ETAG_EMPTY           0x6
#define MAX30001_ECG_ETAG_OVERFLOW        0x7

/* BioZ: BTAG = D[2:0] (datasheet recommends EOF stop, overflow -> FIFO_RST) */
#define MAX30001_BIOZ_BTAG_VALID          0x0
#define MAX30001_BIOZ_BTAG_OVER_UNDER     0x1
#define MAX30001_BIOZ_BTAG_VALID_EOF      0x2
#define MAX30001_BIOZ_BTAG_OU_EOF         0x3
#define MAX30001_BIOZ_BTAG_EMPTY          0x6
#define MAX30001_BIOZ_BTAG_OVERFLOW       0x7

/* ────────── FIFO word helpers (24-bit data in a u32) ────────── */
static inline uint8_t max30001_ecg_etag(uint32_t w24)  { return (uint8_t)((w24 >> 3) & 0x7u); }
static inline uint8_t max30001_ecg_ptag(uint32_t w24)  { return (uint8_t)(w24 & 0x7u); }
static inline uint8_t max30001_bioz_btag(uint32_t w24) { return (uint8_t)(w24 & 0x7u); }

static inline int32_t max30001_sign_extend(uint32_t v, uint8_t bits)
{
	uint32_t m = 1u << (bits - 1u);
	v &= (1u << bits) - 1u;
	return (int32_t)((v ^ m) - m);
}

/* ECG sample: 18-bit two's-complement, left-justified in D[23:6] */
static inline int32_t max30001_ecg_sample(uint32_t w24)
{
	uint32_t raw18 = (w24 >> 6) & 0x3FFFFu;
	return max30001_sign_extend(raw18, 18);
}

/* BioZ sample: 20-bit two's-complement, left-justified in D[23:4] */
static inline int32_t max30001_bioz_sample(uint32_t w24)
{
	uint32_t raw20 = (w24 >> 4) & 0xFFFFFu;
	return max30001_sign_extend(raw20, 20);
}

/* ────────── Driver vtable ────────── */
__subsystem struct max30001_driver_api {
	/* low-level 24-bit register access */
	int (*write_reg)(const struct device *dev, uint8_t reg, uint32_t val24);
	int (*read_reg)(const struct device *dev, uint8_t reg, uint32_t *val24);

	/* commands */
	int (*sw_reset)(const struct device *dev);
	int (*synch)(const struct device *dev);
	int (*fifo_reset)(const struct device *dev);

	/* streaming preset (uses devicetree defaults) */
	int (*stream_on)(const struct device *dev, bool enable_ecg, bool enable_bioz);
	int (*stream_off)(const struct device *dev);

	/* FIFO reads (normal mode, 1 word per call) */
	int (*read_ecg_word)(const struct device *dev, uint32_t *w24);
	int (*read_bioz_word)(const struct device *dev, uint32_t *w24);
};

/* ────────── User-visible syscalls ────────── */

__syscall int max30001_write_reg(const struct device *dev, uint8_t reg, uint32_t val24);
static inline int z_impl_max30001_write_reg(const struct device *dev, uint8_t reg, uint32_t val24)
{
	const struct max30001_driver_api *api = dev->api;
	__ASSERT(api && api->write_reg, "max30001_write_reg not implemented");
	return api->write_reg(dev, reg, val24);
}

__syscall int max30001_read_reg(const struct device *dev, uint8_t reg, uint32_t *val24);
static inline int z_impl_max30001_read_reg(const struct device *dev, uint8_t reg, uint32_t *val24)
{
	const struct max30001_driver_api *api = dev->api;
	__ASSERT(api && api->read_reg, "max30001_read_reg not implemented");
	return api->read_reg(dev, reg, val24);
}

__syscall int max30001_sw_reset(const struct device *dev);
static inline int z_impl_max30001_sw_reset(const struct device *dev)
{
	const struct max30001_driver_api *api = dev->api;
	__ASSERT(api && api->sw_reset, "max30001_sw_reset not implemented");
	return api->sw_reset(dev);
}

__syscall int max30001_synch(const struct device *dev);
static inline int z_impl_max30001_synch(const struct device *dev)
{
	const struct max30001_driver_api *api = dev->api;
	__ASSERT(api && api->synch, "max30001_synch not implemented");
	return api->synch(dev);
}

__syscall int max30001_fifo_reset(const struct device *dev);
static inline int z_impl_max30001_fifo_reset(const struct device *dev)
{
	const struct max30001_driver_api *api = dev->api;
	__ASSERT(api && api->fifo_reset, "max30001_fifo_reset not implemented");
	return api->fifo_reset(dev);
}

__syscall int max30001_stream_on(const struct device *dev, bool enable_ecg, bool enable_bioz);
static inline int z_impl_max30001_stream_on(const struct device *dev, bool enable_ecg, bool enable_bioz)
{
	const struct max30001_driver_api *api = dev->api;
	__ASSERT(api && api->stream_on, "max30001_stream_on not implemented");
	return api->stream_on(dev, enable_ecg, enable_bioz);
}

__syscall int max30001_stream_off(const struct device *dev);
static inline int z_impl_max30001_stream_off(const struct device *dev)
{
	const struct max30001_driver_api *api = dev->api;
	__ASSERT(api && api->stream_off, "max30001_stream_off not implemented");
	return api->stream_off(dev);
}

__syscall int max30001_read_ecg_word(const struct device *dev, uint32_t *w24);
static inline int z_impl_max30001_read_ecg_word(const struct device *dev, uint32_t *w24)
{
	const struct max30001_driver_api *api = dev->api;
	__ASSERT(api && api->read_ecg_word, "max30001_read_ecg_word not implemented");
	return api->read_ecg_word(dev, w24);
}

__syscall int max30001_read_bioz_word(const struct device *dev, uint32_t *w24);
static inline int z_impl_max30001_read_bioz_word(const struct device *dev, uint32_t *w24)
{
	const struct max30001_driver_api *api = dev->api;
	__ASSERT(api && api->read_bioz_word, "max30001_read_bioz_word not implemented");
	return api->read_bioz_word(dev, w24);
}

/* ────────── AUTO-GENERATED WRAPPERS — must be last line ────────── */
#include <syscalls/max30001.h>

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_INCLUDE_DRIVERS_MAX30001_H_ */