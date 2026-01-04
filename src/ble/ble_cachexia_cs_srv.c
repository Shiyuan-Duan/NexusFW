/*  ble_cachexia_cs_srv.c — Cachexia current-source stimulation BLE GATT service */

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/device.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/sys/byteorder.h>
#include <string.h>
#include <errno.h>

#include "ble_cachexia_cs_srv.h"

/* Use your AD5778R driver syscall API */
#include <driver/ad5778r.h>

LOG_MODULE_REGISTER(cachexia_cs_srv, CONFIG_BT_LOG_LEVEL);

/* AD5778R instance (used for immediate safe-off when Switch is written to 0) */
static const struct device *const ad5778r_dev = DEVICE_DT_GET_ONE(dsy_ad5778r);

/* ---- State (atomic for cross-thread access) ---- */
static atomic_t g_switch  = ATOMIC_INIT(0);   /* 0=off, 1=on */
static atomic_t g_amp_pct = ATOMIC_INIT(0);   /* 0..100 */
static atomic_t g_freq_hz = ATOMIC_INIT(5);   /* default 5 Hz */

/* ---- Cached mirrors for READ backs ---- */
static uint8_t  sw_cached;
static uint8_t  amp_cached;
static uint16_t freq_cached_le; /* stored as little-endian for read path */

/* ---- Read helpers ---- */
static ssize_t read_u8_cached(struct bt_conn *conn, const struct bt_gatt_attr *attr,
			      void *buf, uint16_t len, uint16_t offset)
{
	const uint8_t *src = attr->user_data;
	return bt_gatt_attr_read(conn, attr, buf, len, offset, src, 1);
}

static ssize_t read_u16_cached(struct bt_conn *conn, const struct bt_gatt_attr *attr,
			       void *buf, uint16_t len, uint16_t offset)
{
	const uint16_t *src_le = attr->user_data;
	return bt_gatt_attr_read(conn, attr, buf, len, offset, src_le, sizeof(uint16_t));
}

/* ---- Write: switch ---- */
static ssize_t write_switch(struct bt_conn *conn, const struct bt_gatt_attr *attr,
			    const void *buf, uint16_t len, uint16_t offset, uint8_t flags)
{
	if (len != 1 || offset) {
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
	}

	uint8_t v = ((const uint8_t *)buf)[0] ? 1 : 0;
	atomic_set(&g_switch, v);
	sw_cached = v;

	/* If turned off, immediately force DAC output to High-Z + power down for safety.
	 * Datasheet: Code 0 may not be fully off due to offset; High-Z span is the reliable off.
	 */
	if (v == 0 && device_is_ready(ad5778r_dev)) {
		(void)ad5778r_set_span(ad5778r_dev, 0 /*channel*/, AD5778R_SPAN_HIGH_Z);
		(void)ad5778r_power_down(ad5778r_dev, 0 /*channel*/);
	}

	return len;
}

/* ---- Write: amplitude (0..100%) ---- */
static ssize_t write_amplitude(struct bt_conn *conn, const struct bt_gatt_attr *attr,
			       const void *buf, uint16_t len, uint16_t offset, uint8_t flags)
{
	if (len != 1 || offset) {
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
	}

	uint8_t v = ((const uint8_t *)buf)[0];
	if (v > 100) {
		v = 100;
	}

	atomic_set(&g_amp_pct, v);
	amp_cached = v;
	return len;
}

/* ---- Write: frequency Hz (u16 LE) ---- */
static ssize_t write_frequency(struct bt_conn *conn, const struct bt_gatt_attr *attr,
			       const void *buf, uint16_t len, uint16_t offset, uint8_t flags)
{
	if (len != sizeof(uint16_t) || offset) {
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
	}

	uint16_t hz = sys_get_le16(buf);

	/* Clamp: 1..200 Hz (you can widen if you want; 5 Hz default) */
	if (hz < 1) {
		hz = 1;
	} else if (hz > 200) {
		hz = 200;
	}

	atomic_set(&g_freq_hz, hz);
	freq_cached_le = sys_cpu_to_le16(hz);
	return len;
}

/* ---- Read: battery float (placeholder 4.2f) ---- */
static ssize_t read_battery(struct bt_conn *conn, const struct bt_gatt_attr *attr,
			    void *buf, uint16_t len, uint16_t offset)
{
	float f = 4.2f; /* placeholder */
	uint32_t bits;
	memcpy(&bits, &f, sizeof(bits));
	bits = sys_cpu_to_le32(bits);
	return bt_gatt_attr_read(conn, attr, buf, len, offset, &bits, sizeof(bits));
}

/* ---- GATT table ---- */
BT_GATT_SERVICE_DEFINE(cachexia_cs_svc,
	BT_GATT_PRIMARY_SERVICE(BT_UUID_CACHEXIA_SVC),

	BT_GATT_CHARACTERISTIC(BT_UUID_CACHEXIA_SWITCH,
		BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE,
		BT_GATT_PERM_READ | BT_GATT_PERM_WRITE,
		read_u8_cached, write_switch, &sw_cached),

	BT_GATT_CHARACTERISTIC(BT_UUID_CACHEXIA_AMPLITUDE,
		BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE,
		BT_GATT_PERM_READ | BT_GATT_PERM_WRITE,
		read_u8_cached, write_amplitude, &amp_cached),

	BT_GATT_CHARACTERISTIC(BT_UUID_CACHEXIA_FREQUENCY,
		BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE,
		BT_GATT_PERM_READ | BT_GATT_PERM_WRITE,
		read_u16_cached, write_frequency, &freq_cached_le),

	BT_GATT_CHARACTERISTIC(BT_UUID_CACHEXIA_BATTERY,
		BT_GATT_CHRC_READ,
		BT_GATT_PERM_READ,
		read_battery, NULL, NULL),
);

int ble_cachexia_cs_srv_init(void)
{
	if (!device_is_ready(ad5778r_dev)) {
		LOG_ERR("AD5778R device not ready");
		return -ENODEV;
	}

	/* defaults */
	sw_cached = 0;
	amp_cached = 0;
	atomic_clear(&g_switch);
	atomic_set(&g_amp_pct, 0);
	atomic_set(&g_freq_hz, 5);
	freq_cached_le = sys_cpu_to_le16(5);

	/* Ensure output is safely off: High-Z + powerdown */
	(void)ad5778r_set_span(ad5778r_dev, 0 /*channel*/, AD5778R_SPAN_HIGH_Z);
	(void)ad5778r_power_down(ad5778r_dev, 0 /*channel*/);

	LOG_INF("Cachexia CS BLE service ready (default 5 Hz)");
	return 0;
}

uint8_t ble_cachexia_cs_get_switch(void)
{
	return (uint8_t)atomic_get(&g_switch);
}

uint8_t ble_cachexia_cs_get_amplitude_percent(void)
{
	return (uint8_t)atomic_get(&g_amp_pct);
}

uint16_t ble_cachexia_cs_get_frequency_hz(void)
{
	return (uint16_t)atomic_get(&g_freq_hz);
}
