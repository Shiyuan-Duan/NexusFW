/* ========================= ble_cachexia_cs_srv.c =========================
 * Cachexia current-source stimulation BLE GATT service
 */

#include <zephyr/kernel.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/uuid.h>
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
static atomic_t g_switch   = ATOMIC_INIT(0);   /* 0=off, 1=on */
static atomic_t g_amp_pct  = ATOMIC_INIT(0);   /* 0..100 (back-compat UI control) */
static atomic_t g_amp_code = ATOMIC_INIT(0);   /* 0..65535 (real DAC control) */
static atomic_t g_freq_hz  = ATOMIC_INIT(5);   /* default 5 Hz */

/* ---- Cached mirrors for READ backs ---- */
static uint8_t  sw_cached;
static uint8_t  amp_pct_cached;
static uint16_t amp_code_cached_le; /* stored LE for read path */
static uint16_t freq_cached_le;     /* stored LE for read path */

/* ---- IMU cached payload + notify state ---- */
static atomic_t g_imu_notify = ATOMIC_INIT(0);
static uint8_t  imu_payload[CACHEXIA_IMU_PAYLOAD_LEN];
K_MUTEX_DEFINE(imu_payload_lock);

/* We resolve the attribute pointer once (or lazily) for bt_gatt_notify() */
static const struct bt_gatt_attr *imu_value_attr;

/* ---- Conversions (percent <-> code) ---- */
static uint16_t pct_to_code(uint8_t pct)
{
	if (pct >= 100) {
		return (uint16_t)CACHEXIA_MAX_AMP_CODE;
	}
	uint32_t code = ((uint32_t)pct * 65535u) / 100u;
	if (code > CACHEXIA_MAX_AMP_CODE) {
		code = CACHEXIA_MAX_AMP_CODE;
	}
	return (uint16_t)code;
}

static uint8_t code_to_pct(uint16_t code)
{
	/* Round to nearest % */
	uint32_t pct = ((uint32_t)code * 100u + 32767u) / 65535u;
	if (pct > 100u) {
		pct = 100u;
	}
	return (uint8_t)pct;
}

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

static ssize_t read_imu_cached(struct bt_conn *conn, const struct bt_gatt_attr *attr,
			       void *buf, uint16_t len, uint16_t offset)
{
	ARG_UNUSED(attr);

	ssize_t ret;

	k_mutex_lock(&imu_payload_lock, K_FOREVER);
	ret = bt_gatt_attr_read(conn, attr, buf, len, offset,
				imu_payload, sizeof(imu_payload));
	k_mutex_unlock(&imu_payload_lock);

	return ret;
}

static void imu_ccc_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
	ARG_UNUSED(attr);
	atomic_set(&g_imu_notify, (value == BT_GATT_CCC_NOTIFY) ? 1 : 0);
}

/* ---- Write: switch ---- */
static ssize_t write_switch(struct bt_conn *conn, const struct bt_gatt_attr *attr,
			    const void *buf, uint16_t len, uint16_t offset, uint8_t flags)
{
	ARG_UNUSED(conn);
	ARG_UNUSED(attr);
	ARG_UNUSED(flags);

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

/* ---- Write: amplitude percent (0..100%) (back-compat) ---- */
static ssize_t write_amplitude_percent(struct bt_conn *conn, const struct bt_gatt_attr *attr,
				       const void *buf, uint16_t len, uint16_t offset, uint8_t flags)
{
	ARG_UNUSED(conn);
	ARG_UNUSED(attr);
	ARG_UNUSED(flags);

	if (len != 1 || offset) {
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
	}

	uint8_t pct = ((const uint8_t *)buf)[0];
	if (pct > 100) {
		pct = 100;
	}

	uint16_t code = pct_to_code(pct);

	atomic_set(&g_amp_pct, pct);
	atomic_set(&g_amp_code, code);

	amp_pct_cached = pct;
	amp_code_cached_le = sys_cpu_to_le16(code);

	return len;
}

/* ---- Write: amplitude code (u16 LE 0..65535) (finest control) ---- */
static ssize_t write_amplitude_code(struct bt_conn *conn, const struct bt_gatt_attr *attr,
				    const void *buf, uint16_t len, uint16_t offset, uint8_t flags)
{
	ARG_UNUSED(conn);
	ARG_UNUSED(attr);
	ARG_UNUSED(flags);

	if (len != sizeof(uint16_t) || offset) {
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
	}

	uint16_t code = sys_get_le16(buf);
	if (code > CACHEXIA_MAX_AMP_CODE) {
		code = CACHEXIA_MAX_AMP_CODE;
	}

	uint8_t pct = code_to_pct(code);

	atomic_set(&g_amp_code, code);
	atomic_set(&g_amp_pct, pct);

	amp_code_cached_le = sys_cpu_to_le16(code);
	amp_pct_cached = pct;

	return len;
}

/* ---- Write: frequency Hz (u16 LE) ---- */
static ssize_t write_frequency(struct bt_conn *conn, const struct bt_gatt_attr *attr,
			       const void *buf, uint16_t len, uint16_t offset, uint8_t flags)
{
	ARG_UNUSED(conn);
	ARG_UNUSED(attr);
	ARG_UNUSED(flags);

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
	ARG_UNUSED(attr);

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

	/* Back-compat amplitude percent (u8) */
	BT_GATT_CHARACTERISTIC(BT_UUID_CACHEXIA_AMPLITUDE,
		BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE,
		BT_GATT_PERM_READ | BT_GATT_PERM_WRITE,
		read_u8_cached, write_amplitude_percent, &amp_pct_cached),

	/* NEW: finest-control amplitude code (u16 LE) */
	BT_GATT_CHARACTERISTIC(BT_UUID_CACHEXIA_AMPLITUDE_CODE,
		BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE | BT_GATT_CHRC_WRITE_WITHOUT_RESP,
		BT_GATT_PERM_READ | BT_GATT_PERM_WRITE,
		read_u16_cached, write_amplitude_code, &amp_code_cached_le),

	BT_GATT_CHARACTERISTIC(BT_UUID_CACHEXIA_FREQUENCY,
		BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE,
		BT_GATT_PERM_READ | BT_GATT_PERM_WRITE,
		read_u16_cached, write_frequency, &freq_cached_le),

	BT_GATT_CHARACTERISTIC(BT_UUID_CACHEXIA_BATTERY,
		BT_GATT_CHRC_READ,
		BT_GATT_PERM_READ,
		read_battery, NULL, NULL),

	/* IMU streaming (READ + NOTIFY) */
	BT_GATT_CHARACTERISTIC(BT_UUID_CACHEXIA_IMU,
		BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
		BT_GATT_PERM_READ,
		read_imu_cached, NULL, NULL),

	BT_GATT_CCC(imu_ccc_cfg_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
);

static const struct bt_gatt_attr *find_attr_by_uuid(const struct bt_uuid *uuid)
{
	for (size_t i = 0; i < cachexia_cs_svc.attr_count; i++) {
		if (cachexia_cs_svc.attrs[i].uuid &&
		    bt_uuid_cmp(cachexia_cs_svc.attrs[i].uuid, uuid) == 0) {
			return &cachexia_cs_svc.attrs[i];
		}
	}
	return NULL;
}

int ble_cachexia_cs_srv_init(void)
{
	if (!device_is_ready(ad5778r_dev)) {
		LOG_ERR("AD5778R device not ready");
		return -ENODEV;
	}

	/* defaults */
	sw_cached = 0;
	amp_pct_cached = 0;
	amp_code_cached_le = sys_cpu_to_le16(0);

	atomic_clear(&g_switch);
	atomic_set(&g_amp_pct, 0);
	atomic_set(&g_amp_code, 0);

	atomic_set(&g_freq_hz, 5);
	freq_cached_le = sys_cpu_to_le16(5);

	/* Ensure output is safely off: High-Z + powerdown */
	(void)ad5778r_set_span(ad5778r_dev, 0 /*channel*/, AD5778R_SPAN_HIGH_Z);
	(void)ad5778r_power_down(ad5778r_dev, 0 /*channel*/);

	/* IMU defaults */
	atomic_clear(&g_imu_notify);
	memset(imu_payload, 0, sizeof(imu_payload));
	imu_value_attr = find_attr_by_uuid(BT_UUID_CACHEXIA_IMU);
	if (!imu_value_attr) {
		LOG_WRN("IMU attr not found (will retry lazily)");
	}

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

uint16_t ble_cachexia_cs_get_amplitude_code(void)
{
	return (uint16_t)atomic_get(&g_amp_code);
}

uint16_t ble_cachexia_cs_get_frequency_hz(void)
{
	return (uint16_t)atomic_get(&g_freq_hz);
}

bool ble_cachexia_cs_imu_notify_is_enabled(void)
{
	return atomic_get(&g_imu_notify) != 0;
}

int ble_cachexia_cs_imu_stream(const uint8_t *payload, uint16_t len)
{
	if (!payload || len != CACHEXIA_IMU_PAYLOAD_LEN) {
		return -EINVAL;
	}

	/* Update cached value for READ */
	k_mutex_lock(&imu_payload_lock, K_FOREVER);
	memcpy(imu_payload, payload, len);
	k_mutex_unlock(&imu_payload_lock);

	/* If nobody subscribed, we stop here (still keep cache fresh) */
	if (!ble_cachexia_cs_imu_notify_is_enabled()) {
		return 0;
	}

	/* Resolve attr lazily if needed */
	if (!imu_value_attr) {
		imu_value_attr = find_attr_by_uuid(BT_UUID_CACHEXIA_IMU);
		if (!imu_value_attr) {
			return -ENOENT;
		}
	}

	/* Notify all subscribed connections */
	return bt_gatt_notify(NULL, imu_value_attr, payload, len);
}