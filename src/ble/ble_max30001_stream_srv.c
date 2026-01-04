#include <zephyr/bluetooth/gatt.h>
#include <zephyr/device.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/atomic.h>
#include <string.h>
#include <errno.h>

#include "ble_max30001_stream_srv.h"
#include <driver/max30001.h>

LOG_MODULE_REGISTER(max30001_stream_srv, CONFIG_BT_LOG_LEVEL);

static const struct device *const max30001_dev = DEVICE_DT_GET_ONE(dsy_max30001);

static atomic_t g_switch = ATOMIC_INIT(0);
static uint8_t sw_cached;

static atomic_t g_biov_notify = ATOMIC_INIT(0);
static atomic_t g_bioz_notify = ATOMIC_INIT(0);

static void biov_ccc_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
	ARG_UNUSED(attr);
	atomic_set(&g_biov_notify, (value == BT_GATT_CCC_NOTIFY) ? 1 : 0);
}

static void bioz_ccc_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
	ARG_UNUSED(attr);
	atomic_set(&g_bioz_notify, (value == BT_GATT_CCC_NOTIFY) ? 1 : 0);
}

static ssize_t read_u8_cached(struct bt_conn *conn, const struct bt_gatt_attr *attr,
			      void *buf, uint16_t len, uint16_t offset)
{
	const uint8_t *src = attr->user_data;
	return bt_gatt_attr_read(conn, attr, buf, len, offset, src, 1);
}

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

	if (v == 0 && device_is_ready(max30001_dev)) {
		(void)max30001_stream_off(max30001_dev);
	}

	return len;
}

BT_GATT_SERVICE_DEFINE(max30001_stream_svc,
	BT_GATT_PRIMARY_SERVICE(BT_UUID_MAX30001_SVC),

	BT_GATT_CHARACTERISTIC(BT_UUID_MAX30001_SWITCH,
		BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE,
		BT_GATT_PERM_READ | BT_GATT_PERM_WRITE,
		read_u8_cached, write_switch, &sw_cached),

	/* BioV stream notify */
	BT_GATT_CHARACTERISTIC(BT_UUID_MAX30001_BIOV,
		BT_GATT_CHRC_NOTIFY,
		BT_GATT_PERM_NONE,
		NULL, NULL, NULL),
	BT_GATT_CCC(biov_ccc_cfg_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),

	/* BioZ stream notify */
	BT_GATT_CHARACTERISTIC(BT_UUID_MAX30001_BIOZ,
		BT_GATT_CHRC_NOTIFY,
		BT_GATT_PERM_NONE,
		NULL, NULL, NULL),
	BT_GATT_CCC(bioz_ccc_cfg_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
);

int ble_max30001_stream_srv_init(void)
{
	if (!device_is_ready(max30001_dev)) {
		LOG_ERR("MAX30001 device not ready");
		return -ENODEV;
	}

	sw_cached = 0;
	atomic_clear(&g_switch);
	atomic_clear(&g_biov_notify);
	atomic_clear(&g_bioz_notify);

	(void)max30001_stream_off(max30001_dev);

	LOG_INF("MAX30001 stream BLE service ready");
	return 0;
}

uint8_t ble_max30001_get_switch(void)
{
	return (uint8_t)atomic_get(&g_switch);
}

int ble_max30001_notify_biov(const uint8_t *data, uint16_t len)
{
	if (!atomic_get(&g_biov_notify)) {
		return -EACCES;
	}
	return bt_gatt_notify_uuid(NULL, BT_UUID_MAX30001_BIOV,
				   max30001_stream_svc.attrs, data, len);
}

int ble_max30001_notify_bioz(const uint8_t *data, uint16_t len)
{
	if (!atomic_get(&g_bioz_notify)) {
		return -EACCES;
	}
	return bt_gatt_notify_uuid(NULL, BT_UUID_MAX30001_BIOZ,
				   max30001_stream_svc.attrs, data, len);
}