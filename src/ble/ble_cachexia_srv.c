/*  Cachexia Stimulation BLE GATT Service
 *  - Characteristic 1: Switch (uint8, 0/1) controls stimulation on/off
 *  - Characteristic 2: Amplitude (uint8, 0..100) maps to 0x0000..0xFFFF
 *  - Characteristic 3: Battery (float32, read-only), placeholder 4.2f
 *  Uses AD5689 channel A (single-channel behavior) only.
 */
// ble_cachexia_srv.c

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/device.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/atomic.h>
#include <string.h>

#include "ble_cachexia_srv.h"
#include "ad5689.h"

LOG_MODULE_REGISTER(cachexia_srv, CONFIG_BT_LOG_LEVEL);

/* AD5689 instance (required to 0-out when turning switch off) */
static const struct device *const ad5689_dev = DEVICE_DT_GET_ONE(dsy_ad5689);

/* State (atomic for cross-thread access) */
static atomic_t g_switch = ATOMIC_INIT(0);     /* 0=off, 1=on */
static atomic_t g_amp_pct = ATOMIC_INIT(0);    /* 0..100 */

/* --- Read helpers --- */
static ssize_t read_u8_cached(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                              void *buf, uint16_t len, uint16_t offset)
{
    const uint8_t *src = attr->user_data;
    return bt_gatt_attr_read(conn, attr, buf, len, offset, src, 1);
}

/* --- Write: switch --- */
static uint8_t sw_cached;  /* mirror for READ backs */
static ssize_t write_switch(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                            const void *buf, uint16_t len, uint16_t offset, uint8_t flags)
{
    if (len != 1 || offset) {
        return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
    }
    uint8_t v = ((const uint8_t *)buf)[0] ? 1 : 0;
    atomic_set(&g_switch, v);
    sw_cached = v;

    /* If turned off, immediately zero DAC A for safety. */
    if (v == 0 && device_is_ready(ad5689_dev)) {
        (void)ad5689_write(ad5689_dev, 0 /*A*/, 0);
    }
    return len;
}

/* --- Write: amplitude --- */
static uint8_t amp_cached; /* 0..100 */
static ssize_t write_amplitude(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                               const void *buf, uint16_t len, uint16_t offset, uint8_t flags)
{
    if (len != 1 || offset) {
        return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
    }
    uint8_t v = ((const uint8_t *)buf)[0];
    if (v > 100) v = 100;
    atomic_set(&g_amp_pct, v);
    amp_cached = v;
    return len;
}

/* --- Read: battery float (placeholder 4.2f) --- */
static ssize_t read_battery(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                            void *buf, uint16_t len, uint16_t offset)
{
    float f = 4.2f; /* placeholder */
    uint32_t bits;
    memcpy(&bits, &f, sizeof(bits));
    bits = sys_cpu_to_le32(bits);
    return bt_gatt_attr_read(conn, attr, buf, len, offset, &bits, sizeof(bits));
}

/* GATT table */
BT_GATT_SERVICE_DEFINE(cachexia_svc,
    BT_GATT_PRIMARY_SERVICE(BT_UUID_CACHEXIA_SVC),

    BT_GATT_CHARACTERISTIC(BT_UUID_CACHEXIA_SWITCH,
        BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE,
        BT_GATT_PERM_READ | BT_GATT_PERM_WRITE,
        read_u8_cached, write_switch, &sw_cached),

    BT_GATT_CHARACTERISTIC(BT_UUID_CACHEXIA_AMPLITUDE,
        BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE,
        BT_GATT_PERM_READ | BT_GATT_PERM_WRITE,
        read_u8_cached, write_amplitude, &amp_cached),

    BT_GATT_CHARACTERISTIC(BT_UUID_CACHEXIA_BATTERY,
        BT_GATT_CHRC_READ,
        BT_GATT_PERM_READ,
        read_battery, NULL, NULL),
);

/* Public API */
int ble_cachexia_srv_init(void)
{
    if (!device_is_ready(ad5689_dev)) {
        LOG_ERR("AD5689 device not ready");
        return -ENODEV;
    }
    /* defaults */
    sw_cached = 0;
    amp_cached = 0;
    atomic_clear(&g_switch);
    atomic_set(&g_amp_pct, 0);

    /* Ensure DAC A is zeroed on init */
    (void)ad5689_write(ad5689_dev, 0, 0);
    LOG_INF("Cachexia BLE service ready");
    return 0;
}

uint8_t ble_cachexia_get_switch(void)
{
    return (uint8_t)atomic_get(&g_switch);
}

uint8_t ble_cachexia_get_amplitude_percent(void)
{
    return (uint8_t)atomic_get(&g_amp_pct);
}
