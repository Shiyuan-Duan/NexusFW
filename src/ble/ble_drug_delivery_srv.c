/* Drug Delivery BLE GATT Service
 * 5 characteristics: uint8 (0x00=off, 0x01=on)
 * Channel-to-GPIO mapping:
 *   ch1 -> P0.28, ch2 -> P0.29, ch3 -> P0.30, ch4 -> P0.25, ch5 -> P0.31
 */

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>
#include <string.h>

#include "ble_drug_delivery_srv.h"

LOG_MODULE_REGISTER(drug_srv, CONFIG_BT_LOG_LEVEL);

static const struct device *gpio0;
static const uint8_t pins[5] = { 28, 29, 30, 25, 31 };
static uint8_t ch_val[5]; /* cached for READ backs */

static int set_pin(size_t idx, uint8_t v)
{
    if (!gpio0) return -ENODEV;
    if (idx >= 5) return -EINVAL;
    return gpio_pin_set(gpio0, pins[idx], v ? 1 : 0);
}

static ssize_t read_u8_cached(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                              void *buf, uint16_t len, uint16_t offset)
{
    const uint8_t *src = attr->user_data;
    return bt_gatt_attr_read(conn, attr, buf, len, offset, src, 1);
}

static ssize_t write_bool(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                          const void *buf, uint16_t len, uint16_t offset, uint8_t flags)
{
    if (len != 1 || offset) {
        return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
    }
    uint8_t v = ((const uint8_t *)buf)[0] ? 1 : 0;

    /* Determine index by pointer arithmetic against ch_val[] */
    size_t idx = ((uint8_t *)attr->user_data) - &ch_val[0];
    if (idx >= 5) {
        return BT_GATT_ERR(BT_ATT_ERR_UNLIKELY);
    }

    if (set_pin(idx, v) != 0) {
        return BT_GATT_ERR(BT_ATT_ERR_UNLIKELY);
    }

    ch_val[idx] = v;
    return len;
}

BT_GATT_SERVICE_DEFINE(drug_svc,
    BT_GATT_PRIMARY_SERVICE(BT_UUID_DRUG_SVC),

    BT_GATT_CHARACTERISTIC(BT_UUID_DRUG_CH1,
        BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE,
        BT_GATT_PERM_READ | BT_GATT_PERM_WRITE,
        read_u8_cached, write_bool, &ch_val[0]),

    BT_GATT_CHARACTERISTIC(BT_UUID_DRUG_CH2,
        BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE,
        BT_GATT_PERM_READ | BT_GATT_PERM_WRITE,
        read_u8_cached, write_bool, &ch_val[1]),

    BT_GATT_CHARACTERISTIC(BT_UUID_DRUG_CH3,
        BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE,
        BT_GATT_PERM_READ | BT_GATT_PERM_WRITE,
        read_u8_cached, write_bool, &ch_val[2]),

    BT_GATT_CHARACTERISTIC(BT_UUID_DRUG_CH4,
        BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE,
        BT_GATT_PERM_READ | BT_GATT_PERM_WRITE,
        read_u8_cached, write_bool, &ch_val[3]),

    BT_GATT_CHARACTERISTIC(BT_UUID_DRUG_CH5,
        BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE,
        BT_GATT_PERM_READ | BT_GATT_PERM_WRITE,
        read_u8_cached, write_bool, &ch_val[4]),
);

int ble_drug_srv_init(void)
{
    gpio0 = DEVICE_DT_GET(DT_NODELABEL(gpio0));
    if (!device_is_ready(gpio0)) {
        LOG_ERR("gpio0 not ready");
        return -ENODEV;
    }
    /* Configure pins as outputs, default OFF (0) */
    for (size_t i = 0; i < 5; ++i) {
        (void)gpio_pin_configure(gpio0, pins[i], GPIO_OUTPUT);
        (void)gpio_pin_set(gpio0, pins[i], 0);
        ch_val[i] = 0;
    }
    LOG_INF("Drug delivery BLE service ready");
    return 0;
}

