#include <zephyr/types.h>
#include <stddef.h>
#include <string.h>
#include <errno.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/sys/reboot.h>
#include <zephyr/sys/atomic.h>
#include "ble.h"

LOG_MODULE_REGISTER(BLE_ADV);

static const struct bt_le_adv_param adv_param = BT_LE_ADV_PARAM_INIT(
    (BT_LE_ADV_OPT_CONNECTABLE | BT_LE_ADV_OPT_USE_IDENTITY),
    BT_GAP_ADV_SLOW_INT_MIN,
    BT_GAP_ADV_SLOW_INT_MAX,
    NULL);

static const struct bt_data ad[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
	BT_DATA(BT_DATA_NAME_COMPLETE, DEVICE_NAME, DEVICE_NAME_LEN),

};

static const struct bt_data sd[] = {
	BT_DATA_BYTES(BT_DATA_UUID128_ALL, BT_UUID_VAL),
};

/* BLE health flag and a keepalive work that reasserts advertising.
 * If advertising is already running, bt_le_adv_start() returns -EALREADY.
 */
static atomic_t g_ble_ok = ATOMIC_INIT(0);
static struct k_work_delayable adv_keepalive_work;

static void adv_keepalive_handler(struct k_work *work)
{
    ARG_UNUSED(work);
    int err = bt_le_adv_start(&adv_param, ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd));
    if (err == 0 || err == -EALREADY) {
        atomic_set(&g_ble_ok, 1);
    } else {
        atomic_set(&g_ble_ok, 0);
        LOG_ERR("Adv keepalive: start err %d", err);
    }
    /* Re-check periodically */
    (void)k_work_schedule(&adv_keepalive_work, K_SECONDS(5));
}

bool ble_is_healthy(void)
{
    return atomic_get(&g_ble_ok) ? true : false;
}


static void _on_connected(struct bt_conn *conn, uint8_t err)
{
    if (err) {
        printk("Connection failed (err %u)\n", err);
        atomic_set(&g_ble_ok, 0);
        return;
    }
    printk("Connected\n");
    atomic_set(&g_ble_ok, 1);

}

static void _on_disconnected(struct bt_conn *conn, uint8_t reason)
{
    printk("Disconnected (reason %u)\n", reason);
    /* Re-start advertising after disconnect */
    int err = bt_le_adv_start(&adv_param, ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd));
    if (err == 0 || err == -EALREADY) {
        atomic_set(&g_ble_ok, 1);
    } else {
        atomic_set(&g_ble_ok, 0);
        LOG_ERR("Advertising restart failed (err %d)", err);
    }
}

struct bt_conn_cb connection_callbacks = {
	.connected = _on_connected,
	.disconnected = _on_disconnected,
};

int t_ble_adv_start(void)
{	

    int err;
    printk("Initiating BLE\n");
    err = bt_enable(NULL);
    if (err) {
        LOG_ERR("Bluetooth init failed (err %d)\n", err);
        atomic_set(&g_ble_ok, 0);
        return -1;
    }

    bt_conn_cb_register(&connection_callbacks);

    LOG_INF("Bluetooth initialized\n");
    err = bt_le_adv_start(&adv_param, ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd));
    if (err) {
        LOG_ERR("Advertising failed to start (err %d)\n", err);
        atomic_set(&g_ble_ok, 0);
        return -1;
    }

    // stream_sensor_data(1);

    LOG_INF("Advertising successfully started with prioirty: %d\n", 1);
    atomic_set(&g_ble_ok, 1);

    /* Start periodic keepalive */
    k_work_init_delayable(&adv_keepalive_work, adv_keepalive_handler);
    (void)k_work_schedule(&adv_keepalive_work, K_SECONDS(5));
    return 0;
}

// K_THREAD_DEFINE(ble_t, 1024, ble_adv_thread_entry, NULL, NULL, NULL, 1, 0, 0);

