#include <zephyr/types.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/logging/log.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/sys/atomic.h>

#include "ble_adv_core.h"

LOG_MODULE_REGISTER(ble_adv_core, CONFIG_BT_LOG_LEVEL);

/* Health flag and a keepalive that reasserts advertising. */
static atomic_t g_ble_ok = ATOMIC_INIT(0);
static struct k_work_delayable adv_keepalive_work;
static atomic_t adv_work_ready = ATOMIC_INIT(0);
static const struct ble_adv_profile *g_prof;

static void adv_keepalive_handler(struct k_work *work)
{
    ARG_UNUSED(work);
    int err = bt_le_adv_start(g_prof->params, g_prof->ad, g_prof->ad_len,
                              g_prof->sd, g_prof->sd_len);
    if (err == 0 || err == -EALREADY) {
        atomic_set(&g_ble_ok, 1);
    } else {
        /* Try stop+restart to recover */
        (void)bt_le_adv_stop();
        k_sleep(K_MSEC(50));
        err = bt_le_adv_start(g_prof->params, g_prof->ad, g_prof->ad_len,
                              g_prof->sd, g_prof->sd_len);
        if (err == 0 || err == -EALREADY) {
            atomic_set(&g_ble_ok, 1);
        } else {
            atomic_set(&g_ble_ok, 0);
            LOG_ERR("Adv keepalive: start err %d", err);
        }
    }
    (void)k_work_schedule(&adv_keepalive_work, K_SECONDS(g_prof->keepalive_sec));
}

bool ble_is_healthy(void)
{
    return atomic_get(&g_ble_ok) ? true : false;
}

static void _on_connected(struct bt_conn *conn, uint8_t err)
{
    if (err) {
        LOG_WRN("Connection failed (err %u)", err);
        printk("Connection failed (err %u)\n", err);
        atomic_set(&g_ble_ok, 0);
        return;
    }
    LOG_INF("Connected");
    printk("Connected\n");
    atomic_set(&g_ble_ok, 1);

    if (g_prof && g_prof->conn_param) {
        (void)bt_conn_le_param_update(conn, g_prof->conn_param);
    }
}

static void _on_disconnected(struct bt_conn *conn, uint8_t reason)
{
    LOG_INF("Disconnected (reason %u)", reason);
    printk("Disconnected (reason %u)\n", reason);
    atomic_set(&g_ble_ok, 0);
    /* If keepalive work is ready, nudge it; otherwise restart directly */
    if (atomic_get(&adv_work_ready)) {
        (void)k_work_reschedule(&adv_keepalive_work, K_MSEC(10));
    } else if (g_prof) {
        int err = bt_le_adv_start(g_prof->params, g_prof->ad, g_prof->ad_len,
                                  g_prof->sd, g_prof->sd_len);
        if (err == 0 || err == -EALREADY) {
            atomic_set(&g_ble_ok, 1);
        } else {
            LOG_WRN("Restart adv in disconnect cb failed: %d", err);
        }
    }
}

static struct bt_conn_cb connection_callbacks = {
    .connected = _on_connected,
    .disconnected = _on_disconnected,
};

void ble_adv_thread(void *p1, void *p2, void *p3)
{
    ARG_UNUSED(p2); ARG_UNUSED(p3);
    g_prof = (const struct ble_adv_profile *)p1;

    if (!g_prof || !g_prof->params || !g_prof->ad || !g_prof->sd) {
        LOG_ERR("Invalid BLE adv profile");
        return;
    }

    int err;
    LOG_INF("Enabling Bluetooth");
    err = bt_enable(NULL);
    if (err) {
        LOG_ERR("Bluetooth init failed (err %d)", err);
        atomic_set(&g_ble_ok, 0);
        return;
    }

    /* Init keepalive before any callback might attempt to reschedule */
    k_work_init_delayable(&adv_keepalive_work, adv_keepalive_handler);
    atomic_set(&adv_work_ready, 1);

    bt_conn_cb_register(&connection_callbacks);

    LOG_INF("Bluetooth initialized");
    err = bt_le_adv_start(g_prof->params, g_prof->ad, g_prof->ad_len,
                          g_prof->sd, g_prof->sd_len);
    if (err) {
        LOG_ERR("Advertising failed to start (err %d)", err);
        atomic_set(&g_ble_ok, 0);
    } else {
        LOG_INF("Advertising started");
        atomic_set(&g_ble_ok, 1);
    }

    /* Start periodic keepalive */
    uint32_t period = g_prof->keepalive_sec ? g_prof->keepalive_sec : 5;
    (void)k_work_schedule(&adv_keepalive_work, K_SECONDS(period));

    /* Idle thread forever; work handles restarts */
    for (;;) {
        k_sleep(K_SECONDS(60));
    }
}
