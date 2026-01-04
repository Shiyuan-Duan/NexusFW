#include <zephyr/types.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/logging/log.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/sys/atomic.h>
#include <errno.h>

#include "ble_adv_core.h"

LOG_MODULE_REGISTER(ble_adv_core, CONFIG_BT_LOG_LEVEL);

/*
 * Health/State flags:
 * - g_ble_ok:   1 => “我们认为 BLE 当前健康（已连接 或 广告已启动）”
 * - g_connected:1 => 当前有连接
 * - g_adv_on:   1 => 我们最近一次启动广告成功（逻辑标记，用于避免 keepalive 反复 start）
 */
static atomic_t g_ble_ok     = ATOMIC_INIT(0);
static atomic_t g_connected  = ATOMIC_INIT(0);
static atomic_t g_adv_on     = ATOMIC_INIT(0);

static struct k_work_delayable adv_keepalive_work;
static atomic_t adv_work_ready = ATOMIC_INIT(0);
static const struct ble_adv_profile *g_prof;

/* 尝试启动广告：成功则更新状态；失败只记录，不做 stop/start */
static int try_start_adv(void)
{
    if (!g_prof) {
        return -EINVAL;
    }

    int err = bt_le_adv_start(g_prof->params,
                             g_prof->ad, g_prof->ad_len,
                             g_prof->sd, g_prof->sd_len);

    if (err == 0 || err == -EALREADY) {
        atomic_set(&g_adv_on, 1);
        atomic_set(&g_ble_ok, 1);
        return 0;
    }

    /* 注意：这里不 stop/start，不折腾控制器 */
    atomic_clear(&g_adv_on);
    atomic_clear(&g_ble_ok);
    LOG_ERR("Adv start failed: %d", err);
    return err;
}

static void adv_keepalive_handler(struct k_work *work)
{
    ARG_UNUSED(work);

    /* 统一在结尾调度下一次 */
    uint32_t period = (g_prof && g_prof->keepalive_sec) ? g_prof->keepalive_sec : 5;

    /*
     * ✅ 关键策略：
     * - 如果已连接：不要碰广告
     * - 如果我们认为广告已经在跑（g_adv_on=1）：不要反复 start
     * - 只有在“未连接 且 广告未确认在跑”时，才尝试 start
     */
    if (atomic_get(&g_connected)) {
        /* 已连接：健康 */
        atomic_set(&g_ble_ok, 1);
        goto out;
    }

    if (atomic_get(&g_adv_on)) {
        /* 我们认为广告在跑：健康 */
        atomic_set(&g_ble_ok, 1);
        goto out;
    }

    /* 未连接且广告没起来：尝试启动一次 */
    (void)try_start_adv();

out:
    (void)k_work_schedule(&adv_keepalive_work, K_SECONDS(period));
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
        atomic_clear(&g_connected);
        atomic_clear(&g_ble_ok);
        /* 连接失败后，让 keepalive 去确保广告还在 */
        atomic_clear(&g_adv_on);
        if (atomic_get(&adv_work_ready)) {
            (void)k_work_reschedule(&adv_keepalive_work, K_MSEC(50));
        }
        return;
    }

    LOG_INF("Connected");
    printk("Connected\n");

    atomic_set(&g_connected, 1);
    atomic_set(&g_ble_ok, 1);

    /*
     * 连接建立后，控制器/栈通常会停止 connectable advertising；
     * 我们把 g_adv_on 清掉，避免 keepalive 误以为广告还在。
     */
    atomic_clear(&g_adv_on);

    if (g_prof && g_prof->conn_param) {
        (void)bt_conn_le_param_update(conn, g_prof->conn_param);
    }
}

static void _on_disconnected(struct bt_conn *conn, uint8_t reason)
{
    ARG_UNUSED(conn);

    LOG_INF("Disconnected (reason %u)", reason);
    printk("Disconnected (reason %u)\n", reason);

    atomic_clear(&g_connected);
    atomic_clear(&g_ble_ok);
    atomic_clear(&g_adv_on);

    /* ❗断开后不要在回调里直接 start，交给 keepalive 统一拉起，避免竞态 */
    if (atomic_get(&adv_work_ready)) {
        (void)k_work_reschedule(&adv_keepalive_work, K_MSEC(20));
    }
}

static struct bt_conn_cb connection_callbacks = {
    .connected = _on_connected,
    .disconnected = _on_disconnected,
};

void ble_adv_thread(void *p1, void *p2, void *p3)
{
    ARG_UNUSED(p2);
    ARG_UNUSED(p3);

    g_prof = (const struct ble_adv_profile *)p1;

    if (!g_prof || !g_prof->params || !g_prof->ad || !g_prof->sd) {
        LOG_ERR("Invalid BLE adv profile");
        return;
    }

    LOG_INF("Enabling Bluetooth");
    int err = bt_enable(NULL);
    if (err) {
        LOG_ERR("Bluetooth init failed (err %d)", err);
        atomic_clear(&g_ble_ok);
        return;
    }

    /* Init keepalive work before callbacks might reschedule */
    k_work_init_delayable(&adv_keepalive_work, adv_keepalive_handler);
    atomic_set(&adv_work_ready, 1);

    bt_conn_cb_register(&connection_callbacks);

    LOG_INF("Bluetooth initialized");

    /* 初次启动广告（一次） */
    err = try_start_adv();
    if (err == 0) {
        LOG_INF("Advertising started");
    } else {
        LOG_WRN("Advertising not started yet (err %d). Keepalive will retry.", err);
    }

    /* Start periodic keepalive */
    uint32_t period = g_prof->keepalive_sec ? g_prof->keepalive_sec : 5;
    (void)k_work_schedule(&adv_keepalive_work, K_SECONDS(period));

    /* Idle thread forever; work handles restarts */
    for (;;) {
        k_sleep(K_SECONDS(60));
    }
}