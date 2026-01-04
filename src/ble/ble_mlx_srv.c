/*  MLX90393 BLE GATT Service
 *  - STREAM characteristic: Notify-only, carries struct mlx_ble_sample
 *  - CTRL characteristic:   uint8 R/W, 0 = stop, non-zero = start
 */

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/logging/log.h>
#include <string.h>

#include "ble_mlx_srv.h"

LOG_MODULE_REGISTER(mlx_srv, CONFIG_BT_LOG_LEVEL);

/* State */
static atomic_t g_running = ATOMIC_INIT(0);
static uint8_t ctrl_cached;   /* mirror for READ backs */

/* Wait-for-start semaphore */
static struct k_sem start_sem;

/* CCC handler: just log */
static void stream_ccc_cfg_changed(const struct bt_gatt_attr *attr,
                                   uint16_t value)
{
    ARG_UNUSED(attr);
    bool enabled = (value == BT_GATT_CCC_NOTIFY);
    LOG_INF("MLX stream notify %s", enabled ? "enabled" : "disabled");
}

/* Simple u8 read helper (for CTRL) */
static ssize_t read_u8_cached(struct bt_conn *conn,
                              const struct bt_gatt_attr *attr,
                              void *buf, uint16_t len, uint16_t offset)
{
    const uint8_t *src = attr->user_data;
    return bt_gatt_attr_read(conn, attr, buf, len, offset, src, 1);
}

/* CTRL write: 1 byte, 0 = stop, non-zero = start */
static ssize_t write_ctrl(struct bt_conn *conn,
                          const struct bt_gatt_attr *attr,
                          const void *buf, uint16_t len,
                          uint16_t offset, uint8_t flags)
{
    ARG_UNUSED(conn);
    ARG_UNUSED(attr);
    ARG_UNUSED(flags);

    if (len != 1 || offset != 0) {
        return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
    }

    uint8_t v = ((const uint8_t *)buf)[0] ? 1 : 0;
    ctrl_cached = v;
    atomic_set(&g_running, v);

    if (v) {
        /* Wake waiting app thread */
        k_sem_give(&start_sem);
    }

    return len;
}

/* GATT table:
 *  0: Primary Service
 *  1: STREAM characteristic declaration
 *  2: STREAM value
 *  3: STREAM CCC
 *  4: CTRL characteristic declaration
 *  5: CTRL value
 */
BT_GATT_SERVICE_DEFINE(mlx_svc,
    BT_GATT_PRIMARY_SERVICE(BT_UUID_MLX_SVC),

    BT_GATT_CHARACTERISTIC(BT_UUID_MLX_STREAM,
        BT_GATT_CHRC_NOTIFY,
        BT_GATT_PERM_NONE,
        NULL, NULL, NULL),
    BT_GATT_CCC(stream_ccc_cfg_changed,
        BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),

    BT_GATT_CHARACTERISTIC(BT_UUID_MLX_CTRL,
        BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE,
        BT_GATT_PERM_READ | BT_GATT_PERM_WRITE,
        read_u8_cached, write_ctrl, &ctrl_cached),
);

/* Index of STREAM value attribute inside mlx_svc.attrs[] */
#define MLX_STREAM_ATTR_IDX 2

int ble_mlx_srv_init(void)
{
    k_sem_init(&start_sem, 0, 1);
    atomic_clear(&g_running);
    ctrl_cached = 0;

    LOG_INF("MLX BLE service ready");
    return 0;
}

bool ble_mlx_is_running(void)
{
    return atomic_get(&g_running) != 0;
}

int ble_mlx_wait_for_start(k_timeout_t timeout)
{
    int ret;

    if (ble_mlx_is_running()) {
        return 0;
    }

    ret = k_sem_take(&start_sem, timeout);
    if (ret < 0) {
        return ret;
    }

    return ble_mlx_is_running() ? 0 : -ETIMEDOUT;
}

int ble_mlx_notify_sample(const struct mlx_ble_sample *s)
{
    if (!s) {
        return -EINVAL;
    }

    if (!ble_mlx_is_running()) {
        return 0;
    }

    const struct bt_gatt_attr *attr = &mlx_svc.attrs[MLX_STREAM_ATTR_IDX];
    return bt_gatt_notify(NULL, attr, s, sizeof(*s));
}
