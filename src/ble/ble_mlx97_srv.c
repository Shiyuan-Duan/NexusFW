/*
 *  MLX90397 BLE GATT Service (dual stream)
 *  - STREAM0 characteristic: Notify-only, carries struct mlx97_ble_sample for sensor0
 *  - STREAM1 characteristic: Notify-only, carries struct mlx97_ble_sample for sensor1
 *  - CTRL  characteristic:   uint8 R/W, 0 = stop, non-zero = start
 */

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/logging/log.h>
#include <string.h>
#include <errno.h>

#include <ble/ble_mlx97_srv.h>

LOG_MODULE_REGISTER(mlx97_srv, CONFIG_BT_LOG_LEVEL);

/* State */
static atomic_t g_running = ATOMIC_INIT(0);
static uint8_t ctrl_cached;   /* mirror for READ backs */
static struct k_sem start_sem;

/* Optional: track CCC enables (so notify can be silent if not subscribed) */
static atomic_t g_ccc0 = ATOMIC_INIT(0);
static atomic_t g_ccc1 = ATOMIC_INIT(0);

static void stream0_ccc_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
    ARG_UNUSED(attr);
    bool enabled = (value == BT_GATT_CCC_NOTIFY);
    atomic_set(&g_ccc0, enabled ? 1 : 0);
    LOG_INF("MLX97 stream0 notify %s", enabled ? "enabled" : "disabled");
}

static void stream1_ccc_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
    ARG_UNUSED(attr);
    bool enabled = (value == BT_GATT_CCC_NOTIFY);
    atomic_set(&g_ccc1, enabled ? 1 : 0);
    LOG_INF("MLX97 stream1 notify %s", enabled ? "enabled" : "disabled");
}

static ssize_t read_u8_cached(struct bt_conn *conn,
                              const struct bt_gatt_attr *attr,
                              void *buf, uint16_t len, uint16_t offset)
{
    const uint8_t *src = attr->user_data;
    return bt_gatt_attr_read(conn, attr, buf, len, offset, src, 1);
}

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
        k_sem_give(&start_sem);
    }

    return len;
}

/* Attribute indices:
 *  0: Primary Service
 *  1: STREAM0 char decl
 *  2: STREAM0 value
 *  3: STREAM0 CCC
 *  4: STREAM1 char decl
 *  5: STREAM1 value
 *  6: STREAM1 CCC
 *  7: CTRL char decl
 *  8: CTRL value
 */
BT_GATT_SERVICE_DEFINE(mlx97_svc,
    BT_GATT_PRIMARY_SERVICE(BT_UUID_MLX97_SVC),

    BT_GATT_CHARACTERISTIC(BT_UUID_MLX97_STREAM0,
        BT_GATT_CHRC_NOTIFY,
        BT_GATT_PERM_NONE,
        NULL, NULL, NULL),
    BT_GATT_CCC(stream0_ccc_cfg_changed,
        BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),

    BT_GATT_CHARACTERISTIC(BT_UUID_MLX97_STREAM1,
        BT_GATT_CHRC_NOTIFY,
        BT_GATT_PERM_NONE,
        NULL, NULL, NULL),
    BT_GATT_CCC(stream1_ccc_cfg_changed,
        BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),

    BT_GATT_CHARACTERISTIC(BT_UUID_MLX97_CTRL,
        BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE,
        BT_GATT_PERM_READ | BT_GATT_PERM_WRITE,
        read_u8_cached, write_ctrl, &ctrl_cached),
);

#define MLX97_STREAM0_ATTR_IDX 2
#define MLX97_STREAM1_ATTR_IDX 5

int ble_mlx97_srv_init(void)
{
    k_sem_init(&start_sem, 0, 1);
    atomic_clear(&g_running);
    atomic_clear(&g_ccc0);
    atomic_clear(&g_ccc1);
    ctrl_cached = 0;

    LOG_INF("MLX97 BLE service ready");
    return 0;
}

bool ble_mlx97_is_running(void)
{
    return atomic_get(&g_running) != 0;
}

int ble_mlx97_wait_for_start(k_timeout_t timeout)
{
    if (ble_mlx97_is_running()) {
        return 0;
    }
    int ret = k_sem_take(&start_sem, timeout);
    if (ret < 0) {
        return ret;
    }
    return ble_mlx97_is_running() ? 0 : -ETIMEDOUT;
}

static int notify_common(int which, const struct mlx97_ble_sample *s)
{
    if (!s) return -EINVAL;
    if (!ble_mlx97_is_running()) return 0;

    /* Optional CCC gating */
    if (which == 0 && atomic_get(&g_ccc0) == 0) return 0;
    if (which == 1 && atomic_get(&g_ccc1) == 0) return 0;

    const struct bt_gatt_attr *attr =
        (which == 0) ? &mlx97_svc.attrs[MLX97_STREAM0_ATTR_IDX]
                     : &mlx97_svc.attrs[MLX97_STREAM1_ATTR_IDX];

    return bt_gatt_notify(NULL, attr, s, sizeof(*s));
}

int ble_mlx97_notify0(const struct mlx97_ble_sample *s) { return notify_common(0, s); }
int ble_mlx97_notify1(const struct mlx97_ble_sample *s) { return notify_common(1, s); }