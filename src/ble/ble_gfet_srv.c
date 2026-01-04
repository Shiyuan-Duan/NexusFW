/*  GFET BLE GATT Service — notify-only streaming of gfet_sample */
// ble_gfet_srv.c
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/logging/log.h>
#include <string.h>
#include <zephyr/kernel.h>

#include "ble_gfet_srv.h"

LOG_MODULE_REGISTER(gfet_srv, CONFIG_BT_LOG_LEVEL);

static bool notify_enabled;
static K_SEM_DEFINE(start_sem, 0, 1);
static atomic_t running = ATOMIC_INIT(0);
static uint8_t ctrl_cached; /* mirrors last written control value for READ back */
static struct gfet_cfg g_cfg = {
    .v_start_mV = 200,
    .v_stop_mV = 1200,
    .v_step_mV = 5,
    .dwell_ms = 10,
    .avg_N = 4,
};

static void ccc_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
    notify_enabled = (value == BT_GATT_CCC_NOTIFY);
}

/* Primary service with one notify-only characteristic */
static ssize_t read_u8_cached(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                              void *buf, uint16_t len, uint16_t offset)
{
    const uint8_t *src = attr->user_data;
    return bt_gatt_attr_read(conn, attr, buf, len, offset, src, 1);
}

static ssize_t read_cfg(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                        void *buf, uint16_t len, uint16_t offset)
{
    /* Read full gfet_cfg struct from attribute user_data */
    return bt_gatt_attr_read(conn, attr, buf, len, offset,
                             attr->user_data, sizeof(struct gfet_cfg));
}

static ssize_t write_cfg(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                         const void *buf, uint16_t len, uint16_t offset, uint8_t flags)
{
    /* Support simple writes; reject out-of-bounds */
    if (offset > sizeof(struct gfet_cfg)) {
        return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
    }
    if ((offset + len) > sizeof(struct gfet_cfg)) {
        return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
    }

    memcpy(((uint8_t *)attr->user_data) + offset, buf, len);
    return len;
}

static ssize_t write_ctrl(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                          const void *buf, uint16_t len, uint16_t offset, uint8_t flags)
{
    if (len != 1 || offset) {
        return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
    }
    uint8_t v = ((const uint8_t *)buf)[0];
    ctrl_cached = v;
    /* Non-zero -> START; zero -> STOP */
    if (v) {
        atomic_set(&running, 1);
        (void)k_sem_give(&start_sem);
    } else {
        atomic_set(&running, 0);
    }
    return len;
}

BT_GATT_SERVICE_DEFINE(gfet_svc,
    BT_GATT_PRIMARY_SERVICE(BT_UUID_GFET_SVC),
    /* Stream characteristic (Notify only) */
    BT_GATT_CHARACTERISTIC(BT_UUID_GFET_STREAM,
        BT_GATT_CHRC_NOTIFY,
        0, /* no read */
        NULL, NULL, NULL),
    BT_GATT_CCC(ccc_cfg_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
    /* Control characteristic: write 1 to start */
    BT_GATT_CHARACTERISTIC(BT_UUID_GFET_CTRL,
        BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE,
        BT_GATT_PERM_READ | BT_GATT_PERM_WRITE,
        read_u8_cached, write_ctrl, &ctrl_cached)
    ,
    /* Config characteristic: read/write sweep parameters */
    BT_GATT_CHARACTERISTIC(BT_UUID_GFET_CFG,
        BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE,
        BT_GATT_PERM_READ | BT_GATT_PERM_WRITE,
        read_cfg, write_cfg, &g_cfg)
);

int ble_gfet_srv_init(void)
{
    notify_enabled = false;
    ctrl_cached = 0;
    return 0;
}

int ble_gfet_notify_sample(const struct gfet_sample *s)
{
    if (!notify_enabled || s == NULL) return 0;

    /* pack into little-endian buffer */
    uint8_t buf[sizeof(struct gfet_sample)];
    struct gfet_sample le = {
        .t_ms = sys_cpu_to_le32(s->t_ms),
        .vg_mV = sys_cpu_to_le32((uint32_t)s->vg_mV),
        .id_nA = sys_cpu_to_le32((uint32_t)s->id_nA),
    };
    memcpy(buf, &le, sizeof(le));

    /* Stream characteristic value attribute is at index 2 (0:svc,1:chrc,2:value) */
    return bt_gatt_notify(NULL, &gfet_svc.attrs[2], buf, sizeof(buf));
}

int ble_gfet_wait_for_start(k_timeout_t timeout)
{
    return k_sem_take(&start_sem, timeout);
}

void ble_gfet_get_cfg(struct gfet_cfg *out)
{
    if (!out) return;
    /* Shallow copy; attribute storage is already LE */
    memcpy(out, &g_cfg, sizeof(*out));
}

bool ble_gfet_is_running(void)
{
    return atomic_get(&running) != 0;
}
