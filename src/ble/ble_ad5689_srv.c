/*  AD5689 Dual‑DAC GATT Service – 读操作只回缓存，不再直读硬件  */
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/device.h>
#include <zephyr/logging/log.h>
#include <string.h>
#include "ble_ad5689_srv.h"
#include "ad5689.h"

LOG_MODULE_REGISTER(ad5689_srv, CONFIG_BT_LOG_LEVEL);

/* Devicetree 实例 – 若不存在编译就会报错 */
static const struct device *const ad5689_dev = DEVICE_DT_GET_ONE(dsy_ad5689);

/* 缓冲：大端 16‑bit */
static uint8_t ch1_buf[2];
static uint8_t ch2_buf[2];

/* ───── READ 直接返回缓存 ───── */
static ssize_t ch_read(struct bt_conn *conn,
                       const struct bt_gatt_attr *attr,
                       void *buf, uint16_t len, uint16_t offset)
{
    const uint8_t *src = attr->user_data;     /* 指向 ch1_buf 或 ch2_buf */
    return bt_gatt_attr_read(conn, attr, buf, len, offset, src, 2);
}

/* ───── WRITE 更新 DAC 并刷新缓存 ───── */
static ssize_t ch_write(struct bt_conn *conn,
                        const struct bt_gatt_attr *attr,
                        const void *buf, uint16_t len,
                        uint16_t offset, uint8_t flags)
{
    if (len != 2 || offset)
        return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);

    uint8_t  chan = (attr->user_data == ch1_buf) ? 0 : 1;
    uint16_t code = sys_get_be16(buf);

    if (!device_is_ready(ad5689_dev) ||
        ad5689_write(ad5689_dev, chan, code))
        return BT_GATT_ERR(BT_ATT_ERR_UNLIKELY);

    memcpy(attr->user_data, buf, 2);      /* 刷新缓存 */
    return len;
}

/* ───── GATT 表 ───── */
BT_GATT_SERVICE_DEFINE(ad5689_svc,
    BT_GATT_PRIMARY_SERVICE(BT_UUID_DECLARE_128(
        0xA0,0xA4,0xA6,0x80,0x96,0xBE,0x42,0x22,
        0xB4,0x1E,0x98,0xEA,0x76,0xB0,0x12,0x0C)),

    BT_GATT_CHARACTERISTIC(BT_UUID_DAC_CHANNEL1,
        BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE,
        BT_GATT_PERM_READ | BT_GATT_PERM_WRITE,
        ch_read, ch_write, ch1_buf),

    BT_GATT_CHARACTERISTIC(BT_UUID_DAC_CHANNEL2,
        BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE,
        BT_GATT_PERM_READ | BT_GATT_PERM_WRITE,
        ch_read, ch_write, ch2_buf),
);

/* ───── 初始化 ───── */
int ble_ad5689_srv_init(void)
{
    if (!device_is_ready(ad5689_dev)) {
        LOG_ERR("AD5689 device not ready");
        return -ENODEV;
    }

    /* 尝试读取初值；若驱动不支持 read_dac，则保持 0x0000 */
    const struct ad5689_driver_api *api = ad5689_dev->api;
    uint16_t v;
    if (api && api->read_dac && !api->read_dac(ad5689_dev, 0, &v))
        sys_put_be16(v, ch1_buf);
    if (api && api->read_dac && !api->read_dac(ad5689_dev, 1, &v))
        sys_put_be16(v, ch2_buf);

    LOG_INF("AD5689 GATT service ready");
    return 0;
}

/* ───── 主动 Notify（可选） ───── */
int ble_ad5689_notify_ch1(const uint8_t be16[2])
{
    memcpy(ch1_buf, be16, 2);
    return bt_gatt_notify(NULL, &ad5689_svc.attrs[1], ch1_buf, 2);
}
int ble_ad5689_notify_ch2(const uint8_t be16[2])
{
    memcpy(ch2_buf, be16, 2);
    return bt_gatt_notify(NULL, &ad5689_svc.attrs[3], ch2_buf, 2);
}
