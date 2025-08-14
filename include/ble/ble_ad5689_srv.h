/*  ─── ble_ad5689_srv.h ───  AD5689 Dual‑DAC GATT Service  */
#pragma once

#include <zephyr/bluetooth/uuid.h>
#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ────────── 128‑bit UUID 定义 ────────── */
#define BT_UUID_DAC_CHANNEL1_VAL \
    BT_UUID_128_ENCODE(0xA0A4A681, 0x96BE, 0x4222, 0xB41E, 0x98EA76B0120C)
#define BT_UUID_DAC_CHANNEL2_VAL \
    BT_UUID_128_ENCODE(0xA0A4A682, 0x96BE, 0x4222, 0xB41E, 0x98EA76B0120C)

#define BT_UUID_DAC_CHANNEL1  BT_UUID_DECLARE_128(BT_UUID_DAC_CHANNEL1_VAL)
#define BT_UUID_DAC_CHANNEL2  BT_UUID_DECLARE_128(BT_UUID_DAC_CHANNEL2_VAL)

/* ────────── Public API ────────── */
int ble_ad5689_srv_init(void);               /* 初始化一次 */

int ble_ad5689_notify_ch1(const uint8_t be16[2]); /* 主动推送 */
int ble_ad5689_notify_ch2(const uint8_t be16[2]);

#ifdef __cplusplus
}
#endif
