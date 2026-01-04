/* ble_mlx97_srv.h — BLE service for streaming 2x MLX90397 samples via Notify */
#pragma once

#include <zephyr/bluetooth/uuid.h>
#include <zephyr/kernel.h>
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* 128-bit UUIDs: one service, two stream characteristics, one ctrl */
#define BT_UUID_MLX97_SVC_VAL \
    BT_UUID_128_ENCODE(0xA0A4E690, 0x96BE, 0x4222, 0xB41E, 0x98EA76B0120C)

/* stream0 / stream1 / ctrl */
#define BT_UUID_MLX97_STREAM0_VAL \
    BT_UUID_128_ENCODE(0xA0A4E691, 0x96BE, 0x4222, 0xB41E, 0x98EA76B0120C)
#define BT_UUID_MLX97_STREAM1_VAL \
    BT_UUID_128_ENCODE(0xA0A4E692, 0x96BE, 0x4222, 0xB41E, 0x98EA76B0120C)
#define BT_UUID_MLX97_CTRL_VAL \
    BT_UUID_128_ENCODE(0xA0A4E693, 0x96BE, 0x4222, 0xB41E, 0x98EA76B0120C)

#define BT_UUID_MLX97_SVC     BT_UUID_DECLARE_128(BT_UUID_MLX97_SVC_VAL)
#define BT_UUID_MLX97_STREAM0 BT_UUID_DECLARE_128(BT_UUID_MLX97_STREAM0_VAL)
#define BT_UUID_MLX97_STREAM1 BT_UUID_DECLARE_128(BT_UUID_MLX97_STREAM1_VAL)
#define BT_UUID_MLX97_CTRL    BT_UUID_DECLARE_128(BT_UUID_MLX97_CTRL_VAL)

/* Packed sample on-air (little-endian) */
struct mlx97_ble_sample {
    uint32_t t_ms;   /* uptime ms */
    int16_t  x;
    int16_t  y;
    int16_t  z;
    uint8_t  stat1;
    uint8_t  stat2;
} __packed;

int  ble_mlx97_srv_init(void);
bool ble_mlx97_is_running(void);
int  ble_mlx97_wait_for_start(k_timeout_t timeout);

/* Notify for sensor 0/1 (no-op if not running) */
int ble_mlx97_notify0(const struct mlx97_ble_sample *s);
int ble_mlx97_notify1(const struct mlx97_ble_sample *s);

#ifdef __cplusplus
}
#endif