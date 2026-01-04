/* ble_mlx_srv.h — BLE service for streaming MLX90393 samples via Notify */
#pragma once

#include <zephyr/bluetooth/uuid.h>
#include <zephyr/kernel.h>
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* 128-bit UUIDs for MLX service and its characteristics */
#define BT_UUID_MLX_SVC_VAL \
    BT_UUID_128_ENCODE(0xA0A4E680, 0x96BE, 0x4222, 0xB41E, 0x98EA76B0120C)
#define BT_UUID_MLX_STREAM_VAL \
    BT_UUID_128_ENCODE(0xA0A4E681, 0x96BE, 0x4222, 0xB41E, 0x98EA76B0120C)
#define BT_UUID_MLX_CTRL_VAL \
    BT_UUID_128_ENCODE(0xA0A4E682, 0x96BE, 0x4222, 0xB41E, 0x98EA76B0120C)

#define BT_UUID_MLX_SVC    BT_UUID_DECLARE_128(BT_UUID_MLX_SVC_VAL)
#define BT_UUID_MLX_STREAM BT_UUID_DECLARE_128(BT_UUID_MLX_STREAM_VAL)
#define BT_UUID_MLX_CTRL   BT_UUID_DECLARE_128(BT_UUID_MLX_CTRL_VAL)

/* Packed sample on-air (little-endian) */
struct mlx_ble_sample {
    uint32_t t_ms;   /* uptime ms */
    int16_t  x;
    int16_t  y;
    int16_t  z;
    int16_t  t;      /* temperature raw code */
} __packed;

/* Initialize GATT service */
int ble_mlx_srv_init(void);

/* Notify one sample (no-op if not running or not connected) */
int ble_mlx_notify_sample(const struct mlx_ble_sample *s);

/* Blocks until a start command is received (CTRL written non-zero),
 * or timeout expires.
 *   0          : start received
 *  -ETIMEDOUT  : timeout
 *  other < 0   : internal error
 */
int ble_mlx_wait_for_start(k_timeout_t timeout);

/* Current running flag (CTRL != 0) */
bool ble_mlx_is_running(void);

#ifdef __cplusplus
}
#endif
