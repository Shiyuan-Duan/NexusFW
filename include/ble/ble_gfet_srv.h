/*  ble_gfet_srv.h — BLE service for streaming GFET samples via Notify */
#pragma once

#include <zephyr/bluetooth/uuid.h>
#include <stdint.h>
#include <stddef.h>
#include <zephyr/kernel.h>

#ifdef __cplusplus
extern "C" {
#endif

/* 128-bit UUIDs for GFET service and its streaming characteristic */
#define BT_UUID_GFET_SVC_VAL \
    BT_UUID_128_ENCODE(0xA0A4F680, 0x96BE, 0x4222, 0xB41E, 0x98EA76B0120C)
#define BT_UUID_GFET_STREAM_VAL \
    BT_UUID_128_ENCODE(0xA0A4F681, 0x96BE, 0x4222, 0xB41E, 0x98EA76B0120C)
/* Control characteristic to trigger measurement start */
#define BT_UUID_GFET_CTRL_VAL \
    BT_UUID_128_ENCODE(0xA0A4F682, 0x96BE, 0x4222, 0xB41E, 0x98EA76B0120C)
/* Configuration characteristic to set sweep params */
#define BT_UUID_GFET_CFG_VAL \
    BT_UUID_128_ENCODE(0xA0A4F683, 0x96BE, 0x4222, 0xB41E, 0x98EA76B0120C)

#define BT_UUID_GFET_SVC     BT_UUID_DECLARE_128(BT_UUID_GFET_SVC_VAL)
#define BT_UUID_GFET_STREAM  BT_UUID_DECLARE_128(BT_UUID_GFET_STREAM_VAL)
#define BT_UUID_GFET_CTRL    BT_UUID_DECLARE_128(BT_UUID_GFET_CTRL_VAL)
#define BT_UUID_GFET_CFG     BT_UUID_DECLARE_128(BT_UUID_GFET_CFG_VAL)

/* Packed sample structure (little-endian on-air) */
struct gfet_sample {
    uint32_t t_ms;     /* uptime ms */
    int32_t  vg_mV;    /* gate bias in mV */
    int32_t  id_nA;    /* measured drain current in nA */
} __packed;

/* Sweep configuration (little-endian on-air) */
struct gfet_cfg {
    uint16_t v_start_mV;   /* default 200 */
    uint16_t v_stop_mV;    /* default 800 */
    uint16_t v_step_mV;    /* default 20 */
    uint16_t dwell_ms;     /* default 50 */
    uint8_t  avg_N;        /* default 4 */
    uint8_t  _rsv[3];      /* align to 10 bytes total */
} __packed;

int ble_gfet_srv_init(void);
int ble_gfet_notify_sample(const struct gfet_sample *s);
/* Blocks until a start command is received (1-byte write to CTRL), or timeout. */
int ble_gfet_wait_for_start(k_timeout_t timeout);
/* Returns current configuration (copy). */
void ble_gfet_get_cfg(struct gfet_cfg *out);
/* Returns whether app is currently marked running (CTRL!=0). */
bool ble_gfet_is_running(void);

#ifdef __cplusplus
}
#endif
