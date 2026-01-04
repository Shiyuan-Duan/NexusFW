/*  ble_cachexia_srv.h — BLE service for cachexia stimulation control */
//ble_cachexia_srv.h
#pragma once

#include <zephyr/bluetooth/uuid.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* 128-bit UUIDs for the Cachexia service and its characteristics */
#define BT_UUID_CACHEXIA_SVC_VAL \
    BT_UUID_128_ENCODE(0xA0A4C680, 0x96BE, 0x4222, 0xB41E, 0x98EA76B0120C)
#define BT_UUID_CACHEXIA_SWITCH_VAL \
    BT_UUID_128_ENCODE(0xA0A4C681, 0x96BE, 0x4222, 0xB41E, 0x98EA76B0120C)
#define BT_UUID_CACHEXIA_AMPLITUDE_VAL \
    BT_UUID_128_ENCODE(0xA0A4C682, 0x96BE, 0x4222, 0xB41E, 0x98EA76B0120C)
#define BT_UUID_CACHEXIA_BATTERY_VAL \
    BT_UUID_128_ENCODE(0xA0A4C683, 0x96BE, 0x4222, 0xB41E, 0x98EA76B0120C)

#define BT_UUID_CACHEXIA_SVC       BT_UUID_DECLARE_128(BT_UUID_CACHEXIA_SVC_VAL)
#define BT_UUID_CACHEXIA_SWITCH    BT_UUID_DECLARE_128(BT_UUID_CACHEXIA_SWITCH_VAL)
#define BT_UUID_CACHEXIA_AMPLITUDE BT_UUID_DECLARE_128(BT_UUID_CACHEXIA_AMPLITUDE_VAL)
#define BT_UUID_CACHEXIA_BATTERY   BT_UUID_DECLARE_128(BT_UUID_CACHEXIA_BATTERY_VAL)

/* Public API */
int ble_cachexia_srv_init(void);

/* State getters for the application thread */
uint8_t ble_cachexia_get_switch(void);
uint8_t ble_cachexia_get_amplitude_percent(void);

#ifdef __cplusplus
}
#endif

