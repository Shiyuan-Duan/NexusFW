/*  ble_cachexia_cs_srv.h — BLE service for cachexia stimulation (current source)
 *
 *  Characteristics:
 *   - Switch (u8): 0/1 enable stimulation
 *   - Amplitude (u8): 0..100 (%) -> mapped to 0..0xFFFF code in app thread
 *   - Frequency (u16, Hz): default 5 Hz, adjustable over BLE
 *   - Battery (float32, placeholder)
 */

#pragma once

#include <zephyr/bluetooth/uuid.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Keep the same 128-bit base UUID as your original Cachexia service */
#define BT_UUID_CACHEXIA_SVC_VAL \
	BT_UUID_128_ENCODE(0xA0A4C680, 0x96BE, 0x4222, 0xB41E, 0x98EA76B0120C)
#define BT_UUID_CACHEXIA_SWITCH_VAL \
	BT_UUID_128_ENCODE(0xA0A4C681, 0x96BE, 0x4222, 0xB41E, 0x98EA76B0120C)
#define BT_UUID_CACHEXIA_AMPLITUDE_VAL \
	BT_UUID_128_ENCODE(0xA0A4C682, 0x96BE, 0x4222, 0xB41E, 0x98EA76B0120C)
#define BT_UUID_CACHEXIA_BATTERY_VAL \
	BT_UUID_128_ENCODE(0xA0A4C683, 0x96BE, 0x4222, 0xB41E, 0x98EA76B0120C)

/* New: Frequency characteristic */
#define BT_UUID_CACHEXIA_FREQUENCY_VAL \
	BT_UUID_128_ENCODE(0xA0A4C684, 0x96BE, 0x4222, 0xB41E, 0x98EA76B0120C)

#define BT_UUID_CACHEXIA_SVC       BT_UUID_DECLARE_128(BT_UUID_CACHEXIA_SVC_VAL)
#define BT_UUID_CACHEXIA_SWITCH    BT_UUID_DECLARE_128(BT_UUID_CACHEXIA_SWITCH_VAL)
#define BT_UUID_CACHEXIA_AMPLITUDE BT_UUID_DECLARE_128(BT_UUID_CACHEXIA_AMPLITUDE_VAL)
#define BT_UUID_CACHEXIA_FREQUENCY BT_UUID_DECLARE_128(BT_UUID_CACHEXIA_FREQUENCY_VAL)
#define BT_UUID_CACHEXIA_BATTERY   BT_UUID_DECLARE_128(BT_UUID_CACHEXIA_BATTERY_VAL)

/* Public API */
int ble_cachexia_cs_srv_init(void);

/* State getters (atomic-backed) for the application thread */
uint8_t  ble_cachexia_cs_get_switch(void);
uint8_t  ble_cachexia_cs_get_amplitude_percent(void);
uint16_t ble_cachexia_cs_get_frequency_hz(void);

#ifdef __cplusplus
}
#endif
