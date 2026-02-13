/* ========================= ble_cachexia_cs_srv.h =========================
 * BLE service for cachexia stimulation (current source)
 *
 * Characteristics:
 *  - Switch (u8): 0/1 enable stimulation
 *  - AmplitudePercent (u8): 0..100 (%) (backward compatible UI control)
 *  - AmplitudeCode (u16 LE): 0..65535 (finest DAC control)  <-- NEW
 *  - Frequency (u16 LE, Hz): default 5 Hz, adjustable over BLE
 *  - Battery (float32 LE, placeholder)
 *  - IMU (16 bytes, READ+NOTIFY): timestamp + accel + gyro (see format below)
 */

#pragma once

#include <zephyr/bluetooth/uuid.h>
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Keep the same 128-bit base UUID as your original Cachexia service */
#define BT_UUID_CACHEXIA_SVC_VAL \
	BT_UUID_128_ENCODE(0xA0A4C680, 0x96BE, 0x4222, 0xB41E, 0x98EA76B0120C)
#define BT_UUID_CACHEXIA_SWITCH_VAL \
	BT_UUID_128_ENCODE(0xA0A4C681, 0x96BE, 0x4222, 0xB41E, 0x98EA76B0120C)

/* Existing: Amplitude percent (u8 0..100) */
#define BT_UUID_CACHEXIA_AMPLITUDE_VAL \
	BT_UUID_128_ENCODE(0xA0A4C682, 0x96BE, 0x4222, 0xB41E, 0x98EA76B0120C)

#define BT_UUID_CACHEXIA_BATTERY_VAL \
	BT_UUID_128_ENCODE(0xA0A4C683, 0x96BE, 0x4222, 0xB41E, 0x98EA76B0120C)

/* Frequency characteristic */
#define BT_UUID_CACHEXIA_FREQUENCY_VAL \
	BT_UUID_128_ENCODE(0xA0A4C684, 0x96BE, 0x4222, 0xB41E, 0x98EA76B0120C)

/* IMU streaming characteristic (READ + NOTIFY) */
#define BT_UUID_CACHEXIA_IMU_VAL \
	BT_UUID_128_ENCODE(0xA0A4C685, 0x96BE, 0x4222, 0xB41E, 0x98EA76B0120C)

/* NEW: Amplitude code (u16 LE 0..65535). Next UUID in your sequence. */
#define BT_UUID_CACHEXIA_AMPLITUDE_CODE_VAL \
	BT_UUID_128_ENCODE(0xA0A4C686, 0x96BE, 0x4222, 0xB41E, 0x98EA76B0120C)

#define BT_UUID_CACHEXIA_SVC            BT_UUID_DECLARE_128(BT_UUID_CACHEXIA_SVC_VAL)
#define BT_UUID_CACHEXIA_SWITCH         BT_UUID_DECLARE_128(BT_UUID_CACHEXIA_SWITCH_VAL)
#define BT_UUID_CACHEXIA_AMPLITUDE      BT_UUID_DECLARE_128(BT_UUID_CACHEXIA_AMPLITUDE_VAL)
#define BT_UUID_CACHEXIA_AMPLITUDE_CODE BT_UUID_DECLARE_128(BT_UUID_CACHEXIA_AMPLITUDE_CODE_VAL)
#define BT_UUID_CACHEXIA_FREQUENCY      BT_UUID_DECLARE_128(BT_UUID_CACHEXIA_FREQUENCY_VAL)
#define BT_UUID_CACHEXIA_BATTERY        BT_UUID_DECLARE_128(BT_UUID_CACHEXIA_BATTERY_VAL)
#define BT_UUID_CACHEXIA_IMU            BT_UUID_DECLARE_128(BT_UUID_CACHEXIA_IMU_VAL)

/* IMU payload format (little-endian), total 16 bytes:
 * [0..3]  uint32  t_ms (k_uptime_get_32)
 * [4..5]  int16   ax_mg
 * [6..7]  int16   ay_mg
 * [8..9]  int16   az_mg
 * [10..11] int16  gx_dps_x10   (deg/s * 10)
 * [12..13] int16  gy_dps_x10
 * [14..15] int16  gz_dps_x10
 */
#define CACHEXIA_IMU_PAYLOAD_LEN 16

/* Optional safety clamp for amplitude code.
 * Set this to a smaller value to cap max current.
 */
#ifndef CACHEXIA_MAX_AMP_CODE
#define CACHEXIA_MAX_AMP_CODE 0xFFFFu
#endif

/* Public API */
int ble_cachexia_cs_srv_init(void);

/* State getters (atomic-backed) for the application thread */
uint8_t  ble_cachexia_cs_get_switch(void);

/* Back-compat getter (0..100) */
uint8_t  ble_cachexia_cs_get_amplitude_percent(void);

/* NEW: Finest control getter (0..65535) */
uint16_t ble_cachexia_cs_get_amplitude_code(void);

uint16_t ble_cachexia_cs_get_frequency_hz(void);

/* IMU streaming helpers */
bool ble_cachexia_cs_imu_notify_is_enabled(void);

/* Update cached IMU payload; if notifications are enabled, also notify.
 * payload must be exactly CACHEXIA_IMU_PAYLOAD_LEN bytes.
 */
int ble_cachexia_cs_imu_stream(const uint8_t *payload, uint16_t len);

#ifdef __cplusplus
}
#endif