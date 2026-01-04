#pragma once

#include <zephyr/bluetooth/uuid.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define BT_UUID_MAX30001_SVC_VAL \
	BT_UUID_128_ENCODE(0xA0A4D680, 0x96BE, 0x4222, 0xB41E, 0x98EA76B0120C)

#define BT_UUID_MAX30001_SWITCH_VAL \
	BT_UUID_128_ENCODE(0xA0A4D681, 0x96BE, 0x4222, 0xB41E, 0x98EA76B0120C)

#define BT_UUID_MAX30001_BIOV_VAL \
	BT_UUID_128_ENCODE(0xA0A4D682, 0x96BE, 0x4222, 0xB41E, 0x98EA76B0120C)

#define BT_UUID_MAX30001_BIOZ_VAL \
	BT_UUID_128_ENCODE(0xA0A4D683, 0x96BE, 0x4222, 0xB41E, 0x98EA76B0120C)

#define BT_UUID_MAX30001_SVC    BT_UUID_DECLARE_128(BT_UUID_MAX30001_SVC_VAL)
#define BT_UUID_MAX30001_SWITCH BT_UUID_DECLARE_128(BT_UUID_MAX30001_SWITCH_VAL)
#define BT_UUID_MAX30001_BIOV   BT_UUID_DECLARE_128(BT_UUID_MAX30001_BIOV_VAL)
#define BT_UUID_MAX30001_BIOZ   BT_UUID_DECLARE_128(BT_UUID_MAX30001_BIOZ_VAL)

int ble_max30001_stream_srv_init(void);

uint8_t ble_max30001_get_switch(void);

/* app thread push */
int ble_max30001_notify_biov(const uint8_t *data, uint16_t len);
int ble_max30001_notify_bioz(const uint8_t *data, uint16_t len);

#ifdef __cplusplus
}
#endif