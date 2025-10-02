/*  ble_drug_delivery_srv.h — BLE service for 5 GPIO on/off channels */
#pragma once

#include <zephyr/bluetooth/uuid.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* 128-bit UUIDs for Drug Delivery service and its 5 characteristics */
#define BT_UUID_DRUG_SVC_VAL \
    BT_UUID_128_ENCODE(0xA0A4D680, 0x96BE, 0x4222, 0xB41E, 0x98EA76B0120C)
#define BT_UUID_DRUG_CH1_VAL \
    BT_UUID_128_ENCODE(0xA0A4D681, 0x96BE, 0x4222, 0xB41E, 0x98EA76B0120C)
#define BT_UUID_DRUG_CH2_VAL \
    BT_UUID_128_ENCODE(0xA0A4D682, 0x96BE, 0x4222, 0xB41E, 0x98EA76B0120C)
#define BT_UUID_DRUG_CH3_VAL \
    BT_UUID_128_ENCODE(0xA0A4D683, 0x96BE, 0x4222, 0xB41E, 0x98EA76B0120C)
#define BT_UUID_DRUG_CH4_VAL \
    BT_UUID_128_ENCODE(0xA0A4D684, 0x96BE, 0x4222, 0xB41E, 0x98EA76B0120C)
#define BT_UUID_DRUG_CH5_VAL \
    BT_UUID_128_ENCODE(0xA0A4D685, 0x96BE, 0x4222, 0xB41E, 0x98EA76B0120C)

#define BT_UUID_DRUG_SVC  BT_UUID_DECLARE_128(BT_UUID_DRUG_SVC_VAL)
#define BT_UUID_DRUG_CH1  BT_UUID_DECLARE_128(BT_UUID_DRUG_CH1_VAL)
#define BT_UUID_DRUG_CH2  BT_UUID_DECLARE_128(BT_UUID_DRUG_CH2_VAL)
#define BT_UUID_DRUG_CH3  BT_UUID_DECLARE_128(BT_UUID_DRUG_CH3_VAL)
#define BT_UUID_DRUG_CH4  BT_UUID_DECLARE_128(BT_UUID_DRUG_CH4_VAL)
#define BT_UUID_DRUG_CH5  BT_UUID_DECLARE_128(BT_UUID_DRUG_CH5_VAL)

/* Public init to configure GPIO and reset state */
int ble_drug_srv_init(void);

#ifdef __cplusplus
}
#endif

