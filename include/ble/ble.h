/* ble.h */
#ifndef BLE_H_
#define BLE_H_

#ifdef __cplusplus
extern "C" {
#endif

#define DEVICE_NAME CONFIG_BT_DEVICE_NAME
#define DEVICE_NAME_LEN (sizeof(DEVICE_NAME) - 1)

#define BT_UUID_VAL \
	BT_UUID_128_ENCODE(0xA0A43180, 0x96BE, 0x4222, 0xB41E, 0x98EA76B0120C)
    
#define BT_UUID           						BT_UUID_DECLARE_128(BT_UUID_VAL)





int t_ble_adv_start(void);   





#ifdef __cplusplus
}
#endif

#endif /* BLE_H_ */
