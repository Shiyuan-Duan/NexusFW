#pragma once

#include <zephyr/bluetooth/uuid.h>
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ---- Limits ---- */
#define CACHEXIA_FG_MAX_BLOCKS   64
#define CACHEXIA_FG_MAX_ABS_UA   25000  /* +/-25mA in microamps */

/* Wire format for APPEND characteristic: 7 bytes
 * [0..1]  int16  amp_ua (little-endian)
 * [2..5]  uint32 dur_ms (little-endian)
 * [6]     uint8  flags (reserved, set 0)
 */
#define CACHEXIA_FG_BLOCK_WIRE_LEN 7

/* Service UUIDs (new function generator service) */
#define BT_UUID_CACHEXIA_FG_SVC_VAL \
	BT_UUID_128_ENCODE(0xA0A4C690, 0x96BE, 0x4222, 0xB41E, 0x98EA76B0120C)
#define BT_UUID_CACHEXIA_FG_SWITCH_VAL \
	BT_UUID_128_ENCODE(0xA0A4C691, 0x96BE, 0x4222, 0xB41E, 0x98EA76B0120C)
#define BT_UUID_CACHEXIA_FG_CLEAR_VAL \
	BT_UUID_128_ENCODE(0xA0A4C692, 0x96BE, 0x4222, 0xB41E, 0x98EA76B0120C)
#define BT_UUID_CACHEXIA_FG_APPEND_VAL \
	BT_UUID_128_ENCODE(0xA0A4C693, 0x96BE, 0x4222, 0xB41E, 0x98EA76B0120C)
#define BT_UUID_CACHEXIA_FG_COMMIT_VAL \
	BT_UUID_128_ENCODE(0xA0A4C694, 0x96BE, 0x4222, 0xB41E, 0x98EA76B0120C)
#define BT_UUID_CACHEXIA_FG_INFO_VAL \
	BT_UUID_128_ENCODE(0xA0A4C695, 0x96BE, 0x4222, 0xB41E, 0x98EA76B0120C)

#define BT_UUID_CACHEXIA_FG_SVC    BT_UUID_DECLARE_128(BT_UUID_CACHEXIA_FG_SVC_VAL)
#define BT_UUID_CACHEXIA_FG_SWITCH BT_UUID_DECLARE_128(BT_UUID_CACHEXIA_FG_SWITCH_VAL)
#define BT_UUID_CACHEXIA_FG_CLEAR  BT_UUID_DECLARE_128(BT_UUID_CACHEXIA_FG_CLEAR_VAL)
#define BT_UUID_CACHEXIA_FG_APPEND BT_UUID_DECLARE_128(BT_UUID_CACHEXIA_FG_APPEND_VAL)
#define BT_UUID_CACHEXIA_FG_COMMIT BT_UUID_DECLARE_128(BT_UUID_CACHEXIA_FG_COMMIT_VAL)
#define BT_UUID_CACHEXIA_FG_INFO   BT_UUID_DECLARE_128(BT_UUID_CACHEXIA_FG_INFO_VAL)

/* Internal representation */
struct cachexia_fg_block {
	int16_t  amp_ua;   /* signed microamps: [-25000..25000] */
	uint32_t dur_ms;   /* duration in ms, 0xFFFFFFFF = hold forever */
	uint8_t  flags;    /* reserved */
};

struct cachexia_fg_pattern {
	uint8_t count;
	struct cachexia_fg_block blocks[CACHEXIA_FG_MAX_BLOCKS];
};

/* Public API */
int      ble_cachexia_fg_srv_init(void);

uint8_t  ble_cachexia_fg_get_switch(void);

/* Pattern generation increments on COMMIT */
uint32_t ble_cachexia_fg_get_pattern_gen(void);

/* Copy active pattern snapshot (thread-safe) */
int      ble_cachexia_fg_copy_active_pattern(struct cachexia_fg_pattern *out);

#ifdef __cplusplus
}
#endif
