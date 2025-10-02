/* ble_adv_core.h — Shared BLE advertising core */
#pragma once

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <stddef.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

struct ble_adv_profile {
    const struct bt_le_adv_param *params;   /* advertising params */
    const struct bt_data *ad;               /* advertising data */
    size_t ad_len;
    const struct bt_data *sd;               /* scan response */
    size_t sd_len;
    const struct bt_le_conn_param *conn_param; /* optional, can be NULL */
    uint32_t keepalive_sec;                 /* periodic reassert interval */
};

/* Thread entry: pass a pointer to a static ble_adv_profile as p1 */
void ble_adv_thread(void *p1, void *p2, void *p3);

/* Health indicator: true when BLE is enabled and advertising/connected. */
bool ble_is_healthy(void);

#ifdef __cplusplus
}
#endif

