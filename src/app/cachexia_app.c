/* Cachexia application thread: drives a 50 Hz square wave on AD5689 VoutA
 * based on BLE service state. Do not modify main.c; this thread is started
 * here with K_THREAD_DEFINE.
 */
//cachexia_app.c

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/logging/log.h>
#include <string.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/uuid.h>


#include "ble_adv_core.h"

#include "ad5689.h"
#include "ble_cachexia_srv.h"

LOG_MODULE_REGISTER(cachexia_app, CONFIG_LOG_DEFAULT_LEVEL);

/* Half-period for 50 Hz square wave: 10 ms */
#define HALF_PERIOD_MS 100

static const struct device *const ad5689_dev = DEVICE_DT_GET_ONE(dsy_ad5689);

static uint16_t map_pct_to_code(uint8_t pct)
{
    if (pct >= 100) return 0xFFFF;
    /* scale 0..100 to 0..65535 */
    uint32_t code = ((uint32_t)pct * 65535u) / 100u;
    return (uint16_t)code;
}

static void cachexia_thread(void *a, void *b, void *c)
{
    ARG_UNUSED(a); ARG_UNUSED(b); ARG_UNUSED(c);

    if (!device_is_ready(ad5689_dev)) {
        LOG_ERR("AD5689 device not ready");
        return;
    }

    /* Initialize BLE service */
    if (ble_cachexia_srv_init() != 0) {
        LOG_ERR("Failed to init cachexia BLE service");
        return;
    }

    bool high = false;
    bool zeroed = false;

    for (;;) {
        if (ble_cachexia_get_switch()) {
            zeroed = false;
            uint8_t pct = ble_cachexia_get_amplitude_percent();
            uint16_t amp = map_pct_to_code(pct);

            uint16_t code = high ? amp : 0;
            (void)ad5689_write(ad5689_dev, 0 /* channel A */, code);
            high = !high;

            k_sleep(K_MSEC(HALF_PERIOD_MS));
        } else {
            if (!zeroed) {
                (void)ad5689_write(ad5689_dev, 0 /* A */, 0);
                zeroed = true;
            }
            /* sleep longer when idle to save power */
            k_sleep(K_MSEC(50));
        }
    }
}

K_THREAD_DEFINE(cachexia_t, 1024, cachexia_thread, NULL, NULL, NULL,
                2 /*prio*/, 0, 0);

/* ---- BLE advertising profile/thread (moved from cachexia_adv.c) ---- */
#define DEVICE_NAME CONFIG_BT_DEVICE_NAME
#define DEVICE_NAME_LEN (sizeof(DEVICE_NAME) - 1)

/* Advertising parameters: connectable, identity address, slow interval */
static const struct bt_le_adv_param adv_param = BT_LE_ADV_PARAM_INIT(
    (BT_LE_ADV_OPT_CONNECTABLE | BT_LE_ADV_OPT_USE_IDENTITY),
    BT_GAP_ADV_SLOW_INT_MIN,
    BT_GAP_ADV_SLOW_INT_MAX,
    NULL);

/* Advertising data: flags + complete name */
static const struct bt_data ad[] = {
    BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
    BT_DATA(BT_DATA_NAME_COMPLETE, DEVICE_NAME, DEVICE_NAME_LEN),
};

/* Scan response: advertise primary Cachexia service UUID */
static const struct bt_data sd[] = {
    BT_DATA_BYTES(BT_DATA_UUID128_ALL, BT_UUID_CACHEXIA_SVC_VAL),
};

/* Prefer shorter timeout to drop ghost connections faster */
static const struct bt_le_conn_param conn_param_val = {
    .interval_min = 24,
    .interval_max = 40,
    .latency = 0,
    .timeout = 200,
};

static const struct ble_adv_profile cachexia_profile = {
    .params = &adv_param,
    .ad = ad,
    .ad_len = ARRAY_SIZE(ad),
    .sd = sd,
    .sd_len = ARRAY_SIZE(sd),
    .conn_param = &conn_param_val,
    .keepalive_sec = 5,
};

/* BLE advertising/thread for Cachexia app */
K_THREAD_DEFINE(ble_t, 1024, ble_adv_thread, (void *)&cachexia_profile, NULL, NULL,
                1 /*prio*/, 0, 0);
