/* Drug Delivery application: exposes 5 on/off BLE characteristics
 * and directly toggles GPIO pins P0.28, P0.29, P0.30, P0.25, P0.31.
 * BLE advertising/thread uses the shared ble_adv_core.
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/uuid.h>

#include "ble_adv_core.h"
#include "ble_drug_delivery_srv.h"

LOG_MODULE_REGISTER(drug_app, CONFIG_LOG_DEFAULT_LEVEL);

/* Advertising profile for Drug Delivery app */
#define DEVICE_NAME CONFIG_BT_DEVICE_NAME
#define DEVICE_NAME_LEN (sizeof(DEVICE_NAME) - 1)

static const struct bt_le_adv_param adv_param = BT_LE_ADV_PARAM_INIT(
    (BT_LE_ADV_OPT_CONNECTABLE | BT_LE_ADV_OPT_USE_IDENTITY),
    BT_GAP_ADV_SLOW_INT_MIN,
    BT_GAP_ADV_SLOW_INT_MAX,
    NULL);

static const struct bt_data ad[] = {
    BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
    BT_DATA(BT_DATA_NAME_COMPLETE, DEVICE_NAME, DEVICE_NAME_LEN),
};

/* Advertise primary Drug Delivery service UUID in scan response */
static const struct bt_data sd[] = {
    BT_DATA_BYTES(BT_DATA_UUID128_ALL, BT_UUID_DRUG_SVC_VAL),
};

static const struct bt_le_conn_param conn_param_val = {
    .interval_min = 24,
    .interval_max = 40,
    .latency = 0,
    .timeout = 200,
};

static const struct ble_adv_profile drug_profile = {
    .params = &adv_param,
    .ad = ad,
    .ad_len = ARRAY_SIZE(ad),
    .sd = sd,
    .sd_len = ARRAY_SIZE(sd),
    .conn_param = &conn_param_val,
    .keepalive_sec = 5,
};

/* Minimal app thread: initialize the BLE service and idle. */
static void drug_app_thread(void *a, void *b, void *c)
{
    ARG_UNUSED(a); ARG_UNUSED(b); ARG_UNUSED(c);
    if (ble_drug_srv_init() != 0) {
        LOG_ERR("Drug service init failed");
        return;
    }
    for (;;) {
        k_sleep(K_SECONDS(60));
    }
}

/* Start both: the BLE advertiser and the app thread */
K_THREAD_DEFINE(ble_t, 1024, ble_adv_thread, (void *)&drug_profile, NULL, NULL,
                1, 0, 0);
K_THREAD_DEFINE(drug_t, 768, drug_app_thread, NULL, NULL, NULL,
                2, 0, 0);

