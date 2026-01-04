/* MLX90393 measurement application
 * - Uses MLX90393 driver to sample X/Y/Z/T
 * - Streams samples over BLE via MLX service notifications
 * - mlx_app.c
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/logging/log.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/uuid.h>

#include "ble_adv_core.h"
#include "ble_mlx_srv.h"
#include "mlx90393.h"

LOG_MODULE_REGISTER(mlx_app, CONFIG_LOG_DEFAULT_LEVEL);

/* ======================= BLE advertising profile ======================= */

#define DEVICE_NAME     CONFIG_BT_DEVICE_NAME
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

/* Advertise MLX primary service UUID in scan response */
static const struct bt_data sd[] = {
    BT_DATA_BYTES(BT_DATA_UUID128_ALL, BT_UUID_MLX_SVC_VAL),
};

static const struct bt_le_conn_param conn_param_val = {
    .interval_min = 24,
    .interval_max = 40,
    .latency      = 0,
    .timeout      = 200,
};

static const struct ble_adv_profile mlx_profile = {
    .params        = &adv_param,
    .ad            = ad,
    .ad_len        = ARRAY_SIZE(ad),
    .sd            = sd,
    .sd_len        = ARRAY_SIZE(sd),
    .conn_param    = &conn_param_val,
    .keepalive_sec = 5,
};

/* ======================= Device handle ================================= */

static const struct device *const mlx_dev = DEVICE_DT_GET_ONE(dsy_mlx90393);

/* ======================= Measurement thread ============================ */

#define MLX_SAMPLE_PERIOD_MS  20  /* ~50 Hz */
#define MLX_CONV_DELAY_MS      5  /* conversion time after SM */

static void mlx_thread(void *a, void *b, void *c)
{
    ARG_UNUSED(a); ARG_UNUSED(b); ARG_UNUSED(c);

    /* Init BLE service */
    (void)ble_mlx_srv_init();

    if (!device_is_ready(mlx_dev)) {
        LOG_ERR("MLX90393 device not ready");
        return;
    }

    printk("MLX app thread started");

    const uint8_t zyxt =
        MLX90393_ZYXT_T |
        MLX90393_ZYXT_X |
        MLX90393_ZYXT_Y |
        MLX90393_ZYXT_Z;

    for (;;) {
        /* Wait for phone to write CTRL != 0 */
        int ret = ble_mlx_wait_for_start(K_FOREVER);
        if (ret < 0) {
            continue;
        }

        LOG_INF("MLX streaming started");

        while (ble_mlx_is_running()) {
            uint8_t status = 0;
            int16_t x = 0, y = 0, z = 0, t = 0;

            /* Start single conversion */
            ret = mlx90393_sm(mlx_dev, zyxt, &status);
            if (ret < 0) {
                LOG_WRN("SM failed: %d", ret);
                k_sleep(K_MSEC(MLX_SAMPLE_PERIOD_MS));
                continue;
            }

            k_sleep(K_MSEC(MLX_CONV_DELAY_MS));

            ret = mlx90393_rm(mlx_dev, zyxt, &x, &y, &z, &t, &status);
            if (ret < 0) {
                LOG_WRN("RM failed: %d", ret);
                k_sleep(K_MSEC(MLX_SAMPLE_PERIOD_MS));
                continue;
            }

            struct mlx_ble_sample s = {
                .t_ms = (uint32_t)k_uptime_get_32(),
                .x    = x,
                .y    = y,
                .z    = z,
                .t    = t,
            };
            //add a printk to log xyz t
            (void)ble_mlx_notify_sample(&s);

            /* 控制整体采样频率 */
            k_sleep(K_MSEC(MLX_SAMPLE_PERIOD_MS - MLX_CONV_DELAY_MS));
        }

        LOG_INF("MLX streaming stopped");
        /* Loop back to wait for next start */
    }
}

/* ======================= BLE advertising thread ======================= */
/* ble_adv_thread(const struct ble_adv_profile *profile, ...) 由 ble_adv_core 提供 */
K_THREAD_DEFINE(mlx_ble_t, 1024, ble_adv_thread,
                (void *)&mlx_profile, NULL, NULL,
                1 /* prio */, 0, 0);

/* MLX measurement thread */
K_THREAD_DEFINE(mlx_t, 1024, mlx_thread, NULL, NULL, NULL,
                2 /* prio */, 0, 0);
