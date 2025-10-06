/* Graphene FET measurement application
 * - Uses AD5689 DAC to sweep/hold gate bias
 * - Uses AD5941 AFE to measure drain current
 * - Streams samples over BLE via GFET service notifications
 * - gfet_app.c (bidirectional sweep version)
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/logging/log.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/uuid.h>

#include "ble_adv_core.h"
#include "ble_gfet_srv.h"
#include "ad5941.h"

LOG_MODULE_REGISTER(gfet_app, CONFIG_LOG_DEFAULT_LEVEL);

/* Advertising profile for GFET app */
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

/* Advertise GFET primary service UUID in scan response */
static const struct bt_data sd[] = {
    BT_DATA_BYTES(BT_DATA_UUID128_ALL, BT_UUID_GFET_SVC_VAL),
};

static const struct bt_le_conn_param conn_param_val = {
    .interval_min = 24,
    .interval_max = 40,
    .latency = 0,
    .timeout = 200,
};

static const struct ble_adv_profile gfet_profile = {
    .params = &adv_param,
    .ad = ad,
    .ad_len = ARRAY_SIZE(ad),
    .sd = sd,
    .sd_len = ARRAY_SIZE(sd),
    .conn_param = &conn_param_val,
    .keepalive_sec = 5,
};

/* Devices */
static const struct device *const afe_dev = DEVICE_DT_GET_ANY(dsy_ad5941);

/* ======================= Helpers for sweeping ======================= */

static inline int32_t take_avg_id_nA(const struct device *afe, uint8_t avg_N)
{
    uint8_t n = avg_N ? avg_N : 1;
    int64_t acc = 0;
    for (uint8_t i = 0; i < n; ++i) {
        int32_t sample = 0;
        (void)ad5941_read_current_nA(afe, &sample);
        acc += sample;
    }
    return (int32_t)(acc / n);
}

/* Sweep once from v_from to v_to (inclusive), step magnitude = step_abs (>0) */
static void sweep_once(const struct device *afe,
                       int32_t v_from, int32_t v_to, int32_t step_abs,
                       uint16_t dwell_ms, uint8_t avg_N, uint32_t t0_ms)
{
    int32_t step = (v_to >= v_from) ? +step_abs : -(int32_t)step_abs;

    /* Ensure DAC is at the first point and let it settle */
    (void)ad5941_set_vgs_mV(afe, v_from);
    k_sleep(K_MSEC(dwell_ms));

    for (int32_t vg_mv = v_from;
         (step > 0) ? (vg_mv <= v_to) : (vg_mv >= v_to);
         vg_mv += step) {

        if (!ble_gfet_is_running()) {
            break;
        }

        if (vg_mv != v_from) {
            (void)ad5941_set_vgs_mV(afe, vg_mv);
            k_sleep(K_MSEC(dwell_ms));
        }

        int32_t id_na = take_avg_id_nA(afe, avg_N);

        struct gfet_sample s = {
            .t_ms  = k_uptime_get_32() - t0_ms,
            .vg_mV = vg_mv,
            .id_nA = id_na,
        };
        (void)ble_gfet_notify_sample(&s);
    }
}

/* ======================= Measurement thread ======================= */

static void gfet_thread(void *a, void *b, void *c)
{
    ARG_UNUSED(a); ARG_UNUSED(b); ARG_UNUSED(c);

    /* BLE service for streaming */
    (void)ble_gfet_srv_init();

    /* Optional: reset and baseline-config the AFE if present */
    if (afe_dev && device_is_ready(afe_dev)) {
        (void)ad5941_reset(afe_dev);
        /* Configure LPTIA: mid-supply VZERO ~1.1V and 10kΩ RTIA as a safe default */
        (void)ad5941_lptia_config(afe_dev, 200 /* mV */, 200 /* ohm */);
    }

    for (;;) {
        /* Wait for mobile to write start=1 to control characteristic */
        (void)ble_gfet_wait_for_start(K_FOREVER);

        struct gfet_cfg cfg;
        ble_gfet_get_cfg(&cfg);

        /* sanitize */
        int32_t v_start = cfg.v_start_mV;
        int32_t v_stop  = cfg.v_stop_mV;

        int32_t step_abs = (cfg.v_step_mV == 0) ? 1 : (int32_t)cfg.v_step_mV;
        if (step_abs < 0) step_abs = -step_abs;

        uint16_t dwell_ms = cfg.dwell_ms ? cfg.dwell_ms : 10;
        if (dwell_ms > 2000) dwell_ms = 2000;

        uint8_t avg_N = cfg.avg_N ? cfg.avg_N : 1;
        if (avg_N > 32) avg_N = 32;

        if (afe_dev && device_is_ready(afe_dev)) {
            /* First stabilize at the start point once per run */
            (void)ad5941_set_vgs_mV(afe_dev, v_start);
            /* Allow EDL / reference / frontend to settle */
            k_sleep(K_SECONDS(5));
            /* Discard a few warm-up readings (not uploaded) */
            for (int i = 0; i < 5; ++i) {
                int32_t dummy = 0;
                (void)ad5941_read_current_nA(afe_dev, &dummy);
            }
        }

        uint32_t t0 = k_uptime_get_32();

        /* ---- Forward sweep: v_start → v_stop ---- */
        if (afe_dev && device_is_ready(afe_dev) && ble_gfet_is_running()) {
            sweep_once(afe_dev, v_start, v_stop, step_abs, dwell_ms, avg_N, t0);
        }

        /* ---- Reverse sweep: v_stop → v_start (no duplicated endpoint) ---- */
        if (afe_dev && device_is_ready(afe_dev) && ble_gfet_is_running() && (v_stop != v_start)) {
            int32_t dir = (v_start < v_stop) ? -1 : +1;
            int32_t first_back = v_stop + dir * step_abs;

            /* Move to the first reverse point (skip v_stop duplication) and settle */
            (void)ad5941_set_vgs_mV(afe_dev, first_back);
            k_sleep(K_MSEC(dwell_ms));

            /* Sweep back to v_start (inclusive) */
            sweep_once(afe_dev, first_back, v_start, step_abs, dwell_ms, avg_N, t0);
        }

        /* Loop back to wait for next start */
    }
}

/* ======================= BLE advertising thread ======================= */
/* Provided elsewhere: ble_adv_thread(const struct ble_adv_profile *profile, ...) */
K_THREAD_DEFINE(ble_t, 1024, ble_adv_thread, (void *)&gfet_profile, NULL, NULL,
                1 /* prio */, 0, 0);

/* GFET measurement thread */
K_THREAD_DEFINE(gfet_t, 1024, gfet_thread, NULL, NULL, NULL,
                2 /* prio */, 0, 0);
