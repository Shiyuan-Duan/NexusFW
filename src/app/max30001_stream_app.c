#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/logging/log.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/uuid.h>

#include "ble_adv_core.h"
#include "ble_max30001_stream_srv.h"

#include <driver/max30001.h>

LOG_MODULE_REGISTER(max30001_stream_app, CONFIG_LOG_DEFAULT_LEVEL);

static const struct device *const max30001_dev = DEVICE_DT_GET_ONE(dsy_max30001);

/* 每个样本 24bit => 3 bytes；默认 MTU=23 时 payload=20 bytes，保守打 6 个样本=18 bytes */
#define PACK_WORDS_PER_NOTIF 6

static inline void pack_u24_le(uint8_t *dst, uint32_t w24)
{
	dst[0] = (uint8_t)(w24 & 0xFF);
	dst[1] = (uint8_t)((w24 >> 8) & 0xFF);
	dst[2] = (uint8_t)((w24 >> 16) & 0xFF);
}

static void stream_thread(void *a, void *b, void *c)
{
	ARG_UNUSED(a); ARG_UNUSED(b); ARG_UNUSED(c);

	if (!device_is_ready(max30001_dev)) {
		LOG_ERR("MAX30001 not ready");
		return;
	}

	if (ble_max30001_stream_srv_init() != 0) {
		LOG_ERR("BLE MAX30001 stream service init failed");
		return;
	}

	/* 确保默认关闭 */
	(void)max30001_stream_off(max30001_dev);

	bool active = false;

	for (;;) {
		if (ble_max30001_get_switch()) {
			if (!active) {
				active = true;
				(void)max30001_stream_on(max30001_dev, true, true);
				LOG_INF("MAX30001 stream ON");
			}

			/* ---- ECG (BioV) ---- */
			{
				uint8_t buf[3 * PACK_WORDS_PER_NOTIF];
				uint8_t cnt = 0;

				/* ECG FIFO depth 通常 32 words；这里每轮最多读 32 次避免卡死 */
				for (int i = 0; i < 32; i++) {
					uint32_t w24;
					if (max30001_read_ecg_word(max30001_dev, &w24) != 0) {
						break;
					}

					uint8_t etag = max30001_ecg_etag(w24);

					if (etag == MAX30001_ECG_ETAG_EMPTY) {
						break;
					}
					if (etag == MAX30001_ECG_ETAG_OVERFLOW) {
						(void)max30001_fifo_reset(max30001_dev);
						break;
					}

					pack_u24_le(&buf[3 * cnt], w24);
					cnt++;

					if (cnt == PACK_WORDS_PER_NOTIF) {
						(void)ble_max30001_notify_biov(buf, sizeof(buf));
						cnt = 0;
					}

					if (etag == MAX30001_ECG_ETAG_VALID_EOF ||
					    etag == MAX30001_ECG_ETAG_OU_EOF) {
						break;
					}
				}

				if (cnt > 0) {
					(void)ble_max30001_notify_biov(buf, 3u * cnt);
				}
			}

			/* ---- BioZ ---- */
			{
				uint8_t buf[3 * PACK_WORDS_PER_NOTIF];
				uint8_t cnt = 0;

				/* BioZ FIFO depth = 8 words（datasheet） */
				for (int i = 0; i < 8; i++) {
					uint32_t w24;
					if (max30001_read_bioz_word(max30001_dev, &w24) != 0) {
						break;
					}

					uint8_t btag = max30001_bioz_btag(w24);

					if (btag == MAX30001_BIOZ_BTAG_EMPTY) {
						break;
					}
					if (btag == MAX30001_BIOZ_BTAG_OVERFLOW) {
						(void)max30001_fifo_reset(max30001_dev);
						break;
					}

					pack_u24_le(&buf[3 * cnt], w24);
					cnt++;

					if (cnt == PACK_WORDS_PER_NOTIF) {
						(void)ble_max30001_notify_bioz(buf, sizeof(buf));
						cnt = 0;
					}

					if (btag == MAX30001_BIOZ_BTAG_VALID_EOF ||
					    btag == MAX30001_BIOZ_BTAG_OU_EOF) {
						break;
					}
				}

				if (cnt > 0) {
					(void)ble_max30001_notify_bioz(buf, 3u * cnt);
				}
			}

			/* 5ms 一轮，足够避免 BioZ FIFO (8 words) 溢出 */
			k_sleep(K_MSEC(5));

		} else {
			if (active) {
				(void)max30001_stream_off(max30001_dev);
				active = false;
				LOG_INF("MAX30001 stream OFF");
			}
			k_sleep(K_MSEC(50));
		}
	}
}

K_THREAD_DEFINE(max30001_stream_t, 2048, stream_thread,
		NULL, NULL, NULL,
		2 /*prio*/, 0, 0);

/* ---- BLE advertising profile/thread（照你原来的模式） ---- */
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

static const struct bt_data sd[] = {
	BT_DATA_BYTES(BT_DATA_UUID128_ALL, BT_UUID_MAX30001_SVC_VAL),
};

static const struct bt_le_conn_param conn_param_val = {
	.interval_min = 12, /* 更快一点吞吐更舒服 */
	.interval_max = 24,
	.latency = 0,
	.timeout = 200,
};

static const struct ble_adv_profile max30001_profile = {
	.params = &adv_param,
	.ad = ad,
	.ad_len = ARRAY_SIZE(ad),
	.sd = sd,
	.sd_len = ARRAY_SIZE(sd),
	.conn_param = &conn_param_val,
	.keepalive_sec = 5,
};

K_THREAD_DEFINE(ble_t, 1024, ble_adv_thread, (void *)&max30001_profile, NULL, NULL,
		1 /*prio*/, 0, 0);