#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/util.h>
#include <stdint.h>
#include <string.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>

#include "ble_adv_core.h"
#include "ble_cachexia_fg_srv.h"

#include <driver/ad5778r.h>

LOG_MODULE_REGISTER(cachexia_fg_app, CONFIG_LOG_DEFAULT_LEVEL);

/* ---- DAC ---- */
#define CACHEXIA_CS_DAC_CH 0
#define CACHEXIA_CS_DEFAULT_SPAN AD5778R_SPAN_25_MA  /* +/-25mA realized by electrode swap */

/* ---- ADG5436 control: both IN1+IN2 tied to P0.14 ---- */
#define ADG5436_GPIO_NODE DT_NODELABEL(gpio0)
#define ADG5436_CTRL_PIN  14

/* Optional gap around polarity change */
#define INTERPHASE_GAP_US 0

static const struct device *const ad5778r_dev = DEVICE_DT_GET_ONE(dsy_ad5778r);
static const struct device *const gpio0_dev = DEVICE_DT_GET(ADG5436_GPIO_NODE);

static int adg5436_init(void)
{
	if (!device_is_ready(gpio0_dev)) {
		return -ENODEV;
	}

	int ret = gpio_pin_configure(gpio0_dev, ADG5436_CTRL_PIN, GPIO_OUTPUT_INACTIVE);
	if (ret != 0) {
		return ret;
	}

	gpio_pin_set(gpio0_dev, ADG5436_CTRL_PIN, 0);
	return 0;
}

static inline void adg5436_set_in(uint8_t level)
{
	gpio_pin_set(gpio0_dev, ADG5436_CTRL_PIN, level ? 1 : 0);
}

static inline uint16_t map_abs_ua_to_code(uint32_t abs_ua)
{
	if (abs_ua >= CACHEXIA_FG_MAX_ABS_UA) {
		return 0xFFFF;
	}
	/* code = abs_ua / 25000 * 65535 */
	uint32_t code = (abs_ua * 65535u) / (uint32_t)CACHEXIA_FG_MAX_ABS_UA;
	return (uint16_t)code;
}

static void dac_safe_off(void)
{
	(void)ad5778r_set_span(ad5778r_dev, CACHEXIA_CS_DAC_CH, AD5778R_SPAN_HIGH_Z);
	(void)ad5778r_power_down(ad5778r_dev, CACHEXIA_CS_DAC_CH);
}

static void fg_apply_block(int16_t amp_ua, uint8_t *phase_in_io, bool *dac_on_io, uint16_t *last_code_io)
{
	uint32_t abs_ua = (amp_ua >= 0) ? (uint32_t)amp_ua : (uint32_t)(-amp_ua);

	if (abs_ua == 0) {
		/* For true 0, use High-Z + powerdown (same safety reasoning as your old app) */
		dac_safe_off();
		*dac_on_io = false;
		*last_code_io = 0xFFFF;
		return;
	}

	uint16_t code = map_abs_ua_to_code(abs_ua);
	uint8_t desired_phase = (amp_ua >= 0) ? 1 : 0;

	/* Ensure span is correct (also “wakes” DAC if you powered down before) */
	if (!(*dac_on_io)) {
		(void)ad5778r_set_span(ad5778r_dev, CACHEXIA_CS_DAC_CH, CACHEXIA_CS_DEFAULT_SPAN);
		*dac_on_io = true;
	}

	/* Polarity change via electrode swap */
	if (desired_phase != *phase_in_io) {
#if (INTERPHASE_GAP_US > 0)
		(void)ad5778r_write(ad5778r_dev, CACHEXIA_CS_DAC_CH, 0);
		k_busy_wait(INTERPHASE_GAP_US);
#endif
		*phase_in_io = desired_phase;
		adg5436_set_in(*phase_in_io);
#if (INTERPHASE_GAP_US > 0)
		k_busy_wait(INTERPHASE_GAP_US);
#endif
		/* Re-apply code after switching */
		*last_code_io = 0xFFFF;
	}

	if (code != *last_code_io) {
		(void)ad5778r_write(ad5778r_dev, CACHEXIA_CS_DAC_CH, code);
		*last_code_io = code;
	}
}

/* Abortible sleep: check switch/gen periodically so stop/update is responsive */
static bool fg_wait_abortible(uint32_t dur_ms, uint32_t *gen_io, struct cachexia_fg_pattern *pat_io, uint8_t *idx_io)
{
	const uint32_t quantum_ms = 10;

	/* Infinite hold */
	if (dur_ms == 0xFFFFFFFFu) {
		for (;;) {
			if (!ble_cachexia_fg_get_switch()) {
				return true;
			}
			uint32_t gen_now = ble_cachexia_fg_get_pattern_gen();
			if (gen_now != *gen_io) {
				(void)ble_cachexia_fg_copy_active_pattern(pat_io);
				*gen_io = gen_now;
				*idx_io = 0;
				return true;
			}
			k_sleep(K_MSEC(quantum_ms));
		}
	}

	uint32_t remain = dur_ms;
	while (remain > 0) {
		if (!ble_cachexia_fg_get_switch()) {
			return true;
		}
		uint32_t gen_now = ble_cachexia_fg_get_pattern_gen();
		if (gen_now != *gen_io) {
			(void)ble_cachexia_fg_copy_active_pattern(pat_io);
			*gen_io = gen_now;
			*idx_io = 0;
			return true;
		}

		uint32_t step = MIN(remain, quantum_ms);
		k_sleep(K_MSEC(step));
		remain -= step;
	}
	return false;
}

static void cachexia_fg_thread(void *a, void *b, void *c)
{
	ARG_UNUSED(a); ARG_UNUSED(b); ARG_UNUSED(c);

	if (!device_is_ready(ad5778r_dev)) {
		LOG_ERR("AD5778R device not ready");
		return;
	}
	if (adg5436_init() != 0) {
		LOG_ERR("ADG5436 GPIO init failed");
		return;
	}

	if (ble_cachexia_fg_srv_init() != 0) {
		LOG_ERR("FG BLE service init failed");
		return;
	}

	/* Ensure safe off */
	dac_safe_off();
	adg5436_set_in(0);

	struct cachexia_fg_pattern pat;
	memset(&pat, 0, sizeof(pat));

	uint32_t gen = ble_cachexia_fg_get_pattern_gen();
	(void)ble_cachexia_fg_copy_active_pattern(&pat);

	bool dac_on = false;
	uint16_t last_code = 0xFFFF;
	uint8_t phase_in = 0;

	uint8_t idx = 0;

	for (;;) {
		if (!ble_cachexia_fg_get_switch()) {
			/* Stop */
			dac_safe_off();
			dac_on = false;
			last_code = 0xFFFF;
			phase_in = 0;
			adg5436_set_in(phase_in);
			k_sleep(K_MSEC(50));
			continue;
		}

		/* Reload pattern if committed */
		uint32_t gen_now = ble_cachexia_fg_get_pattern_gen();
		if (gen_now != gen) {
			gen = gen_now;
			(void)ble_cachexia_fg_copy_active_pattern(&pat);
			idx = 0;
		}

		/* No blocks -> output remains off but switch is ON */
		if (pat.count == 0) {
			dac_safe_off();
			dac_on = false;
			k_sleep(K_MSEC(50));
			continue;
		}

		if (idx >= pat.count) {
			idx = 0;
		}

		struct cachexia_fg_block *blk = &pat.blocks[idx];

		/* dur=0 means skip */
		if (blk->dur_ms == 0) {
			idx++;
			continue;
		}

		fg_apply_block(blk->amp_ua, &phase_in, &dac_on, &last_code);

		/* Wait duration; abort early if switch off or new commit */
		bool aborted = fg_wait_abortible(blk->dur_ms, &gen, &pat, &idx);
		if (!aborted) {
			idx++;
		}
	}
}

K_THREAD_DEFINE(cachexia_fg_t, 2048, cachexia_fg_thread, NULL, NULL, NULL,
		2 /*prio*/, 0, 0);

/* ---- BLE advertising (same pattern as your previous app) ---- */
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
	BT_DATA_BYTES(BT_DATA_UUID128_ALL, BT_UUID_CACHEXIA_FG_SVC_VAL),
};

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

K_THREAD_DEFINE(ble_t, 1024, ble_adv_thread, (void *)&cachexia_profile, NULL, NULL,
		1 /*prio*/, 0, 0);
