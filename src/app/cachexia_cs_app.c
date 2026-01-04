/* Cachexia current-source application thread:
 * - Drives AD5778R OUT0 current amplitude (0..100% -> 16-bit code)
 * - Generates biphasic stimulation by toggling ADG5436 IN1/IN2 (both tied to P0.14)
 * - Default frequency 5 Hz, adjustable over BLE (Hz, integer)
 *
 * Hardware assumption for true biphasic (electrode swap) with IN1==IN2:
 *   - D1 -> Electrode A
 *   - D2 -> Electrode B
 *   - S1A -> DAC OUT (current source)
 *   - S1B -> GND
 *   - S2A -> GND
 *   - S2B -> DAC OUT
 *
 * With ADG5436 truth table: IN=1 => SxA ON, IN=0 => SxB ON. :contentReference[oaicite:3]{index=3}
 * So:
 *   IN=1 -> A gets DAC, B gets GND
 *   IN=0 -> A gets GND, B gets DAC  (current direction reverses through the load)
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/gpio.h>
#include <string.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/uuid.h>

#include "ble_adv_core.h"
#include "ble_cachexia_cs_srv.h"

#include <driver/ad5778r.h>

LOG_MODULE_REGISTER(cachexia_cs_app, CONFIG_LOG_DEFAULT_LEVEL);

/* ---- AD5778R usage ---- */
#define CACHEXIA_CS_DAC_CH 0

/* Choose a default span for your stim system.
 * You can change this to AD5778R_SPAN_50_MA / 100mA / 200mA / 300mA, etc.
 */
#define CACHEXIA_CS_DEFAULT_SPAN AD5778R_SPAN_25_MA

/* ---- ADG5436 control GPIO: both IN1 and IN2 tied to P0.14 ---- */
#define ADG5436_GPIO_NODE DT_NODELABEL(gpio0)
#define ADG5436_CTRL_PIN  14

/* Optional interphase “quiet” gap (microseconds).
 * If nonzero, we momentarily set DAC code to 0 around the switch toggle.
 * Default 0 keeps the output current constant and only swaps electrodes.
 */
#define INTERPHASE_GAP_US 0

static const struct device *const ad5778r_dev = DEVICE_DT_GET_ONE(dsy_ad5778r);
static const struct device *const gpio0_dev = DEVICE_DT_GET(ADG5436_GPIO_NODE);

static uint16_t map_pct_to_code(uint8_t pct)
{
	if (pct >= 100) {
		return 0xFFFF;
	}
	uint32_t code = ((uint32_t)pct * 65535u) / 100u;
	return (uint16_t)code;
}

static int adg5436_init(void)
{
	if (!device_is_ready(gpio0_dev)) {
		return -ENODEV;
	}

	/* ADG5436 inputs are 3V logic compatible (VINH min 2.0V, VINL max 0.8V). :contentReference[oaicite:4]{index=4} */
	int ret = gpio_pin_configure(gpio0_dev, ADG5436_CTRL_PIN, GPIO_OUTPUT_INACTIVE);
	if (ret != 0) {
		return ret;
	}

	/* Start in a defined phase (IN=0) */
	gpio_pin_set(gpio0_dev, ADG5436_CTRL_PIN, 0);
	return 0;
}

static inline void adg5436_set_in(uint8_t level)
{
	gpio_pin_set(gpio0_dev, ADG5436_CTRL_PIN, level ? 1 : 0);
}

static void cachexia_cs_thread(void *a, void *b, void *c)
{
	ARG_UNUSED(a); ARG_UNUSED(b); ARG_UNUSED(c);

	if (!device_is_ready(ad5778r_dev)) {
		LOG_ERR("AD5778R device not ready");
		return;
	}

	int ret = adg5436_init();
	if (ret != 0) {
		LOG_ERR("ADG5436 ctrl GPIO init failed (%d)", ret);
		return;
	}

	/* Initialize BLE service */
	if (ble_cachexia_cs_srv_init() != 0) {
		LOG_ERR("Failed to init cachexia CS BLE service");
		return;
	}

	/* Ensure output safely off initially */
	(void)ad5778r_set_span(ad5778r_dev, CACHEXIA_CS_DAC_CH, AD5778R_SPAN_HIGH_Z);
	(void)ad5778r_power_down(ad5778r_dev, CACHEXIA_CS_DAC_CH);

	bool active = false;
	uint16_t last_code = 0xFFFF; /* force first write */
	uint16_t last_freq = 0;

	/* Phase state: 0 -> IN=0, 1 -> IN=1 */
	uint8_t phase_in = 0;

	for (;;) {
		if (ble_cachexia_cs_get_switch()) {
			uint8_t pct = ble_cachexia_cs_get_amplitude_percent();
			uint16_t code = map_pct_to_code(pct);

			uint16_t freq_hz = ble_cachexia_cs_get_frequency_hz();
			if (freq_hz < 1) {
				freq_hz = 1;
			}

			/* On first activation: program span, power up by doing an update-containing op */
			if (!active) {
				active = true;
				phase_in = 0;
				adg5436_set_in(phase_in);

				/* Set span to your chosen current range and power up */
				(void)ad5778r_set_span(ad5778r_dev, CACHEXIA_CS_DAC_CH, CACHEXIA_CS_DEFAULT_SPAN);

				/* Set initial current code */
				(void)ad5778r_write(ad5778r_dev, CACHEXIA_CS_DAC_CH, code);
				last_code = code;
				last_freq = freq_hz;

				LOG_INF("Stim ON: span=%u code=0x%04x freq=%uHz", (unsigned)CACHEXIA_CS_DEFAULT_SPAN,
					(unsigned)code, (unsigned)freq_hz);
			}

			/* If amplitude changed, update DAC */
			if (code != last_code) {
				(void)ad5778r_write(ad5778r_dev, CACHEXIA_CS_DAC_CH, code);
				last_code = code;
			}

			/* Compute half-period in ms: T/2 = 1000 / (2*Hz) */
			uint32_t half_period_ms = 1000u / (2u * (uint32_t)freq_hz);
			if (half_period_ms == 0) {
				half_period_ms = 1; /* guard for high freq values */
			}

			/* Toggle phase (swap electrodes through ADG5436) */
#if (INTERPHASE_GAP_US > 0)
			(void)ad5778r_write(ad5778r_dev, CACHEXIA_CS_DAC_CH, 0);
			k_busy_wait(INTERPHASE_GAP_US);
#endif
			phase_in ^= 1u;
			adg5436_set_in(phase_in);
#if (INTERPHASE_GAP_US > 0)
			k_busy_wait(INTERPHASE_GAP_US);
			(void)ad5778r_write(ad5778r_dev, CACHEXIA_CS_DAC_CH, code);
#endif

			last_freq = freq_hz;
			k_sleep(K_MSEC(half_period_ms));
		} else {
			if (active) {
				/* Safe off: set span to High-Z + power down.
				 * Datasheet: to fully turn off output, High-Z span is recommended. :contentReference[oaicite:5]{index=5}
				 */
				(void)ad5778r_set_span(ad5778r_dev, CACHEXIA_CS_DAC_CH, AD5778R_SPAN_HIGH_Z);
				(void)ad5778r_power_down(ad5778r_dev, CACHEXIA_CS_DAC_CH);

				/* Put switch in a defined state */
				phase_in = 0;
				adg5436_set_in(phase_in);

				active = false;
				last_code = 0xFFFF;
				last_freq = 0;
				LOG_INF("Stim OFF");
			}
			k_sleep(K_MSEC(50));
		}
	}
}

/* Give stim thread a bit lower priority than BLE adv thread if you want */
K_THREAD_DEFINE(cachexia_cs_t, 1024, cachexia_cs_thread, NULL, NULL, NULL,
		2 /*prio*/, 0, 0);

/* ---- BLE advertising profile/thread (same pattern as your original) ---- */
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
	BT_DATA_BYTES(BT_DATA_UUID128_ALL, BT_UUID_CACHEXIA_SVC_VAL),
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
