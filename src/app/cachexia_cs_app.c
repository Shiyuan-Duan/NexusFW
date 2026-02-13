/* ========================= cachexia_cs_app.c =========================
 * Cachexia current-source application thread:
 * - Drives AD5778R OUT0 current amplitude using *AmplitudeCode* (u16 over BLE)
 * - Generates biphasic stimulation by toggling ADG5436 IN1/IN2 (both tied to P0.14)
 * - Default frequency 5 Hz, adjustable over BLE (Hz, integer)
 *
 * + IMU (BMI270) streaming:
 *   - Periodically reads accel/gyro via Zephyr sensor API
 *   - Streams 16-byte payload over BLE notify (Cachexia IMU characteristic)
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/sys/byteorder.h>
#include <stdint.h>
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

/* Choose a default span for your stim system. */
#define CACHEXIA_CS_DEFAULT_SPAN AD5778R_SPAN_25_MA

/* ---- ADG5436 control GPIO: both IN1 and IN2 tied to P0.14 ---- */
#define ADG5436_GPIO_NODE DT_NODELABEL(gpio0)
#define ADG5436_CTRL_PIN  14

/* Optional interphase “quiet” gap (microseconds). */
#define INTERPHASE_GAP_US 0

/* ---- BMI270 IMU streaming (polling) ---- */
#define CACHEXIA_IMU_STREAM_HZ        50u
#define CACHEXIA_IMU_STREAM_PERIOD_MS (1000u / CACHEXIA_IMU_STREAM_HZ)
/* When nobody subscribes to IMU notifications, sample less often to save power */
#define CACHEXIA_IMU_IDLE_PERIOD_MS   200u

static const struct device *const ad5778r_dev = DEVICE_DT_GET_ONE(dsy_ad5778r);
static const struct device *const gpio0_dev = DEVICE_DT_GET(ADG5436_GPIO_NODE);

/* BMI270 device (only compiled if a bosch,bmi270 node exists and is "okay") */
#if DT_HAS_COMPAT_STATUS_OKAY(bosch_bmi270)
#define CACHEXIA_HAVE_BMI270 1
static const struct device *const bmi270_dev =
	DEVICE_DT_GET(DT_COMPAT_GET_ANY_STATUS_OKAY(bosch_bmi270));
#else
#define CACHEXIA_HAVE_BMI270 0
#endif

static int adg5436_init(void)
{
	if (!device_is_ready(gpio0_dev)) {
		return -ENODEV;
	}

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
			/* NEW: finest control (16-bit DAC code) */
			uint16_t code = ble_cachexia_cs_get_amplitude_code();

			uint16_t freq_hz = ble_cachexia_cs_get_frequency_hz();
			if (freq_hz < 1) {
				freq_hz = 1;
			}

			/* On first activation: program span, power up by doing an update-containing op */
			if (!active) {
				active = true;
				phase_in = 0;
				adg5436_set_in(phase_in);

				(void)ad5778r_set_span(ad5778r_dev, CACHEXIA_CS_DAC_CH,
						       CACHEXIA_CS_DEFAULT_SPAN);

				(void)ad5778r_write(ad5778r_dev, CACHEXIA_CS_DAC_CH, code);
				last_code = code;
				last_freq = freq_hz;

				LOG_INF("Stim ON: span=%u code=0x%04x freq=%uHz",
					(unsigned)CACHEXIA_CS_DEFAULT_SPAN,
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
				half_period_ms = 1;
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
				(void)ad5778r_set_span(ad5778r_dev, CACHEXIA_CS_DAC_CH,
						       AD5778R_SPAN_HIGH_Z);
				(void)ad5778r_power_down(ad5778r_dev, CACHEXIA_CS_DAC_CH);

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

/* ---- BMI270 IMU polling + BLE streaming thread ---- */
#if CACHEXIA_HAVE_BMI270

static int16_t clamp_i64_to_i16(int64_t x)
{
	if (x > (int64_t)INT16_MAX) {
		return INT16_MAX;
	}
	if (x < (int64_t)INT16_MIN) {
		return INT16_MIN;
	}
	return (int16_t)x;
}

static int16_t accel_ms2_to_mg(const struct sensor_value *v)
{
	int64_t micro = (int64_t)v->val1 * 1000000LL + (int64_t)v->val2;

	const int64_t denom = 9806650LL; /* 9.80665 * 1e6 */
	int64_t num = micro * 1000LL;

	int64_t mg = (num >= 0) ? ((num + denom / 2) / denom)
				: ((num - denom / 2) / denom);

	return clamp_i64_to_i16(mg);
}

static int16_t gyro_rads_to_dps10(const struct sensor_value *v)
{
	int64_t micro = (int64_t)v->val1 * 1000000LL + (int64_t)v->val2;

	const int64_t k_num = 572957795LL;
	const int64_t denom = 1000000000000LL;

	int64_t num = micro * k_num;
	int64_t dps10 = (num >= 0) ? ((num + denom / 2) / denom)
				   : ((num - denom / 2) / denom);

	return clamp_i64_to_i16(dps10);
}

static void cachexia_imu_thread(void *a, void *b, void *c)
{
	ARG_UNUSED(a); ARG_UNUSED(b); ARG_UNUSED(c);

	if (!device_is_ready(bmi270_dev)) {
		LOG_ERR("BMI270 device not ready");
		return;
	}

	struct sensor_value odr = { .val1 = 100, .val2 = 0 };
	(void)sensor_attr_set(bmi270_dev, SENSOR_CHAN_ACCEL_XYZ,
			      SENSOR_ATTR_SAMPLING_FREQUENCY, &odr);
	(void)sensor_attr_set(bmi270_dev, SENSOR_CHAN_GYRO_XYZ,
			      SENSOR_ATTR_SAMPLING_FREQUENCY, &odr);

	bool warned_fetch = false;
	bool warned_chan  = false;

	for (;;) {
		int rc = sensor_sample_fetch(bmi270_dev);
		if (rc == 0) {
			struct sensor_value acc[3];
			struct sensor_value gyr[3];

			int rc_a = sensor_channel_get(bmi270_dev, SENSOR_CHAN_ACCEL_XYZ, acc);
			int rc_g = sensor_channel_get(bmi270_dev, SENSOR_CHAN_GYRO_XYZ,  gyr);

			if (rc_a == 0 && rc_g == 0) {
				int16_t ax_mg    = accel_ms2_to_mg(&acc[0]);
				int16_t ay_mg    = accel_ms2_to_mg(&acc[1]);
				int16_t az_mg    = accel_ms2_to_mg(&acc[2]);

				int16_t gx_dps10 = gyro_rads_to_dps10(&gyr[0]);
				int16_t gy_dps10 = gyro_rads_to_dps10(&gyr[1]);
				int16_t gz_dps10 = gyro_rads_to_dps10(&gyr[2]);

				uint8_t payload[CACHEXIA_IMU_PAYLOAD_LEN];

				sys_put_le32(k_uptime_get_32(), &payload[0]);
				sys_put_le16((uint16_t)ax_mg,    &payload[4]);
				sys_put_le16((uint16_t)ay_mg,    &payload[6]);
				sys_put_le16((uint16_t)az_mg,    &payload[8]);
				sys_put_le16((uint16_t)gx_dps10, &payload[10]);
				sys_put_le16((uint16_t)gy_dps10, &payload[12]);
				sys_put_le16((uint16_t)gz_dps10, &payload[14]);

				(void)ble_cachexia_cs_imu_stream(payload, sizeof(payload));
			} else if (!warned_chan) {
				warned_chan = true;
				LOG_WRN("BMI270 channel_get failed (acc=%d gyro=%d)", rc_a, rc_g);
			}
		} else if (!warned_fetch) {
			warned_fetch = true;
			LOG_WRN("BMI270 sample_fetch failed (%d)", rc);
		}

		uint32_t period_ms = ble_cachexia_cs_imu_notify_is_enabled() ?
				     CACHEXIA_IMU_STREAM_PERIOD_MS :
				     CACHEXIA_IMU_IDLE_PERIOD_MS;

		if (period_ms == 0) {
			period_ms = 1;
		}
		k_sleep(K_MSEC(period_ms));
	}
}

K_THREAD_DEFINE(cachexia_imu_t, 1024, cachexia_imu_thread, NULL, NULL, NULL,
		3 /*prio*/, 0, 0);

#endif /* CACHEXIA_HAVE_BMI270 */

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