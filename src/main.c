#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/logging/log.h>

#include "ble.h"
#include "ad5689.h"
#include "ble_ad5689_srv.h"
LOG_MODULE_REGISTER(app, CONFIG_LOG_DEFAULT_LEVEL);

/* ── BLE advertising thread (unchanged) ───────────────────────────────── */
K_THREAD_DEFINE(ble_t, 1024, t_ble_adv_start, NULL, NULL, NULL,
		1 /* prio */, 0, 0);

/* 1 kHz square wave params */
#define SQUARE_HZ        1000
#define HALF_PERIOD_US   (1000000 / (2 * SQUARE_HZ))  /* 500 µs */

/* ── Application entry point ──────────────────────────────────────────── */
int main(void)
{
	const struct device *dac = DEVICE_DT_GET_ONE(dsy_ad5689);

	if (!device_is_ready(dac)) {
		LOG_ERR("AD5689 device not ready");
		return -ENODEV;
	}

	LOG_INF("Starting 1 kHz square wave on AD5689 channel A");

	while (true) {
		/* high level (full-scale) */
		ad5689_write(dac, 0 /* ch A */, 0xFFFF);
		k_sleep(K_USEC(HALF_PERIOD_US));

		/* low level (zero) */
		ad5689_write(dac, 0 /* ch A */, 0x0000);
		k_sleep(K_USEC(HALF_PERIOD_US));
	}

	/* never reached */
	return 0;
}
