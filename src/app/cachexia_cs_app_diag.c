#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/logging/log.h>

#include <driver/ad5778r.h>

LOG_MODULE_REGISTER(cs_diag, LOG_LEVEL_INF);

static const struct device *dac = DEVICE_DT_GET_ONE(dsy_ad5778r);

static void diag_thread(void *a, void *b, void *c)
{
	ARG_UNUSED(a); ARG_UNUSED(b); ARG_UNUSED(c);

	if (!device_is_ready(dac)) {
		LOG_ERR("AD5778R not ready");
		return;
	}

	LOG_INF("DIAG: set span=12.5mA on OUT0, write code=0xFFFF");

	/* span 12.5mA, then full-scale code */
	(void)ad5778r_set_span(dac, 0, AD5778R_SPAN_12P5_MA);
	(void)ad5778r_write(dac, 0, 0xFFFF);

	while (1) {
		uint8_t fr = 0;
		(void)ad5778r_read_fault(dac, &fr);
		k_sleep(K_SECONDS(1));
	}
}

K_THREAD_DEFINE(cs_diag_t, 1024, diag_thread, NULL, NULL, NULL, 2, 0, 0);
