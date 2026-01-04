#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/gpio.h>
#include <driver/ad5778r.h>

LOG_MODULE_REGISTER(mux_probe, LOG_LEVEL_INF);

static const struct device *dac = DEVICE_DT_GET_ONE(dsy_ad5778r);
static const struct device *gpio0 = DEVICE_DT_GET(DT_NODELABEL(gpio0));

#define MARK_PIN 13  // 随便挑个空脚，当示波器/万用表“状态标记”

static void mux_probe_thread(void *a, void *b, void *c)
{
    ARG_UNUSED(a); ARG_UNUSED(b); ARG_UNUSED(c);

    if (!device_is_ready(dac)) {
        LOG_ERR("AD5778R not ready");
        return;
    }
    if (device_is_ready(gpio0)) {
        gpio_pin_configure(gpio0, MARK_PIN, GPIO_OUTPUT_INACTIVE);
    }

    for (;;) {
        LOG_INF("MUX -> VREF (expect ~1.25V)");
        if (device_is_ready(gpio0)) gpio_pin_set(gpio0, MARK_PIN, 1);
        (void)ad5778r_monitor_mux(dac, AD5778R_MUX_VREF);
        k_sleep(K_SECONDS(5));

        LOG_INF("MUX -> GND (expect ~0V)");
        if (device_is_ready(gpio0)) gpio_pin_set(gpio0, MARK_PIN, 0);
        (void)ad5778r_monitor_mux(dac, AD5778R_MUX_GND);
        k_sleep(K_SECONDS(5));
    }
}

K_THREAD_DEFINE(mux_probe_t, 1024, mux_probe_thread, NULL, NULL, NULL, 2, 0, 0);
