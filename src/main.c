#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/gpio.h>

#include "ble_adv_core.h"
// #include "ad5689.h"
// #include "ble_ad5689_srv.h"
#include "mlx90393.h"
LOG_MODULE_REGISTER(app, CONFIG_LOG_DEFAULT_LEVEL);

/* BLE advertising thread is defined in the app-specific module. */

int main(void)
{
    /* Quick sanity: confirm the MLX90393 device instance is ready */
    /* Configure P0.23 as indicator LED (active-low):
     * - Pull the pin low for 100 ms (LED on)
     * - Then release to Hi-Z for 900 ms (LED off)
     * Use k_sleep to avoid starving lower-priority threads.
     */
    // const struct device *gpio0 = DEVICE_DT_GET(DT_NODELABEL(gpio0));
    // const uint32_t pin = 23U; /* P0.23 */

    // if (!device_is_ready(gpio0)) {
    //     LOG_ERR("gpio0 not ready");
    //     return 0;
    // }

    /* Ensure LED is off initially by setting Hi-Z (input). */
    // (void)gpio_pin_configure(gpio0, pin, GPIO_INPUT);

    while (1) {
        printk("Running...\n");
        // bool healthy = ble_is_healthy();

        // if (healthy) {
        //     /* Normal heartbeat: 100 ms low, 900 ms Hi-Z */
        //     (void)gpio_pin_configure(gpio0, pin, GPIO_OUTPUT);
        //     (void)gpio_pin_set(gpio0, pin, 0);
        //     k_sleep(K_MSEC(100));

        //     (void)gpio_pin_configure(gpio0, pin, GPIO_INPUT);
        //     k_sleep(K_MSEC(900));
        // } else {
        //     /* BLE unhealthy: double-flash pattern within ~1 s window */
        //     for (int i = 0; i < 2; ++i) {
        //         (void)gpio_pin_configure(gpio0, pin, GPIO_OUTPUT);
        //         (void)gpio_pin_set(gpio0, pin, 0);
        //         k_sleep(K_MSEC(100));

        //         (void)gpio_pin_configure(gpio0, pin, GPIO_INPUT);
        //         k_sleep(K_MSEC(100));
        //     }
        //     /* Remaining off time */
            k_sleep(K_MSEC(1000));
        }
    }
