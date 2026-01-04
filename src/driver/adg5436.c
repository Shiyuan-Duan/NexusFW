// #include <zephyr/device.h>
// #include <zephyr/drivers/gpio.h>
// #include <zephyr/devicetree.h>
// #include <zephyr/logging/log.h>
// #include <errno.h>

// #include "adg5436.h"

// LOG_MODULE_REGISTER(adg5436, CONFIG_LOG_DEFAULT_LEVEL);

// /* Required: alias "adg5436-ctrl" -> drives IN1/IN2 (tied together) */
// #if DT_NODE_HAS_STATUS(DT_ALIAS(adg5436_ctrl), okay)
// static const struct gpio_dt_spec g_ctrl = GPIO_DT_SPEC_GET(DT_ALIAS(adg5436_ctrl), gpios);
// #else
// #error "Missing devicetree alias adg5436-ctrl (DT_ALIAS(adg5436_ctrl))"
// #endif

// /* Optional: alias "adg5436-en" -> drives EN (LFCSP only) */
// #if DT_NODE_HAS_STATUS(DT_ALIAS(adg5436_en), okay)
// static const struct gpio_dt_spec g_en = GPIO_DT_SPEC_GET(DT_ALIAS(adg5436_en), gpios);
// #define ADG5436_HAS_EN 1
// #else
// #define ADG5436_HAS_EN 0
// #endif

// int adg5436_init(void)
// {
//     if (!device_is_ready(g_ctrl.port)) {
//         LOG_ERR("ADG5436 ctrl GPIO port not ready");
//         return -ENODEV;
//     }

//     int ret = gpio_pin_configure_dt(&g_ctrl, GPIO_OUTPUT_INACTIVE);
//     if (ret) {
//         LOG_ERR("Failed to configure ADG5436 ctrl GPIO: %d", ret);
//         return ret;
//     }

// #if ADG5436_HAS_EN
//     if (!device_is_ready(g_en.port)) {
//         LOG_ERR("ADG5436 EN GPIO port not ready");
//         return -ENODEV;
//     }
//     ret = gpio_pin_configure_dt(&g_en, GPIO_OUTPUT_INACTIVE); /* default disabled */
//     if (ret) {
//         LOG_ERR("Failed to configure ADG5436 EN GPIO: %d", ret);
//         return ret;
//     }
// #endif

//     /* Default safe state */
//     (void)gpio_pin_set_dt(&g_ctrl, 0);
// #if ADG5436_HAS_EN
//     (void)gpio_pin_set_dt(&g_en, 0);
// #endif

//     LOG_INF("ADG5436 GPIO control ready");
//     return 0;
// }

// int adg5436_set_phase(bool phase)
// {
//     return gpio_pin_set_dt(&g_ctrl, phase ? 1 : 0);
// }

// int adg5436_enable(bool enable)
// {
// #if ADG5436_HAS_EN
//     return gpio_pin_set_dt(&g_en, enable ? 1 : 0);
// #else
//     ARG_UNUSED(enable);
//     return 0;
// #endif
// }
