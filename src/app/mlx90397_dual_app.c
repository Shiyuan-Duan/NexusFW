#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/util.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>

#include <ble/ble_adv_core.h>
#include <ble/ble_mlx97_srv.h>

LOG_MODULE_REGISTER(mlx97_app, CONFIG_LOG_DEFAULT_LEVEL);

/* ======================= MLX90397 registers / bits ======================= */
#define MLX97_REG_STAT1   0x00
#define MLX97_REG_X_L     0x01
#define MLX97_REG_STAT2   0x07
#define MLX97_REG_CTRL    0x0E

#define MLX97_STAT1_DRDY  BIT(0)

/* CTRL bits */
#define MLX97_CTRL_X_EN   BIT(4)
#define MLX97_CTRL_Y_EN   BIT(5)
#define MLX97_CTRL_Z_EN   BIT(6)

/* CTRL.MODE[3:0] values */
#define MLX97_MODE_POWERDOWN  0
#define MLX97_MODE_CONT_100HZ 5

static int mlx97_read_regs(const struct i2c_dt_spec *bus,
                           uint8_t start_reg, uint8_t *buf, size_t len)
{
    return i2c_write_read_dt(bus, &start_reg, 1, buf, len);
}

static int mlx97_write_u8(const struct i2c_dt_spec *bus, uint8_t reg, uint8_t val)
{
    uint8_t tx[2] = { reg, val };
    return i2c_write_dt(bus, tx, sizeof(tx));
}

static int mlx97_set_mode_xyz(const struct i2c_dt_spec *bus, uint8_t mode,
                              bool x, bool y, bool z)
{
    uint8_t ctrl = 0;
    if (x) ctrl |= MLX97_CTRL_X_EN;
    if (y) ctrl |= MLX97_CTRL_Y_EN;
    if (z) ctrl |= MLX97_CTRL_Z_EN;
    ctrl |= (mode & 0x0F);

    /* datasheet note: change mode via powerdown first */
    int ret = mlx97_write_u8(bus, MLX97_REG_CTRL, (ctrl & ~0x0F) | MLX97_MODE_POWERDOWN);
    if (ret < 0) return ret;
    k_sleep(K_MSEC(2));
    return mlx97_write_u8(bus, MLX97_REG_CTRL, ctrl);
}

static int mlx97_wait_drdy(const struct i2c_dt_spec *bus, int32_t timeout_ms, uint8_t *st1_out)
{
    int64_t deadline = k_uptime_get() + timeout_ms;
    uint8_t st1 = 0;

    while (1) {
        int ret = mlx97_read_regs(bus, MLX97_REG_STAT1, &st1, 1);
        if (ret == 0) {
            if (st1_out) *st1_out = st1;
            if (st1 & MLX97_STAT1_DRDY) return 0;
        } else {
            return ret;
        }
        if (timeout_ms >= 0 && k_uptime_get() >= deadline) return -ETIMEDOUT;
        k_sleep(K_MSEC(1));
    }
}

static int mlx97_read_sample(const struct i2c_dt_spec *bus, struct mlx97_ble_sample *out, int32_t timeout_ms)
{
    uint8_t st1 = 0;
    int ret = mlx97_wait_drdy(bus, timeout_ms, &st1);
    if (ret < 0) return ret;

    /* burst read X_L..STAT2 => 6 bytes XYZ + 1 byte STAT2 */
    uint8_t buf[7] = {0};
    ret = mlx97_read_regs(bus, MLX97_REG_X_L, buf, sizeof(buf));
    if (ret < 0) return ret;

    out->t_ms  = (uint32_t)k_uptime_get_32();
    out->x     = (int16_t)((buf[1] << 8) | buf[0]);
    out->y     = (int16_t)((buf[3] << 8) | buf[2]);
    out->z     = (int16_t)((buf[5] << 8) | buf[4]);
    out->stat1 = st1;
    out->stat2 = buf[6];
    return 0;
}

/* ======================= Advertising profile (你的原版) ======================= */
#define DEVICE_NAME     CONFIG_BT_DEVICE_NAME
#define DEVICE_NAME_LEN (sizeof(DEVICE_NAME) - 1)

static const struct bt_le_adv_param adv_param = BT_LE_ADV_PARAM_INIT(
    (BT_LE_ADV_OPT_CONNECTABLE | BT_LE_ADV_OPT_USE_IDENTITY),
    BT_GAP_ADV_FAST_INT_MIN_1,
    BT_GAP_ADV_FAST_INT_MAX_1,
    NULL);

/*
 * Improve macOS/CoreBluetooth discoverability:
 * - Use fast advertising interval
 * - Put 128-bit primary service UUID directly in advertising data (AD), not only in scan response
 * - Use a shortened name in AD to keep total payload <= 31 bytes
 * - Provide complete name in scan response (optional)
 */
static const struct bt_data ad[] = {
    BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
    BT_DATA(BT_DATA_NAME_SHORTENED, DEVICE_NAME, MIN(DEVICE_NAME_LEN, 8)),
    BT_DATA_BYTES(BT_DATA_UUID128_ALL, BT_UUID_MLX97_SVC_VAL),
};

static const struct bt_data sd[] = {
    BT_DATA(BT_DATA_NAME_COMPLETE, DEVICE_NAME, DEVICE_NAME_LEN),
};

static const struct bt_le_conn_param conn_param_val = {
    .interval_min = 24,
    .interval_max = 40,
    .latency      = 0,
    .timeout      = 200,
};

static const struct ble_adv_profile mlx97_profile = {
    .params        = &adv_param,
    .ad            = ad,
    .ad_len        = ARRAY_SIZE(ad),
    .sd            = sd,
    .sd_len        = ARRAY_SIZE(sd),
    .conn_param    = &conn_param_val,
    .keepalive_sec = 5,
};

/* ======================= Devicetree sensors ======================= */
#define MLX0_NODE DT_NODELABEL(mlx90397_0)
#define MLX1_NODE DT_NODELABEL(mlx90397_1)

#if !DT_NODE_HAS_STATUS(MLX0_NODE, okay)
#error "mlx90397_0 node not okay in devicetree"
#endif
#if !DT_NODE_HAS_STATUS(MLX1_NODE, okay)
#error "mlx90397_1 node not okay in devicetree"
#endif

static const struct i2c_dt_spec mlx0 = I2C_DT_SPEC_GET(MLX0_NODE);
static const struct i2c_dt_spec mlx1 = I2C_DT_SPEC_GET(MLX1_NODE);

/* ======================= Sampling params ======================= */
#define APP_PRINT_EVERY 20
#define APP_PERIOD_MS   10

static void mlx97_thread(void *a, void *b, void *c)
{
    ARG_UNUSED(a); ARG_UNUSED(b); ARG_UNUSED(c);

    (void)ble_mlx97_srv_init();

    if (!device_is_ready(mlx0.bus) || !device_is_ready(mlx1.bus)) {
        LOG_ERR("I2C bus not ready (mlx0=%s mlx1=%s)",
                mlx0.bus ? mlx0.bus->name : "NULL",
                mlx1.bus ? mlx1.bus->name : "NULL");
        return;
    }

    printk("MLX97: bus0=%s addr=0x%02x | bus1=%s addr=0x%02x\n",
           mlx0.bus->name, mlx0.addr, mlx1.bus->name, mlx1.addr);

    (void)mlx97_set_mode_xyz(&mlx0, MLX97_MODE_CONT_100HZ, true, true, true);
    (void)mlx97_set_mode_xyz(&mlx1, MLX97_MODE_CONT_100HZ, true, true, true);

    LOG_INF("MLX97 app thread started, waiting for CTRL start...");

    uint32_t cnt = 0;

    for (;;) {
        (void)ble_mlx97_wait_for_start(K_FOREVER);
        LOG_INF("MLX97 streaming started");

        while (ble_mlx97_is_running()) {
            struct mlx97_ble_sample p0 = {0};
            struct mlx97_ble_sample p1 = {0};

            int r0 = mlx97_read_sample(&mlx0, &p0, 50);
            int r1 = mlx97_read_sample(&mlx1, &p1, 50);

            if (r0 == 0) (void)ble_mlx97_notify0(&p0);
            if (r1 == 0) (void)ble_mlx97_notify1(&p1);

            if ((cnt++ % APP_PRINT_EVERY) == 0) {
                printk("0: x=%d y=%d z=%d st2=0x%02x | 1: x=%d y=%d z=%d st2=0x%02x\n",
                       p0.x, p0.y, p0.z, p0.stat2,
                       p1.x, p1.y, p1.z, p1.stat2);
            }

            k_sleep(K_MSEC(APP_PERIOD_MS));
        }

        LOG_INF("MLX97 streaming stopped");
    }
}

K_THREAD_DEFINE(mlx97_ble_adv_t, 1024, ble_adv_thread,
                (void *)&mlx97_profile, NULL, NULL,
                1, 0, 0);

K_THREAD_DEFINE(mlx97_t, 1024, mlx97_thread,
                NULL, NULL, NULL,
                2, 0, 0);
