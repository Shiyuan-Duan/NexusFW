/* Zephyr driver for AD5941 — datasheet-accurate SPI (2-transaction) + Table 14 init
 * + debug snapshot printing right before/after the SINC2DAT read
 *
 * 外部 API 名称保持不变：
 *  - ad5941_reg_read / ad5941_reg_write
 *  - api_reset / api_set_vgs / api_lptia_cfg / api_read_current
 *
 * 变更点：
 *  1) SPI 两笔事务：SETADDR -> READ/WRITE（CS 中间抬高）
 *  2) READ 修正：READREG 后第一个返回字节是 STATUS（0x80），随后才是数据
 *  3) 上电 Table 14 初始化
 *  4) 明确 16/32bit 访问；CHIPID 等 16 位专用分支
 *  5) Sinc2 路径读数前轮询 INTCFLAG1.FLAG2
 *  6) 在读取前后 printk 关键寄存器快照（用于定位问题）
 */

#define DT_DRV_COMPAT dsy_ad5941

#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>
LOG_MODULE_REGISTER(ad5941, CONFIG_LOG_DEFAULT_LEVEL);

/* ---------------- Register map (subset) ---------------- */
#define REG_ADIID            0x0400u   /* 16-bit, expect 0x4144 ("AD") */
#define REG_CHIPID           0x0404u   /* 16-bit: [15:4] Part=0x550, [3:0] Rev */
#define REG_AFECON           0x2000u   /* ... SINC2EN(16) ADCCONVEN(8) ADCEN(7) DACBUFEN(21) */
#define REG_ADCFILTERCON     0x2044u
#define REG_ADCDAT           0x2074u
#define REG_SINC2DAT         0x2080u
#define REG_LPREFBUFCON      0x2050u
#define REG_LPTIASW0         0x20E4u
#define REG_LPTIACON0        0x20ECu
#define REG_ADCCON           0x21A8u
#define REG_REPEATADCCNV     0x21F0u
#define REG_LPDACDAT0        0x2120u   /* [17:12] VZERO(6b), [11:0] VBIAS(12b) */
#define REG_LPDACSW0         0x2124u
#define REG_LPDACCON0        0x2128u
#define REG_ADCBUFCON        0x238Cu

/* Table 14 (must-write after reset) */
#define REG_KEY              0x0A04u
#define REG_SWRESET          0x0A00u
#define REG_CLKCON           0x0A28u
#define REG_DIGIFILTCON      0x0410u
#define REG_ANA_CAL1         0x0908u
#define REG_ANA_CAL2         0x0C08u
#define REG_PMBW             0x22F0u

/* ---------- Interrupt controller (used for ready flags) ---------- */
#define REG_INTCCLR          0x3004u   /* Interrupt clear (Write-1-to-Clear) */
#define REG_INTCFLAG1        0x3014u   /* Interrupt flags (block 1) */
#define INTCFLAG_SINC2_RDY   BIT(2)    /* FLAG2: Sinc2 filter result ready */

/* Bits */
#define AFECON_DACBUFEN      BIT(21)
#define AFECON_ADCCONVEN     BIT(8)
#define AFECON_ADCEN         BIT(7)
#define AFECON_SINC2EN       BIT(16)   /* 50/60 Hz notch (Sinc2) enable */

#define ADCFILTERCON_LPFBYPEN BIT(4)   /* 0 => write notch result to SINC2DAT */

/* ADC MUX codes */
#define ADCCON_MUXP_LPTIA_P  (0b100001)   /* [5:0] */
#define ADCCON_MUXN_LPTIA_N  (0b00010)    /* [12:8] */
#define ADCCON_GNPGA_1X      (0u << 16)

/* LPDAC constants (LP ref path) */
#define LPDAC_VZERO_LSB_uV   34375    /* 34.375 mV/LSB */
#define LPDAC_VBIAS_LSB_uV   537      /* 0.537 mV/LSB */
#define LPDAC_MIN_mV         200
#define LPDAC_MAX_VBIAS_mV   2400
#define LPDAC_MAX_VZERO_mV   2366

/* ADC reference */
#define ADC_INT_VREF_mV      1820

#define CONFIG_AD5941_INIT_PRIORITY 90

/* --------- SPI command opcodes (per datasheet) --------- */
#define AD5941_CMD_SETADDR   0x20
#define AD5941_CMD_WRITEREG  0x2D
#define AD5941_CMD_READREG   0x6D

/* --------- Device config/data --------- */
struct ad5941_config {
    struct spi_dt_spec  spi;
    struct gpio_dt_spec reset;
    struct gpio_dt_spec intp;
};
struct ad5941_data {
    struct k_mutex lock;
    int32_t vgs_mv;
    int32_t vzero_mv;
    uint32_t rtia_ohm;
};

/* ---------------- SPI helpers (datasheet-accurate) ---------------- */
#define AD5941_SPI_FLAGS (SPI_WORD_SET(8) | SPI_TRANSFER_MSB )

static inline int ad5941_spi_setaddr(const struct device *dev, uint16_t reg)
{
    const struct ad5941_config *cfg = dev->config;
    uint8_t tx[3] = { AD5941_CMD_SETADDR, (uint8_t)(reg >> 8), (uint8_t)reg };
    struct spi_buf buf = { .buf = tx, .len = sizeof(tx) };
    struct spi_buf_set txs = { .buffers = &buf, .count = 1 };
    return spi_write_dt(&cfg->spi, &txs);   /* CS toggles between transactions */
}

/* 16-bit write */
static int ad5941_reg_write16_exact(const struct device *dev, uint16_t reg, uint16_t val)
{
    const struct ad5941_config *cfg = dev->config;
    int ret = ad5941_spi_setaddr(dev, reg);
    if (ret) return ret;
    uint8_t tx[3] = { AD5941_CMD_WRITEREG, (uint8_t)(val >> 8), (uint8_t)val };
    struct spi_buf buf = { .buf = tx, .len = sizeof(tx) };
    struct spi_buf_set txs = { .buffers = &buf, .count = 1 };
    return spi_write_dt(&cfg->spi, &txs);
}

/* 32-bit write */
static int ad5941_reg_write32_exact(const struct device *dev, uint16_t reg, uint32_t val)
{
    const struct ad5941_config *cfg = dev->config;
    int ret = ad5941_spi_setaddr(dev, reg);
    if (ret) return ret;
    uint8_t tx[5] = { AD5941_CMD_WRITEREG,
                      (uint8_t)(val >> 24), (uint8_t)(val >> 16),
                      (uint8_t)(val >> 8), (uint8_t)val };
    struct spi_buf buf = { .buf = tx, .len = sizeof(tx) };
    struct spi_buf_set txs = { .buffers = &buf, .count = 1 };
    return spi_write_dt(&cfg->spi, &txs);
}

/* 16-bit read — FIXED: include STATUS byte after opcode, then data */
static int ad5941_reg_read16_exact(const struct device *dev, uint16_t reg, uint16_t *out)
{
    const struct ad5941_config *cfg = dev->config;
    if (!out) return -EINVAL;
    int ret = ad5941_spi_setaddr(dev, reg);
    if (ret) return ret;

    /* Clock 1(opcode) + 1(status) + 2(data) = 4 bytes */
    uint8_t tx[4] = { AD5941_CMD_READREG, 0x00, 0x00, 0x00 };
    uint8_t rx[4] = {0};
    struct spi_buf bufs[2] = { { .buf = tx, .len = 4 }, { .buf = rx, .len = 4 } };
    struct spi_buf_set txs = { .buffers = &bufs[0], .count = 1 };
    struct spi_buf_set rxs = { .buffers = &bufs[1], .count = 1 };

    ret = spi_transceive_dt(&cfg->spi, &txs, &rxs);
    if (ret) return ret;

    /* rx[1]=STATUS (0x80), rx[2..3]=MSB..LSB */
    *out = ((uint16_t)rx[2] << 8) | rx[3];
    return 0;
}

/* 32-bit read — FIXED: include STATUS byte after opcode, then data */
static int ad5941_reg_read32_exact(const struct device *dev, uint16_t reg, uint32_t *out)
{
    const struct ad5941_config *cfg = dev->config;
    if (!out) return -EINVAL;
    int ret = ad5941_spi_setaddr(dev, reg);
    if (ret) return ret;

    /* Clock 1(opcode) + 1(status) + 4(data) = 6 bytes */
    uint8_t tx[6] = { AD5941_CMD_READREG, 0,0,0,0,0 };
    uint8_t rx[6] = {0};
    struct spi_buf bufs[2] = { { .buf = tx, .len = 6 }, { .buf = rx, .len = 6 } };
    struct spi_buf_set txs = { .buffers = &bufs[0], .count = 1 };
    struct spi_buf_set rxs = { .buffers = &bufs[1], .count = 1 };

    ret = spi_transceive_dt(&cfg->spi, &txs, &rxs);
    if (ret) return ret;

    /* rx[1]=STATUS (0x80), rx[2..5]=MSB..LSB */
    *out = ((uint32_t)rx[2] << 24) | ((uint32_t)rx[3] << 16) | ((uint32_t)rx[4] << 8) | rx[5];
    return 0;
}

/* -------- Compatibility wrappers -------- */
static inline int ad5941_reg_write(const struct device *dev, uint16_t reg, uint32_t val)
{
    switch (reg) {
        case REG_ADIID:
        case REG_CHIPID:
        case REG_DIGIFILTCON:
        case REG_REPEATADCCNV:
        case REG_CLKCON:
        case REG_ANA_CAL1:
        case REG_ANA_CAL2:
        case REG_PMBW:
            return ad5941_reg_write16_exact(dev, reg, (uint16_t)val);
        default:
            return ad5941_reg_write32_exact(dev, reg, val);
    }
}
static inline int ad5941_reg_read(const struct device *dev, uint16_t reg, uint32_t *val)
{
    switch (reg) {
        case REG_ADIID:
        case REG_CHIPID:
        case REG_DIGIFILTCON:
        case REG_REPEATADCCNV:
        case REG_CLKCON:
        case REG_ANA_CAL1:
        case REG_ANA_CAL2:
        case REG_PMBW: {
            uint16_t v16; int r = ad5941_reg_read16_exact(dev, reg, &v16);
            if (!r && val) *val = v16;
            return r;
        }
        default:
            return ad5941_reg_read32_exact(dev, reg, val);
    }
}

static int reg_update(const struct device *dev, uint16_t reg, uint32_t mask, uint32_t val)
{
    uint32_t cur; int ret = ad5941_reg_read(dev, reg, &cur); if (ret) return ret;
    cur = (cur & ~mask) | (val & mask);
    return ad5941_reg_write(dev, reg, cur);
}

/* ---- Debug: one-shot dump of the key registers we need ---- */
static void ad5941_dbg_dump(const struct device *dev, const char *tag)
{
    uint32_t v; int r;
#define RD(reg) do { \
    v = 0xDEADBEEF; r = ad5941_reg_read(dev, (reg), &v); \
    printk("AD5941[%s] %s(0x%04X)=0x%08X %s\n", \
           tag, #reg, (unsigned)(reg), (unsigned)v, r ? "(ERR)" : ""); \
} while (0)
    RD(REG_ADIID);        /* 0x0400 */
    RD(REG_CHIPID);       /* 0x0404 */
    RD(REG_AFECON);       /* 0x2000 */
    RD(REG_ADCFILTERCON); /* 0x2044 */
    RD(REG_ADCCON);       /* 0x21A8 */
    RD(REG_REPEATADCCNV); /* 0x21F0 */
    RD(REG_INTCFLAG1);    /* 0x3014 */
    RD(REG_SINC2DAT);     /* 0x2080 */
    RD(REG_ADCDAT);       /* 0x2074 */
#undef RD
}

/* ---------------- Table 14: must-run power-on init ---------------- */
static int ad5941_power_on_init(const struct device *dev)
{
    int ret = 0;
    ret |= ad5941_reg_write(dev, REG_KEY, 0x4859u);
    ret |= ad5941_reg_write(dev, REG_KEY, 0xF27Bu);
    k_sleep(K_USEC(20));

    ret |= ad5941_reg_write(dev, REG_SWRESET, 0x8009u);
    k_sleep(K_USEC(20));

    ret |= ad5941_reg_write(dev, REG_ANA_CAL1, 0x02C9u);
    ret |= ad5941_reg_write(dev, REG_ANA_CAL2, 0x206Cu);
    ret |= ad5941_reg_write(dev, REG_DIGIFILTCON, 0x02C9u);
    ret |= ad5941_reg_write(dev, REG_CLKCON,      0x0009u);
    ret |= ad5941_reg_write(dev, REG_ADCBUFCON,   0x0104u);
    ret |= ad5941_reg_write(dev, REG_REPEATADCCNV,0x0010u);
    ret |= ad5941_reg_write(dev, REG_PMBW,        0x0000u);
    return ret ? -EIO : 0;
}

/* ---------------- LPDAC / LPTIA / ADC blocks ---------------- */
static int lprefs_enable(const struct device *dev)
{
    /* Enable low-power 2.5 V reference + buffer (bits cleared = enabled). */
    return ad5941_reg_write(dev, REG_LPREFBUFCON, 0x00000000u);
}
static int lpdac_cfg_transfer(const struct device *dev)
{
    /* REFSEL=0 (LP 2.5V), VBIASMUX=0 (12b->VBIAS0), VZEROMUX=0 (6b->VZERO0),
       PWDEN=0 (on), RSTEN=1 (write enable). */
    return ad5941_reg_write(dev, REG_LPDACCON0, 0x00000001u);
}
static int lpdac_route_transfer(const struct device *dev)
{
    /* LPMODEDIS(5)=1, SW4|SW3|SW2|SW1=1, SW0=0  */
    uint32_t sw = BIT(5) | BIT(4) | BIT(3) | BIT(2) | BIT(1);
    return ad5941_reg_write(dev, REG_LPDACSW0, sw);
}
static int lptia_cfg_transfer(const struct device *dev, uint32_t rtia_ohm)
{
    (void)ad5941_reg_write(dev, REG_LPTIASW0, 0x0000302Cu);

    uint32_t tiarf = (0b111u << 13);  /* LPF 1M */
    uint32_t tiarl = (0b000u << 10);  /* RLOAD=0 */
    uint32_t tiagain;
    switch (rtia_ohm) {
        case 200:    tiagain = (0b00001u << 5); break;
        case 1000:   tiagain = (0b00010u << 5); break;
        case 10000:  tiagain = (0b00011u << 5); break;
        case 20000:  tiagain = (0b00100u << 5); break;
        case 40000:  tiagain = (0b00101u << 5); break;
        default:     tiagain = (0b00011u << 5); rtia_ohm = 10000; break;
    }
    uint32_t lptia = tiarf | tiarl | tiagain; /* power bits 0 = on */
    return ad5941_reg_write(dev, REG_LPTIACON0, lptia);
}
static int adc_cfg_lptia(const struct device *dev)
{
    (void)ad5941_reg_write(dev, REG_ADCBUFCON, 0x005F3D04u);
    uint32_t adccfg = ADCCON_GNPGA_1X
                    | ((uint32_t)ADCCON_MUXN_LPTIA_N << 8)
                    | ((uint32_t)ADCCON_MUXP_LPTIA_P);
    (void)ad5941_reg_write(dev, REG_ADCCON, adccfg);

    /* Ensure notch path writes to SINC2DAT: LPFBYPEN=0 */
    uint32_t fcfg = 0;
    (void)ad5941_reg_read(dev, REG_ADCFILTERCON, &fcfg);
    fcfg &= ~ADCFILTERCON_LPFBYPEN;
    (void)ad5941_reg_write(dev, REG_ADCFILTERCON, fcfg);
    return 0;
}

/* ---------------- Public API ---------------- */
static int api_reset(const struct device *dev)
{
    const struct ad5941_config *cfg = dev->config;
    if (cfg->reset.port) {
        /* robust pulse: inactive -> assert -> deassert */
        gpio_pin_configure_dt(&cfg->reset, GPIO_OUTPUT_INACTIVE);
        k_sleep(K_MSEC(1));
        gpio_pin_set_dt(&cfg->reset, 1);
        k_sleep(K_MSEC(2));
        gpio_pin_set_dt(&cfg->reset, 0);
        k_sleep(K_MSEC(5));
    }
    return 0;
}

/* Utilities */
static inline uint32_t clamp_u32(uint32_t v, uint32_t lo, uint32_t hi){ return v<lo?lo:(v>hi?hi:v); }
static inline uint8_t  vzero_code_from_mV(uint32_t mV)
{
    mV = clamp_u32(mV, LPDAC_MIN_mV, LPDAC_MAX_VZERO_mV);
    uint32_t uV = mV*1000u - (LPDAC_MIN_mV*1000u);
    uint32_t code = (uV + (LPDAC_VZERO_LSB_uV/2)) / LPDAC_VZERO_LSB_uV;
    if (code > 0x3F) code = 0x3F;
    return (uint8_t)code;
}
static inline uint16_t vbias_code_from_mV(uint32_t mV)
{
    mV = clamp_u32(mV, LPDAC_MIN_mV, LPDAC_MAX_VBIAS_mV);
    uint32_t uV = mV*1000u - (LPDAC_MIN_mV*1000u);
    uint32_t code = (uV + (LPDAC_VBIAS_LSB_uV/2)) / LPDAC_VBIAS_LSB_uV;
    if (code > 0x0FFF) code = 0x0FFF;
    return (uint16_t)code;
}
static inline uint32_t lpdac_pack(uint8_t vzero6, uint16_t vbias12)
{
    return (((uint32_t)vzero6 & 0x3Fu) << 12) | (vbias12 & 0x0FFFu);
}

static int api_set_vgs(const struct device *dev, int32_t mv)
{
    struct ad5941_data *d = dev->data;
    k_mutex_lock(&d->lock, K_FOREVER);
    d->vgs_mv = mv;

    (void)lprefs_enable(dev);
    (void)lpdac_cfg_transfer(dev);
    (void)lpdac_route_transfer(dev);

    uint32_t vzero_mv = d->vzero_mv ? (uint32_t)d->vzero_mv : 1100u;
    uint8_t  vzero6   = vzero_code_from_mV(vzero_mv);
    uint16_t vbias12  = vbias_code_from_mV((uint32_t)mv);

    /* 若 VBIAS >= VZERO，可将 VBIAS 代码减 1 LSB 以补偿耦合（可选） */
    if (vbias12 >= ((uint16_t)vzero6 << 6)) {
        if (vbias12) vbias12 -= 1;
    }

    (void)ad5941_reg_write(dev, REG_LPDACDAT0, lpdac_pack(vzero6, vbias12));
    k_mutex_unlock(&d->lock);
    return 0;
}

static int api_lptia_cfg(const struct device *dev, int32_t vzero_mV, uint32_t rtia_ohm)
{
    struct ad5941_data *d = dev->data;
    k_mutex_lock(&d->lock, K_FOREVER);
    d->vzero_mv = vzero_mV; d->rtia_ohm = rtia_ohm;

    (void)lprefs_enable(dev);
    (void)lpdac_cfg_transfer(dev);
    (void)lpdac_route_transfer(dev);
    (void)lptia_cfg_transfer(dev, rtia_ohm);
    (void)adc_cfg_lptia(dev);

    uint32_t cur; (void)ad5941_reg_read(dev, REG_LPDACDAT0, &cur);
    uint8_t vzero6 = vzero_code_from_mV((uint32_t)vzero_mV);
    uint16_t vbias12 = (uint16_t)(cur & 0x0FFFu);
    (void)ad5941_reg_write(dev, REG_LPDACDAT0, lpdac_pack(vzero6, vbias12));
    k_mutex_unlock(&d->lock);
    return 0;
}

static int api_read_current(const struct device *dev, int32_t *nA)
{
    if (!nA) return -EINVAL;
    struct ad5941_data *d = dev->data;
    k_mutex_lock(&d->lock, K_FOREVER);

    /* Enable ADC + conversions + Sinc2 path */
    (void)reg_update(dev, REG_AFECON, AFECON_ADCEN,     AFECON_ADCEN);
    (void)reg_update(dev, REG_AFECON, AFECON_ADCCONVEN, AFECON_ADCCONVEN);
    (void)reg_update(dev, REG_AFECON, AFECON_SINC2EN,   AFECON_SINC2EN);

    /* Repeat conversions: EN_P=1, NUM=1 (NUM must be nonzero) */
    uint32_t rpt = (1u << 0) | (1u << 4);
    (void)ad5941_reg_write(dev, REG_REPEATADCCNV, rpt);

    /* Poll “Sinc2 filter result ready” (FLAG2) and clear with INTCCLR (W1C) */
    (void)ad5941_reg_write(dev, REG_INTCCLR, INTCFLAG_SINC2_RDY); /* clear stale */
    uint32_t flags = 0;
    for (uint32_t tries = 0; tries < 2000; ++tries) { /* ~40 ms safety timeout */
        (void)ad5941_reg_read(dev, REG_INTCFLAG1, &flags);
        if (flags & INTCFLAG_SINC2_RDY) break;
        k_busy_wait(20); /* ~20 us backoff */
    }
    /* Dump state right before the SINC2DAT read */
    ad5941_dbg_dump(dev, "BEFORE_READ");
    (void)ad5941_reg_write(dev, REG_INTCCLR, INTCFLAG_SINC2_RDY); /* ack */

    uint32_t code; (void)ad5941_reg_read(dev, REG_SINC2DAT, &code);
    uint16_t adc = (uint16_t)(code & 0xFFFFu);

    /* Also show what we actually read */
    printk("AD5941[AFTER_READ] SINC2DAT=0x%08X (adc16=0x%04X)\n",
           (unsigned)code, (unsigned)adc);
    uint32_t raw; (void)ad5941_reg_read(dev, REG_ADCDAT, &raw);
    printk("AD5941[AFTER_READ] ADCDAT  =0x%08X (raw16=0x%04X)\n",
           (unsigned)raw, (unsigned)(raw & 0xFFFF));

    int32_t vmV = (int32_t)((uint64_t)adc * ADC_INT_VREF_mV / 65535u);
    int32_t inA = (d->rtia_ohm ? (int32_t)((int64_t)vmV * 1000 / (int32_t)d->rtia_ohm) : 0); /* nA */
    *nA = inA;

    k_mutex_unlock(&d->lock);
    return 0;
}

/* ---------------- Boilerplate / init ---------------- */
struct ad5941_driver_api {
    int (*reset)(const struct device *dev);
    int (*reg_write)(const struct device *dev, uint16_t reg, uint32_t val);
    int (*reg_read)(const struct device *dev, uint16_t reg, uint32_t *val);
    int (*read_current_nA)(const struct device *dev, int32_t *nA);
    int (*set_vgs_mV)(const struct device *dev, int32_t mv);
    int (*lptia_config)(const struct device *dev, int32_t vzero_mV, uint32_t rtia_ohm);
};

static int api_reg_write(const struct device *dev, uint16_t reg, uint32_t val)
{ return ad5941_reg_write(dev, reg, val); }
static int api_reg_read (const struct device *dev, uint16_t reg, uint32_t *val)
{ return ad5941_reg_read(dev, reg, val); }

static int ad5941_init(const struct device *dev)
{
    const struct ad5941_config *cfg = dev->config;
    struct ad5941_data *data = dev->data;
    if (!device_is_ready(cfg->spi.bus)) return -ENODEV;
    if (cfg->reset.port && !device_is_ready(cfg->reset.port)) return -ENODEV;
    if (cfg->intp.port  && !device_is_ready(cfg->intp.port))  return -ENODEV;

    k_mutex_init(&data->lock);
    /* sane defaults so first read is not forced to 0 by rtia_ohm==0 */
    data->vzero_mv = 1100;
    data->rtia_ohm = 10000;

    api_reset(dev);

    /* 必须：Table 14 上电序列 */
    (void)ad5941_power_on_init(dev);

    /* 自检：读 ADIID / CHIPID （16-bit） */
    uint32_t v = 0;
    if (!ad5941_reg_read(dev, REG_ADIID, &v)) {
        printk("AD5941 ADIID=0x%04X\n", (uint16_t)v);
    }
    if (!ad5941_reg_read(dev, REG_CHIPID, &v)) {
        uint16_t chip = (uint16_t)v;
        printk("AD5941 CHIPID=0x%04X (part=0x%03X, rev=0x%X)\n",
               chip, (chip>>4)&0x0FFF, chip&0xF);
    } else {
        printk("AD5941: read CHIPID failed\n");
    }

    /* Dump once at init, too */
    ad5941_dbg_dump(dev, "INIT");

    return 0;
}

/* SPI config flags: Mode0 / 8-bit / MSB */
#define AD5941_CFG(inst) \
{ .spi = SPI_DT_SPEC_INST_GET(inst, AD5941_SPI_FLAGS, 0), \
  .reset = GPIO_DT_SPEC_INST_GET_OR(inst, reset_gpios, {0}), \
  .intp  = GPIO_DT_SPEC_INST_GET_OR(inst, int_gpios, {0}), }

static const struct ad5941_driver_api ad5941_api = {
    .reset = api_reset,
    .reg_write = api_reg_write,
    .reg_read = api_reg_read,
    .read_current_nA = api_read_current,
    .set_vgs_mV = api_set_vgs,
    .lptia_config = api_lptia_cfg,
};

#define AD5941_DEFINE(inst) \
  static struct ad5941_data ad5941_data_##inst; \
  static const struct ad5941_config ad5941_cfg_##inst = AD5941_CFG(inst); \
  DEVICE_DT_INST_DEFINE(inst, ad5941_init, NULL, \
    &ad5941_data_##inst, &ad5941_cfg_##inst, \
    POST_KERNEL, CONFIG_AD5941_INIT_PRIORITY, &ad5941_api);
DT_INST_FOREACH_STATUS_OKAY(AD5941_DEFINE)
