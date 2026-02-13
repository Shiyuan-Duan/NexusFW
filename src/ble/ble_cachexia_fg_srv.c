#include <zephyr/kernel.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/device.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/sys/byteorder.h>
#include <string.h>
#include <errno.h>

#include "ble_cachexia_fg_srv.h"

/* Your DAC driver */
#include <driver/ad5778r.h>

LOG_MODULE_REGISTER(cachexia_fg_srv, CONFIG_BT_LOG_LEVEL);

static const struct device *const ad5778r_dev = DEVICE_DT_GET_ONE(dsy_ad5778r);

/* ---- State ---- */
static atomic_t g_switch = ATOMIC_INIT(0);
static atomic_t g_gen    = ATOMIC_INIT(0);

/* Cached mirror for READ (switch) */
static uint8_t sw_cached;

/* Pattern buffers */
static struct cachexia_fg_pattern active_pat;
static struct cachexia_fg_pattern edit_pat;
K_MUTEX_DEFINE(pat_lock);

/* ---- Helpers ---- */
static int16_t clamp_amp_ua(int32_t x)
{
	if (x >  (int32_t)CACHEXIA_FG_MAX_ABS_UA) return  (int16_t)CACHEXIA_FG_MAX_ABS_UA;
	if (x < -(int32_t)CACHEXIA_FG_MAX_ABS_UA) return -(int16_t)CACHEXIA_FG_MAX_ABS_UA;
	return (int16_t)x;
}

static ssize_t read_u8_cached(struct bt_conn *conn, const struct bt_gatt_attr *attr,
			      void *buf, uint16_t len, uint16_t offset)
{
	const uint8_t *src = attr->user_data;
	return bt_gatt_attr_read(conn, attr, buf, len, offset, src, 1);
}

/* INFO: [0]=count, [1]=max, [2..5]=gen_le */
static ssize_t read_info(struct bt_conn *conn, const struct bt_gatt_attr *attr,
			 void *buf, uint16_t len, uint16_t offset)
{
	ARG_UNUSED(attr);

	uint8_t out[6];

	k_mutex_lock(&pat_lock, K_FOREVER);
	out[0] = active_pat.count;
	k_mutex_unlock(&pat_lock);

	out[1] = CACHEXIA_FG_MAX_BLOCKS;

	uint32_t gen = (uint32_t)atomic_get(&g_gen);
	sys_put_le32(gen, &out[2]);

	return bt_gatt_attr_read(conn, attr, buf, len, offset, out, sizeof(out));
}

/* ---- Write: SWITCH ---- */
static ssize_t write_switch(struct bt_conn *conn, const struct bt_gatt_attr *attr,
			    const void *buf, uint16_t len, uint16_t offset, uint8_t flags)
{
	ARG_UNUSED(conn);
	ARG_UNUSED(attr);
	ARG_UNUSED(flags);

	if (len != 1 || offset) {
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
	}

	uint8_t v = ((const uint8_t *)buf)[0] ? 1 : 0;
	atomic_set(&g_switch, v);
	sw_cached = v;

	/* Immediate safe-off on stop (same safety logic as your previous app) */
	if (v == 0 && device_is_ready(ad5778r_dev)) {
		(void)ad5778r_set_span(ad5778r_dev, 0 /*ch*/, AD5778R_SPAN_HIGH_Z);
		(void)ad5778r_power_down(ad5778r_dev, 0 /*ch*/);
	}

	return len;
}

/* ---- Write: CLEAR (edit buffer) ---- */
static ssize_t write_clear(struct bt_conn *conn, const struct bt_gatt_attr *attr,
			   const void *buf, uint16_t len, uint16_t offset, uint8_t flags)
{
	ARG_UNUSED(conn);
	ARG_UNUSED(attr);
	ARG_UNUSED(flags);

	if (len != 1 || offset) {
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
	}

	uint8_t v = ((const uint8_t *)buf)[0];
	if (v == 0) {
		return len;
	}

	k_mutex_lock(&pat_lock, K_FOREVER);
	memset(&edit_pat, 0, sizeof(edit_pat));
	k_mutex_unlock(&pat_lock);

	return len;
}

/* ---- Write: APPEND block into edit buffer ---- */
static ssize_t write_append(struct bt_conn *conn, const struct bt_gatt_attr *attr,
			    const void *buf, uint16_t len, uint16_t offset, uint8_t flags)
{
	ARG_UNUSED(conn);
	ARG_UNUSED(attr);
	ARG_UNUSED(flags);

	if (len != CACHEXIA_FG_BLOCK_WIRE_LEN || offset) {
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
	}

	const uint8_t *p = (const uint8_t *)buf;
	int16_t amp_ua = (int16_t)sys_get_le16(&p[0]);
	uint32_t dur_ms = sys_get_le32(&p[2]);
	uint8_t bflags = p[6];

	amp_ua = clamp_amp_ua((int32_t)amp_ua);

	k_mutex_lock(&pat_lock, K_FOREVER);

	if (edit_pat.count >= CACHEXIA_FG_MAX_BLOCKS) {
		k_mutex_unlock(&pat_lock);
		return BT_GATT_ERR(BT_ATT_ERR_INSUFFICIENT_RESOURCES);
	}

	struct cachexia_fg_block *b = &edit_pat.blocks[edit_pat.count];
	b->amp_ua = amp_ua;
	b->dur_ms = dur_ms;
	b->flags  = bflags;

	edit_pat.count++;

	k_mutex_unlock(&pat_lock);

	return len;
}

/* ---- Write: COMMIT edit -> active ---- */
static ssize_t write_commit(struct bt_conn *conn, const struct bt_gatt_attr *attr,
			    const void *buf, uint16_t len, uint16_t offset, uint8_t flags)
{
	ARG_UNUSED(conn);
	ARG_UNUSED(attr);
	ARG_UNUSED(flags);

	if (len != 1 || offset) {
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
	}

	uint8_t v = ((const uint8_t *)buf)[0];
	if (v == 0) {
		return len;
	}

	k_mutex_lock(&pat_lock, K_FOREVER);
	active_pat = edit_pat;
	k_mutex_unlock(&pat_lock);

	(void)atomic_inc(&g_gen);

	return len;
}

/* ---- GATT table ---- */
BT_GATT_SERVICE_DEFINE(cachexia_fg_svc,
	BT_GATT_PRIMARY_SERVICE(BT_UUID_CACHEXIA_FG_SVC),

	BT_GATT_CHARACTERISTIC(BT_UUID_CACHEXIA_FG_SWITCH,
		BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE,
		BT_GATT_PERM_READ | BT_GATT_PERM_WRITE,
		read_u8_cached, write_switch, &sw_cached),

	BT_GATT_CHARACTERISTIC(BT_UUID_CACHEXIA_FG_CLEAR,
		BT_GATT_CHRC_WRITE,
		BT_GATT_PERM_WRITE,
		NULL, write_clear, NULL),

	BT_GATT_CHARACTERISTIC(BT_UUID_CACHEXIA_FG_APPEND,
		BT_GATT_CHRC_WRITE,
		BT_GATT_PERM_WRITE,
		NULL, write_append, NULL),

	BT_GATT_CHARACTERISTIC(BT_UUID_CACHEXIA_FG_COMMIT,
		BT_GATT_CHRC_WRITE,
		BT_GATT_PERM_WRITE,
		NULL, write_commit, NULL),

	BT_GATT_CHARACTERISTIC(BT_UUID_CACHEXIA_FG_INFO,
		BT_GATT_CHRC_READ,
		BT_GATT_PERM_READ,
		read_info, NULL, NULL),
);

int ble_cachexia_fg_srv_init(void)
{
	/* Defaults */
	atomic_clear(&g_switch);
	sw_cached = 0;

	k_mutex_lock(&pat_lock, K_FOREVER);
	memset(&active_pat, 0, sizeof(active_pat));
	memset(&edit_pat, 0, sizeof(edit_pat));
	k_mutex_unlock(&pat_lock);

	atomic_clear(&g_gen);

	/* Ensure output safely off at boot */
	if (device_is_ready(ad5778r_dev)) {
		(void)ad5778r_set_span(ad5778r_dev, 0 /*ch*/, AD5778R_SPAN_HIGH_Z);
		(void)ad5778r_power_down(ad5778r_dev, 0 /*ch*/);
	}

	LOG_INF("Cachexia FG BLE service ready");
	return 0;
}

uint8_t ble_cachexia_fg_get_switch(void)
{
	return (uint8_t)atomic_get(&g_switch);
}

uint32_t ble_cachexia_fg_get_pattern_gen(void)
{
	return (uint32_t)atomic_get(&g_gen);
}

int ble_cachexia_fg_copy_active_pattern(struct cachexia_fg_pattern *out)
{
	if (!out) {
		return -EINVAL;
	}

	k_mutex_lock(&pat_lock, K_FOREVER);
	*out = active_pat;
	k_mutex_unlock(&pat_lock);

	return 0;
}
