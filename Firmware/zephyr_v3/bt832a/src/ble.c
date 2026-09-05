#include "bio.h"
#include "transport.h"
#include <string.h>
#include <stdio.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(bio_ble, LOG_LEVEL_INF);
#define UUID_BYTES(n) BT_UUID_128_ENCODE(((n) << 16) | 0xcafe, 0xb0ba, 0x8bad, 0xf00d, 0xdeadbeef0000)
#define UUID_VAR(name, n) static const struct bt_uuid_128 name = BT_UUID_INIT_128(UUID_BYTES(n))
UUID_VAR(service_uuid, 0);
UUID_VAR(ads0_uuid, 2); UUID_VAR(max_uuid, 3); UUID_VAR(mpu_uuid, 4);
UUID_VAR(ads1_uuid, 5); UUID_VAR(bme_uuid, 6); UUID_VAR(qmc_uuid, 8);
UUID_VAR(control_uuid, 9); UUID_VAR(beacon_uuid, 10);
UUID_VAR(text_uuid, 11); UUID_VAR(reply_uuid, 12); UUID_VAR(stream_uuid, 13);
static struct bt_conn *connection;
K_MUTEX_DEFINE(connection_lock);
K_MUTEX_DEFINE(reply_lock);
static atomic_t initialized, dropped, next_sequence, scanning;
static struct bio_link link;
#define INITIAL_REPLY "ready; write help to text characteristic"
static char last_reply[BIO_PACKET_MAX] = INITIAL_REPLY;
struct tx_packet { uint32_t session; uint16_t len, seq; uint8_t sensor; uint8_t data[BIO_PACKET_MAX]; };
K_MSGQ_DEFINE(data_queue, sizeof(struct tx_packet), CONFIG_BIO_TX_DEPTH, 4);
K_MSGQ_DEFINE(reply_queue, sizeof(struct tx_packet), CONFIG_BIO_REPLY_DEPTH, 4);
K_SEM_DEFINE(tx_wake, 0, 1);

static ssize_t write_command(struct bt_conn *conn, const struct bt_gatt_attr *attr,
	const void *buf, uint16_t len, uint16_t offset, uint8_t flags)
{
	if (offset) { return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET); }
	if (flags & BT_GATT_WRITE_FLAG_PREPARE) { return BT_GATT_ERR(BT_ATT_ERR_NOT_SUPPORTED); }
	bool binary = bt_uuid_cmp(attr->uuid, &control_uuid.uuid) == 0;
	if (!len || len >= BIO_COMMAND_MAX || (binary && len < 3)) { return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN); }
	k_mutex_lock(&connection_lock, K_FOREVER);
	uint32_t token = connection == conn ? bio_link_token(&link) : 0;
	k_mutex_unlock(&connection_lock);
	if (!token) { return BT_GATT_ERR(BT_ATT_ERR_UNLIKELY); }
	int err = bio_submit(BIO_ORIGIN_BLE, buf, len, binary, token);
	return err ? BT_GATT_ERR(BT_ATT_ERR_INSUFFICIENT_RESOURCES) : len;
}
static ssize_t read_reply(struct bt_conn *conn, const struct bt_gatt_attr *attr,
	void *buf, uint16_t len, uint16_t offset)
{
	k_mutex_lock(&reply_lock, K_FOREVER);
	ssize_t ret = bt_gatt_attr_read(conn, attr, buf, len, offset, last_reply, strlen(last_reply));
	k_mutex_unlock(&reply_lock);
	return ret;
}
#define DATA_CHARACTERISTIC(id) \
	BT_GATT_CHARACTERISTIC(&(id).uuid, BT_GATT_CHRC_NOTIFY, BT_GATT_PERM_NONE, NULL, NULL, NULL), \
	BT_GATT_CCC(NULL, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE)
BT_GATT_SERVICE_DEFINE(bio_service,
	BT_GATT_PRIMARY_SERVICE(&service_uuid),
	BT_GATT_CHARACTERISTIC(&control_uuid.uuid, BT_GATT_CHRC_WRITE, BT_GATT_PERM_WRITE, NULL, write_command, NULL),
	DATA_CHARACTERISTIC(ads0_uuid),
#if defined(CONFIG_BIO_ADS_SECOND)
	DATA_CHARACTERISTIC(ads1_uuid),
#endif
#if defined(CONFIG_BIO_I2C_SENSORS)
	DATA_CHARACTERISTIC(max_uuid), DATA_CHARACTERISTIC(mpu_uuid),
	DATA_CHARACTERISTIC(bme_uuid), DATA_CHARACTERISTIC(qmc_uuid),
#endif
#if defined(CONFIG_BIO_BEACONS)
	DATA_CHARACTERISTIC(beacon_uuid),
#endif
	BT_GATT_CHARACTERISTIC(&text_uuid.uuid, BT_GATT_CHRC_WRITE, BT_GATT_PERM_WRITE, NULL, write_command, NULL),
	BT_GATT_CHARACTERISTIC(&reply_uuid.uuid, BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY, BT_GATT_PERM_READ, read_reply, NULL, NULL),
	BT_GATT_CCC(NULL, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
	DATA_CHARACTERISTIC(stream_uuid)
);
static const struct bt_gatt_attr *attribute(const struct bt_uuid_128 *uuid)
{
	return bt_gatt_find_by_uuid(bio_service.attrs, bio_service.attr_count, &uuid->uuid);
}
static const struct bt_uuid_128 *legacy_uuid(unsigned sensor)
{
	switch (sensor) {
	case BIO_ADS0: return &ads0_uuid; case BIO_ADS1: return &ads1_uuid;
	case BIO_MAX: return &max_uuid; case BIO_MPU: return &mpu_uuid;
	case BIO_BME: return &bme_uuid; case BIO_QMC: return &qmc_uuid;
	case BIO_BEACON: return &beacon_uuid; default: return NULL;
	}
}
static struct bt_conn *get_connection(uint32_t token)
{
	k_mutex_lock(&connection_lock, K_FOREVER);
	struct bt_conn *conn = connection && bio_link_accepts(&link, token) ? bt_conn_ref(connection) : NULL;
	k_mutex_unlock(&connection_lock);
	return conn;
}
static int notify(struct bt_conn *conn, const struct bt_gatt_attr *attr, const uint8_t *data, size_t len, uint32_t token)
{
	int err = -ENOTCONN;
	for (unsigned retry = 0; retry < 3 && token == bio_ble_session(); retry++) {
		err = bt_gatt_notify(conn, attr, data, len);
		if (err != -ENOMEM && err != -EAGAIN) { break; }
		k_msleep(5);
	}
	return err;
}
static void send_packet(struct tx_packet *p)
{
	struct bt_conn *conn = get_connection(p->session);
	if (!conn) { return; }
	size_t limit = bt_gatt_get_mtu(conn) - 3;
	const struct bt_uuid_128 *legacy = legacy_uuid(p->sensor);
	if (legacy) {
		const struct bt_gatt_attr *attr = attribute(legacy);
		if (attr && bt_gatt_is_subscribed(conn, attr, BT_GATT_CCC_NOTIFY)) {
			// A legacy packet is atomic: never truncate or silently split it.
			if (p->len > limit || notify(conn, attr, p->data, p->len, p->session)) { atomic_inc(&dropped); }
		}
	}
	const struct bt_gatt_attr *attr = attribute(p->sensor == BIO_RESPONSE ? &reply_uuid : &stream_uuid);
	if (bt_gatt_is_subscribed(conn, attr, BT_GATT_CCC_NOTIFY)) {
		uint8_t frame[BIO_PACKET_MAX + BIO_FRAME_HEADER];
		size_t offset = 0;
		while (offset < p->len) {
			size_t len = bio_frame(frame, MIN(limit, sizeof(frame)), p->sensor, p->seq, p->data, p->len, offset);
			if (!len || notify(conn, attr, frame, len, p->session)) { atomic_inc(&dropped); break; }
			offset += len - BIO_FRAME_HEADER;
		}
	}
	bt_conn_unref(conn);
}
static void transmit(void *a, void *b, void *c)
{
	struct tx_packet p;
	while (true) {
		k_sem_take(&tx_wake, K_FOREVER);
		while (!k_msgq_get(&reply_queue, &p, K_NO_WAIT) || !k_msgq_get(&data_queue, &p, K_NO_WAIT)) { send_packet(&p); }
	}
}
K_THREAD_DEFINE(ble_tx_thread, 1536, transmit, NULL, NULL, NULL, 6, 0, 0);
static int enqueue(uint8_t sensor, const void *data, size_t len, uint32_t token)
{
	if (!atomic_get(&initialized)) { return -ENOTCONN; }
	if (!data || !len || len > BIO_PACKET_MAX) { return -EMSGSIZE; }
	struct bt_conn *conn = get_connection(token);
	if (!conn) { return -ENOTCONN; }
	bt_conn_unref(conn);
	struct tx_packet p = { .session = token, .sensor = sensor, .len = len, .seq = atomic_inc(&next_sequence) };
	memcpy(p.data, data, len);
	int err = k_msgq_put(sensor == BIO_RESPONSE ? &reply_queue : &data_queue, &p, K_NO_WAIT);
	if (err) { atomic_inc(&dropped); }
	else { k_sem_give(&tx_wake); }
	return err;
}
int bio_ble_send(uint8_t sensor, const void *data, size_t len) { return enqueue(sensor, data, len, bio_ble_session()); }
void bio_ble_reply(const char *reply, uint32_t token)
{
	/* Lock order is always connection -> reply. Disconnect cannot expose an old reply. */
	k_mutex_lock(&connection_lock, K_FOREVER);
	if (!bio_link_accepts(&link, token)) {
		k_mutex_unlock(&connection_lock);
		return;
	}
	k_mutex_lock(&reply_lock, K_FOREVER);
	snprintf(last_reply, sizeof(last_reply), "%s", reply);
	k_mutex_unlock(&reply_lock);
	k_mutex_unlock(&connection_lock);
	(void)enqueue(BIO_RESPONSE, reply, strlen(reply), token);
}
uint32_t bio_ble_session(void)
{
	k_mutex_lock(&connection_lock, K_FOREVER);
	uint32_t token = bio_link_token(&link);
	k_mutex_unlock(&connection_lock);
	return token;
}
unsigned bio_ble_dropped(void) { return atomic_get(&dropped); }
bool bio_ble_ready(void) { return atomic_get(&initialized) != 0; }
bool bio_ble_scanning(void) { return atomic_get(&scanning) != 0; }
static const struct bt_data advertising[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR),
	BT_DATA_BYTES(BT_DATA_UUID128_ALL, UUID_BYTES(0)),
};
static const struct bt_data scan_response[] = { BT_DATA(BT_DATA_NAME_COMPLETE, CONFIG_BT_DEVICE_NAME, sizeof(CONFIG_BT_DEVICE_NAME) - 1) };
static void advertise(struct k_work *work);
K_WORK_DELAYABLE_DEFINE(advertise_work, advertise);
static void advertise(struct k_work *work)
{
	int err = bt_le_adv_start(BT_LE_ADV_CONN_FAST_1, advertising, ARRAY_SIZE(advertising), scan_response, ARRAY_SIZE(scan_response));
	if (err && err != -EALREADY) { LOG_WRN("Advertising: %d; retrying", err); k_work_reschedule(&advertise_work, K_SECONDS(1)); }
}
static void connected(struct bt_conn *conn, uint8_t err)
{
	if (err) { k_work_reschedule(&advertise_work, K_MSEC(100)); return; }
	k_mutex_lock(&connection_lock, K_FOREVER);
	if (!connection) {
		connection = bt_conn_ref(conn);
		bio_link_set(&link, true);
		k_mutex_lock(&reply_lock, K_FOREVER);
		strcpy(last_reply, INITIAL_REPLY);
		k_mutex_unlock(&reply_lock);
	}
	k_mutex_unlock(&connection_lock);
	LOG_INF("BLE connected");
}
static void disconnected(struct bt_conn *conn, uint8_t reason)
{
	k_mutex_lock(&connection_lock, K_FOREVER);
	if (connection == conn) {
		bt_conn_unref(connection);
		connection = NULL;
		bio_link_set(&link, false);
		k_mutex_lock(&reply_lock, K_FOREVER);
		strcpy(last_reply, INITIAL_REPLY);
		k_mutex_unlock(&reply_lock);
		k_msgq_purge(&data_queue);
		k_msgq_purge(&reply_queue);
	}
	k_mutex_unlock(&connection_lock);
	LOG_INF("BLE disconnected: %u", reason);
	k_work_reschedule(&advertise_work, K_MSEC(100));
}
BT_CONN_CB_DEFINE(connection_callbacks) = { .connected = connected, .disconnected = disconnected };
int bio_ble_init(void)
{
	int err = bt_enable(NULL);
	if (!err) { atomic_set(&initialized, 1); k_work_reschedule(&advertise_work, K_NO_WAIT); }
	return err;
}
#if defined(CONFIG_BIO_BEACONS)
struct beacon_context { const bt_addr_le_t *addr; int8_t rssi; };
static bool beacon_data(struct bt_data *data, void *user)
{
	struct beacon_context *ctx = user;
	// Apple manufacturer ID, iBeacon type and length, all checked before access.
	if (data->type == BT_DATA_MANUFACTURER_DATA && data->data_len == 25 &&
	    data->data[0] == 0x4c && data->data[1] == 0 && data->data[2] == 2 && data->data[3] == 0x15) {
		uint8_t report[28];
		memcpy(report, ctx->addr->a.val, 6); report[6] = ctx->rssi;
		memcpy(report + 7, data->data + 4, 21);
		bio_publish(BIO_BEACON, report, sizeof(report));
	}
	return true;
}
static void scan_report(const bt_addr_le_t *addr, int8_t rssi, uint8_t type, struct net_buf_simple *ad)
{
	struct beacon_context context = { .addr = addr, .rssi = rssi };
	bt_data_parse(ad, beacon_data, &context);
}
#endif
int bio_ble_scan(bool enable)
{
#if defined(CONFIG_BIO_BEACONS)
	if (!atomic_get(&initialized)) { return -EAGAIN; }
	int err = enable ? bt_le_scan_start(BT_LE_SCAN_PASSIVE, scan_report) : bt_le_scan_stop();
	if (!err || err == -EALREADY) { atomic_set(&scanning, enable); }
	return err == -EALREADY ? 0 : err;
#else
	return -ENOTSUP;
#endif
}
