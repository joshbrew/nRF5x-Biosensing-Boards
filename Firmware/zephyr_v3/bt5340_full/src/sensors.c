#include "bio.h"
#include <string.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/byteorder.h>
LOG_MODULE_REGISTER(bio_sensors, LOG_LEVEL_INF);
static const struct device *const bus = DEVICE_DT_GET(DT_ALIAS(sensor_i2c));
static const struct device *const bme = DEVICE_DT_GET(DT_NODELABEL(bio_bme));
K_MUTEX_DEFINE(sensor_lock);
struct sensor_state { bool ready, running; uint8_t count, sequence, packet[244]; };
static struct sensor_state state[4]; // MPU, MAX, BME, QMC, matching IDs 4..7.
static uint8_t bme_id;
static uint8_t max_temp[2];
static int read_bytes(unsigned addr, unsigned reg, void *buf, size_t len) { return i2c_burst_read(bus, addr, reg, buf, len); }
static int read_byte(unsigned addr, unsigned reg, uint8_t *value) { return i2c_reg_read_byte(bus, addr, reg, value); }
static int write_byte(unsigned addr, unsigned reg, unsigned value) { return i2c_reg_write_byte(bus, addr, reg, value); }
static int registers(unsigned addr, const uint8_t (*values)[2], size_t count)
{
	for (size_t i = 0; i < count; i++) { int err = write_byte(addr, values[i][0], values[i][1]); if (err) { return err; } }
	return 0;
}
static int init_max(void)
{
	uint8_t id, mode;
	int err = read_byte(0x57, 0xff, &id);
	if (err || id != 0x15) { return err ? err : -ENODEV; }
	err = write_byte(0x57, 0x09, 0x40);
	if (err) { return err; }
	bool reset = false;
	for (unsigned i = 0; i < 20; i++) {
		k_msleep(5);
		err = read_byte(0x57, 0x09, &mode);
		if (err) { return err; }
		if (!(mode & 0x40)) { reset = true; break; }
	}
	if (!reset) { return -ETIMEDOUT; }
	// Red + IR, 100 SPS, 18-bit samples, low initial LED currents; idle in shutdown.
	static const uint8_t values[][2] = {{0x02,0},{0x03,0},{0x04,0},{0x05,0},{0x06,0},
		{0x08,0x10},{0x0a,0x27},{0x0c,0x10},{0x0d,0x10},{0x09,0x83}};
	return registers(0x57, values, ARRAY_SIZE(values));
}
static int init_mpu(void)
{
	uint8_t id;
	int err = read_byte(0x68, 0x75, &id);
	if (err || (id & 0x7e) != 0x68) { return err ? err : -ENODEV; }
	err = write_byte(0x68, 0x6b, 0x80);
	if (err) { return err; }
	k_msleep(100);
	// 100 Hz, +/-4 g, +/-500 dps. Interrupt status is polled; no disputed IRQ pins.
	static const uint8_t values[][2] = {{0x19,9},{0x1a,1},{0x1b,8},{0x1c,8},
		{0x23,0},{0x37,0},{0x38,1},{0x6a,0},{0x6c,0},{0x6b,0x41}};
	return registers(0x68, values, ARRAY_SIZE(values));
}
static int init_qmc(void)
{
	uint8_t id;
	int err = read_byte(0x0d, 0x0d, &id);
	if (err || id != 0xff) { return err ? err : -ENODEV; }
	static const uint8_t values[][2] = {{0x0a,0x80}};
	err = registers(0x0d, values, ARRAY_SIZE(values));
	if (err) { return err; }
	k_msleep(5);
	err = write_byte(0x0d, 0x0b, 1);
	if (!err) { err = write_byte(0x0d, 0x09, 0x18); } // 100 Hz, 8 G, standby.
	return err;
}
static void poll_mpu(struct sensor_state *s)
{
	uint8_t ready, raw[14];
	if (read_byte(0x68, 0x3a, &ready) || !(ready & 1) || read_bytes(0x68, 0x3b, raw, sizeof(raw))) { return; }
	memcpy(s->packet + 1 + 12 * s->count, raw, 6);
	memcpy(s->packet + 7 + 12 * s->count, raw + 8, 6);
	if (++s->count == 20) {
		s->packet[0] = s->sequence++;
		memcpy(s->packet + 241, raw + 6, 2);
		bio_publish(BIO_MPU, s->packet, 243); s->count = 0;
	}
}
static void poll_qmc(struct sensor_state *s)
{
	uint8_t ready, raw[6];
	if (read_byte(0x0d, 6, &ready) || !(ready & 1) || (ready & 2) || read_bytes(0x0d, 0, raw, sizeof(raw))) { return; }
	memcpy(s->packet + 1 + 6 * s->count, raw, 6);
	if (++s->count == 40) {
		s->packet[0] = s->sequence++;
		if (!read_bytes(0x0d, 7, s->packet + 241, 2)) { bio_publish(BIO_QMC, s->packet, 243); }
		s->count = 0;
	}
}
static void poll_max(struct sensor_state *s)
{
	uint8_t pointers[3], ready;
	if (!read_byte(0x57, 1, &ready) && (ready & 2)) { (void)read_bytes(0x57, 0x1f, max_temp, 2); }
	if (read_bytes(0x57, 4, pointers, sizeof(pointers))) { return; }
	unsigned available = (pointers[0] - pointers[2]) & 31;
	if (pointers[1]) {
		// Discard overflowed FIFO, rather than label overwritten samples as continuous.
		(void)write_byte(0x57, 4, 0); (void)write_byte(0x57, 5, 0); (void)write_byte(0x57, 6, 0);
		s->count = 0; s->sequence++; return;
	}
	while (available--) {
		if (read_bytes(0x57, 7, s->packet + 1 + s->count * 6, 6)) { s->count = 0; return; }
		if (++s->count == 32) {
			s->packet[0] = s->sequence++;
			memcpy(s->packet + 193, max_temp, 2);
			bio_publish(BIO_MAX, s->packet, 195); s->count = 0;
			(void)write_byte(0x57, 0x21, 1); // Temperature result is collected asynchronously.
		}
	}
}
static void put_sensor_value(uint8_t *dest, const struct sensor_value *v)
{
	sys_put_le32(v->val1, dest); sys_put_le32(v->val2, dest + 4);
}
static void poll_bme(struct sensor_state *s)
{
	struct sensor_value temp, pressure, humidity;
	int err = sensor_sample_fetch(bme);
	if (!err) { err = sensor_channel_get(bme, SENSOR_CHAN_AMBIENT_TEMP, &temp); }
	if (!err) { err = sensor_channel_get(bme, SENSOR_CHAN_PRESS, &pressure); }
	if (!err && bme_id == 0x60) { err = sensor_channel_get(bme, SENSOR_CHAN_HUMIDITY, &humidity); }
	if (err) { return; }
	unsigned stride = bme_id == 0x60 ? 24 : 16;
	uint8_t *dest = s->packet + 2 + s->count * stride;
	put_sensor_value(dest, &temp); put_sensor_value(dest + 8, &pressure);
	if (bme_id == 0x60) { put_sensor_value(dest + 16, &humidity); }
	if (++s->count == 3) {
		s->packet[0] = bme_id; s->packet[1] = s->sequence++;
		bio_publish(BIO_BME, s->packet, 2 + 3 * stride); s->count = 0;
	}
}
static void polling(void *a, void *b, void *c)
{
	int64_t next_bme = 0;
	while (true) {
		k_mutex_lock(&sensor_lock, K_FOREVER);
		if (state[0].running) { poll_mpu(&state[0]); }
		if (state[1].running) { poll_max(&state[1]); }
		if (state[3].running) { poll_qmc(&state[3]); }
		int64_t now = k_uptime_get();
		if (state[2].running && now >= next_bme) { poll_bme(&state[2]); next_bme = now + 1000; }
		k_mutex_unlock(&sensor_lock);
		k_msleep(5);
	}
}
K_THREAD_DEFINE(sensor_thread, 2048, polling, NULL, NULL, NULL, 5, 0, SYS_FOREVER_MS);
int bio_sensors_init(void)
{
	if (!device_is_ready(bus)) { return -ENODEV; }
	state[0].ready = init_mpu() == 0;
	state[1].ready = init_max() == 0;
	state[3].ready = init_qmc() == 0;
	state[2].ready = device_is_ready(bme) && !read_byte(DT_REG_ADDR(DT_NODELABEL(bio_bme)), 0xd0, &bme_id);
	LOG_INF("I2C present: MPU=%d MAX=%d BME/BMP=%d QMC=%d", state[0].ready, state[1].ready, state[2].ready, state[3].ready);
	k_thread_start(sensor_thread);
	return bio_sensors_ready() == (BIT(BIO_MPU) | BIT(BIO_MAX) | BIT(BIO_BME) | BIT(BIO_QMC)) ? 0 : -ENODEV;
}
int bio_sensors_start(unsigned id, bool enable)
{
	if (id < BIO_MPU || id > BIO_QMC || !state[id - BIO_MPU].ready) { return -ENODEV; }
	k_mutex_lock(&sensor_lock, K_FOREVER);
	struct sensor_state *s = &state[id - BIO_MPU];
	int err = 0;
	if (s->running == enable) { goto done; }
	switch (id) {
	case BIO_MPU: err = write_byte(0x68, 0x6b, enable ? 1 : 0x41); break;
	case BIO_MAX:
		if (enable) {
			err = write_byte(0x57, 4, 0);
			if (!err) { err = write_byte(0x57, 5, 0); }
			if (!err) { err = write_byte(0x57, 6, 0); }
			if (!err) { err = write_byte(0x57, 0x21, 1); }
		}
		if (!err) { err = write_byte(0x57, 9, enable ? 3 : 0x83); }
		break;
	case BIO_QMC: err = write_byte(0x0d, 9, enable ? 0x19 : 0x18); break;
	default: break; // BME sampling is driven through the public Zephyr sensor API.
	}
	if (!err) { s->running = enable; s->count = 0; }
done:
	k_mutex_unlock(&sensor_lock);
	return err;
}
unsigned bio_sensors_ready(void)
{
	unsigned mask = 0;
	for (unsigned i = 0; i < 4; i++) { if (state[i].ready) { mask |= BIT(i + BIO_MPU); } }
	return mask;
}

unsigned bio_sensors_running(void)
{
	unsigned mask = 0;
	k_mutex_lock(&sensor_lock, K_FOREVER);
	for (unsigned i = 0; i < ARRAY_SIZE(state); i++) {
		if (state[i].running) { mask |= BIT(i + BIO_MPU); }
	}
	k_mutex_unlock(&sensor_lock);
	return mask;
}
