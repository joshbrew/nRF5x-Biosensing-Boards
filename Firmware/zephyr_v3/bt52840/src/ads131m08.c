// Fixed 24-bit, 10-word ADS131M08 frames. Register layout follows the old driver.
#include "bio.h"
#include <string.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/byteorder.h>
LOG_MODULE_REGISTER(bio_ads, LOG_LEVEL_INF);
#define ADS_COUNT (1 + IS_ENABLED(CONFIG_BIO_ADS_SECOND))
#define ADS_SPEC(node) { .spi = SPI_DT_SPEC_GET(node, SPI_WORD_SET(8) | SPI_MODE_CPHA), \
	.drdy = GPIO_DT_SPEC_GET(node, drdy_gpios), .reset = GPIO_DT_SPEC_GET(node, reset_gpios) }
struct ads_device {
	struct spi_dt_spec spi;
	struct gpio_dt_spec drdy, reset;
	struct gpio_callback cb;
	bool ready, running;
	atomic_t edges;
	uint8_t count, packet[227];
};
static struct ads_device adc[ADS_COUNT] = {
	ADS_SPEC(DT_ALIAS(ads0)),
#if defined(CONFIG_BIO_ADS_SECOND)
	ADS_SPEC(DT_ALIAS(ads1)),
#endif
};
K_MUTEX_DEFINE(ads_lock);
K_SEM_DEFINE(ads_wake, 0, 1);
static atomic_t pending, coalesced, spi_errors, crc_errors;

static int transfer(struct ads_device *dev, uint8_t *tx, uint8_t *rx)
{
	struct spi_buf tx_buf = { .buf = tx, .len = 30 };
	struct spi_buf rx_buf = { .buf = rx, .len = 30 };
	struct spi_buf_set tx_set = { .buffers = &tx_buf, .count = 1 };
	struct spi_buf_set rx_set = { .buffers = &rx_buf, .count = 1 };
	int err = spi_transceive_dt(&dev->spi, &tx_set, &rx_set);
	if (err) { atomic_inc(&spi_errors); }
	return err;
}
static int reg_read(struct ads_device *dev, unsigned reg, uint16_t *value)
{
	uint8_t tx[30] = {0}, rx[30];
	sys_put_be16(0xa000 | (reg << 7), tx);
	int err = transfer(dev, tx, rx);
	if (err) { return err; }
	memset(tx, 0, sizeof(tx));
	err = transfer(dev, tx, rx);
	if (!err) { *value = sys_get_be16(rx); }
	return err;
}
static int reg_write(struct ads_device *dev, unsigned reg, uint16_t value)
{
	uint8_t tx[30] = {0}, rx[30];
	sys_put_be16(0x6000 | (reg << 7), tx);
	sys_put_be16(value, tx + 3);
	int err = transfer(dev, tx, rx);
	if (err) { return err; }
	memset(tx, 0, sizeof(tx));
	err = transfer(dev, tx, rx);
	if (err) { return err; }
	if (sys_get_be16(rx) != (0x4000 | (reg << 7))) { return -EIO; }
	uint16_t actual;
	err = reg_read(dev, reg, &actual);
	return err ? err : (actual == value ? 0 : -EIO);
}
static void drdy_irq(const struct device *port, struct gpio_callback *cb, gpio_port_pins_t pins)
{
	struct ads_device *dev = CONTAINER_OF(cb, struct ads_device, cb);
	unsigned index = dev - adc;
	atomic_inc(&dev->edges);
	if (atomic_test_and_set_bit(&pending, index)) { atomic_inc(&coalesced); }
	k_sem_give(&ads_wake);
}
static bool crc_valid(const uint8_t *frame)
{
	uint16_t crc = 0xffff;
	for (unsigned i = 0; i < 27; i++) {
		crc ^= (uint16_t)frame[i] << 8;
		for (unsigned j = 0; j < 8; j++) { crc = (crc & 0x8000) ? (crc << 1) ^ 0x1021 : crc << 1; }
	}
	return crc == sys_get_be16(frame + 27);
}
static void acquisition(void *a, void *b, void *c)
{
	while (true) {
		k_sem_take(&ads_wake, K_FOREVER);
		k_mutex_lock(&ads_lock, K_FOREVER);
		for (unsigned i = 0; i < ADS_COUNT; i++) {
			struct ads_device *dev = &adc[i];
			if (!atomic_test_and_clear_bit(&pending, i) || !dev->running) { continue; }
			uint8_t tx[30] = {0}, rx[30];
			uint8_t sequence = atomic_get(&dev->edges);
			int err = transfer(dev, tx, rx);
			if (err) { dev->count = 0; continue; }
			if (!crc_valid(rx)) {
				atomic_inc(&crc_errors);
				dev->count = 0;
				continue;
			}
			// Keep the existing client payload: 9 * (8 signed BE24 channels + counter), CR LF.
			memcpy(dev->packet + 25 * dev->count, rx + 3, 24);
			dev->packet[25 * dev->count + 24] = sequence;
			if (++dev->count == 9) {
				dev->packet[225] = '\r'; dev->packet[226] = '\n';
				bio_publish(BIO_ADS0 + i, dev->packet, sizeof(dev->packet));
				dev->count = 0;
			}
		}
		k_mutex_unlock(&ads_lock);
	}
}
K_THREAD_DEFINE(ads_thread, CONFIG_BIO_ADS_STACK, acquisition, NULL, NULL, NULL, 4, 0, SYS_FOREVER_MS);
int bio_ads_init(void)
{
	int result = 0;
	for (unsigned i = 0; i < ADS_COUNT; i++) {
		struct ads_device *dev = &adc[i];
		int err = 0;
		if (!spi_is_ready_dt(&dev->spi) || !gpio_is_ready_dt(&dev->drdy) || !gpio_is_ready_dt(&dev->reset)) { err = -ENODEV; }
		if (!err) { err = gpio_pin_configure_dt(&dev->reset, GPIO_OUTPUT_ACTIVE); }
		if (!err) {
			k_msleep(2);
			err = gpio_pin_set_dt(&dev->reset, 0);
			k_msleep(10);
		}
		if (!err) { err = gpio_pin_configure_dt(&dev->drdy, GPIO_INPUT); }
		uint16_t id = 0;
		if (!err) { err = reg_read(dev, 0, &id); }
		if (!err && (id & 0xff00) != 0x2800) { err = -ENODEV; }
		if (!err) { err = reg_write(dev, 2, 0x0110); } // 24-bit, CCITT output CRC, clear RESET.
		if (!err) { err = reg_write(dev, 3, 0xff1f); } // 250 SPS with an 8.192 MHz CLKIN.
		if (!err) { err = reg_write(dev, 4, 0x5555); }
		if (!err) { err = reg_write(dev, 5, 0x5555); } // All channels gain 32.
		if (!err) {
			gpio_init_callback(&dev->cb, drdy_irq, BIT(dev->drdy.pin));
			err = gpio_add_callback(dev->drdy.port, &dev->cb);
		}
		dev->ready = !err;
		if (err) { result = err; LOG_WRN("ADS%u ID=0x%04x unavailable (%d)", i, id, err); }
		else { LOG_INF("ADS%u ID=0x%04x ready, sampling stopped", i, id); }
	}
	k_thread_start(ads_thread);
	return result;
}
int bio_ads_start(unsigned index, bool enable)
{
	if (index >= ADS_COUNT || !adc[index].ready) { return -ENODEV; }
	k_mutex_lock(&ads_lock, K_FOREVER);
	struct ads_device *dev = &adc[index];
	int err = 0;
	if (dev->running == enable) { goto done; }
	if (enable) {
		// Drain stale conversion words before enabling data-ready interrupts.
		uint8_t tx[30] = {0}, rx[30];
		err = transfer(dev, tx, rx);
		if (!err) { err = transfer(dev, tx, rx); }
	}
	if (!err) { err = gpio_pin_interrupt_configure_dt(&dev->drdy, enable ? GPIO_INT_EDGE_TO_ACTIVE : GPIO_INT_DISABLE); }
	if (!err) { dev->running = enable; dev->count = 0; atomic_clear_bit(&pending, index); }
done:
	k_mutex_unlock(&ads_lock);
	return err;
}
int bio_ads_control(const struct bio_command *cmd, uint16_t *value)
{
	if (cmd->id >= ADS_COUNT || !adc[cmd->id].ready) { return -ENODEV; }
	k_mutex_lock(&ads_lock, K_FOREVER);
	struct ads_device *dev = &adc[cmd->id];
	int err = -EBUSY;
	if (dev->running) { goto done; }
	switch (cmd->op) {
	case BIO_ADS_READ: err = reg_read(dev, cmd->reg, value); break;
	case BIO_ADS_WRITE:
		// The parser disallows MODE: changing word length or CRC breaks the transport.
		err = reg_write(dev, cmd->reg, cmd->value); break;
	case BIO_ADS_GAIN: {
		unsigned code = 0;
		for (unsigned g = cmd->value; g > 1; g >>= 1) { code++; }
		err = reg_write(dev, 4, code * 0x1111);
		if (!err) { err = reg_write(dev, 5, code * 0x1111); }
		break;
	}
	case BIO_ADS_RATE: {
		unsigned osr = cmd->value == 250 ? 7 : cmd->value == 500 ? 6 : cmd->value == 1000 ? 5 : 4;
		err = reg_read(dev, 3, value);
		if (!err) { err = reg_write(dev, 3, (*value & 0xff00) | (osr << 2) | 3); }
		break;
	}
	default: err = -EINVAL;
	}
done:
	k_mutex_unlock(&ads_lock);
	return err;
}
unsigned bio_ads_ready(void)
{
	unsigned mask = 0;
	for (unsigned i = 0; i < ADS_COUNT; i++) { if (adc[i].ready) { mask |= BIT(i); } }
	return mask;
}
struct bio_adc_stats bio_ads_stats(void)
{
	return (struct bio_adc_stats){ .irq = atomic_get(&coalesced),
		.spi = atomic_get(&spi_errors), .crc = atomic_get(&crc_errors) };
}

unsigned bio_ads_missed(void)
{
	struct bio_adc_stats stats = bio_ads_stats();
	return stats.irq + stats.spi + stats.crc;
}

unsigned bio_ads_running(void)
{
	unsigned mask = 0;
	k_mutex_lock(&ads_lock, K_FOREVER);
	for (unsigned i = 0; i < ADS_COUNT; i++) {
		if (adc[i].running) { mask |= BIT(i); }
	}
	k_mutex_unlock(&ads_lock);
	return mask;
}
