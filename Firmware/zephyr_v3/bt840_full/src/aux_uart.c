#include "bio.h"
#include "transport.h"
#include <string.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/sys/ring_buffer.h>
static const struct device *const uart = DEVICE_DT_GET(DT_NODELABEL(uart2));
RING_BUF_DECLARE(aux_tx, 512);
static struct k_spinlock aux_lock;
static bool ready;
static struct bio_line line;
static atomic_t tx_dropped, rx_dropped;
struct rx_line { size_t len; char data[BIO_COMMAND_MAX]; };
K_MSGQ_DEFINE(aux_rx, sizeof(struct rx_line), 4, 4);
static void publish_rx(struct k_work *work)
{
	struct rx_line item;
	while (!k_msgq_get(&aux_rx, &item, K_NO_WAIT)) { bio_publish(BIO_UART, item.data, item.len); }
}
K_WORK_DEFINE(aux_rx_work, publish_rx);
static void aux_irq(const struct device *dev, void *context)
{
	while (uart_irq_update(dev) && uart_irq_is_pending(dev)) {
		if (uart_irq_rx_ready(dev)) {
			uint8_t ch;
			while (uart_fifo_read(dev, &ch, 1) == 1) {
				enum bio_line_result result = bio_line_feed(&line, ch);
				if (result == BIO_LINE_READY) {
					/* RP2040 replies are data, not firmware commands. */
					struct rx_line item = { .len = strlen(line.data) };
					memcpy(item.data, line.data, item.len);
					if (k_msgq_put(&aux_rx, &item, K_NO_WAIT)) { atomic_inc(&rx_dropped); }
					else { k_work_submit(&aux_rx_work); }
				} else if (result == BIO_LINE_OVERFLOW) { atomic_inc(&rx_dropped); }
			}
		}
		if (uart_irq_tx_ready(dev)) {
			k_spinlock_key_t key = k_spin_lock(&aux_lock);
			uint8_t *data;
			uint32_t len = ring_buf_get_claim(&aux_tx, &data, 32);
			int count = len ? uart_fifo_fill(dev, data, len) : 0;
			ring_buf_get_finish(&aux_tx, count > 0 ? count : 0);
			if (!len) { uart_irq_tx_disable(dev); }
			k_spin_unlock(&aux_lock, key);
			if (len && count <= 0) { break; }
		}
	}
}
int bio_aux_init(void)
{
	if (!device_is_ready(uart)) { return -ENODEV; }
	int err = uart_irq_callback_user_data_set(uart, aux_irq, NULL);
	if (!err) { ready = true; uart_irq_rx_enable(uart); }
	return err;
}
int bio_aux_send(const char *text)
{
	if (!ready) { return -ENODEV; }
	size_t len = strlen(text);
	if (!len || len >= BIO_COMMAND_MAX) { return -EMSGSIZE; }
	k_spinlock_key_t key = k_spin_lock(&aux_lock);
	int err = 0;
	if (ring_buf_space_get(&aux_tx) < len + 1) { atomic_inc(&tx_dropped); err = -ENOBUFS; }
	else { ring_buf_put(&aux_tx, (const uint8_t *)text, len); ring_buf_put(&aux_tx, (const uint8_t *)"\n", 1); }
	k_spin_unlock(&aux_lock, key);
	if (!err) { uart_irq_tx_enable(uart); }
	return err;
}
bool bio_aux_ready(void) { return ready; }
struct bio_io_stats bio_aux_stats(void)
{
	return (struct bio_io_stats){ .tx_drop = atomic_get(&tx_dropped), .rx_drop = atomic_get(&rx_dropped) };
}
