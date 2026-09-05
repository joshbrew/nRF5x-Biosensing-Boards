#include "bio.h"
#include "transport.h"
#include <string.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/usb/usbd.h>
#include <zephyr/sys/ring_buffer.h>
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(bio_usb, LOG_LEVEL_INF);
static const struct device *const uart = DEVICE_DT_GET(DT_NODELABEL(bio_usb));
USBD_DEVICE_DEFINE(bio_usbd, DEVICE_DT_GET(DT_NODELABEL(zephyr_udc0)), CONFIG_BIO_USB_VID, CONFIG_BIO_USB_PID);
USBD_DESC_LANG_DEFINE(usb_language);
USBD_DESC_MANUFACTURER_DEFINE(usb_manufacturer, "Biosensing development");
USBD_DESC_PRODUCT_DEFINE(usb_product, "Biosensing v3");
USBD_DESC_CONFIG_DEFINE(usb_config_string, "CDC data");
USBD_CONFIGURATION_DEFINE(usb_config, USB_SCD_SELF_POWERED, 2, &usb_config_string);
RING_BUF_DECLARE(usb_tx, 2048);
/* One lock protects session changes, partial RX lines, and application TX bytes. */
static struct k_spinlock state_lock;
static atomic_t usb_ready, sequence, tx_dropped, rx_dropped;
static struct bio_link link;
static struct bio_line line;

static void set_host_open(bool open)
{
	k_spinlock_key_t key = k_spin_lock(&state_lock);
	if (bio_link_set(&link, open) || !open) {
		bio_line_reset(&line);
		ring_buf_reset(&usb_tx);
	}
	k_spin_unlock(&state_lock, key);
}

static void receive_byte(uint8_t ch)
{
	/* Copy a completed line before releasing the lock to the USB event callback. */
	char command[BIO_COMMAND_MAX];
	k_spinlock_key_t key = k_spin_lock(&state_lock);
	uint32_t token = bio_link_token(&link);
	enum bio_line_result result = token ? bio_line_feed(&line, ch) : BIO_LINE_MORE;
	if (result == BIO_LINE_READY) { strcpy(command, line.data); }
	k_spin_unlock(&state_lock, key);
	if (result == BIO_LINE_READY) {
		int err = bio_submit(BIO_ORIGIN_USB, command, strlen(command), false, token);
		if (err) {
			atomic_inc(&rx_dropped);
			(void)bio_usb_reply("error command queue", token);
		}
	} else if (result == BIO_LINE_OVERFLOW) {
		atomic_inc(&rx_dropped);
		(void)bio_usb_reply("error invalid or overlong line", token);
	}
}
static void serial_irq(const struct device *dev, void *context)
{
	while (uart_irq_update(dev) && uart_irq_is_pending(dev)) {
		if (uart_irq_rx_ready(dev)) {
			uint8_t ch;
			while (uart_fifo_read(dev, &ch, 1) == 1) {
				receive_byte(ch);
			}
		}
		if (uart_irq_tx_ready(dev)) {
			k_spinlock_key_t key = k_spin_lock(&state_lock);
			uint8_t *data;
			uint32_t len = ring_buf_get_claim(&usb_tx, &data, 64);
			int sent = len ? uart_fifo_fill(dev, data, len) : 0;
			ring_buf_get_finish(&usb_tx, sent > 0 ? sent : 0);
			if (!len) { uart_irq_tx_disable(dev); }
			k_spin_unlock(&state_lock, key);
			if (len && sent <= 0) { break; }
		}
	}
}
static void usb_event(struct usbd_context *ctx, const struct usbd_msg *msg)
{
	if (msg->type == USBD_MSG_VBUS_READY) {
		int err = usbd_enable(ctx); if (err) { LOG_WRN("USB enable: %d", err); }
	} else if (msg->type == USBD_MSG_VBUS_REMOVED) {
		set_host_open(false);
		(void)usbd_disable(ctx);
	}
	else if (msg->type == USBD_MSG_CDC_ACM_CONTROL_LINE_STATE) {
		uint32_t dtr = 0;
		(void)uart_line_ctrl_get(msg->dev, UART_LINE_CTRL_DTR, &dtr);
		set_host_open(dtr != 0);
	}
	if (msg->type == USBD_MSG_RESET) { set_host_open(false); }
}
int bio_usb_init(void)
{
	if (!device_is_ready(uart)) { return -ENODEV; }
	int err = usbd_add_descriptor(&bio_usbd, &usb_language);
	if (!err) { err = usbd_add_descriptor(&bio_usbd, &usb_manufacturer); }
	if (!err) { err = usbd_add_descriptor(&bio_usbd, &usb_product); }
	if (!err) { err = usbd_add_configuration(&bio_usbd, USBD_SPEED_FS, &usb_config); }
	if (!err) { err = usbd_register_class(&bio_usbd, "cdc_acm_0", USBD_SPEED_FS, 1); }
	if (!err) { err = usbd_device_set_code_triple(&bio_usbd, USBD_SPEED_FS, USB_BCC_MISCELLANEOUS, 2, 1); }
	if (!err) { err = usbd_msg_register_cb(&bio_usbd, usb_event); }
	if (!err) { err = usbd_init(&bio_usbd); }
	if (!err) { err = uart_irq_callback_user_data_set(uart, serial_irq, NULL); }
	if (!err && !usbd_can_detect_vbus(&bio_usbd)) { err = usbd_enable(&bio_usbd); }
	if (!err) { atomic_set(&usb_ready, 1); uart_irq_rx_enable(uart); }
	return err;
}
static int send_frame(uint8_t sensor, const void *data, size_t len, uint32_t token)
{
	if (!atomic_get(&usb_ready) || !token) { return -ENOTCONN; }
	uint8_t frame[BIO_PACKET_MAX + BIO_FRAME_HEADER];
	size_t size = bio_frame(frame, sizeof(frame), sensor, atomic_inc(&sequence), data, len, 0);
	if (!size) { return -EMSGSIZE; }
	k_spinlock_key_t key = k_spin_lock(&state_lock);
	int err = 0;
	if (!bio_link_accepts(&link, token)) { err = -ENOTCONN; }
	else if (ring_buf_space_get(&usb_tx) < size) {
		atomic_inc(&tx_dropped);
		err = -ENOBUFS;
	}
	else { ring_buf_put(&usb_tx, frame, size); }
	k_spin_unlock(&state_lock, key);
	if (!err) { uart_irq_tx_enable(uart); }
	return err;
}

uint32_t bio_usb_session(void)
{
	k_spinlock_key_t key = k_spin_lock(&state_lock);
	uint32_t token = bio_link_token(&link);
	k_spin_unlock(&state_lock, key);
	return token;
}

int bio_usb_send(uint8_t sensor, const void *data, size_t len)
{
	return send_frame(sensor, data, len, bio_usb_session());
}

int bio_usb_reply(const char *text, uint32_t token)
{
	return send_frame(BIO_RESPONSE, text, strlen(text), token);
}

bool bio_usb_ready(void) { return atomic_get(&usb_ready) != 0; }
struct bio_io_stats bio_usb_stats(void)
{
	return (struct bio_io_stats){ .tx_drop = atomic_get(&tx_dropped), .rx_drop = atomic_get(&rx_dropped) };
}
