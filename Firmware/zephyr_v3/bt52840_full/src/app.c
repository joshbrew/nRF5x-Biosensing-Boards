#include "bio.h"
#include <string.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/printk.h>
#if defined(CONFIG_SHELL)
#include <zephyr/shell/shell.h>
#endif
LOG_MODULE_REGISTER(bio_app, LOG_LEVEL_INF);
struct request { enum bio_origin origin; uint32_t session; bool binary;
	uint16_t len; char data[BIO_COMMAND_MAX]; };
K_MSGQ_DEFINE(commands, sizeof(struct request), CONFIG_BIO_COMMAND_DEPTH, 4);
static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(DT_ALIAS(bio_led), gpios);
static bool led_ready;
static atomic_t app_ready, commands_dropped, stale_commands;

void bio_publish(uint8_t sensor, const void *data, size_t len)
{
	// Each transport copies the packet; neither blocks acquisition on host traffic.
	(void)bio_ble_send(sensor, data, len);
	(void)bio_usb_send(sensor, data, len);
}
int bio_submit(enum bio_origin origin, const void *data, size_t len, bool binary, uint32_t session)
{
	if (!atomic_get(&app_ready)) { return -EAGAIN; }
	if (!data || !len || len >= BIO_COMMAND_MAX || (!binary && memchr(data, 0, len))) { return -EINVAL; }
	struct request req = { .origin = origin, .session = session, .binary = binary, .len = len };
	memcpy(req.data, data, len);
	int err = k_msgq_put(&commands, &req, K_NO_WAIT);
	if (err) { atomic_inc(&commands_dropped); }
	return err;
}

static void status_reply(char *reply, size_t capacity)
{
	unsigned built = (IS_ENABLED(CONFIG_BIO_ADS) ? BIT(BIO_ADS0) : 0) |
		(IS_ENABLED(CONFIG_BIO_ADS_SECOND) ? BIT(BIO_ADS1) : 0) |
		(IS_ENABLED(CONFIG_BIO_I2C_SENSORS) ? BIT(BIO_MPU) | BIT(BIO_MAX) | BIT(BIO_BME) | BIT(BIO_QMC) : 0) |
		(IS_ENABLED(CONFIG_BIO_DMIC) ? BIT(BIO_PCM) : 0) |
		(IS_ENABLED(CONFIG_BIO_BEACONS) ? BIT(BIO_BEACON) : 0) |
		(IS_ENABLED(CONFIG_BIO_AUX_UART) ? BIT(BIO_UART) : 0) |
		(IS_ENABLED(CONFIG_BIO_AUDIO) ? BIO_CAP_AUDIO : 0) |
		(IS_ENABLED(CONFIG_BIO_USB) ? BIO_CAP_USB : 0);
	unsigned ready = (bio_ads_ready() << BIO_ADS0) | bio_sensors_ready() |
		(bio_dmic_ready() ? BIT(BIO_PCM) : 0) |
		(IS_ENABLED(CONFIG_BIO_BEACONS) && bio_ble_ready() ? BIT(BIO_BEACON) : 0) |
		(bio_aux_ready() ? BIT(BIO_UART) : 0) |
		(bio_audio_ready() ? BIO_CAP_AUDIO : 0) |
		(bio_usb_ready() ? BIO_CAP_USB : 0);
	unsigned running = (bio_ads_running() << BIO_ADS0) | bio_sensors_running() |
		(bio_dmic_running() ? BIT(BIO_PCM) : 0) |
		(bio_ble_scanning() ? BIT(BIO_BEACON) : 0) |
		(bio_aux_ready() ? BIT(BIO_UART) : 0) |
		(bio_audio_running() ? BIO_CAP_AUDIO : 0) |
		(bio_usb_session() ? BIO_CAP_USB : 0);
	snprintk(reply, capacity, "%s built=0x%x ready=0x%x running=0x%x ble=%u ads_mask=0x%x ads_missed=%u ble_drop=%u",
		CONFIG_BIO_PROFILE, built, ready, running, bio_ble_session() != 0,
		bio_ads_ready(), bio_ads_missed(), bio_ble_dropped());
}

static void stats_reply(char *reply, size_t capacity)
{
	struct bio_adc_stats adc = bio_ads_stats();
	struct bio_io_stats usb = bio_usb_stats(), uart = bio_aux_stats();
	snprintk(reply, capacity,
		"ads_irq=%u ads_spi=%u ads_crc=%u ble_drop=%u usb_tx_drop=%u usb_rx_drop=%u uart_tx_drop=%u uart_rx_drop=%u cmd_drop=%u stale_cmd=%u",
		adc.irq, adc.spi, adc.crc, bio_ble_dropped(), usb.tx_drop, usb.rx_drop,
		uart.tx_drop, uart.rx_drop, (unsigned)atomic_get(&commands_dropped), (unsigned)atomic_get(&stale_commands));
}
static int execute(const struct bio_command *cmd, char *reply, size_t capacity)
{
	uint16_t value = 0;
	switch (cmd->op) {
	case BIO_HELP:
		snprintk(reply, capacity, "status; stats; sensor ads0|ads1|mpu|max|bme|qmc start|stop; ads 0|1 read REG|write REG VALUE|gain 1..128|rate 250..2000; led on|off; scan|audio|dmic start|stop; uart TEXT");
		return 0;
	case BIO_STATUS:
		status_reply(reply, capacity);
		return 0;
	case BIO_STATS:
		stats_reply(reply, capacity);
		return 0;
	case BIO_SAMPLE:
		return cmd->id <= BIO_ADS1 ? bio_ads_start(cmd->id - BIO_ADS0, cmd->enable) : bio_sensors_start(cmd->id, cmd->enable);
	case BIO_ADS_READ: case BIO_ADS_WRITE: case BIO_ADS_GAIN: case BIO_ADS_RATE: {
		int err = bio_ads_control(cmd, &value);
		if (!err && cmd->op == BIO_ADS_READ) { snprintk(reply, capacity, "ok 0x%04x", value); }
		return err;
	}
	case BIO_LED: return led_ready ? gpio_pin_set_dt(&led, cmd->enable) : -ENODEV;
	case BIO_SCAN: return bio_ble_scan(cmd->enable);
	case BIO_AUDIO: return bio_audio_set(cmd->enable);
	case BIO_MIC: return bio_dmic_set(cmd->enable);
	case BIO_UART_SEND: return bio_aux_send(cmd->text);
	default: return -ENOTSUP;
	}
}
void bio_app_run(void)
{
	struct request req;
	while (true) {
		k_msgq_get(&commands, &req, K_FOREVER);
		/* Queued commands are valid only for the connection that submitted them.
		 * Commands already executing are not cancelled by a disconnect. */
		if ((req.origin == BIO_ORIGIN_BLE && (!req.session || req.session != bio_ble_session())) ||
		    (req.origin == BIO_ORIGIN_USB && (!req.session || req.session != bio_usb_session()))) {
			atomic_inc(&stale_commands);
			continue;
		}
		struct bio_command cmd;
		char reply[BIO_PACKET_MAX] = "ok";
		int err = req.binary ? bio_parse_legacy((uint8_t *)req.data, req.len, &cmd) : bio_parse(req.data, &cmd);
		if (!err) { err = execute(&cmd, reply, sizeof(reply)); }
		if (err) { snprintk(reply, sizeof(reply), "error %d (use help; stop ADC before changing registers)", err); }
		if (req.origin == BIO_ORIGIN_BLE) { bio_ble_reply(reply, req.session); }
		else if (req.origin == BIO_ORIGIN_USB) { bio_usb_reply(reply, req.session); }
		else if (req.origin == BIO_ORIGIN_AUX) { bio_aux_send(reply); }
		else { LOG_INF("%s", reply); }
	}
}
void bio_app_start(void)
{
	led_ready = gpio_is_ready_dt(&led) && gpio_pin_configure_dt(&led, GPIO_OUTPUT_INACTIVE) == 0;
	atomic_set(&app_ready, 1);
}
#if defined(CONFIG_SHELL)
static int shell_bio(const struct shell *sh, size_t argc, char **argv)
{
	char line[BIO_COMMAND_MAX] = "";
	size_t used = 0;
	if (argc < 2) { strcpy(line, "help"); }
	for (size_t i = 1; i < argc; i++) {
		size_t len = strlen(argv[i]);
		if (used + len + 1 >= sizeof(line)) { return -EMSGSIZE; }
		if (used) { line[used++] = ' '; }
		memcpy(line + used, argv[i], len + 1); used += len;
	}
	int err = bio_submit(BIO_ORIGIN_SHELL, line, strlen(line), false, 0);
	if (err) { shell_error(sh, "Command queue: %d", err); }
	return err;
}
SHELL_CMD_REGISTER(bio, NULL, "Biosensing commands (bio help)", shell_bio);
#endif
