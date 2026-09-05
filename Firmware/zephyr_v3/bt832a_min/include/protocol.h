#ifndef BIO_PROTOCOL_H
#define BIO_PROTOCOL_H
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#define BIO_COMMAND_MAX 128
#define BIO_PACKET_MAX 244
#define BIO_FRAME_HEADER 8
enum bio_sensor { BIO_RESPONSE = 0, BIO_ADS0 = 2, BIO_ADS1 = 3,
	BIO_MPU = 4, BIO_MAX = 5, BIO_BME = 6, BIO_QMC = 7,
	BIO_PCM = 8, BIO_BEACON = 9, BIO_UART = 10 };
enum bio_operation { BIO_HELP, BIO_STATUS, BIO_SAMPLE, BIO_ADS_READ,
	BIO_ADS_WRITE, BIO_ADS_GAIN, BIO_ADS_RATE, BIO_LED, BIO_SCAN,
	BIO_AUDIO, BIO_MIC, BIO_UART_SEND, BIO_STATS };
struct bio_command { enum bio_operation op; unsigned id, value, reg; bool enable;
	char text[BIO_COMMAND_MAX]; };
int bio_parse(const char *line, struct bio_command *cmd);
int bio_parse_legacy(const uint8_t *data, size_t len, struct bio_command *cmd);
size_t bio_frame(uint8_t *out, size_t capacity, uint8_t sensor, uint16_t seq,
	const uint8_t *data, size_t total, size_t offset);
#endif
