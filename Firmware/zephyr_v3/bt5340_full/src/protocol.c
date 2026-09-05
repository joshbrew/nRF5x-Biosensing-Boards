#include "protocol.h"
#include <errno.h>
#include <stdlib.h>
#include <string.h>

static int number(const char *s, unsigned max, unsigned *out)
{
	char *end;
	if (!s || !*s || *s == '-' || *s == '+') { return -EINVAL; }
	errno = 0;
	unsigned long v = strtoul(s, &end, 0);
	if (errno || *end || v > max) { return -EINVAL; }
	*out = (unsigned)v;
	return 0;
}
static int toggle(const char *s, bool *out)
{
	if (!strcmp(s, "start") || !strcmp(s, "on")) { *out = true; return 0; }
	if (!strcmp(s, "stop") || !strcmp(s, "off")) { *out = false; return 0; }
	return -EINVAL;
}
int bio_parse(const char *line, struct bio_command *cmd)
{
	char buf[BIO_COMMAND_MAX], *argv[5], *p;
	size_t argc = 0;
	if (!line || !cmd || strlen(line) >= sizeof(buf)) { return -EINVAL; }
	memset(cmd, 0, sizeof(*cmd));
	strcpy(buf, line);
	p = buf;
	while (*p == ' ' || *p == '\t') { p++; }
	if (!strncmp(p, "uart ", 5)) {
		if (!p[5]) { return -EINVAL; }
		cmd->op = BIO_UART_SEND;
		strcpy(cmd->text, p + 5);
		return 0;
	}
	while (*p) {
		while (*p == ' ' || *p == '\t' || *p == '\r' || *p == '\n') { p++; }
		if (!*p) { break; }
		if (argc == 5) { return -EINVAL; }
		argv[argc++] = p;
		while (*p && *p != ' ' && *p != '\t' && *p != '\r' && *p != '\n') { p++; }
		if (*p) { *p++ = 0; }
	}
	if (!argc) { return -EINVAL; }
	if (argc == 1 && !strcmp(argv[0], "help")) { cmd->op = BIO_HELP; return 0; }
	if (argc == 1 && !strcmp(argv[0], "status")) { cmd->op = BIO_STATUS; return 0; }
	if (argc == 1 && !strcmp(argv[0], "stats")) { cmd->op = BIO_STATS; return 0; }
	if (argc == 3 && !strcmp(argv[0], "sensor")) {
		static const char *const names[] = { "ads0", "ads1", "mpu", "max", "bme", "qmc" };
		for (unsigned i = 0; i < 6; ++i) {
			if (!strcmp(argv[1], names[i])) {
				cmd->op = BIO_SAMPLE; cmd->id = i + 2;
				return toggle(argv[2], &cmd->enable);
			}
		}
		return -EINVAL;
	}
	if (argc == 2) {
		if (!strcmp(argv[0], "led")) { cmd->op = BIO_LED; }
		else if (!strcmp(argv[0], "scan")) { cmd->op = BIO_SCAN; }
		else if (!strcmp(argv[0], "audio")) { cmd->op = BIO_AUDIO; }
		else if (!strcmp(argv[0], "dmic")) { cmd->op = BIO_MIC; }
		else { return -EINVAL; }
		return toggle(argv[1], &cmd->enable);
	}
	if (argc >= 4 && !strcmp(argv[0], "ads")) {
		if (number(argv[1], 1, &cmd->id)) { return -EINVAL; }
		if (argc == 4 && !strcmp(argv[2], "read")) {
			cmd->op = BIO_ADS_READ;
			return number(argv[3], 0x3e, &cmd->reg);
		}
		if (argc == 5 && !strcmp(argv[2], "write")) {
			cmd->op = BIO_ADS_WRITE;
			if (number(argv[3], 0x30, &cmd->reg) || cmd->reg < 3) { return -EINVAL; }
			return number(argv[4], 65535, &cmd->value);
		}
		if (argc == 4 && !strcmp(argv[2], "gain")) {
			cmd->op = BIO_ADS_GAIN;
			if (number(argv[3], 128, &cmd->value) || !cmd->value ||
			    (cmd->value & (cmd->value - 1))) { return -EINVAL; }
			return 0;
		}
		if (argc == 4 && !strcmp(argv[2], "rate")) {
			cmd->op = BIO_ADS_RATE;
			if (number(argv[3], 2000, &cmd->value)) { return -EINVAL; }
			return (cmd->value == 250 || cmd->value == 500 || cmd->value == 1000 || cmd->value == 2000) ? 0 : -EINVAL;
		}
	}
	return -EINVAL;
}
int bio_parse_legacy(const uint8_t *data, size_t len, struct bio_command *cmd)
{
	if (!data || !cmd || len < 3 || len > BIO_COMMAND_MAX) { return -EINVAL; }
	memset(cmd, 0, sizeof(*cmd));
	if (data[0] == 9 && (data[1] == 3 || data[1] == 4)) {
		cmd->op = BIO_SCAN; cmd->enable = data[1] == 3; return 0;
	}
	if (data[1] != 1 && data[1] != 2) { return -ENOTSUP; }
	cmd->enable = data[1] == 1;
	cmd->op = BIO_SAMPLE;
	switch (data[0]) {
	case 1: if (data[2] > 1) { return -EINVAL; } cmd->id = BIO_ADS0 + data[2]; break;
	case 2: cmd->id = BIO_BME; break;
	case 3: cmd->id = BIO_MAX; break;
	case 4: cmd->id = BIO_MPU; break;
	case 5: cmd->id = BIO_QMC; break;
	case 6: cmd->op = BIO_AUDIO; break;
	case 7: cmd->op = BIO_MIC; break;
	default: return -ENOTSUP;
	}
	return 0;
}
size_t bio_frame(uint8_t *out, size_t cap, uint8_t sensor, uint16_t seq,
	const uint8_t *data, size_t total, size_t offset)
{
	if (!out || !data || !total || total > BIO_PACKET_MAX || offset >= total || cap <= BIO_FRAME_HEADER) { return 0; }
	size_t len = total - offset;
	if (len > cap - BIO_FRAME_HEADER) { len = cap - BIO_FRAME_HEADER; }
	out[0] = 0xb3; out[1] = sensor;
	out[2] = (uint8_t)seq; out[3] = (uint8_t)(seq >> 8);
	out[4] = (uint8_t)offset; out[5] = (uint8_t)(offset >> 8);
	out[6] = (uint8_t)total; out[7] = (uint8_t)(total >> 8);
	memcpy(out + BIO_FRAME_HEADER, data + offset, len);
	return len + BIO_FRAME_HEADER;
}
