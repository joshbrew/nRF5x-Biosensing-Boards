#ifndef BIO_TRANSPORT_H
#define BIO_TRANSPORT_H

#include "protocol.h"

/* Portable state helpers. Callers must serialize access (mutex or spinlock). */
struct bio_link {
	uint32_t generation;
	bool open;
};

bool bio_link_set(struct bio_link *link, bool open);
uint32_t bio_link_token(const struct bio_link *link);
bool bio_link_accepts(const struct bio_link *link, uint32_t token);

struct bio_line {
	char data[BIO_COMMAND_MAX];
	size_t used;
	bool overflow;
};

enum bio_line_result { BIO_LINE_MORE, BIO_LINE_READY, BIO_LINE_OVERFLOW };
void bio_line_reset(struct bio_line *line);
/* READY leaves a NUL-terminated command in data until the next feed/reset. */
enum bio_line_result bio_line_feed(struct bio_line *line, uint8_t byte);

#endif
