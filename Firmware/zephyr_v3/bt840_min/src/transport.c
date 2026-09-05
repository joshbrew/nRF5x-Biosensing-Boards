#include "transport.h"
#include <string.h>

bool bio_link_set(struct bio_link *link, bool open)
{
	if (link->open == open) {
		return false;
	}
	link->open = open;
	if (++link->generation == 0) {
		++link->generation; /* Zero is always an invalid token. */
	}
	return true;
}

uint32_t bio_link_token(const struct bio_link *link)
{
	return link->open ? link->generation : 0;
}

bool bio_link_accepts(const struct bio_link *link, uint32_t token)
{
	return token != 0 && token == bio_link_token(link);
}

void bio_line_reset(struct bio_line *line)
{
	memset(line, 0, sizeof(*line));
}

enum bio_line_result bio_line_feed(struct bio_line *line, uint8_t byte)
{
	if (byte == '\r' || byte == '\n') {
		enum bio_line_result result = line->overflow ? BIO_LINE_OVERFLOW :
			(line->used ? BIO_LINE_READY : BIO_LINE_MORE);
		line->data[line->used] = '\0';
		line->used = 0;
		line->overflow = false;
		return result;
	}
	/* Reject binary/NUL input as a whole line, not a silently shortened command. */
	if (!byte || line->used >= sizeof(line->data) - 1) {
		line->overflow = true;
	}
	if (!line->overflow) {
		line->data[line->used++] = (char)byte;
	}
	return BIO_LINE_MORE;
}
