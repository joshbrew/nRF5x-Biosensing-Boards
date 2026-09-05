#ifndef BIO_H
#define BIO_H
#include "protocol.h"
#include <zephyr/kernel.h>
#include <errno.h>

/* Capability masks reuse stream IDs. Audio output and USB have no sensor ID. */
#define BIO_CAP_AUDIO BIT(11)
#define BIO_CAP_USB BIT(12)
struct bio_adc_stats { uint32_t irq, spi, crc; };
struct bio_io_stats { uint32_t tx_drop, rx_drop; };
enum bio_origin { BIO_ORIGIN_SHELL, BIO_ORIGIN_BLE, BIO_ORIGIN_USB, BIO_ORIGIN_AUX };
void bio_app_start(void);
void bio_app_run(void);
int bio_submit(enum bio_origin origin, const void *data, size_t len, bool binary, uint32_t session);
void bio_publish(uint8_t sensor, const void *data, size_t len);
int bio_ble_init(void);
int bio_ble_send(uint8_t sensor, const void *data, size_t len);
void bio_ble_reply(const char *reply, uint32_t session);
uint32_t bio_ble_session(void);
unsigned bio_ble_dropped(void);
int bio_ble_scan(bool enable);
bool bio_ble_scanning(void);
bool bio_ble_ready(void);

#if defined(CONFIG_BIO_ADS)
int bio_ads_init(void);
int bio_ads_start(unsigned index, bool enable);
int bio_ads_control(const struct bio_command *cmd, uint16_t *value);
unsigned bio_ads_ready(void);
unsigned bio_ads_missed(void);
unsigned bio_ads_running(void);
struct bio_adc_stats bio_ads_stats(void);
#else
static inline int bio_ads_init(void) { return 0; }
static inline int bio_ads_start(unsigned i, bool e) { return -ENOTSUP; }
static inline int bio_ads_control(const struct bio_command *c, uint16_t *v) { return -ENOTSUP; }
static inline unsigned bio_ads_ready(void) { return 0; }
static inline unsigned bio_ads_missed(void) { return 0; }
static inline unsigned bio_ads_running(void) { return 0; }
static inline struct bio_adc_stats bio_ads_stats(void) { return (struct bio_adc_stats){0}; }
#endif
#if defined(CONFIG_BIO_I2C_SENSORS)
int bio_sensors_init(void);
int bio_sensors_start(unsigned id, bool enable);
unsigned bio_sensors_ready(void);
unsigned bio_sensors_running(void);
#else
static inline int bio_sensors_init(void) { return 0; }
static inline int bio_sensors_start(unsigned i, bool e) { return -ENOTSUP; }
static inline unsigned bio_sensors_ready(void) { return 0; }
static inline unsigned bio_sensors_running(void) { return 0; }
#endif
#if defined(CONFIG_BIO_USB)
int bio_usb_init(void);
int bio_usb_send(uint8_t sensor, const void *data, size_t len);
int bio_usb_reply(const char *text, uint32_t token);
uint32_t bio_usb_session(void);
bool bio_usb_ready(void);
struct bio_io_stats bio_usb_stats(void);
#else
static inline int bio_usb_init(void) { return 0; }
static inline int bio_usb_send(uint8_t s, const void *d, size_t l) { return -ENOTSUP; }
static inline int bio_usb_reply(const char *s, uint32_t t) { return -ENOTSUP; }
static inline uint32_t bio_usb_session(void) { return 0; }
static inline bool bio_usb_ready(void) { return false; }
static inline struct bio_io_stats bio_usb_stats(void) { return (struct bio_io_stats){0}; }
#endif
#if defined(CONFIG_BIO_AUX_UART)
int bio_aux_init(void);
int bio_aux_send(const char *text);
bool bio_aux_ready(void);
struct bio_io_stats bio_aux_stats(void);
#else
static inline int bio_aux_init(void) { return 0; }
static inline int bio_aux_send(const char *s) { return -ENOTSUP; }
static inline bool bio_aux_ready(void) { return false; }
static inline struct bio_io_stats bio_aux_stats(void) { return (struct bio_io_stats){0}; }
#endif
#if defined(CONFIG_BIO_AUDIO)
int bio_audio_set(bool enable);
bool bio_audio_ready(void);
bool bio_audio_running(void);
#else
static inline int bio_audio_set(bool e) { return -ENOTSUP; }
static inline bool bio_audio_ready(void) { return false; }
static inline bool bio_audio_running(void) { return false; }
#endif
#if defined(CONFIG_BIO_DMIC)
int bio_dmic_set(bool enable);
bool bio_dmic_ready(void);
bool bio_dmic_running(void);
#else
static inline int bio_dmic_set(bool e) { return -ENOTSUP; }
static inline bool bio_dmic_ready(void) { return false; }
static inline bool bio_dmic_running(void) { return false; }
#endif
#endif
