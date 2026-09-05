#include "bio.h"
#include <zephyr/drivers/i2s.h>
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(bio_audio, LOG_LEVEL_INF);
#define BLOCK_BYTES 256
static const struct device *const output = DEVICE_DT_GET(DT_NODELABEL(i2s0));
K_MEM_SLAB_DEFINE(audio_slab, BLOCK_BYTES, 4, 4);
K_SEM_DEFINE(audio_wake, 0, 1);
static atomic_t requested, active;
static unsigned phase;
static int queue_tone(void)
{
	void *block;
	int err = k_mem_slab_alloc(&audio_slab, &block, K_MSEC(100));
	if (err) { return err; }
	int16_t *samples = block;
	for (unsigned i = 0; i < BLOCK_BYTES / 4; i++) {
		// Quiet 1 kHz stereo square wave at 16 kHz; no large embedded alarm array.
		int16_t value = (phase++ & 15) < 8 ? 512 : -512;
		samples[i * 2] = value; samples[i * 2 + 1] = value;
	}
	err = i2s_write(output, block, BLOCK_BYTES);
	if (err) { k_mem_slab_free(&audio_slab, block); }
	return err;
}
static void audio_worker(void *a, void *b, void *c)
{
	while (true) {
		k_sem_take(&audio_wake, K_FOREVER);
		if (!atomic_get(&requested)) { continue; }
		struct i2s_config cfg = {
			.word_size = 16, .channels = 2, .format = I2S_FMT_DATA_FORMAT_I2S,
			.options = I2S_OPT_BIT_CLK_MASTER | I2S_OPT_FRAME_CLK_MASTER,
			.frame_clk_freq = 16000, .mem_slab = &audio_slab,
			.block_size = BLOCK_BYTES, .timeout = 100,
		};
		int err = i2s_configure(output, I2S_DIR_TX, &cfg);
		if (!err) { err = queue_tone(); }
		if (!err) { err = queue_tone(); }
		if (!err) { err = i2s_trigger(output, I2S_DIR_TX, I2S_TRIGGER_START); }
		atomic_set(&active, !err);
		while (!err && atomic_get(&requested)) { err = queue_tone(); }
		(void)i2s_trigger(output, I2S_DIR_TX, I2S_TRIGGER_DROP);
		atomic_clear(&active);
		if (err) { atomic_clear(&requested); LOG_ERR("Audio stopped: %d", err); }
	}
}
K_THREAD_DEFINE(audio_thread, 1536, audio_worker, NULL, NULL, NULL, 5, 0, 0);
int bio_audio_set(bool enable)
{
	if (!device_is_ready(output)) { return -ENODEV; }
	atomic_set(&requested, enable);
	if (enable) { k_sem_give(&audio_wake); }
	return 0;
}
bool bio_audio_ready(void) { return device_is_ready(output); }
bool bio_audio_running(void) { return atomic_get(&active) != 0; }
