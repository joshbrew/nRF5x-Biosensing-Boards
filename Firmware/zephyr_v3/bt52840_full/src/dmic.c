#include "bio.h"
#include <zephyr/audio/dmic.h>
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(bio_dmic, LOG_LEVEL_INF);
static const struct device *const microphone = DEVICE_DT_GET(DT_NODELABEL(pdm0));
// 5 ms * 16000 samples/s * 2 bytes; each block fits one application packet.
#define PCM_BYTES 160
K_MEM_SLAB_DEFINE(pcm_slab, PCM_BYTES, 4, 4);
K_SEM_DEFINE(mic_wake, 0, 1);
static atomic_t requested, active;
static void mic_worker(void *a, void *b, void *c)
{
	while (true) {
		k_sem_take(&mic_wake, K_FOREVER);
		if (!atomic_get(&requested)) { continue; }
		struct pcm_stream_cfg stream = { .pcm_rate = 16000, .pcm_width = 16,
			.block_size = PCM_BYTES, .mem_slab = &pcm_slab };
		struct dmic_cfg cfg = {
			.io = { .min_pdm_clk_freq = 1000000, .max_pdm_clk_freq = 3500000,
				.min_pdm_clk_dc = 40, .max_pdm_clk_dc = 60 },
			.streams = &stream,
			.channel = { .req_num_streams = 1, .req_num_chan = 1,
				.req_chan_map_lo = dmic_build_channel_map(0, 0, PDM_CHAN_LEFT) },
		};
		int err = dmic_configure(microphone, &cfg);
		if (!err) { err = dmic_trigger(microphone, DMIC_TRIGGER_START); }
		atomic_set(&active, !err);
		while (!err && atomic_get(&requested)) {
			void *block = NULL; size_t size = 0;
			err = dmic_read(microphone, 0, &block, &size, 100);
			if (!err) {
				if (size <= BIO_PACKET_MAX) { bio_publish(BIO_PCM, block, size); }
				k_mem_slab_free(&pcm_slab, block);
			}
		}
		(void)dmic_trigger(microphone, DMIC_TRIGGER_STOP);
		atomic_clear(&active);
		// Return completed-but-unread blocks before another start.
		void *block; size_t size;
		while (!dmic_read(microphone, 0, &block, &size, 0)) { k_mem_slab_free(&pcm_slab, block); }
		if (err) { atomic_clear(&requested); LOG_ERR("DMIC stopped: %d", err); }
	}
}
K_THREAD_DEFINE(mic_thread, 1536, mic_worker, NULL, NULL, NULL, 5, 0, 0);
int bio_dmic_set(bool enable)
{
	if (!device_is_ready(microphone)) { return -ENODEV; }
	atomic_set(&requested, enable);
	if (enable) { k_sem_give(&mic_wake); }
	return 0;
}
bool bio_dmic_ready(void) { return device_is_ready(microphone); }
bool bio_dmic_running(void) { return atomic_get(&active) != 0; }
