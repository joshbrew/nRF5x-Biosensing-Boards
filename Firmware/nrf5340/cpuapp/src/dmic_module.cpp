#include <zephyr/logging/log.h>
#include <zephyr/sys/printk.h>
#include "dmic_module.hpp"
#include "ble_service.hpp"
#include "usb_comm_handler.hpp"
// For registering callback
#include "ble_service.hpp"

#define DEVICE_NODE DT_NODELABEL(dmicdev)

/** Register log module */
LOG_MODULE_REGISTER(dmic, LOG_LEVEL_INF);

/* Define a new Memory Slab which consists of NUM_BLOCKS blocks
   __________________________________________________________________________
  |    Block 0   |    Block 1   |    Block 2   |    Block 3   |    Block 4   |
  |    0...31    |    0...31    |    0...31    |    0...31    |    0...31    |
  |______________|______________|______________|______________|______________|
*/
K_MEM_SLAB_DEFINE_STATIC(mem_slab, MAX_BLOCK_SIZE, BLOCK_COUNT, 4);

DmicModule::DmicModule() {
    LOG_DBG("DmicModule Constructor!");
}

int DmicModule::Initialize() {

    LOG_DBG("Starting DmicModule Initialization...");
    dmic_dev = DEVICE_DT_GET(DEVICE_NODE);
	
    if (!device_is_ready(dmic_dev)) {
		LOG_ERR("%s is not ready", dmic_dev->name);
		return -1;
	}

	struct pcm_stream_cfg stream = {
		.pcm_width = SAMPLE_BIT_WIDTH,
		.mem_slab  = &mem_slab,
	};
	struct dmic_cfg cfg = {
		.io = {
			/* These fields can be used to limit the PDM clock
			 * configurations that the driver is allowed to use
			 * to those supported by the microphone.
			 */
			.min_pdm_clk_freq = 1000000,
			.max_pdm_clk_freq = 3500000,
			.min_pdm_clk_dc   = 40,
			.max_pdm_clk_dc   = 60,
		},
		.streams = &stream,
		.channel = {
			.req_num_streams = 1,
		},
	};

	cfg.channel.req_num_chan = 1;
	cfg.channel.req_chan_map_lo =
		dmic_build_channel_map(0, 0, PDM_CHAN_LEFT);
	cfg.streams[0].pcm_rate = MAX_SAMPLE_RATE;
	cfg.streams[0].block_size =
		BLOCK_SIZE(cfg.streams[0].pcm_rate, cfg.channel.req_num_chan);

	LOG_DBG("PCM output rate: %u, channels: %u",
		cfg.streams[0].pcm_rate, cfg.channel.req_num_chan);

	int ret = dmic_configure(dmic_dev, &cfg);
	if (ret < 0) {
		LOG_ERR("Failed to configure DMIC driver: %d", ret);
		return ret;
	}

    // memset((uint16_t*)mem_blocks, 0, (NUM_SAMPLES * NUM_BLOCKS));

    Bluetooth::GattRegisterControlCallback(CommandId::DmicCmd,
        [this](const uint8_t *buffer, Bluetooth::CommandKey key, Bluetooth::BleLength length, Bluetooth::BleOffset offset)
        {
            return OnBleCommand(buffer, key, length, offset);
        });

    // Start working thread
    k_thread_create(&worker, pollStackAreaDmic, K_THREAD_STACK_SIZEOF(pollStackAreaDmic),
                    &WorkingThreadDmic, this, nullptr, nullptr, taskPriorityDmic, 0, K_NO_WAIT);
    k_thread_suspend(&worker);

    return ret;    
}

bool DmicModule::OnBleCommand(const uint8_t *buffer, Bluetooth::CommandKey key, Bluetooth::BleLength length, Bluetooth::BleOffset offset){
    if (offset.value != 0 || length.value == 0)
    {
        return false;
    }
    LOG_DBG("DmicModule BLE Command received");

    Bluetooth::CommandKey bleCommand;
    memcpy(&bleCommand, &key, sizeof(key));

    switch(bleCommand.key[0]){
        case static_cast<uint8_t>(BleCommand::StartSampling):
            StartSampling();
            break;
        case static_cast<uint8_t>(BleCommand::StopSampling):
            StopSampling();
            break;

        default:
            break;
    }

    return true;
}

int DmicModule::StartSampling(){
    /* Start taking DMIC data samples */
	int ret = dmic_trigger(dmic_dev, DMIC_TRIGGER_START);
	if (ret < 0) {
		LOG_ERR("START DMIC trigger failed: %d", ret);
    } else {
        LOG_DBG("Resuming thread...");
        k_thread_resume(&worker);
    }

    return ret;
}

int DmicModule::StopSampling(){
	/* Stot taking DMIC data samples */
	int ret = dmic_trigger(dmic_dev, DMIC_TRIGGER_STOP);
	if (ret < 0) {
		LOG_ERR("STOP DMIC trigger failed: %d", ret);
	} else {
        LOG_DBG("Suspending thread...");
        k_thread_suspend(&worker);
    }

    return ret;
}

/**
 * @brief Main working thread. Used to perform I2S transport.
 * It required a separate stack to not break main BLE stack state machine
 *
 * @param data pointer to this
 */
void DmicModule::WorkingThreadDmic(void *data, void *, void *){
    DmicModule *self = static_cast<DmicModule *>(data);
    int ret = 0;
    static arm_rfft_instance_q15 RealFFT_Instance;
    // arm_cfft_radix4_instance_q15 MyComplexFFT_Instance;
    static volatile int16_t MicFFT[2*FFT_LENGTH];
    static int16_t MicFFT_Mag[2*FFT_LENGTH];

    // Initialize the FFT Structures
    ret = arm_rfft_init_q15(&RealFFT_Instance,
                    //&MyComplexFFT_Instance,
                    FFT_LENGTH,
                    0,
                    1); //  Bit Reverse Flag enabled

    if (ret < 0){
        LOG_ERR("arm_rfft_init_q15: %d", ret);
    } else {
        LOG_DBG("arm_rfft_init_q15: SUCESS");
    }

    for (;;)
    {
        for (int i = 0; i < BLOCK_COUNT; ++i) {
            void *buffer;
            uint32_t size;

            ret = dmic_read(self->dmic_dev, 0, &buffer, &size, READ_TIMEOUT);
            if (ret < 0) {
                LOG_ERR("%d - read failed: %d", i, ret);                    
            }

            // Compute FFT
            arm_rfft_q15(&RealFFT_Instance,
                        (q15_t *)buffer,
                        (q15_t *)MicFFT);

            // Scale the input before computing magnitude
            for(int k = 0; k < 2*FFT_LENGTH; k++){
                MicFFT[k]<<=6;
            }

            // FFT functions returns the real/imaginary values. We need to compute the magnitude
            arm_cmplx_mag_q15((q15_t *)MicFFT,
                            (q15_t *)MicFFT_Mag,
                            FFT_LENGTH);

            LOG_HEXDUMP_INF(MicFFT_Mag, FFT_LENGTH, "micFFT_Mag");

            k_mem_slab_free(&mem_slab, &buffer);
        }
    }
}