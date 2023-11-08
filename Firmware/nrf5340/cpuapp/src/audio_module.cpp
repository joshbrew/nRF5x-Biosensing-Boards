#include <logging/log.h>
#include <sys/printk.h>

#include "audio_module.hpp"
#include "ble_service.hpp"
#include "usb_comm_handler.hpp"
// For registering callback
#include "ble_service.hpp"

/** Register log module */
LOG_MODULE_REGISTER(i2s, LOG_LEVEL_INF);

/* Define a new Memory Slab which consistes of NUM_BLOCKS blocks
   __________________________________________________________________________
  |    Block 0   |    Block 1   |    Block 2   |    Block 3   |    Block 4   |
  |    0...31    |    0...31    |    0...31    |    0...31    |    0...31    |
  |______________|______________|______________|______________|______________|
*/
static K_MEM_SLAB_DEFINE(mem_slab, BLOCK_SIZE, NUM_BLOCKS, NUM_SAMPLES);

AudioModule::AudioModule() {
    LOG_DBG("AudioModule Constructor!");
}

int AudioModule::Initialize() {

    LOG_DBG("Starting AudioModule Initialization...");

    i2s_dev = DEVICE_DT_GET(DT_NODELABEL(i2s_rxtx));
    if (!device_is_ready(i2s_dev)) {
        LOG_ERR("%s is not ready\n", i2s_dev->name);
        return -1;
    } else {
        LOG_DBG("i2s_dev ready!");
    }

    struct i2s_config i2s_cfg;
    i2s_cfg.word_size = 16; // due to int16_t in data_frame declaration
    i2s_cfg.channels = 2; // L + R channel
    i2s_cfg.format = I2S_FMT_DATA_FORMAT_I2S;
    i2s_cfg.options = I2S_OPT_BIT_CLK_MASTER | I2S_OPT_FRAME_CLK_MASTER;
    i2s_cfg.frame_clk_freq = 44100;
    i2s_cfg.mem_slab = &mem_slab;
    i2s_cfg.block_size = BLOCK_SIZE;
    i2s_cfg.timeout = 1000;
    
    int ret = i2s_configure(i2s_dev, I2S_DIR_TX, &i2s_cfg);
    if (ret < 0) {
        LOG_ERR("Failed to configure the I2S stream: (%d)\n", ret);
        return ret;
    } else {
        LOG_DBG("i2s_configured successfully!");
    }

    ret = k_mem_slab_alloc(&mem_slab, &mem_blocks, K_NO_WAIT);
    if (ret < 0) {
        LOG_ERR("Failed to allocate the memory blocks: %d\n", ret);
        return ret;
    } else {
        LOG_DBG("slab allocated successfully!");
    }

    memset((uint16_t*)mem_blocks, 0, (NUM_SAMPLES * NUM_BLOCKS));

    Bluetooth::GattRegisterControlCallback(CommandId::AlarmCmd,
        [this](const uint8_t *buffer, Bluetooth::CommandKey key, Bluetooth::BleLength length, Bluetooth::BleOffset offset)
        {
            return OnBleCommand(buffer, key, length, offset);
        });

    // Start working thread
    k_thread_create(&worker, pollStackAreaAudio, K_THREAD_STACK_SIZEOF(pollStackAreaAudio),
                    &WorkingThreadAudio, this, nullptr, nullptr, taskPriorityAudio, 0, K_NO_WAIT);
    k_thread_suspend(&worker);

    return ret;    
}

bool AudioModule::OnBleCommand(const uint8_t *buffer, Bluetooth::CommandKey key, Bluetooth::BleLength length, Bluetooth::BleOffset offset){
    if (offset.value != 0 || length.value == 0)
    {
        return false;
    }
    LOG_DBG("AudioModule BLE Command received");

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

int AudioModule::StartSampling(){
    /* Start the transmission of data */
	int ret = i2s_trigger(i2s_dev, I2S_DIR_TX, I2S_TRIGGER_START);
	if (ret < 0) {
		LOG_ERR("Failed to start the transmission: %d", ret);
	} else {
        LOG_DBG("Resuming thread...");
        k_thread_resume(&worker);
    }
    return ret;
}

int AudioModule::StopSampling(){
	/* Start the transmission of data */
	int ret = i2s_trigger(i2s_dev, I2S_DIR_TX, I2S_TRIGGER_DRAIN);
	if (ret < 0) {
		LOG_ERR("Failed to stop the transmission: %d", ret);
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
    void AudioModule::WorkingThreadAudio(void *data, void *, void *){
        AudioModule *self = static_cast<AudioModule *>(data);
        int ret = 0;
        uint32_t idx;
        uint32_t j = 0;

        for (;;)
        {
            /* Put data into the tx buffer */
            for (int i = 0; i < NUM_SAMPLES * NUM_BLOCKS; i++) {
                idx = 2 * (NUM_BLOCKS * NUM_SAMPLES) * j + 2*i;
                ((uint16_t*)self->mem_blocks)[i] = ((self->rawAudioData[idx+1]) << 8) | self->rawAudioData[idx];
                if (j >= self->J_LIMIT){
                    ((uint16_t*)self->mem_blocks)[i] = 0;
                }
            }

            j++;
            if (j >= 2*self->J_LIMIT){
                j = 0;
            }
            /* Write Data */
            ret = i2s_buf_write(self->i2s_dev, self->mem_blocks, BLOCK_SIZE);
            if (ret < 0) {
                LOG_ERR("Error: i2s_write failed with %d", ret);
            }
        }

    }