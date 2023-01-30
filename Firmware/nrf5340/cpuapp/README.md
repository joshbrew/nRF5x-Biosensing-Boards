# nrf5340 Zephyr
Working repository for the nRF52 Zephyr firmware, made for a [BT40/BC40M prototype](https://github.com/moothyknight/nRF52-Biosensing-Boards)

Recommended build tools: nRFConnect with VSCode. 

In the nRFConnect for VSCode extension, install the zephyr toolchain and select `nrf52840dke_nrf52840` board.

Include the extra Cmake argument:
- `-DDTC_OVERLAY_FILE:STRING="path/to/nrf5340dk_nrf5340.overlay"`

- You may edit pinouts in the main.cpp and overlay files.

You may need to manually install gnu-arm-embedded software. 

We used the nRF52 development board which is a J-LINK device for flashing. 

Was also able to use a 1 dollar STM32 blue pill to flash a built hex file manually. 

Device sensor test: http://modules.brainsatplay.com/

Debugger demo: https://devicedebugger.netlify.app/ 
