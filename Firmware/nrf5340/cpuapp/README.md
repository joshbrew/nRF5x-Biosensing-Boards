# nrf5340 Zephyr
Working repository for the nRF52 Zephyr firmware, made for a [BT40/BC40M prototype](https://github.com/moothyknight/nRF52-Biosensing-Boards)

Recommended build tools: nRFConnect with VSCode. 

For pinouts refer to datasheet: https://www.mouser.com/datasheet/2/915/Fanstel_08262021_BT40F_Product_Specificationsv1_01-2530184.pdf or see the BT-series CAD folder for pinout screenshots.

Note, the BT-series design is using the BT840 footprint, so correlate the pins via the datasheet.

Instructions:
- [Install](https://nrfconnect.github.io/vscode-nrf-connect/) nRFConnect for VSCode.  We used SDK version 1.9.1 in the toolchain manager.
- Open this repo in VSCode and launch the nRFConnect menu within VSCode
- In VSCode, in the nRFConnect extension under the APPLICATIONS tab, select the "Add Build Configuration" button to the right of the main folder name (cpuapp). Select the correct board, nrf5340dk_nrf5340_cpuapp, and click Build Configuration. Make sure prj.conf is selected as the configuration below it.
- Connect the nRF52-dk via usb with the flash pins wired to the custom PCB. Note: wire VTG to VDD on the DK, then VDDnRF to the custom PCB's VDD pin, then the rest of the SW/Reset/Gnd pins.
- In VSCode, in the nRFConnect extension under the ACTIONS tab, select "Flash" to build and flash the code


Include the extra Cmake argument in the build configuration:
- `-DDTC_OVERLAY_FILE:STRING="path/to/nrf5340dk_nrf5340.overlay"`

For cpunet do the same thing, but without the overlay file.

- You may edit pinouts in the main.cpp and overlay files.

You may need to manually install gnu-arm-embedded software. 

We used the nRF53 (53! The nRF52DK did not work for us) development board which is a J-LINK device for flashing. 

Was also able to use a 1 dollar STM32 blue pill to flash a built hex file manually.

Device sensor test: http://modules.brainsatplay.com/

Debugger demo: https://devicedebugger.netlify.app/ 

