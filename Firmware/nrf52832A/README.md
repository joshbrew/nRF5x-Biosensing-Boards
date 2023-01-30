# BT832_Zephyr
Working repository for the nRF52 Zephyr firmware, made for a [BT832A prototype (24KB)](https://github.com/moothyknight/nRF52-Biosensing-Boards/tree/main/CAD/BT832_OLD)

Instructions:
- [Install](https://nrfconnect.github.io/vscode-nrf-connect/) nRFConnect for VSCode
- Open this repo in VSCode
- In VSCode, in the nRFConnect extension under the APPLICATIONS tab, select the "Add Build Configuration" button to the right of the repo name, BT832_Zephyr. Select the correct board, nrf52dk_nrf52810, and click Build Configuration.
- Connect the nRF52DK via usb with the flash pins wired to the custom PCB. Note: wire VTG to VDD on the DK, then VDDnRF to the custom PCB's VDD pin, then the rest of the SW/Reset/Gnd pins.
- In VSCode, in the nRFConnect extension under the ACTIONS tab, select "Flash" to build and flash the code

Extra CMAKE Argument:
`-DDTC_OVERLAY_FILE:STRING="path/to/zephyr/nrf52dk_nrf52810.overlay"`

You may edit the pin configurations in the main.cpp and overlay files.

You may need to manually install gnu-arm-embedded software. 

We used the nRF52 development board which is a J-LINK device for flashing.


Device sensor test: http://modules.brainsatplay.com/

Debugger demo: https://devicedebugger.netlify.app/ 

The BT832A can only run the EMG chip due to the 24kb size limit, but it's dirt cheap.

