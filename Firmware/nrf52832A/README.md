# BT832_Zephyr
Working repository for the nRF52 Zephyr firmware, made for a [BT832A prototype (24KB)](https://github.com/moothyknight/nRF52-Biosensing-Boards/tree/main/CAD/BT832_OLD)

Instructions:
- [Install](https://nrfconnect.github.io/vscode-nrf-connect/) nRFConnect for VSCode
- Open this repo in VSCode
- In VSCode, in the nRFConnect extension under the APPLICATIONS tab, select the "Add Build Configuration" button to the right of the repo name, BT832_Zephyr. Select the correct board, nrf52840dk_nrf52840, and click Build Configuration.
- Connect the nRF52-dk via usb
- In VSCode, in the nRFConnect extension under the ACTIONS tab, select "Flash" to build and flash the code

Extra CMAKE Argument:
`-DDTC_OVERLAY_FILE:STRING="path/to/zephyr/nrf52dk_nrf52810.overlay"`

You may need to manually install gnu-arm-embedded software. 

We used the nRF52 development board which is a J-LINK device for flashing.


