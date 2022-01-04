# nRF52 Biosensing Boards

This contains our working prototypes for using nRF52 microcontrollers (ARM + BLE5) with low cost sensors. [Doc](https://docs.google.com/document/d/1gOrWiBDynYziCYDMF4r1Rw85PXo9JbuBrYJEEkW2U-c/edit?usp=sharing). 

Features Zephyr RTOS test firmware, which is hardware-agnostic. [Firmware for BT832A prototype](https://github.com/moothyknight/BT832_Zephyr)

The current prototypes here are breakout boards, they are pin compatible with several different modules while it's easy to spin up new designs with similar products. Next versions will be further miniaturized.

The goal will be to have enough designs and versatility here to create any LOW COST biosensor combinations we want with the same hardware/firmware base. We need enough designs flowing to outmaneuver shortages and obsolescence, too. This keeps development overhead to a minimum for creating a wide array of products. Who wants to crash the market?

![breakout](Capture.PNG)

License: AGPL v3.0

Joshua Brewster, Jacob Tinkhauser, Bojan Jovanovich

Developed for creating open source [Brains@Play](https://brainsatplay.com) and [MyAlyce](https://github.com/myalyce/myalyce) wearables for neurofeedback and treatment recovery monitoring.
