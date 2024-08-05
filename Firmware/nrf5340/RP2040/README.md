# RPI PICO C++ Project

[Windows SDK installation](https://www.raspberrypi.com/news/raspberry-pi-pico-windows-installer/)

You also need VSCommunity 2022 in the current config to access the C/C++/ASM compilers and NMake, and ensure the tool path is in your System or User ENV Path:

e.g. C:\Program Files\Microsoft Visual Studio\2022\Community\VC\Tools\MSVC\14.39.33519\bin\Hostx64\x64

Also set the env variables (or declare manually in CMakeLists.txt)

PICO_SDK_PATH=C:\Program Files\Raspberry Pi\Pico SDK v1.5.1\pico-sdk

PICO_TOOLCHAIN_PATH=C:\Program Files\Raspberry Pi\Pico SDK v1.5.1\gcc-arm-none-eabi

In Windows, if using the installer provided at the previous link:

[Pico SDK github](https://github.com/raspberrypi/pico-sdk)

[Raspberry Pi Pico SDK PDF (linux oriented)](https://datasheets.raspberrypi.com/pico/getting-started-with-pico.pdf)


# Build

In VSCode, open RP2040

`cd App` (if starting in this README directory)

`mkdir binaries` or `cd ./binaries`

`cmake -G "NMMake Makefiles" ..`

`nmake`

# UF2 Binary

In `./binaries` after bundling, you can drag and drop main.uf2 into your RP2040 plugged in via USB to program it.

This will remove the USB detection when it flashes successfully. To reset, on the RP2040, hold down the reset button, then hold down the boot button, and release the reset button. This will put it back to factory settings. 

# COMMANDS

The RP2040 expects commands to begin alternating LEDs, you can set up more with in the main.cpp

- `A\n` - 250Hz period for pins defined in main();
- `B\n` - 500Hz period
- `C\n` - 1000Hz period
- `D\n` - 2000Hz period

Stop PWM Command:

- `STOP\n`: Stops the PWM operation.
Enter Deep Sleep Command:

- `SLEEP\n`: Puts the system into deep sleep for a specified duration.
Wake Up and Resume PWM Command:

- `WAKE\n`: Wakes up the system and resumes PWM operation.
Set Custom Parameters:

Commands with a parameter followed by a value, separated by a space.
Examples:
- `PULSEWIDTH 300\n`: Sets pulseWidthUs to 300 microseconds.
- `PERIOD 4000\n`: Sets periodUs to 4000 microseconds.
- `INITIALDELAY 500\n`: Sets initialDelayUs to 500 microseconds.