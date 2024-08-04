# RPI PICO C++ Project


[Windows SDK installation](https://www.raspberrypi.com/news/raspberry-pi-pico-windows-installer/)

You need to set two environment variables afterward.

PICO_SDK_PATH

PICO_TOOLCHAIN_PATH

In Windows, if using the installer provided at the previous link:

PICO_SDK_PATH=C:\Program Files\Raspberry Pi\Pico SDK v1.5.1\pico-sdk

PICO_TOOLCHAIN_PATH=C:\Program Files\Raspberry Pi\Pico SDK v1.5.1\gcc-arm-none-eabi

[Pico SDK github](https://github.com/raspberrypi/pico-sdk)

[Raspberry Pi Pico SDK PDF (linux oriented)](https://datasheets.raspberrypi.com/pico/getting-started-with-pico.pdf)


# Build

`cmake CMakeLists.txt`

# UF2 Binary

In `./binaries` after bundling, you can drag and drop main.uf2 into your RP2040 plugged in via USB to program it.

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