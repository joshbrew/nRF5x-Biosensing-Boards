# RPI PICO C++ Project Setup Guide

This guide walks you through installing and building a C++ project for Raspberry Pi Pico using Visual Studio Code and the Pico SDK.

## Installation Steps

### Platform-Specific Instructions

If you are on a platform other than Windows, please refer to the official Raspberry Pi Pico documentation for detailed instructions:

[Getting Started with Raspberry Pi Pico (PDF)](https://datasheets.raspberrypi.com/pico/getting-started-with-pico.pdf)

1. **Download the Latest Release:**
   - Visit the [GitHub repository](https://github.com/example/repository).
   - Download the latest release by clicking "Download the latest release."

2. **Install the Executable:**
   - Run the downloaded executable installer.
   - Follow on-screen instructions, ensuring to select "Clone and build example in this folder" before clicking "Finish."
   - ![image](https://i.imgur.com/ddJbKNg.png)
   - If you see this output then everything is installed properly.
   - ![image](https://i.imgur.com/SSjUpyQ.png)


3. **Open Project in Visual Studio Code:**
   - Navigate to the project folder where the executable was installed.
   - Open the folder in Visual Studio Code.

4. **Configure GCC 10.3.1 ARM-none-eabi Compiler:**
   - If not already installed, install the CMake extension.
   - Navigate to the CMake extension in Visual Studio Code.
   - Configure GCC 10.3.1 ARM-none-eabi compiler.
   - ![image](https://i.imgur.com/IxmM66O.png)


## Troubleshooting

If you encounter errors during setup, follow these steps:

- **PICO_SDK_PATH Error:**
  - If encountering errors related to `PICO_SDK_PATH`, add a new environmental variable:
    - Variable Name: `PICO_SDK_PATH`
    - Variable Value: `C:\Program Files\Raspberry Pi\Pico SDK v1.5.1\pico-sdk`

- **Ninja Error:**
  - If encountering errors related to Ninja, add the following path in environmental variables :
    - `C:\Program Files\Raspberry Pi\Pico SDK v1.5.1\ninja`

## Building the Project

Once setup is complete and any issues are addressed:

- Click "Build" within Visual Studio Code to build the project.

## UF2 File Location

After successfully building the project, the UF2 file will be located in the build folder.

## updateWrapValue Calculation Steps

1. **PWM Timer Peripheral Clock Frequency Calculation:**
    - The PWM clock divider divides the Raspberry Pi clock frequency by 256, resulting in a PWM timer peripheral clock frequency of 488,281.25 Hz.

2. **Timer Tick Duration Calculation:**
    - With the PWM timer peripheral clock frequency, each timer tick duration is calculated as 2.048 microseconds (us).

3. **Target PWM Frequency:**
    - The desired PWM frequency is set to 250 Hz. We can select 500, 1000 or 2000.

4. **Counter Value Calculation:**
    - To achieve the desired PWM frequency, the total number of timer ticks required per cycle is calculated by dividing the PWM timer peripheral clock frequency (488,281.25 Hz) by the desired PWM frequency (250 Hz), resulting in approximately 1953.125 ticks.

5. **Total Cycle Duration Calculation:**
    - Multiplying the tick duration (2.048 us) by the total number of ticks (1953) gives the total cycle duration of the PWM signal, which is approximately 4 milliseconds (ms). This results in a PWM signal with a frequency of 250 Hz.

