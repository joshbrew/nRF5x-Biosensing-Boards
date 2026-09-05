cmake_minimum_required(VERSION 3.12)

# Import Pico SDK
include(pico_sdk_import.cmake)

# set(PICO_SDK_PATH "C:/Program Files/Raspberry Pi/Pico SDK v1.5.1/pico-sdk")
# set(PICO_TOOLCHAIN_PATH "C:/Program Files/Raspberry Pi/Pico SDK v1.5.1/gcc-arm-none-eabi")

# Set automatically if you have an enviornment variable PATH set to points to the directory for these 3 depedndencies
# set(CMAKE_C_COMPILER "C:/Program Files/Microsoft Visual Studio/2022/Community/VC/Tools/MSVC/14.39.33519/bin/Hostx64/x64/cl.exe")
# set(CMAKE_CXX_COMPILER "C:/Program Files/Microsoft Visual Studio/2022/Community/VC/Tools/MSVC/14.39.33519/bin/Hostx64/x64/cl.exe")
# set(CMAKE_ASM_COMPILER "C:/Program Files/Microsoft Visual Studio/2022/Community/VC/Tools/MSVC/14.39.33519/bin/Hostx64/x64/ml64.exe")

project(App C CXX ASM)

set(CMAKE_C_STANDARD 11)

set(CMAKE_CXX_STANDARD 17) # Visual Studio 17 2022

# Initialize the SDK

pico_sdk_init()

# include(example_auto_set_url.cmake)

# Add main Folder

add_subdirectory(main)