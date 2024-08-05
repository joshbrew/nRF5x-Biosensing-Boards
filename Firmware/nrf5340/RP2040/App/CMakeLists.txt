cmake_minimum_required(VERSION 3.12)


project(App C CXX ASM)

set(CMAKE_C_STANDARD 11)

set(CMAKE_CXX_STANDARD 17)

set(PICO_EXAMPLES_PATH ${PROJECT_SOURCE_DIR})

# Import Pico SDK
include(pico_sdk_import.cmake)

# Initialize the SDK

pico_sdk_init()

include(example_auto_set_url.cmake)

# Add main Folder

add_subdirectory(main)