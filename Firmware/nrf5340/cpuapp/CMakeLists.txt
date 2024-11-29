cmake_minimum_required(VERSION 3.20.0)

# set(DTC_OVERLAY_FILE "nrf5340dk_nrf5340.overlay")
set(DTS_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/dts")

# Get the parent directory of ZEPHYR_BASE
# get_filename_component(PARENT_DIR ${ZEPHYR_BASE} DIRECTORY)
# set(MODULE_DIR "${PARENT_DIR}/modules")


find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})


FILE(GLOB app_sources src/*.c*)
target_sources(app PRIVATE ${app_sources})

target_include_directories(app PRIVATE
                            include
                            include/Transports)

set_property(TARGET app PROPERTY CXX_STANDARD 17)

project(BT40_port_to_3.5)