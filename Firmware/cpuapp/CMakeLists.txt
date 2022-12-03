# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.17.0)

set(DTC_OVERLAY_FILE "nrf5340dk_nrf5340.overlay")

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(hello_world)

FILE(GLOB app_sources src/*.c*)
target_sources(app PRIVATE ${app_sources})

target_include_directories(app PRIVATE
    include
    include/Transports)

set_property(TARGET app PROPERTY CXX_STANDARD 17)    
