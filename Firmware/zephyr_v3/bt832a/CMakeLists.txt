cmake_minimum_required(VERSION 3.20.0)
# Standalone application: no external source tree or configuration fragments.
include(${CMAKE_CURRENT_LIST_DIR}/board.cmake)
set(USER_CACHE_DIR "${CMAKE_BINARY_DIR}/.cache" CACHE PATH "Local compiler cache")
# The SDK DK runner also targets its onboard nRF52832; this is a real nRF52810.
# This application hook runs as the SDK finalizes each runner (NCS v3.3.1).
macro(app_set_runner_args)
  get_property(bio_jlink_args GLOBAL PROPERTY BOARD_RUNNER_ARGS_EXPLICIT_jlink)
  list(FILTER bio_jlink_args EXCLUDE REGEX "^--device=")
  set_property(GLOBAL PROPERTY BOARD_RUNNER_ARGS_EXPLICIT_jlink ${bio_jlink_args})
  board_runner_args(jlink "--device=nRF52810_xxAA")
  # The DK finalizes OpenOCD after pyOCD, before writing runner metadata.
  # Remove its already-registered nRF52832 pyOCD target in this hook.
  get_property(bio_runners GLOBAL PROPERTY ZEPHYR_RUNNERS)
  list(REMOVE_ITEM bio_runners pyocd)
  set_property(GLOBAL PROPERTY ZEPHYR_RUNNERS ${bio_runners})
endmacro()
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(biosensing)
if(CONFIG_SOC_NRF52810)
  # SDK nrf52dk/nrf52810 emulates an 810 on a 832. This carrier has real 810 silicon.
  get_property(bio_defs TARGET zephyr_interface PROPERTY INTERFACE_COMPILE_DEFINITIONS)
  list(REMOVE_ITEM bio_defs DEVELOP_IN_NRF52832 NRFX_COREDEP_DELAY_US_LOOP_CYCLES=3)
  set_property(TARGET zephyr_interface PROPERTY INTERFACE_COMPILE_DEFINITIONS "${bio_defs}")
endif()
target_include_directories(app PRIVATE include)
target_sources(app PRIVATE src/main.c src/app.c src/protocol.c src/transport.c src/ble.c)
target_sources_ifdef(CONFIG_BIO_ADS app PRIVATE src/ads131m08.c)
target_sources_ifdef(CONFIG_BIO_I2C_SENSORS app PRIVATE src/sensors.c)
target_sources_ifdef(CONFIG_BIO_USB app PRIVATE src/usb.c)
target_sources_ifdef(CONFIG_BIO_AUX_UART app PRIVATE src/aux_uart.c)
target_sources_ifdef(CONFIG_BIO_AUDIO app PRIVATE src/audio.c)
target_sources_ifdef(CONFIG_BIO_DMIC app PRIVATE src/dmic.c)
