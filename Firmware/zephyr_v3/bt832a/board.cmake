# Prevent a different SDK board from silently using this carrier's pin mapping.
if(DEFINED BOARD AND NOT BOARD STREQUAL "nrf52dk/nrf52810")
  message(FATAL_ERROR "bt832a: select board nrf52dk/nrf52810; this folder has a fixed carrier pin mapping.")
endif()
