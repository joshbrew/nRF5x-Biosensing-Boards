# Prevent a different SDK board from silently using this carrier's pin mapping.
if(DEFINED BOARD AND NOT BOARD STREQUAL "nrf52840dk/nrf52840")
  message(FATAL_ERROR "bt840_full: select board nrf52840dk/nrf52840; this folder has a fixed carrier pin mapping.")
endif()
