# Prevent a different SDK board from silently using this carrier's pin mapping.
if(DEFINED BOARD AND NOT BOARD STREQUAL "nrf5340dk/nrf5340/cpuapp")
  message(FATAL_ERROR "bt5340_min: select board nrf5340dk/nrf5340/cpuapp; this folder has a fixed carrier pin mapping.")
endif()
