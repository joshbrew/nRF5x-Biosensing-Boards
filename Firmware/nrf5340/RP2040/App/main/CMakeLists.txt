if (TARGET tinyusb_device)
    add_executable(main 
        main.cpp 
        pwmcontroller.cpp 
        uartcontroller.cpp
        WS2812.cpp
    )


    pico_generate_pio_header(main ${CMAKE_CURRENT_LIST_DIR}/WS2812.pio)

    pico_enable_stdio_usb(main 1)
    pico_enable_stdio_uart(main 0)

    # pull in common dependencies
    target_link_libraries(main pico_stdlib pico_multicore hardware_pwm hardware_rtc hardware_clocks hardware_pio)


    # create map/bin/hex/uf2 file etc.
    pico_add_extra_outputs(main)

elseif(PICO_ON_DEVICE)
    message(WARNING "not building hello_usb because TinyUSB submodule is not initialized in the SDK")
endif()
