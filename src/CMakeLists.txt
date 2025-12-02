add_executable(${PROJECT_NAME}
    main.c
)

if(PICO_PLATFORM STREQUAL "host")
    # Host build specific settings
    target_link_libraries(${PROJECT_NAME}
        freertos_kernel
        freertos_config
        freertos_heap
    )
else()
    # Pico build specific settings
    target_link_libraries(${PROJECT_NAME}
        pico_stdlib
        freertos_kernel
        freertos_config
        freertos_heap
    )

    # Enable USB output
    pico_enable_stdio_usb(${PROJECT_NAME} 1)
    pico_enable_stdio_uart(${PROJECT_NAME} 1)

    # Create UF2 file
    pico_add_extra_outputs(${PROJECT_NAME})
endif()
