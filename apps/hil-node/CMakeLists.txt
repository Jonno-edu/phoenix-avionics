add_executable(hil_node
    src/main.c
)

# Reference definitions
target_compile_definitions(hil_node PRIVATE
    PICO_BUILD=1
)

# Enable USB output, Disable UART output (optional, but cleaner)
pico_enable_stdio_usb(hil_node 1)
pico_enable_stdio_uart(hil_node 0)

# Link libraries (SIMPLEST)
target_link_libraries(hil_node
    pico_stdlib
    pico_cyw43_arch_none
)

# Generate UF2
pico_add_extra_outputs(hil_node)
