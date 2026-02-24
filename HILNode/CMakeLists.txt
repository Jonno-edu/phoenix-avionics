# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.2.0)
set(toolchainVersion 14_2_Rel1)
set(picotoolVersion 2.2.0-a4)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
set(PICO_BOARD pico2 CACHE STRING "Board type")

cmake_minimum_required(VERSION 3.13)

# Include Pico SDK import (must be before project)
include(pico_sdk_import.cmake)

# Set project name
project(embedded-hil-simulator C CXX ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Initialize the SDK
pico_sdk_init()

# Add your executable
add_executable(embedded-hil-simulator
    src/main.c
    # Add other source files here
)

# Link libraries
target_link_libraries(embedded-hil-simulator 
    pico_stdlib
    hardware_i2c
    pico_i2c_slave
    # Add other libraries here
)

# Project include directory for local headers (e.g. include/hil_protocol.h)
target_include_directories(embedded-hil-simulator PRIVATE ${CMAKE_SOURCE_DIR}/include)
# Enable USB/UART output
pico_enable_stdio_usb(embedded-hil-simulator 1)
pico_enable_stdio_uart(embedded-hil-simulator 0)

# Create additional outputs (UF2, etc.)
pico_add_extra_outputs(embedded-hil-simulator)
