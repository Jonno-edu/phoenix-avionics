cmake_minimum_required(VERSION 3.15)

# The name of our project
set(PROJECT_NAME phoenix-avionics)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 23)

# Detect if we're building for host (Mac) - check PICO_PLATFORM first
if(PICO_PLATFORM STREQUAL "host")
    set(BUILD_FOR_HOST TRUE)
    set(PICO_BOARD "host" CACHE STRING "Board type" FORCE)
    message(STATUS "Building for HOST (Mac) - Unit Tests")
elseif(PICO_BOARD STREQUAL "host")
    set(BUILD_FOR_HOST TRUE)
    message(STATUS "Building for HOST (Mac) - Unit Tests")
else()
    set(BUILD_FOR_HOST FALSE)
    # The board name (pico2_w for RP2350 + WiFi, unless overridden by variant)
    if(NOT PICO_BOARD)
        set(PICO_BOARD pico2_w)
    endif()
    message(STATUS "Building for Pico Board: ${PICO_BOARD}")
    
    # Pull in Raspberry Pi Pico SDK (must be before project)
    set(PICO_SDK_PATH "${CMAKE_CURRENT_LIST_DIR}/lib/pico-sdk")
    include(${PICO_SDK_PATH}/external/pico_sdk_import.cmake)
endif()

# Project declaration must come BEFORE pico_sdk_init()
project(${PROJECT_NAME} C CXX ASM)

# Initialise the Raspberry Pi Pico SDK (only for Pico targets)
if(NOT BUILD_FOR_HOST)
    pico_sdk_init()
    
    # Hide boot_stage2 targets from CMake Tools launch/debug UI
    set_property(GLOBAL PROPERTY CTEST_TARGETS_ADDED 1)
endif()

# --- ROBUST FREERTOS SETUP START ---

# 1. Define where we expect FreeRTOS to be
if(NOT FREERTOS_KERNEL_PATH)
    set(FREERTOS_KERNEL_PATH ${CMAKE_CURRENT_SOURCE_DIR}/lib/FreeRTOS-Kernel)
endif()

# 2. Check if it is fully valid (looks for tasks.c)
if(EXISTS "${FREERTOS_KERNEL_PATH}/tasks.c")
    message(STATUS "Using local FreeRTOS found at: ${FREERTOS_KERNEL_PATH}")
else()
    # 3. If the folder exists but tasks.c is missing, it's likely corrupt/empty.
    # We MUST delete it, otherwise git clone will fail with Error 1.
    if(EXISTS "${FREERTOS_KERNEL_PATH}")
        message(STATUS "FreeRTOS folder exists but is incomplete. Deleting to re-download...")
        file(REMOVE_RECURSE "${FREERTOS_KERNEL_PATH}")
    endif()

    message(STATUS "FreeRTOS not found locally. Downloading from GitHub...")
    include(FetchContent)
    FetchContent_Declare(freertos_kernel
        GIT_REPOSITORY https://github.com/FreeRTOS/FreeRTOS-Kernel.git
        GIT_TAG V11.2.0
        GIT_SUBMODULES "portable/ThirdParty/Community-Supported-Ports"
        SOURCE_DIR ${FREERTOS_KERNEL_PATH}
    )
    FetchContent_MakeAvailable(freertos_kernel)
endif()

# --- ROBUST FREERTOS SETUP END ---

# Include FreeRTOS for both Pico and Mac builds
if(NOT BUILD_FOR_HOST)
    include(FreeRTOS_Kernel_import.cmake)
    # Ensure the Kernel library sees the PICO_BUILD definition so FreeRTOSConfig.h works correctly
    target_compile_definitions(FreeRTOS-Kernel INTERFACE PICO_BUILD=1)
else()
    # For Mac host builds, manually add FreeRTOS
    add_library(FreeRTOS-Kernel STATIC
        ${FREERTOS_KERNEL_PATH}/tasks.c
        ${FREERTOS_KERNEL_PATH}/queue.c
        ${FREERTOS_KERNEL_PATH}/list.c
        ${FREERTOS_KERNEL_PATH}/timers.c
        ${FREERTOS_KERNEL_PATH}/event_groups.c
        ${FREERTOS_KERNEL_PATH}/stream_buffer.c
        ${FREERTOS_KERNEL_PATH}/portable/ThirdParty/GCC/Posix/port.c
        ${FREERTOS_KERNEL_PATH}/portable/ThirdParty/GCC/Posix/utils/wait_for_event.c
    )
    
    target_include_directories(FreeRTOS-Kernel PUBLIC
        ${FREERTOS_KERNEL_PATH}/include
        ${FREERTOS_KERNEL_PATH}/portable/ThirdParty/GCC/Posix
        ${FREERTOS_KERNEL_PATH}/portable/ThirdParty/GCC/Posix/utils
    )
    
    target_compile_definitions(FreeRTOS-Kernel PUBLIC
        _DARWIN_C_SOURCE
    )
    
    target_compile_options(FreeRTOS-Kernel PRIVATE
        -pthread
    )
    
    add_library(FreeRTOS-Kernel-Heap3 STATIC
        ${FREERTOS_KERNEL_PATH}/portable/MemMang/heap_3.c
    )
    
    target_link_libraries(FreeRTOS-Kernel-Heap3 PUBLIC FreeRTOS-Kernel)
    target_link_libraries(FreeRTOS-Kernel PUBLIC pthread)
endif()

set(TARGET_NAME ${PROJECT_NAME})

# This is required to let FreeRTOS find our FreeRTOSConfig.h config file
add_library(freertos_config INTERFACE)
target_include_directories(freertos_config SYSTEM INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/include)

# Link freertos_config to FreeRTOS-Kernel for Mac builds too
if(BUILD_FOR_HOST)
    target_link_libraries(FreeRTOS-Kernel PUBLIC freertos_config)
endif()

add_executable(${TARGET_NAME}
    src/main.c
    src/serial.c
    lib/esl-comms/src/rs485_protocol.c
    src/system_data.c
    lib/esl-sensors/src/baroMS5607.c
    lib/esl-sensors/src/i2c.c
    lib/esl-math/src/dcm.c
    lib/esl-math/src/matrix.c
    lib/esl-math/src/quaternion.c
    lib/esl-math/src/vector2.c
    lib/esl-math/src/vector3.c
)

target_compile_definitions(${TARGET_NAME} PRIVATE
    $<$<NOT:$<BOOL:${BUILD_FOR_HOST}>>:PICO_BUILD=1>
)

target_include_directories(${TARGET_NAME} PRIVATE
    include
    lib/esl-comms/include
    lib/esl-sensors/include
    lib/esl-math/include
)

if(BUILD_FOR_HOST)
    # Mac host build - only link FreeRTOS for unit testing
    target_compile_definitions(${TARGET_NAME} PRIVATE HOST_BUILD=1)
    target_link_libraries(${TARGET_NAME} PRIVATE
        freertos_config
        FreeRTOS-Kernel
        FreeRTOS-Kernel-Heap3
    )
else()
    # Pico build - link everything
    target_link_libraries(${TARGET_NAME} PRIVATE
        pico_stdlib
        hardware_gpio
        hardware_i2c
        pico_multicore
        pico_cyw43_arch_none

        freertos_config
        FreeRTOS-Kernel
        FreeRTOS-Kernel-Heap3
    )

    pico_add_extra_outputs(${TARGET_NAME})

    # Disable UART StdIO (unless you are using a Debug Probe)
    pico_enable_stdio_uart(${TARGET_NAME} 0)

    # Enable USB StdIO (For Serial Monitor & Picotool via USB)
    pico_enable_stdio_usb(${TARGET_NAME} 1)
endif()

# Convenience target: flash UF2 to Pico using picotool
if(NOT BUILD_FOR_HOST)
    find_program(PICOTOOL picotool)
    if(PICOTOOL)
        add_custom_target(flash
            COMMAND ${PICOTOOL} reboot -f -u || echo "Put Pico in BOOTSEL mode..."
            COMMAND ${CMAKE_COMMAND} -E sleep 2
            COMMAND ${PICOTOOL} load --ignore-partitions -x $<TARGET_FILE_DIR:${TARGET_NAME}>/${TARGET_NAME}.uf2
            WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
            DEPENDS ${TARGET_NAME}
            COMMENT "Flashing ${TARGET_NAME}.uf2 to Pico via picotool"
        )
    else()
        message(STATUS "picotool not found; 'flash' target will not be available")
    endif()
else()
    # Dummy flash target for host builds to prevent errors if 'flash' is selected
    add_custom_target(flash
        COMMAND ${CMAKE_COMMAND} -E echo "Flash target is not applicable for Host Mac builds"
        COMMENT "Ignoring flash request for Host build"
    )

    # 'run' target for Host builds to allow "Build & Run" workflow
    add_custom_target(run
        COMMAND ${CMAKE_COMMAND} -E echo "--- Starting Application ---"
        COMMAND $<TARGET_FILE:${TARGET_NAME}>
        DEPENDS ${TARGET_NAME}
        COMMENT "Running ${TARGET_NAME}..."
        USES_TERMINAL
    )
endif()
