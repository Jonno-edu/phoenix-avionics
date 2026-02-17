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

cmake_minimum_required(VERSION 3.13)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Default board (can be overridden via -DPICO_BOARD=...)
set(PICO_BOARD pico2 CACHE STRING "Board type")

include(pico_sdk_import.cmake)
project(phoenix_avionics_monorepo)

pico_sdk_init()

add_subdirectory(lib)

# Determine host vs Pico board
if(PICO_PLATFORM STREQUAL "host")
    set(BUILD_FOR_HOST TRUE)
    set(PICO_BOARD "host" CACHE STRING "Board type" FORCE)
    message(STATUS "Building for HOST (Mac) - Unit Tests")
elseif(PICO_BOARD STREQUAL "host")
    set(BUILD_FOR_HOST TRUE)
    message(STATUS "Building for HOST (Mac) - Unit Tests")
else()
    set(BUILD_FOR_HOST FALSE)
    if(NOT PICO_BOARD)
        set(PICO_BOARD pico2_w)
    endif()
    message(STATUS "Building for Pico Board: ${PICO_BOARD}")
endif()

# --- FreeRTOS setup (mirrors previous apps/main-obc behavior) ---
if(NOT FREERTOS_KERNEL_PATH)
    set(FREERTOS_KERNEL_PATH ${CMAKE_SOURCE_DIR}/lib/FreeRTOS-Kernel)
endif()

if(EXISTS "${FREERTOS_KERNEL_PATH}/tasks.c")
    message(STATUS "Using local FreeRTOS found at: ${FREERTOS_KERNEL_PATH}")
else()
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

# The application target
set(TARGET_NAME phoenix_avionics_monorepo)

# This is required to let FreeRTOS find our FreeRTOSConfig.h config file
add_library(freertos_config INTERFACE)
target_include_directories(freertos_config SYSTEM INTERFACE ${CMAKE_SOURCE_DIR}/include)

# Common source files
set(COMMON_SOURCES
    src/main.c
    bsp/bsp_init.c
    hal/platform_hal.c
    src/tasks/task_manager.c
    src/tasks/queue_manager.c
    src/modules/flight_control/pilot_tasks.c
    src/modules/communication/command_handler_task.c
    src/tasks/heartbeat_task.c
    src/modules/communication/rs485_task.c
    src/modules/communication/telemetry_task.c
    src/tasks/sensors_task.c
    src/tasks/hil_sensor_task.c
    src/core/usb_console.c
    ${CMAKE_SOURCE_DIR}/lib/esl-comms/src/rs485_protocol.c
    src/modules/communication/tctlm.c
    src/modules/data_store/obc_data.c
    src/core/obc_commands.c
    src/modules/data_store/rocket_data.c
    src/modules/subsystems/eps_node.c
    src/modules/subsystems/tracking_radio_node.c
    src/core/rs485_monitor.c
    ${CMAKE_SOURCE_DIR}/lib/esl-math/src/dcm.c
    ${CMAKE_SOURCE_DIR}/lib/esl-math/src/matrix.c
    ${CMAKE_SOURCE_DIR}/lib/esl-math/src/quaternion.c
    ${CMAKE_SOURCE_DIR}/lib/esl-math/src/vector2.c
    ${CMAKE_SOURCE_DIR}/lib/esl-math/src/vector3.c
    src/core/logging_shim.c
)

set(HARDWARE_SOURCES
    hal/rs485_hal.c
    ${CMAKE_SOURCE_DIR}/lib/esl-sensors/src/baroMS5607.c
    ${CMAKE_SOURCE_DIR}/lib/esl-sensors/src/tempTMP117.c
    ${CMAKE_SOURCE_DIR}/lib/esl-sensors/src/i2c.c
)

if(BUILD_FOR_HOST)
    add_executable(${TARGET_NAME} ${COMMON_SOURCES})
else()
    add_executable(${TARGET_NAME} ${COMMON_SOURCES} ${HARDWARE_SOURCES})
endif()

target_compile_definitions(${TARGET_NAME} PRIVATE
    $<$<NOT:$<BOOL:${BUILD_FOR_HOST}>>:PICO_BUILD=1>
)

target_include_directories(${TARGET_NAME} PRIVATE
    include
    ${CMAKE_SOURCE_DIR}/common/include
    ${CMAKE_SOURCE_DIR}/lib/esl-comms/include
    ${CMAKE_SOURCE_DIR}/lib/esl-math/include
    ${CMAKE_SOURCE_DIR}/src
    ${CMAKE_SOURCE_DIR}/src/tasks
    ${CMAKE_SOURCE_DIR}/src/modules/flight_control
    ${CMAKE_SOURCE_DIR}/src/bsp
    ${CMAKE_SOURCE_DIR}/src/modules
    ${CMAKE_SOURCE_DIR}/src/modules/communication
    ${CMAKE_SOURCE_DIR}
)

if(NOT BUILD_FOR_HOST)
    target_include_directories(${TARGET_NAME} PRIVATE
        ${CMAKE_SOURCE_DIR}/lib/esl-sensors/include
    )
endif()

if(BUILD_FOR_HOST)
    target_compile_definitions(${TARGET_NAME} PRIVATE HOST_BUILD=1)
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

    target_link_libraries(${TARGET_NAME} PRIVATE
        freertos_config
        FreeRTOS-Kernel
        FreeRTOS-Kernel-Heap3
    )
else()
    # For Pico builds, include FreeRTOS import helper (searches FREERTOS_KERNEL_PATH)
    include(FreeRTOS_Kernel_import.cmake)

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

    pico_enable_stdio_uart(${TARGET_NAME} 0)
    pico_enable_stdio_usb(${TARGET_NAME} 1)
endif()

# Convenience flash/run targets
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
    add_custom_target(flash
        COMMAND ${CMAKE_COMMAND} -E echo "Flash target is not applicable for Host builds"
        COMMENT "Ignoring flash request for Host build"
    )
    add_custom_target(run
        COMMAND ${CMAKE_COMMAND} -E echo "--- Starting Application ---"
        COMMAND $<TARGET_FILE:${TARGET_NAME}>
        DEPENDS ${TARGET_NAME}
        COMMENT "Running ${TARGET_NAME}..."
        USES_TERMINAL
    )
endif()
