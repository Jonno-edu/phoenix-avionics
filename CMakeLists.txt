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

# --- FreeRTOS setup ---
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

# FreeRTOS config - now at root
add_library(freertos_config INTERFACE)
target_include_directories(freertos_config SYSTEM INTERFACE ${CMAKE_SOURCE_DIR})

###########################################
# Source Files Organized by Module
###########################################

# Main entry point
set(MAIN_SOURCES
    src/main.c
)

# Board Support Package
set(BSP_SOURCES
    bsp/bsp_init.c
)

# Hardware Abstraction Layer
set(HAL_SOURCES
    hal/platform_hal.c
)

# Flight Control Module (Core 0)
# set(FLIGHT_CONTROL_SOURCES
#     src/modules/flight_control/pilot_tasks.c
# )

# Communication Module (Core 1)
# set(COMMUNICATION_SOURCES
#     src/modules/communication/command_handler_task.c
#     src/modules/communication/rs485_task.c
#     src/modules/communication/telemetry_task.c
#     src/modules/communication/tctlm.c
#     src/modules/communication/rs485_monitor.c
#     src/modules/communication/obc_commands.c
# )

# Data Store Module
# set(DATA_STORE_SOURCES
#     src/modules/data_store/obc_data.c
#     src/modules/data_store/rocket_data.c
# )

# Task Manager & Queues
# set(TASK_SOURCES
#     src/tasks/queue_manager.c
# )

# Utils & Console
# set(UTIL_SOURCES
#     common/logging_shim.c
#     common/usb_console.c
# )

# # Subsystems Module
# set(SUBSYSTEMS_SOURCES
#     src/modules/subsystems/eps_node.c
#     src/modules/subsystems/tracking_radio_node.c
# )

# # Shared Libraries
# set(COMMS_LIB_SOURCES
#     ${CMAKE_SOURCE_DIR}/lib/esl-comms/src/rs485_protocol.c
# )

# set(MATH_LIB_SOURCES
#     ${CMAKE_SOURCE_DIR}/lib/esl-math/src/dcm.c
#     ${CMAKE_SOURCE_DIR}/lib/esl-math/src/matrix.c
#     ${CMAKE_SOURCE_DIR}/lib/esl-math/src/quaternion.c
#     ${CMAKE_SOURCE_DIR}/lib/esl-math/src/vector2.c
#     ${CMAKE_SOURCE_DIR}/lib/esl-math/src/vector3.c
# )

# Hardware-specific sources (only for Pico builds)
# set(HARDWARE_SOURCES
#     hal/rs485_hal.c
#     ${CMAKE_SOURCE_DIR}/lib/esl-sensors/src/baroMS5607.c
#     ${CMAKE_SOURCE_DIR}/lib/esl-sensors/src/tempTMP117.c
#     ${CMAKE_SOURCE_DIR}/lib/esl-sensors/src/i2c.c
# )

# Combine common sources
set(COMMON_SOURCES
    ${MAIN_SOURCES}
    # ${BSP_SOURCES}
    # ${HAL_SOURCES}
    # ${FLIGHT_CONTROL_SOURCES}
    # ${COMMUNICATION_SOURCES}
    # ${DATA_STORE_SOURCES}
    # ${TASK_SOURCES}
    # ${SUBSYSTEMS_SOURCES}
    # ${UTIL_SOURCES}
    # ${COMMS_LIB_SOURCES}
    # ${MATH_LIB_SOURCES}
)

###########################################
# Build Executable
###########################################

if(BUILD_FOR_HOST)
    add_executable(${TARGET_NAME} ${COMMON_SOURCES})
else()
    add_executable(${TARGET_NAME} ${COMMON_SOURCES} ${HARDWARE_SOURCES})
endif()

target_sources(${TARGET_NAME} PRIVATE
    src/FreeRTOS-openocd.c
)

# Preserve FreeRTOS symbols for debugger awareness and force debug optimization
target_compile_options(${TARGET_NAME} PRIVATE 
    $<$<CONFIG:Debug>:-O0 -fno-lto -fno-optimize-sibling-calls>
)

target_link_options(${TARGET_NAME} PRIVATE
    -Wl,--undefined=pxCurrentTCB
    -Wl,--undefined=pxReadyTasksLists
    -Wl,--undefined=uxCurrentNumberOfTasks
)

target_compile_definitions(${TARGET_NAME} PRIVATE
    $<$<NOT:$<BOOL:${BUILD_FOR_HOST}>>:PICO_BUILD=1>
)

###########################################
# Include Directories (NEW MODULE STRUCTURE)
###########################################

target_include_directories(${TARGET_NAME} PRIVATE
    .                                         # For FreeRTOSConfig.h at root
    src                                       # For modules/xxx/yyy.h includes
    # bsp                                       # BSP headers
    # hal                                       # HAL headers
    # ${CMAKE_SOURCE_DIR}/common/include        # Common utilities
    # ${CMAKE_SOURCE_DIR}/lib/esl-comms/include # RS485 protocol
    # ${CMAKE_SOURCE_DIR}/lib/esl-math/include  # Math utilities
)

if(NOT BUILD_FOR_HOST)
    target_include_directories(${TARGET_NAME} PRIVATE
        ${CMAKE_SOURCE_DIR}/lib/esl-sensors/include
    )
endif()

###########################################
# Build Type Specific Configuration
###########################################

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
    # For Pico builds
    include(FreeRTOS_Kernel_import.cmake)

    target_link_libraries(${TARGET_NAME} PRIVATE
        pico_stdlib
        hardware_gpio
        hardware_i2c
        pico_multicore

        freertos_config
        FreeRTOS-Kernel
        FreeRTOS-Kernel-Heap3
    )

    pico_add_extra_outputs(${TARGET_NAME})

    pico_enable_stdio_uart(${TARGET_NAME} 0)
    pico_enable_stdio_usb(${TARGET_NAME} 1)
endif()

###########################################
# Custom Targets
###########################################

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
