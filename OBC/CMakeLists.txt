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

# Default board (can be overridden via -DPICO_BOARD=...).
# Do NOT set a default when targeting the host platform — the Pico SDK does not
# need a board definition in that case and will error if given a hardware board.
if(NOT PICO_PLATFORM STREQUAL "host")
    set(PICO_BOARD pico2 CACHE STRING "Board type")
endif()

include(pico_sdk_import.cmake)
project(phoenix_avionics_monorepo)

pico_sdk_init()

add_subdirectory(lib)

# Determine host vs Pico board
if(PICO_PLATFORM STREQUAL "host")
    set(BUILD_FOR_HOST TRUE)
    set(PICO_BOARD "host" CACHE STRING "Board type" FORCE)
    set(PICO_BUILD 0 CACHE STRING "Building for Pico" FORCE)
    message(STATUS "Building for HOST (Mac) - Unit Tests")
elseif(PICO_BOARD STREQUAL "host")
    set(BUILD_FOR_HOST TRUE)
    set(PICO_BUILD 0 CACHE STRING "Building for Pico" FORCE)
    message(STATUS "Building for HOST (Mac) - Unit Tests")
else()
    set(BUILD_FOR_HOST FALSE)
    if(NOT PICO_BUILD)
        set(PICO_BUILD 1 CACHE STRING "Building for Pico" FORCE)
    endif()
    if(NOT PICO_BOARD)
        set(PICO_BOARD pico2_w)
    endif()
    message(STATUS "Building for Pico Board: ${PICO_BOARD}")
endif()

# Global definition for IntelliSense and code gating
add_definitions(-DPICO_BUILD=${PICO_BUILD})

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

# --- Estimator module (owns Eigen3, SymForce gen, estimator_lib, and unit tests) ---
add_subdirectory(src/modules/estimator)

###########################################
# nORB Topic Code Generation
###########################################

set(NORB_MSG_DIR ${CMAKE_SOURCE_DIR}/msg)
set(NORB_GEN_DIR ${CMAKE_BINARY_DIR}/norb_generated)

# Glob .msg files. CONFIGURE_DEPENDS: CMake re-runs configure when files are
# added or removed from the msg/ directory.
file(GLOB NORB_MSG_FILES CONFIGURE_DEPENDS "${NORB_MSG_DIR}/*.msg")

if(NOT NORB_MSG_FILES)
    message(FATAL_ERROR "No .msg files found in ${NORB_MSG_DIR}")
endif()

# Build the expected output list so CMake knows what the command produces.
set(NORB_GEN_OUTPUTS
    ${NORB_GEN_DIR}/norb/topics.h
    ${NORB_GEN_DIR}/norb_autogen.c
)
foreach(msg_file ${NORB_MSG_FILES})
    get_filename_component(topic_name ${msg_file} NAME_WE)
    list(APPEND NORB_GEN_OUTPUTS ${NORB_GEN_DIR}/norb/topic_defs/${topic_name}.h)
endforeach()

add_custom_command(
    OUTPUT  ${NORB_GEN_OUTPUTS}
    COMMAND python3 ${CMAKE_SOURCE_DIR}/tools/generate_norb_topics.py
            --outdir  ${NORB_GEN_DIR}
            --topics  ${NORB_MSG_FILES}
    DEPENDS ${NORB_MSG_FILES} ${CMAKE_SOURCE_DIR}/tools/generate_norb_topics.py
    COMMENT "Generating nORB topic headers and init code from .msg files"
    VERBATIM
)

# Convenience target so IDEs can trigger generation without a full build.
add_custom_target(norb_generate DEPENDS ${NORB_GEN_OUTPUTS})

# estimator_lib compiles imu_predictor.c which includes vehicle_imu.h.
# Ensure the nORB headers are generated first.
add_dependencies(estimator_lib norb_generate)

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

set(MATH_LIB_SOURCES
    ${CMAKE_SOURCE_DIR}/lib/esl-math/src/dcm.c
    ${CMAKE_SOURCE_DIR}/lib/esl-math/src/matrix.c
    ${CMAKE_SOURCE_DIR}/lib/esl-math/src/quaternion.c
    ${CMAKE_SOURCE_DIR}/lib/esl-math/src/vector2.c
    ${CMAKE_SOURCE_DIR}/lib/esl-math/src/vector3.c
)

# Hardware-specific sources (only for Pico builds)
set(HARDWARE_SOURCES
    hal/rs485_hal.c
    ${CMAKE_SOURCE_DIR}/lib/esl-sensors/src/baroMS5607.c
    ${CMAKE_SOURCE_DIR}/lib/esl-sensors/src/tempTMP117.c
    ${CMAKE_SOURCE_DIR}/lib/esl-sensors/src/i2c.c
)

# Combine common sources
set(COMMON_SOURCES
    ${MAIN_SOURCES}
    src/norb/norb.c
    ${NORB_GEN_DIR}/norb_autogen.c   # generated — populated by norb_generate
    src/modules/sensors/sensors.c
    src/modules/sensors/imu.c
    src/modules/sensors/integrator.c
    src/modules/sensors/sim/sim_imu.c
    src/modules/estimator/ekf.c
    src/modules/datalink/datalink.c
    src/modules/datalink/receiver.c
    src/modules/datalink/tctlm.c
    src/modules/datalink/norb_streamer.c
    src/modules/nodes/tracking_radio/tracking_radio.c
    src/modules/nodes/eps/eps.c
    src/modules/housekeeping/housekeeping.c
    ${CMAKE_SOURCE_DIR}/lib/esl-comms/src/rs485_protocol.c
    ${BSP_SOURCES}
    ${HAL_SOURCES}
    # ${FLIGHT_CONTROL_SOURCES}
    # ${COMMUNICATION_SOURCES}
    # ${DATA_STORE_SOURCES}
    # ${TASK_SOURCES}
    # ${SUBSYSTEMS_SOURCES}
    # ${UTIL_SOURCES}
    # ${COMMS_LIB_SOURCES}
    ${MATH_LIB_SOURCES}
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

    # # ── NASA Power of Ten / DO-178C strict auditing ────────────────────────
    # # Applied to project sources only (not FreeRTOS/Pico SDK) via PRIVATE.

    # # Rule 10: Zero Warnings — treat every warning as a hard error
    # -Wall               # Standard warning set
    # -Wextra             # Extra warnings (sign comparisons, unused params, etc.)
    # -Wpedantic          # Enforce ISO C; reject non-portable GNU extensions
    # -Werror             # Warnings are build failures

    # # Rule 7: Check all return values
    # -Werror=unused-result   # Ignoring a return value is a build error

    # # Rule 6: Smallest possible scope — catch name shadowing
    # -Wshadow            # Local variable shadows a global/outer-scope variable

    # # Rules 3 & 9: Memory and pointer safety
    # -Wcast-qual         # Casting away const is forbidden
    # -Wconversion        # Implicit narrowing conversions (e.g. uint32_t → int16_t)
    # -Wnull-dereference  # Statically detectable null-pointer dereferences

    # # DO-178C: No hidden dynamic stack allocation
    # -Wvla               # Variable Length Arrays are forbidden
    # -Walloca            # alloca() is forbidden
)



target_link_options(${TARGET_NAME} PRIVATE
    -Wl,--undefined=pxCurrentTCB
    -Wl,--undefined=pxReadyTasksLists
    -Wl,--undefined=uxCurrentNumberOfTasks
)

###########################################
# Include Directories
###########################################

target_include_directories(${TARGET_NAME} PRIVATE
    ${NORB_GEN_DIR}                          # Generated: norb/topics.h, norb/topic_defs/*.h
    .                                        # For FreeRTOSConfig.h at root
    src                                      # For modules/xxx/yyy.h includes
    ${CMAKE_SOURCE_DIR}/lib/phoenix-icd/include   # phoenix_icd.h (shared ICD)
    bsp                                      # BSP headers
    hal                                      # HAL headers
    ${CMAKE_SOURCE_DIR}/lib/esl-comms/include # RS485 protocol
    ${CMAKE_SOURCE_DIR}/lib/esl-math/include  # Math utilities
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
    
    target_link_libraries(FreeRTOS-Kernel PUBLIC pthread)

    target_link_libraries(${TARGET_NAME} PRIVATE
        freertos_config
        FreeRTOS-Kernel
        estimator_lib
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
        estimator_lib
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
