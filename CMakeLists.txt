cmake_minimum_required(VERSION 3.13)
include(pico_sdk_import.cmake)
project(phoenix_avionics_monorepo)

pico_sdk_init()
add_subdirectory(lib)

# Ensure BUILD_TARGET is cached so external tasks can read it
if(NOT DEFINED BUILD_TARGET)
    set(BUILD_TARGET "ALL" CACHE STRING "Target to build (OBC, SENSOR, ALL)" FORCE)
else()
    set(BUILD_TARGET "${BUILD_TARGET}" CACHE STRING "Target to build (OBC, SENSOR, ALL)" FORCE)
endif()

message(STATUS "Selected Build Target: ${BUILD_TARGET}")

# Logic to select subdirectories
if(BUILD_TARGET STREQUAL "OBC" OR BUILD_TARGET STREQUAL "ALL")
    add_subdirectory(apps/main-obc)
endif()

if(BUILD_TARGET STREQUAL "SENSOR" OR BUILD_TARGET STREQUAL "ALL")
    add_subdirectory(apps/sensor-node)
endif()
