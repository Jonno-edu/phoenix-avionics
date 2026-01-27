add_executable(sensor_node
    src/main.c
)

# Include the root libraries
target_include_directories(sensor_node PRIVATE
    ${CMAKE_SOURCE_DIR}/lib/esl-comms/include
    # Add other includes as needed
)

# Link libraries
target_link_libraries(sensor_node
    pico_stdlib
    # Add FreeRTOS or other libs if needed
)

# Generate UF2
pico_add_extra_outputs(sensor_node)
