#pragma once

/**
 * @brief Initialise all simulated sensor drivers.
 *
 * Call once from sensors_init() when building without physical hardware.
 * Creates the simulated IMU task which publishes TOPIC_SENSOR_IMU at 1 kHz.
 */
void sim_sensors_init(void);
