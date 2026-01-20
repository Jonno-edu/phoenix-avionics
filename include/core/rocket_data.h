#ifndef ROCKET_DATA_H
#define ROCKET_DATA_H

#include "telemetry_defs.h" // From esl-comms library

/**
 * @brief Initialize the rocket data store (clear all fields)
 */
void rocket_data_init(void);

/**
 * @brief Fill the store with dummy test values for protocol testing
 */
void rocket_data_fill_test_values(void);

// ============================================================================
// SETTERS (Called by Sensor Tasks / Producers)
// ============================================================================

/**
 * @brief Update GPS data
 * @param lat Latitude (deg * 1e7)
 * @param lon Longitude (deg * 1e7)
 * @param alt Altitude MSL (mm)
 * @param vel_n North velocity (m/s * 10)
 * @param vel_e East velocity (m/s * 10)
 * @param vel_d Down velocity (m/s * 10)
 * @param itow GPS time-of-week (ms)
 * @param week GPS week number
 */
void rocket_data_update_gps(int32_t lat, int32_t lon, int32_t alt, 
                            int32_t vel_n, int32_t vel_e, int32_t vel_d,
                            uint32_t itow, uint16_t week);

/**
 * @brief Update IMU accelerometer data (raw LSB)
 */
void rocket_data_update_accel(int16_t x, int16_t y, int16_t z);

/**
 * @brief Update IMU gyroscope data (raw LSB)
 */
void rocket_data_update_gyro(int16_t x, int16_t y, int16_t z);

/**
 * @brief Update magnetometer data (raw LSB)
 */
void rocket_data_update_mag(int16_t x, int16_t y, int16_t z);

/**
 * @brief Update estimated state (from Kalman filter, etc.)
 * @param lat Estimated latitude (deg * 1e7)
 * @param lon Estimated longitude (deg * 1e7)
 * @param alt Estimated altitude (mm)
 * @param vel_n Estimated North velocity (m/s * 10)
 * @param vel_e Estimated East velocity (m/s * 10)
 * @param vel_d Estimated Down velocity (m/s * 10)
 */
void rocket_data_update_est_state(int32_t lat, int32_t lon, int32_t alt,
                                  int16_t vel_n, int16_t vel_e, int16_t vel_d);

/**
 * @brief Update barometer data
 * @param pressure_pa Pressure (Pa)
 * @param temp_c_x100 Temperature (°C * 100)
 */
void rocket_data_update_baro(uint32_t pressure_pa, int32_t temp_c_x100);

/**
 * @brief Update stack temperature sensor
 * @param temp_c_x128 Temperature (°C * 128)
 */
void rocket_data_update_temp_stack(int16_t temp_c_x128);

/**
 * @brief Update system runtime
 */
void rocket_data_update_runtime(uint16_t seconds, uint16_t milliseconds);

/**
 * @brief Update battery current
 * @param current_ma Current in milliamps
 */
void rocket_data_update_battery(uint16_t current_ma);

/**
 * @brief Update avionics state
 * @param state Avionics state enum value
 * @param apogee_count Apogee counter
 */
void rocket_data_update_state(uint8_t state, uint8_t apogee_count);

/**
 * @brief Update GPS time validity flags
 */
void rocket_data_update_time_flags(uint8_t flags);

/**
 * @brief Update apogee vote bits
 * @param votes 3-bit apogee votes (bit 0-2)
 */
void rocket_data_update_apogee_votes(uint8_t votes);

// ============================================================================
// GETTERS (Called by Telemetry Task / Consumers)
// ============================================================================

/**
 * @brief Get a thread-safe copy of the complete tracking beacon
 * @param out_beacon Pointer to destination struct
 */
void rocket_data_get_beacon(TlmTrackingBeaconPayload_t *out_beacon);

#endif // ROCKET_DATA_H
