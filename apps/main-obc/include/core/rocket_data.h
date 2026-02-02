#ifndef ROCKET_DATA_H
#define ROCKET_DATA_H

#include "telemetry_defs.h" // From esl-comms library
#include "telemetry.h"      // For TrackingBeacon_t

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
 * @param lat_deg Latitude (Degrees)
 * @param lon_deg Longitude (Degrees)
 * @param alt_m Altitude (m)
 * @param vel_n_ms North velocity (m/s)
 * @param vel_e_ms East velocity (m/s)
 * @param vel_d_ms Down velocity (m/s)
 * @param itow GPS time-of-week (ms)
 * @param week GPS week number
 */
void rocket_data_update_gps(double lat_deg, double lon_deg, float alt_m, 
                            float vel_n_ms, float vel_e_ms, float vel_d_ms,
                            uint32_t itow, uint16_t week);

/**
 * @brief Update IMU accelerometer data (SI: m/s^2)
 */
void rocket_data_update_accel(float ax_ms2, float ay_ms2, float az_ms2);

/**
 * @brief Update IMU gyroscope data (SI: rad/s)
 */
void rocket_data_update_gyro(float gx_rads, float gy_rads, float gz_rads);

/**
 * @brief Update magnetometer data (SI: uT)
 */
void rocket_data_update_mag(float mx_ut, float my_ut, float mz_ut);

/**
 * @brief Update estimated state (Data from Kalman Filter / NAV)
 * @param lat_deg Estimated latitude (Degrees)
 * @param lon_deg Estimated longitude (Degrees)
 * @param alt_m Estimated altitude (m)
 * @param vn_ms Estimated North velocity (m/s)
 * @param ve_ms Estimated East velocity (m/s)
 * @param vd_ms Estimated Down velocity (m/s)
 */
void rocket_data_update_est_state(double lat_deg, double lon_deg, float alt_m,
                                  float vn_ms, float ve_ms, float vd_ms);

/**
 * @brief Update barometer data
 * @param pressure_pa Pressure (Pa)
 * @param temp_c Temperature (C)
 */
void rocket_data_update_baro(float pressure_pa, float temp_c);

/**
 * @brief Update stack temperature sensor
 * @param temp_c Temperature (C)
 */
void rocket_data_update_temp_stack(float temp_c);

/**
 * @brief Update system runtime
 */
void rocket_data_update_runtime(uint16_t seconds, uint16_t milliseconds);

/**
 * @brief Update battery current
 * @param current_ma Current in milliamps
 */
void rocket_data_update_battery(float current_ma);

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
void getRocketTrackingInfo(TrackingBeacon_t *out_beacon);

// --- SI UNIT GETTERS ---

// IMU (m/s^2 and rad/s)
float rocket_data_get_accel_x_si(void);
float rocket_data_get_accel_y_si(void);
float rocket_data_get_accel_z_si(void);
float rocket_data_get_gyro_x_si(void);
float rocket_data_get_gyro_y_si(void);
float rocket_data_get_gyro_z_si(void);

// Baro (Pa and °C)
float rocket_data_get_baro_press_si(void);
float rocket_data_get_baro_temp_si(void);

// GPS (deg, m, m/s)
double rocket_data_get_gps_lat_si(void);
double rocket_data_get_gps_lon_si(void);
float  rocket_data_get_gps_alt_si(void);
float  rocket_data_get_gps_vel_n_si(void);
float  rocket_data_get_gps_vel_e_si(void);
float  rocket_data_get_gps_vel_d_si(void);

// Estimation (deg, m, m/s)
double rocket_data_get_est_lat_si(void);
double rocket_data_get_est_lon_si(void);
float  rocket_data_get_est_alt_si(void);
float  rocket_data_get_est_vel_n_si(void);
float  rocket_data_get_est_vel_e_si(void);
float  rocket_data_get_est_vel_d_si(void);

// Magnetometer (uT)
float rocket_data_get_mag_x_si(void);
float rocket_data_get_mag_y_si(void);
float rocket_data_get_mag_z_si(void);

// Stack Temperature (°C)
float rocket_data_get_temp_stack_si(void);

#endif // ROCKET_DATA_H
