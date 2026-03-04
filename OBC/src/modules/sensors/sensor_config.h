/**
 * @file sensor_config.h
 * @brief Simulated Non-Volatile Memory (NVM) for factory bench calibrations
 *        and launch-site geometry constants.
 *
 * In production these values would be read from flash/EEPROM at boot.
 * For now they are hardcoded here and #included wherever sensor
 * pre-compensation or EKF initialisation is performed.
 *
 * Applying these constants in the sensor front-end (sensors.c /
 * process_imu_data) ensures the EKF core only ever receives measurements
 * that have already been stripped of the large, static bench-measured biases.
 * The EKF's own internal bias states then only need to track the small
 * residual turn-on drift that remains after factory compensation.
 */

#ifndef SENSOR_CONFIG_H
#define SENSOR_CONFIG_H

#include <math.h>

/* ── Factory Bench Calibrations ──────────────────────────────────────────────
 * These are the large, static biases measured during bench testing.
 * Subtract from raw sensor output before passing data to the EKF.
 * In the future, a 3×3 cross-axis misalignment matrix and scale-factor
 * matrix would also be applied here.
 */

/** Factory gyroscope bias — X axis (rad/s). */
#define FACTORY_GYRO_BIAS_X   ( 0.0050f)

/** Factory gyroscope bias — Y axis (rad/s). */
#define FACTORY_GYRO_BIAS_Y   (-0.0020f)

/** Factory gyroscope bias — Z axis (rad/s). */
#define FACTORY_GYRO_BIAS_Z   ( 0.0150f)

/** Factory accelerometer bias — X axis (m/s²). */
#define FACTORY_ACCEL_BIAS_X  ( 0.0200f)

/** Factory accelerometer bias — Y axis (m/s²). */
#define FACTORY_ACCEL_BIAS_Y  (-0.0300f)

/** Factory accelerometer bias — Z axis (m/s²). */
#define FACTORY_ACCEL_BIAS_Z  ( 0.0800f)

/* ── Launch Pad Geometry ─────────────────────────────────────────────────────
 * Used by ekf_state_set_launch_rail() to hard-initialise the attitude
 * quaternion from known GSE constraints, bypassing the gravity leveling
 * and magnetometer heading alignment phases.
 *
 * Coordinate conventions (Z-Y-X aerospace Euler):
 *   Elevation: 0 = horizontal, π/2 = perfectly vertical nose-up.
 *   Azimuth:   0 = True North, π/2 = East.
 */

/** Launch rail elevation angle (radians). 85° ≈ 1.4835 rad. */
#define LAUNCH_RAIL_ELEVATION_RAD  (1.48353f)

/** Launch rail azimuth angle (radians). 0 = True North. */
#define LAUNCH_RAIL_AZIMUTH_RAD    (0.0f)

#endif /* SENSOR_CONFIG_H */
