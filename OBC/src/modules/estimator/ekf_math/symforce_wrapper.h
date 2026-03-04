#ifndef EKF_MATH_SYMFORCE_WRAPPER_H
#define EKF_MATH_SYMFORCE_WRAPPER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>

/**
 * @brief Predict the EKF covariance matrix P (15x15)
 * 
 * @param P           Pointer to the 15x15 covariance matrix (flat float[225])
 * @param q           Quaternion [w, x, y, z] (float[4])
 * @param vel         NED velocity [vx, vy, vz] (float[3])
 * @param pos         NED position [x, y, z] (float[3])
 * @param gyro_bias   Gyro bias [wx, wy, wz] (float[3])
 * @param accel_bias  Accel bias [ax, ay, az] (float[3])
 * @param accel       Measured acceleration [ax, ay, az] (float[3])
 * @param accel_var   Acceleration process noise variance (float[3])
 * @param gyro        Measured angular rate [wx, wy, wz] (float[3])
 * @param gyro_var    Gyro process noise variance scalar
 * @param dt          Time step (seconds)
 */
void symforce_predict_covariance(
    float* P,
    const float* q,
    const float* vel,
    const float* pos,
    const float* gyro_bias,
    const float* accel_bias,
    const float* accel,
    const float* accel_var,
    const float* gyro,
    const float gyro_var,
    const float dt
);

/**
 * @brief Zero-velocity and Zero-rotation (stationary) EKF update
 * 
 * @param P           Pointer to the 15x15 covariance matrix (float[225])
 * @param q           Quaternion [w, x, y, z] (float[4])
 * @param vel         NED velocity [vx, vy, vz] (float[3])
 * @param pos         NED position [x, y, z] (float[3])
 * @param gyro_bias   Gyro bias [wx, wy, wz] (float[3])
 * @param accel_bias  Accel bias [ax, ay, az] (float[3])
 * @param gyro_raw    Raw gyro meas [wx, wy, wz] (float[3])
 * @param vel_var     Measurement noise variance for velocity (float[3])
 * @param gyro_var    Measurement noise variance for gyro (float[3])
 */
void symforce_update_stationary(
    float* P, float* q, float* vel, float* pos, float* gyro_bias, float* accel_bias,
    const float* gyro_raw, const float* vel_var, const float* gyro_var
);

/**
 * Barometric altitude update.
 * baro_alt  : measured altitude in metres (positive up, i.e. -p_ned_z).
 * baro_var  : altitude measurement noise variance (m²).
 */
void symforce_update_baro(
    float* P, float* q, float* vel, float* pos, float* gyro_bias, float* accel_bias,
    float baro_alt, float baro_var, float epsilon
);

/**
 * 6-DOF GPS position + velocity update.
 * gps_pos / gps_vel : NED frame, metres / m·s⁻¹.
 * pos_var / vel_var  : per-axis measurement noise variances.
 */
void symforce_update_gps(
    float* P, float* q, float* vel, float* pos, float* gyro_bias, float* accel_bias,
    const float* gps_pos, const float* gps_vel,
    const float* pos_var, const float* vel_var,
    float epsilon
);

/**
 * 3-axis magnetometer update.
 *
 * @param mag_body     Measured magnetic field in body frame (Gauss, float[3]).
 * @param mag_ref_ned  Reference NED field from World Magnetic Model (Gauss, float[3]).
 * @param mag_var      Per-axis measurement noise variance (Gauss², float[3]).
 * @param epsilon      Numerical regularisation scalar.
 *
 * Observation model: h(q) = q⁻¹ ⊗ mag_ref_ned ⊗ q
 * (reference field rotated from NED into body frame)
 * H is [3×15], non-zero only in the attitude columns (indices 0–2).
 */
void symforce_update_mag(
    float* P, float* q, float* vel, float* pos, float* gyro_bias, float* accel_bias,
    const float* mag_body, const float* mag_ref_ned,
    const float* mag_var, float epsilon
);

#ifdef __cplusplus
}
#endif

#endif // EKF_MATH_SYMFORCE_WRAPPER_H
