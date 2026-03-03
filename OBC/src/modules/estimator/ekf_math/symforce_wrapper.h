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

#ifdef __cplusplus
}
#endif

#endif // EKF_MATH_SYMFORCE_WRAPPER_H
