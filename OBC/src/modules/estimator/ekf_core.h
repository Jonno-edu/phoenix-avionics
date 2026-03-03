#ifndef EKF_CORE_H
#define EKF_CORE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

/**
 * @brief EKF 15-state definition
 */
typedef struct {
    float q[4];          // Orientation [w, x, y, z] (body to NED)
    float v_ned[3];      // Velocity [vx, vy, vz] (m/s)
    float p_ned[3];      // Position [x, y, z] (m)
    float gyro_bias[3];  // Gyro bias [wx, wy, wz] (rad/s)
    float accel_bias[3]; // Accel bias [ax, ay, az] (m/s^2)
} ekf_state_t;

/**
 * @brief Main EKF state container
 */
typedef struct {
    ekf_state_t state;
    float P[225];        // 15x15 Covariance flat array
    bool initialized;
} ekf_core_t;

/**
 * @brief Initialize the EKF core
 * 
 * @param ekf Pointer to the EKF core structure
 */
void ekf_core_init(ekf_core_t* ekf);

/**
 * @brief Reset the EKF state and covariance
 * 
 * @param ekf Pointer to the EKF core structure
 */
void ekf_core_reset(ekf_core_t* ekf);

#ifdef __cplusplus
}
#endif

#endif // EKF_CORE_H
