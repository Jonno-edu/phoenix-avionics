#ifndef EKF_CORE_H
#define EKF_CORE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

/**
 * @brief EKF flight mode — tracks progress through the 5-stage warmup sequence.
 *
 * State machine transitions (see ekf_state.c):
 *   UNINITIALIZED → LEVELING → HEADING_ALIGN → ZVU_CALIBRATING → FLIGHT
 *
 * The filter performs meaningful state estimation at all stages.
 * ZVU (Zero-Velocity Update) is active during LEVELING, HEADING_ALIGN, and
 * ZVU_CALIBRATING, then disabled when FLIGHT is triggered by a +2g event.
 */
typedef enum {
    EKF_MODE_UNINITIALIZED  = 0,  /**< No IMU data received yet. */
    EKF_MODE_LEVELING       = 1,  /**< Averaging gravity to determine pitch/roll. */
    EKF_MODE_HEADING_ALIGN  = 2,  /**< Applying mag heading (yaw initialisation). */
    EKF_MODE_ZVU_CALIBRATING = 3, /**< ZVU active, converging gyro/accel biases. */
    EKF_MODE_FLIGHT         = 4,  /**< Liftoff detected. ZVU disabled. Full fusion. */
} ekf_flight_mode_t;

/**
 * @brief EKF 15-state definition
 */
typedef struct {
    float q[4];          // Orientation [w, x, y, z] (body to NED)
    float v_ned[3];      // Velocity [vx, vy, vz] (m/s)
    float p_ned[3];      // Position [x, y, z] (m)
    float gyro_bias[3];  // Gyro bias [wx, wy, wz] (rad/s)
    float accel_bias[3]; // Accel bias [ax, ay, az] (m/s^2)
    float prev_gyro[3];  // Previous un-biased gyro rate [wx, wy, wz] (rad/s) — bookkeeping for angular accel (NOT a filter state)
} ekf_state_t;

/**
 * @brief Main EKF state container
 */
typedef struct {
    ekf_state_t       state;
    float             P[225];        /**< 15×15 covariance, flat row-major float array. */
    ekf_flight_mode_t flight_mode;   /**< Current warmup / flight mode (set by ekf_state.c). */
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
