#include "imu_predictor.h"
#include "../ekf_math/symforce_wrapper.h"
#include "math/ekf_math.h"
#include <math.h>

/**
 * @brief Run the EKF IMU prediction step
 *
 * Implements Phase 3 Core: IMU Propagation, with Phase 5 lever arm compensation.
 * - Bias correction + angular acceleration estimation
 * - Centripetal & tangential lever arm compensation (omega x (omega x r) + alpha x r)
 * - Nominal state kinematics integration via math/ekf_math.h
 * - SymForce covariance propagation
 */
void imu_predict(ekf_core_t* ekf, const imu_history_t* imu) {
    if (imu->dt_s <= 0.0f) return;

    float dt = imu->dt_s;

    // --- State variables (delayed source of truth) ---
    float* q         = ekf->delayed_state.q;
    float* v_ned     = ekf->delayed_state.v_ned;
    float* p_ned     = ekf->delayed_state.p_ned;
    float* bg        = ekf->delayed_state.gyro_bias;
    float* ba        = ekf->delayed_state.accel_bias;
    float* prev_gyro = ekf->delayed_state.prev_gyro;

    // --- IMU lever arm offset from CG (body frame: x=fwd, y=right, z=down) ---
    // Pulled from live-tunable params; default is 10 cm forward of CG.
    const float* imu_lever_arm = ekf->params.imu_lever_arm;

    // --- 1. Correct IMU measurements with current biases & compute angular accel ---
    // Note: vehicle_imu contains delta_angle (rad) and delta_velocity (m/s).
    // Divide by dt to recover rates/accelerations for kinematic integration.
    float gyro_raw[3], accel_raw[3];
    float gyro_corrected[3], accel_corrected[3];
    float angular_accel[3];

    for (int i = 0; i < 3; i++) {
        gyro_raw[i]        = imu->delta_angle[i] / dt;
        accel_raw[i]       = imu->delta_velocity[i] / dt;
        gyro_corrected[i]  = gyro_raw[i] - bg[i];
        accel_corrected[i] = accel_raw[i] - ba[i];

        // Angular acceleration: alpha = (omega_k - omega_{k-1}) / dt
        angular_accel[i] = (gyro_corrected[i] - prev_gyro[i]) / dt;
        prev_gyro[i]     = gyro_corrected[i];
    }

    // --- 2. Lever Arm Compensation ---
    // The IMU measures: a_meas = a_cg + omega x (omega x r) + alpha x r
    // Invert to recover CG acceleration: a_cg = a_meas - centripetal - tangential
    float omega_cross_r[3];
    float centripetal_accel[3];
    float tangential_accel[3];

    vec3_cross(gyro_corrected, imu_lever_arm, omega_cross_r);
    vec3_cross(gyro_corrected, omega_cross_r, centripetal_accel);
    vec3_cross(angular_accel,  imu_lever_arm, tangential_accel);

    for (int i = 0; i < 3; i++) {
        accel_corrected[i] -= centripetal_accel[i] + tangential_accel[i];
    }

    // --- 3. Nominal State Integration (Non-linear Kinematics) ---

    // a. Update Orientation — exact trigonometric exponential map
    quat_integrate_exact(q, gyro_corrected, dt);

    // b. Rotate bias-corrected CG acceleration to NED and add gravity
    float acc_n[3];
    vec3_rotate_body_to_ned(q, accel_corrected, acc_n);
    acc_n[2] += ekf->params.gravity_ms2;

    // c. Position then velocity — trapezoidal: p uses old v + ½a·dt²
    for (int i = 0; i < 3; i++) {
        p_ned[i] += v_ned[i] * dt + 0.5f * acc_n[i] * dt * dt;
        v_ned[i] += acc_n[i] * dt;
    }

    // --- 4. Covariance Propagation (SymForce) ---
    // Process noise variances pulled from live-tunable params.
    symforce_predict_covariance(
        ekf->P,
        ekf->delayed_state.q,
        ekf->delayed_state.v_ned,
        ekf->delayed_state.p_ned,
        ekf->delayed_state.gyro_bias,
        ekf->delayed_state.accel_bias,
        accel_raw,
        ekf->params.accel_noise_var,
        gyro_raw,
        ekf->params.gyro_noise_var,
        dt
    );
}
