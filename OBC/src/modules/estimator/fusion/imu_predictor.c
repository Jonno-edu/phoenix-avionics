#include "imu_predictor.h"
#include "../ekf_math/symforce_wrapper.h"
#include <math.h>

// --- Cross product helper: out = a x b ---
static inline void cross_product(const float a[3], const float b[3], float out[3]) {
    out[0] = a[1]*b[2] - a[2]*b[1];
    out[1] = a[2]*b[0] - a[0]*b[2];
    out[2] = a[0]*b[1] - a[1]*b[0];
}

/**
 * @brief Run the EKF IMU prediction step
 *
 * Implements Phase 3 Core: IMU Propagation, with Phase 5 lever arm compensation.
 * - Bias correction + angular acceleration estimation
 * - Centripetal & tangential lever arm compensation (omega x (omega x r) + alpha x r)
 * - Nominal state kinematics integration
 * - SymForce covariance propagation
 */
void imu_predict(ekf_core_t* ekf, const vehicle_imu_t* imu) {
    if (imu->dt_s <= 0.0f) return;

    float dt = imu->dt_s;

    // --- State variables ---
    float* q         = ekf->state.q;
    float* v_ned     = ekf->state.v_ned;
    float* p_ned     = ekf->state.p_ned;
    float* bg        = ekf->state.gyro_bias;
    float* ba        = ekf->state.accel_bias;
    float* prev_gyro = ekf->state.prev_gyro;

    // --- IMU lever arm offset from CG (body frame: x=fwd, y=right, z=down) ---
    // TODO: pull from a param/config struct when that system exists
    const float imu_lever_arm[3] = {0.1f, 0.0f, 0.0f}; // 10 cm forward of CG

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

    cross_product(gyro_corrected, imu_lever_arm, omega_cross_r);
    cross_product(gyro_corrected, omega_cross_r, centripetal_accel);
    cross_product(angular_accel,  imu_lever_arm, tangential_accel);

    for (int i = 0; i < 3; i++) {
        accel_corrected[i] -= centripetal_accel[i] + tangential_accel[i];
    }

    // --- 3. Nominal State Integration (Non-linear Kinematics) ---

    // a. Update Orientation using Small-Angle-Approximation (dt is 4ms)
    float half_angle[3];
    for (int i = 0; i < 3; i++) {
        half_angle[i] = 0.5f * gyro_corrected[i] * dt;
    }

    // q_new = q * exp(da/2)
    float q_new[4];
    q_new[0] = q[0] - q[1]*half_angle[0] - q[2]*half_angle[1] - q[3]*half_angle[2];
    q_new[1] = q[1] + q[0]*half_angle[0] + q[2]*half_angle[2] - q[3]*half_angle[1];
    q_new[2] = q[2] + q[0]*half_angle[1] + q[3]*half_angle[0] - q[1]*half_angle[2];
    q_new[3] = q[3] + q[0]*half_angle[2] + q[1]*half_angle[1] - q[2]*half_angle[0];

    // Normalize quaternion
    float norm = sqrtf(q_new[0]*q_new[0] + q_new[1]*q_new[1] + q_new[2]*q_new[2] + q_new[3]*q_new[3]);
    if (norm > 0.0f) {
        q[0] = q_new[0] / norm;
        q[1] = q_new[1] / norm;
        q[2] = q_new[2] / norm;
        q[3] = q_new[3] / norm;
    }

    // b. Update Velocity (Rotate body accel to NED, add gravity)
    // R(q) body to NED
    float R[3][3];
    R[0][0] = 1.0f - 2.0f * (q[2]*q[2] + q[3]*q[3]);
    R[0][1] = 2.0f * (q[1]*q[2] - q[0]*q[3]);
    R[0][2] = 2.0f * (q[1]*q[3] + q[0]*q[2]);
    R[1][0] = 2.0f * (q[1]*q[2] + q[0]*q[3]);
    R[1][1] = 1.0f - 2.0f * (q[1]*q[1] + q[3]*q[3]);
    R[1][2] = 2.0f * (q[2]*q[3] - q[0]*q[1]);
    R[2][0] = 2.0f * (q[1]*q[3] - q[0]*q[2]);
    R[2][1] = 2.0f * (q[2]*q[3] + q[0]*q[1]);
    R[2][2] = 1.0f - 2.0f * (q[1]*q[1] + q[2]*q[2]);

    float acc_n[3];
    acc_n[0] = R[0][0]*accel_corrected[0] + R[0][1]*accel_corrected[1] + R[0][2]*accel_corrected[2];
    acc_n[1] = R[1][0]*accel_corrected[0] + R[1][1]*accel_corrected[1] + R[1][2]*accel_corrected[2];
    acc_n[2] = R[2][0]*accel_corrected[0] + R[2][1]*accel_corrected[1] + R[2][2]*accel_corrected[2];

    acc_n[2] += 9.80665f; // Add gravity (downwards positive in NED)

    for (int i = 0; i < 3; i++) {
        v_ned[i] += acc_n[i] * dt;
    }

    // c. Update Position
    for (int i = 0; i < 3; i++) {
        p_ned[i] += v_ned[i] * dt;
    }

    // --- 4. Covariance Propagation (SymForce) ---

    // Process noise variance values
    const float ACCEL_NOISE_VAR[3] = {1.0e-2f, 1.0e-2f, 1.0e-2f};
    const float GYRO_NOISE_VAR = 1.0e-4f;

    symforce_predict_covariance(
        ekf->P,
        ekf->state.q,
        ekf->state.v_ned,
        ekf->state.p_ned,
        ekf->state.gyro_bias,
        ekf->state.accel_bias,
        accel_raw,
        ACCEL_NOISE_VAR,
        gyro_raw,
        GYRO_NOISE_VAR,
        dt
    );
}
