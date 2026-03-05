#include "ekf_core.h"
#include <string.h>
#include <math.h>

/* Forward declaration for the SymForce-based delayed predictor in imu_predictor.c. */
extern void imu_predict(ekf_core_t* ekf, const imu_history_t* imu);

void ekf_core_init(ekf_core_t* ekf) {
    memset(ekf, 0, sizeof(ekf_core_t));
    ekf_core_reset(ekf);
}

void ekf_core_reset(ekf_core_t* ekf) {
    memset(ekf, 0, sizeof(ekf_core_t));

    /* Initial orientation: identity quaternion (level, no heading rotation). */
    ekf->delayed_state.q[0] = 1.0f;
    ekf->delayed_state.q[1] = 0.0f;
    ekf->delayed_state.q[2] = 0.0f;
    ekf->delayed_state.q[3] = 0.0f;

    for (int i = 0; i < 3; i++) {
        ekf->delayed_state.v_ned[i]      = 0.0f;
        ekf->delayed_state.p_ned[i]      = 0.0f;
        ekf->delayed_state.gyro_bias[i]  = 0.0f;
        ekf->delayed_state.accel_bias[i] = 0.0f;
        ekf->delayed_state.prev_gyro[i]  = 0.0f;
    }

    /* Sync head state to the initial delayed state. */
    ekf->head_state = ekf->delayed_state;

    /* Reset ring buffer indices. */
    ekf->head_idx = 0;
    ekf->tail_idx = 0;

    /* Initial covariance P: high uncertainty.
     * Index mapping: 0-2 = attitude, 3-5 = velocity, 6-8 = position,
     *                9-11 = gyro bias, 12-14 = accel bias. */
    const float P_INIT_VEL    = 100.0f;   /* (10 m/s)^2  */
    const float P_INIT_POS    = 100.0f;   /* (10 m)^2    */
    const float P_INIT_ATT    = 0.05f;    /* ~15 deg std */
    const float P_INIT_BIAS_G = 0.001f;
    const float P_INIT_BIAS_A = 0.1f;

    ekf->P[0*15 + 0] = P_INIT_ATT;
    ekf->P[1*15 + 1] = P_INIT_ATT;
    ekf->P[2*15 + 2] = P_INIT_ATT;

    ekf->P[3*15 + 3] = P_INIT_VEL;
    ekf->P[4*15 + 4] = P_INIT_VEL;
    ekf->P[5*15 + 5] = P_INIT_VEL;

    ekf->P[6*15 + 6] = P_INIT_POS;
    ekf->P[7*15 + 7] = P_INIT_POS;
    ekf->P[8*15 + 8] = P_INIT_POS;

    ekf->P[9*15 + 9]   = P_INIT_BIAS_G;
    ekf->P[10*15 + 10] = P_INIT_BIAS_G;
    ekf->P[11*15 + 11] = P_INIT_BIAS_G;

    ekf->P[12*15 + 12] = P_INIT_BIAS_A;
    ekf->P[13*15 + 13] = P_INIT_BIAS_A;
    ekf->P[14*15 + 14] = P_INIT_BIAS_A;

    ekf->flight_mode = EKF_MODE_UNINITIALIZED;
}

void ekf_core_step(ekf_core_t* ekf, const imu_history_t* imu) {
    if (imu->dt_s <= 0.0f) return;

    /* ── STEP 1: Push new IMU sample into the ring buffer at head ── */
    uint8_t next_head = (ekf->head_idx + 1) & EKF_OBS_BUFFER_MASK;

    ekf->imu_buffer[ekf->head_idx] = *imu;

    ekf->head_idx = next_head;

    /* ── STEP 2: Advance the delayed tail ── */
    /* Retrieve the timestamp of the most recently pushed sample. */
    uint8_t newest_idx = (ekf->head_idx - 1) & EKF_OBS_BUFFER_MASK;
    uint64_t head_time = ekf->imu_buffer[newest_idx].timestamp_us;

    while (ekf->tail_idx != ekf->head_idx) {
        uint64_t tail_time = ekf->imu_buffer[ekf->tail_idx].timestamp_us;

        if ((head_time - tail_time) >= EKF_TARGET_DELAY_US) {
            /* Run the heavy SymForce predictor on the delayed state. */
            imu_predict(ekf, &ekf->imu_buffer[ekf->tail_idx]);

            ekf->tail_idx = (ekf->tail_idx + 1) & EKF_OBS_BUFFER_MASK;
        } else {
            /* Haven't reached the 200ms horizon yet. */
            break;
        }
    }

    /* ── STEP 3: Forward-propagate head_state to the present ── */
    /* Resync head to the delayed source of truth. */
    ekf->head_state = ekf->delayed_state;

    /* Rapidly integrate from the delayed tail up to the present head. */
    uint8_t iter = ekf->tail_idx;
    while (iter != ekf->head_idx) {
        imu_propagate_kinematics(&ekf->head_state, &ekf->imu_buffer[iter]);
        iter = (iter + 1) & EKF_OBS_BUFFER_MASK;
    }
}

/**
 * Lightweight kinematic integration used to fast-forward head_state to the
 * present moment.  Does NOT touch the covariance matrix or apply lever-arm
 * compensation — those only run on the delayed state via imu_predict().
 */
void imu_propagate_kinematics(ekf_state_t* state, const imu_history_t* imu) {
    float dt = imu->dt_s;
    float* q = state->q;

    /* 1. Correct raw deltas with latest known biases. */
    float gyro_corrected[3];
    float accel_corrected[3];

    for (int i = 0; i < 3; i++) {
        gyro_corrected[i]  = (imu->delta_angle[i]    / dt) - state->gyro_bias[i];
        accel_corrected[i] = (imu->delta_velocity[i] / dt) - state->accel_bias[i];
    }

    /* 2. Update orientation (small-angle quaternion multiplication). */
    float half_angle[3];
    for (int i = 0; i < 3; i++) {
        half_angle[i] = 0.5f * gyro_corrected[i] * dt;
    }

    float q_new[4];
    q_new[0] = q[0] - q[1]*half_angle[0] - q[2]*half_angle[1] - q[3]*half_angle[2];
    q_new[1] = q[1] + q[0]*half_angle[0] + q[2]*half_angle[2] - q[3]*half_angle[1];
    q_new[2] = q[2] + q[0]*half_angle[1] + q[3]*half_angle[0] - q[1]*half_angle[2];
    q_new[3] = q[3] + q[0]*half_angle[2] + q[1]*half_angle[1] - q[2]*half_angle[0];

    float norm = sqrtf(q_new[0]*q_new[0] + q_new[1]*q_new[1]
                     + q_new[2]*q_new[2] + q_new[3]*q_new[3]);
    if (norm > 0.0f) {
        q[0] = q_new[0] / norm;
        q[1] = q_new[1] / norm;
        q[2] = q_new[2] / norm;
        q[3] = q_new[3] / norm;
    }

    /* 3. Rotate body acceleration to NED frame.  R(q): body to NED. */
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

    /* Add gravity (downwards positive in NED). */
    acc_n[2] += 9.80665f;

    /* 4. Integrate velocity and position. */
    for (int i = 0; i < 3; i++) {
        state->v_ned[i] += acc_n[i] * dt;
        state->p_ned[i] += state->v_ned[i] * dt;
    }
}
