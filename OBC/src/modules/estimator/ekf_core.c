#include "ekf_core.h"
#include <string.h>
#include <math.h>
#include "fusion/gps_fuse.h"
#include "math/ekf_math.h"

/* Forward declaration for the SymForce-based delayed predictor in imu_predictor.c. */
extern void imu_predict(ekf_core_t* ekf, const imu_history_t* imu);

/**
 * @brief Populate ekf->params with safe hard-coded defaults.
 *
 * Called at the end of ekf_core_reset() so the filter is immediately usable
 * without a ground-station parameter uplink.
 */
static void _load_default_params(ekf_params_t* p) {
    /* Physical configuration */
    p->imu_lever_arm[0] = 0.1f;  /* 10 cm forward of CG */
    p->imu_lever_arm[1] = 0.0f;
    p->imu_lever_arm[2] = 0.0f;
    p->gravity_ms2 = 9.80665f;

    /* Process noise */
    p->accel_noise_var[0] = 1.0e-2f;
    p->accel_noise_var[1] = 1.0e-2f;
    p->accel_noise_var[2] = 1.0e-2f;
    p->gyro_noise_var = 1.0e-4f;

    /* Sensor noise */
    p->gps_pos_var[0] = 2.25f; p->gps_pos_var[1] = 2.25f; p->gps_pos_var[2] = 2.25f;
    p->gps_vel_var[0] = 0.01f; p->gps_vel_var[1] = 0.01f; p->gps_vel_var[2] = 0.01f;
    p->baro_noise_var = 0.25f;
    p->mag_noise_var_pad    = 0.05f;
    p->mag_noise_var_flight = 5.0f;

    /* Chi-squared gates (sigma multiples) */
    p->gps_pos_gate       = 3.0f;
    p->gps_vel_gate       = 3.0f;
    p->baro_gate          = 5.0f;
    p->mag_magnitude_gate = 3.0f;

    /* Recovery timeout */
    p->gps_timeout_samples = 50;
}

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

    /* Initialise GPS waiting room. */
    ekf->waiting_gps.is_waiting = false;

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

    /* Zero the debug snapshot (already covered by the memset above, but
     * kept explicit so the intent is clear). */
    memset(&ekf->debug, 0, sizeof(ekf_debug_t));

    /* Populate safe defaults so the filter runs correctly before any ground
     * station parameter uplink has occurred. */
    _load_default_params(&ekf->params);
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
            
            /* -> NEW: Check if the timelines align for a waiting GPS measurement <- */
            if (ekf->waiting_gps.is_waiting && tail_time >= ekf->waiting_gps.timestamp_us) {
                gps_measurement_t g;
                for (int i = 0; i < 3; i++) {
                    g.pos_ned[i] = ekf->waiting_gps.pos_ned[i];
                    g.vel_ned[i] = ekf->waiting_gps.vel_ned[i];
                }
                
                /* Fuse it directly into the delayed state */
                gps_fuse(ekf, &g); 
                ekf->waiting_gps.is_waiting = false;
            }

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
        imu_propagate_kinematics(&ekf->head_state, &ekf->imu_buffer[iter], &ekf->params);
        iter = (iter + 1) & EKF_OBS_BUFFER_MASK;
    }
}

/**
 * Lightweight kinematic integration used to fast-forward head_state to the
 * present moment.  Does NOT touch the covariance matrix or apply lever-arm
 * compensation — those only run on the delayed state via imu_predict().
 *
 * Uses math/ekf_math.h helpers so the implementation stays in one place.
 */
void imu_propagate_kinematics(ekf_state_t* state, const imu_history_t* imu,
                              const ekf_params_t* params) {
    float dt = imu->dt_s;

    /* 1. Correct raw deltas with latest known biases. */
    float gyro_corrected[3], accel_corrected[3];
    for (int i = 0; i < 3; i++) {
        gyro_corrected[i]  = (imu->delta_angle[i]    / dt) - state->gyro_bias[i];
        accel_corrected[i] = (imu->delta_velocity[i] / dt) - state->accel_bias[i];
    }

    /* 2. Update orientation using the small-angle quaternion integrator. */
    quat_integrate_small_angle(state->q, gyro_corrected, dt);

    /* 3. Rotate bias-corrected body acceleration to NED and add gravity. */
    float acc_n[3];
    vec3_rotate_body_to_ned(state->q, accel_corrected, acc_n);
    acc_n[2] += params->gravity_ms2;

    /* 4. Integrate velocity and position. */
    for (int i = 0; i < 3; i++) {
        state->v_ned[i] += acc_n[i] * dt;
        state->p_ned[i] += state->v_ned[i] * dt;
    }
}

void ekf_core_push_gps(ekf_core_t* ekf, uint64_t timestamp_us, const float pos_ned[3], const float vel_ned[3]) {
    ekf->waiting_gps.timestamp_us = timestamp_us;
    for (int i = 0; i < 3; i++) {
        ekf->waiting_gps.pos_ned[i] = pos_ned[i];
        ekf->waiting_gps.vel_ned[i] = vel_ned[i];
    }
    ekf->waiting_gps.is_waiting = true;
}
