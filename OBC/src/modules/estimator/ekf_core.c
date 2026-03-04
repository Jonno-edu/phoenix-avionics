#include "ekf_core.h"
#include <string.h>

void ekf_core_init(ekf_core_t* ekf) {
    memset(ekf, 0, sizeof(ekf_core_t));
    ekf_core_reset(ekf);
}

void ekf_core_reset(ekf_core_t* ekf) {
    // Initial state: Level, zero velocity, zero bias
    ekf->state.q[0] = 1.0f;
    ekf->state.q[1] = 0.0f;
    ekf->state.q[2] = 0.0f;
    ekf->state.q[3] = 0.0f;

    for (int i = 0; i < 3; i++) {
        ekf->state.v_ned[i] = 0.0f;
        ekf->state.p_ned[i] = 0.0f;
        ekf->state.gyro_bias[i] = 0.0f;
        ekf->state.accel_bias[i] = 0.0f;
    }

    // Initial covariance P: High uncertainty initially
    memset(ekf->P, 0, sizeof(ekf->P));
    
    const float P_INIT_VEL = 100.0f;  // (10 m/s)^2
    const float P_INIT_POS = 100.0f;  // (10 m)^2
    const float P_INIT_ATT = 0.05f;   // ~15 degree std
    const float P_INIT_BIAS_G = 0.001f;
    const float P_INIT_BIAS_A = 0.1f;

    // Set diagonal elements of 15x15 P matrix
    // Index mapping: 0,1,2 = att, 3,4,5 = vel, 6,7,8 = pos, 9,10,11 = bg, 12,13,14 = ba
    ekf->P[0*15 + 0] = P_INIT_ATT;
    ekf->P[1*15 + 1] = P_INIT_ATT;
    ekf->P[2*15 + 2] = P_INIT_ATT;

    ekf->P[3*15 + 3] = P_INIT_VEL;
    ekf->P[4*15 + 4] = P_INIT_VEL;
    ekf->P[5*15 + 5] = P_INIT_VEL;

    ekf->P[6*15 + 6] = P_INIT_POS;
    ekf->P[7*15 + 7] = P_INIT_POS;
    ekf->P[8*15 + 8] = P_INIT_POS;

    ekf->P[9*15 + 9] = P_INIT_BIAS_G;
    ekf->P[10*15 + 10] = P_INIT_BIAS_G;
    ekf->P[11*15 + 11] = P_INIT_BIAS_G;

    ekf->P[12*15 + 12] = P_INIT_BIAS_A;
    ekf->P[13*15 + 13] = P_INIT_BIAS_A;
    ekf->P[14*15 + 14] = P_INIT_BIAS_A;

    ekf->flight_mode = EKF_MODE_UNINITIALIZED;
}
