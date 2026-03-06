#include "baro_fuse.h"
#include "../ekf_math/symforce_wrapper.h"
#include "../ekf_core.h"
#include <math.h>

void baro_fuse(ekf_core_t *ekf, const baro_measurement_t *baro)
{
    /* ------------------------------------------------------------------ */
    /* 1. Pad Warmup Phase — EMA filter builds the ground-level bias       */
    /* ------------------------------------------------------------------ */
    if (ekf->flight_mode < EKF_MODE_FLIGHT) {
        const float alpha = 0.05f; /* EMA smoothing factor */

        if (ekf->delayed_state.baro_bias == 0.0f) {
            /* Hard-initialize on the very first valid sample. */
            ekf->delayed_state.baro_bias     = baro->altitude_m;
            ekf->delayed_state.baro_bias_var = 0.5f; /* tight: we are stationary */
        } else {
            /* Smoothly track ambient pressure while sitting on the pad. */
            ekf->delayed_state.baro_bias = (1.0f - alpha) * ekf->delayed_state.baro_bias
                                         + (alpha * baro->altitude_m);
        }

        /* Clamp down-position to exactly 0 AGL on the pad. */
        ekf->delayed_state.p_ned[2] = 0.0f;

        /* Skip SymForce fusion — strictly calibrating. */
        return;
    }

    /* ------------------------------------------------------------------ */
    /* 2. Flight / Coast Phase — 1D baro bias estimator                   */
    /*    Only update when GPS is actively anchoring altitude so that the  */
    /*    bias does not drift during un-observed coast.                    */
    /* ------------------------------------------------------------------ */
    if (!ekf->debug.gps_rejected) {
        /* Innovation: raw baro altitude vs GPS-anchored EKF altitude + current bias. */
        float bias_innov = baro->altitude_m
                         - (-ekf->delayed_state.p_ned[2] + ekf->delayed_state.baro_bias);

        /* 1D Kalman gain. */
        float bias_innov_var = ekf->delayed_state.baro_bias_var + ekf->params.baro_noise_var;
        float K_bias         = ekf->delayed_state.baro_bias_var / bias_innov_var;

        /* State and covariance update. */
        ekf->delayed_state.baro_bias     += K_bias * bias_innov;
        ekf->delayed_state.baro_bias_var  = (1.0f - K_bias) * ekf->delayed_state.baro_bias_var;

        /* Process noise: keeps the filter responsive over long coast phases. */
        ekf->delayed_state.baro_bias_var += 1e-4f;
    }

    /* ------------------------------------------------------------------ */
    /* 3. Main 15-state fusion — bias-stripped altitude fed to SymForce   */
    /* ------------------------------------------------------------------ */
    float corrected_baro_alt = baro->altitude_m - ekf->delayed_state.baro_bias;

    float alt_pred   = -ekf->delayed_state.p_ned[2];
    float innovation = corrected_baro_alt - alt_pred;

    /* Down-position variance from the 15x15 P matrix (index 8). */
    float state_var_pD   = ekf->P[8*15 + 8];
    float innov_var_baro = state_var_pD + ekf->params.baro_noise_var;

    float test_ratio_baro = (innovation * innovation) /
                            ((ekf->params.baro_gate * ekf->params.baro_gate) * innov_var_baro);

    ekf->debug.baro_innov      = innovation;
    ekf->debug.baro_test_ratio = test_ratio_baro;
    ekf->debug.baro_rejected   = (test_ratio_baro > 1.0f);

    /* Gate: reject transonic pressure spikes. */
    if (ekf->debug.baro_rejected) {
        return;
    }

    symforce_update_baro(
        ekf->P,
        ekf->delayed_state.q,
        ekf->delayed_state.v_ned,
        ekf->delayed_state.p_ned,
        ekf->delayed_state.gyro_bias,
        ekf->delayed_state.accel_bias,
        corrected_baro_alt,  /* bias-stripped altitude */
        ekf->params.baro_noise_var,
        1e-6f
    );
}