#include "gps_fuse.h"
#include "../ekf_math/symforce_wrapper.h"
#include <math.h>
#include <stdbool.h>

void gps_fuse(ekf_core_t *ekf, const gps_measurement_t *gps)
{
    /* ── Kinematic validity guard ────────────────────────────────────────── */
    float v0 = ekf->delayed_state.v_ned[0];
    float v1 = ekf->delayed_state.v_ned[1];
    float v2 = ekf->delayed_state.v_ned[2];
    float ekf_vel_mag  = sqrtf(v0*v0 + v1*v1 + v2*v2);
    float ekf_altitude = -ekf->delayed_state.p_ned[2];

    if (ekf_vel_mag > 500.0f || ekf_altitude > 80000.0f) {
        return;   /* Reject: receiver tracking limits exceeded */
    }

    static int rejected_samples = 0;
    bool reject_measurement = false;

    /* ── Chi-Squared Innovation Gates ────────────────────────────────────── */
    for (int i = 0; i < 3; i++) {
        /* Velocity Test Ratio */
        float innov_v = gps->vel_ned[i] - ekf->delayed_state.v_ned[i];
        float state_var_v = ekf->P[(3+i)*15 + (3+i)];
        float innov_var_v = state_var_v + ekf->params.gps_vel_var[i];
        float test_ratio_v = (innov_v * innov_v) /
                             ((ekf->params.gps_vel_gate * ekf->params.gps_vel_gate) * innov_var_v);

        /* Position Test Ratio */
        float innov_p = gps->pos_ned[i] - ekf->delayed_state.p_ned[i];
        float state_var_p = ekf->P[(6+i)*15 + (6+i)];
        float innov_var_p = state_var_p + ekf->params.gps_pos_var[i];
        float test_ratio_p = (innov_p * innov_p) /
                             ((ekf->params.gps_pos_gate * ekf->params.gps_pos_gate) * innov_var_p);

        /* --- DEBUG LOGGING --- */
        ekf->debug.gps_innov_vel[i]      = innov_v;
        ekf->debug.gps_test_ratio_vel[i] = test_ratio_v;
        ekf->debug.gps_innov_pos[i]      = innov_p;
        ekf->debug.gps_test_ratio_pos[i] = test_ratio_p;

        if (test_ratio_v > 1.0f || test_ratio_p > 1.0f) {
            reject_measurement = true;
            /* No break: continue so all 3 axes are evaluated and logged. */
        }
    }

    ekf->debug.gps_rejected              = reject_measurement;
    ekf->debug.gps_consecutive_rejections = (uint32_t)rejected_samples;

    /* ── Timeout & Hard Reset Logic ──────────────────────────────────────── */
    if (reject_measurement) {
        rejected_samples++;
        
        if (rejected_samples >= (int)ekf->params.gps_timeout_samples) {
            /* Execute Hard Reset */
            for (int i = 0; i < 3; i++) {
                /* Overwrite state values */
                ekf->delayed_state.v_ned[i] = gps->vel_ned[i];
                ekf->delayed_state.p_ned[i] = gps->pos_ned[i];
                
                /* Overwrite state variances */
                ekf->P[(3+i)*15 + (3+i)] = ekf->params.gps_vel_var[i];
                ekf->P[(6+i)*15 + (6+i)] = ekf->params.gps_pos_var[i];
                
                /* Decorrelate state: zero out non-diagonals for these rows/cols */
                for (int j = 0; j < 15; j++) {
                    if (j != (3+i)) {
                        ekf->P[(3+i)*15 + j] = 0.0f;
                        ekf->P[j*15 + (3+i)] = 0.0f;
                    }
                    if (j != (6+i)) {
                        ekf->P[(6+i)*15 + j] = 0.0f;
                        ekf->P[j*15 + (6+i)] = 0.0f;
                    }
                }
            }
            rejected_samples = 0;
        }
        return; 
    }

    rejected_samples = 0;

    symforce_update_gps(
        ekf->P,
        ekf->delayed_state.q,
        ekf->delayed_state.v_ned,
        ekf->delayed_state.p_ned,
        ekf->delayed_state.gyro_bias,
        ekf->delayed_state.accel_bias,
        gps->pos_ned,
        gps->vel_ned,
        ekf->params.gps_pos_var,
        ekf->params.gps_vel_var,
        1e-6f
    );
}