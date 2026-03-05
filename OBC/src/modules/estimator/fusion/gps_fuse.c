#include "gps_fuse.h"
#include "../ekf_math/symforce_wrapper.h"
#include <math.h>
#include <stdbool.h>

/* GPS measurement noise variances */
static const float GPS_POS_VAR[3] = {2.25f, 2.25f, 2.25f};  /* (1.5 m)^2 */
static const float GPS_VEL_VAR[3] = {0.01f, 0.01f, 0.01f};  /* (0.1 m/s)^2 */

/* Dynamic Gates and Timeout */
#define GPS_POS_GATE 3.0f /* 3-sigma gate */
#define GPS_VEL_GATE 3.0f /* 3-sigma gate */
#define GPS_TIMEOUT_SAMPLES 50 /* 5 seconds at assumed 10Hz */

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
        float innov_var_v = state_var_v + GPS_VEL_VAR[i];
        float test_ratio_v = (innov_v * innov_v) / ((GPS_VEL_GATE * GPS_VEL_GATE) * innov_var_v);

        /* Position Test Ratio */
        float innov_p = gps->pos_ned[i] - ekf->delayed_state.p_ned[i];
        float state_var_p = ekf->P[(6+i)*15 + (6+i)];
        float innov_var_p = state_var_p + GPS_POS_VAR[i];
        float test_ratio_p = (innov_p * innov_p) / ((GPS_POS_GATE * GPS_POS_GATE) * innov_var_p);

        if (test_ratio_v > 1.0f || test_ratio_p > 1.0f) {
            reject_measurement = true;
            break; 
        }
    }

    /* ── Timeout & Hard Reset Logic ──────────────────────────────────────── */
    if (reject_measurement) {
        rejected_samples++;
        
        if (rejected_samples >= GPS_TIMEOUT_SAMPLES) {
            /* Execute Hard Reset */
            for (int i = 0; i < 3; i++) {
                /* Overwrite state values */
                ekf->delayed_state.v_ned[i] = gps->vel_ned[i];
                ekf->delayed_state.p_ned[i] = gps->pos_ned[i];
                
                /* Overwrite state variances */
                ekf->P[(3+i)*15 + (3+i)] = GPS_VEL_VAR[i];
                ekf->P[(6+i)*15 + (6+i)] = GPS_POS_VAR[i];
                
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
        GPS_POS_VAR,
        GPS_VEL_VAR,
        1e-6f
    );
}