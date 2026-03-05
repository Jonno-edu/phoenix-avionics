#include "mag_fuse.h"
#include "ekf_math/symforce_wrapper.h"
#include "math/ekf_math.h"
#include <math.h>
#include <stdbool.h>

bool mag_fuse(ekf_core_t           *ekf,
              const mag_measurement_t *mag,
              const float           mag_ref_ned[3],
              float                 mag_var)
{
    float pred[3];
    vec3_rotate_ned_to_body(ekf->delayed_state.q, mag_ref_ned, pred);

    const float *m = mag->field_gauss;

    float dot       = m[0]*pred[0] + m[1]*pred[1] + m[2]*pred[2];
    float norm_m    = sqrtf(m[0]*m[0]    + m[1]*m[1]    + m[2]*m[2]);
    float norm_pred = sqrtf(pred[0]*pred[0] + pred[1]*pred[1] + pred[2]*pred[2]);

    if (norm_m < 1e-6f || norm_pred < 1e-6f) {
        return false; 
    }

    /* ── 1. Angular Innovation Gate ─────────────────────────────────────────── */
    float cos_angle = dot / (norm_m * norm_pred);
    if (cos_angle < MAG_GATE_COS) {
        return false;  
    }

    /* ── 2. Magnitude Chi-Squared Gate ──────────────────────────────────────── */
    /* An external EM pulse will radically alter the magnitude of the reading.
     * Since body rotation does not alter the length of the Earth's magnetic 
     * vector, the state uncertainty regarding the magnitude is essentially 0.
     * Therefore, innov_var is purely the sensor noise (mag_var).
     */
    float innov_mag = norm_m - norm_pred;
    float test_ratio_mag = (innov_mag * innov_mag) /
                           ((ekf->params.mag_magnitude_gate * ekf->params.mag_magnitude_gate) * mag_var);

    if (test_ratio_mag > 1.0f) {
        return false; /* High-current pyrotechnic spike detected — reject */
    }

    /* ── 3. Apply SymForce-generated Kalman Update ─────────────────────────── */
    const float mag_var3[3] = { mag_var, mag_var, mag_var };

    symforce_update_mag(
        ekf->P,
        ekf->delayed_state.q, ekf->delayed_state.v_ned, ekf->delayed_state.p_ned,
        ekf->delayed_state.gyro_bias, ekf->delayed_state.accel_bias,
        mag->field_gauss, mag_ref_ned, mag_var3, 1e-6f
    );

    return true;
}