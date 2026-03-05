#include "mag_fuse.h"
#include "ekf_math/symforce_wrapper.h"
#include <math.h>
#include <stdbool.h>

#define MAG_MAGNITUDE_GATE 3.0f /* 3-sigma gate for field strength */

static void _rotate_ned_to_body(const float q[4], const float v[3], float out[3])
{
    const float w = q[0], x = q[1], y = q[2], z = q[3];

    const float RT00 = 1.0f - 2.0f*(y*y + z*z);
    const float RT01 = 2.0f*(x*y + w*z);
    const float RT02 = 2.0f*(x*z - w*y);

    const float RT10 = 2.0f*(x*y - w*z);
    const float RT11 = 1.0f - 2.0f*(x*x + z*z);
    const float RT12 = 2.0f*(y*z + w*x);

    const float RT20 = 2.0f*(x*z + w*y);
    const float RT21 = 2.0f*(y*z - w*x);
    const float RT22 = 1.0f - 2.0f*(x*x + y*y);

    out[0] = RT00*v[0] + RT01*v[1] + RT02*v[2];
    out[1] = RT10*v[0] + RT11*v[1] + RT12*v[2];
    out[2] = RT20*v[0] + RT21*v[1] + RT22*v[2];
}

bool mag_fuse(ekf_core_t           *ekf,
              const mag_measurement_t *mag,
              const float           mag_ref_ned[3],
              float                 mag_var)
{
    float pred[3];
    _rotate_ned_to_body(ekf->delayed_state.q, mag_ref_ned, pred);

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
    float test_ratio_mag = (innov_mag * innov_mag) / ((MAG_MAGNITUDE_GATE * MAG_MAGNITUDE_GATE) * mag_var);

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