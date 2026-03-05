#include "baro_fuse.h"
#include "../ekf_math/symforce_wrapper.h"
#include <math.h>

void baro_fuse(ekf_core_t *ekf, const baro_measurement_t *baro)
{
    float alt_pred  = -ekf->delayed_state.p_ned[2];
    float innovation = baro->altitude_m - alt_pred;

    /* Get State Variance from the P matrix (Index 8 is Down Position) */
    float state_var_pD = ekf->P[8*15 + 8];

    /* Calculate Innovation Variance */
    float innov_var_baro = state_var_pD + ekf->params.baro_noise_var;

    /* Calculate Chi-Squared Test Ratio */
    float test_ratio_baro = (innovation * innovation) /
                            ((ekf->params.baro_gate * ekf->params.baro_gate) * innov_var_baro);

    /* Gate: Reject-Only for transonic pressure waves */
    if (test_ratio_baro > 1.0f) {
        return;
    }

    symforce_update_baro(
        ekf->P,
        ekf->delayed_state.q,
        ekf->delayed_state.v_ned,
        ekf->delayed_state.p_ned,
        ekf->delayed_state.gyro_bias,
        ekf->delayed_state.accel_bias,
        baro->altitude_m,
        ekf->params.baro_noise_var,
        1e-6f
    );
}