#include "baro_fuse.h"
#include "../ekf_math/symforce_wrapper.h"
#include <math.h>

/* Baro measurement noise variance: (0.5 m)^2 = 0.25 m^2 */
#define BARO_NOISE_VAR   0.25f

/* 5-sigma dynamic gate to handle standard aerodynamic buffeting */
#define BARO_GATE        5.0f

void baro_fuse(ekf_core_t *ekf, const baro_measurement_t *baro)
{
    float alt_pred  = -ekf->delayed_state.p_ned[2];
    float innovation = baro->altitude_m - alt_pred;

    /* Get State Variance from the P matrix (Index 8 is Down Position) */
    float state_var_pD = ekf->P[8*15 + 8];

    /* Calculate Innovation Variance */
    float innov_var_baro = state_var_pD + BARO_NOISE_VAR;

    /* Calculate Chi-Squared Test Ratio */
    float test_ratio_baro = (innovation * innovation) / ((BARO_GATE * BARO_GATE) * innov_var_baro);

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
        BARO_NOISE_VAR,
        1e-6f
    );
}