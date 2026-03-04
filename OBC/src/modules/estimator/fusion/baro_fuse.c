#include "baro_fuse.h"
#include "../ekf_math/symforce_wrapper.h"
#include <math.h>

/* Baro measurement noise variance: (0.5 m)^2 = 0.25 m^2 */
#define BARO_NOISE_VAR   0.25f

/* Innovation gate: reject measurements more than 5 m from the prediction */
#define BARO_INNOV_GATE  5.0f

void baro_fuse(ekf_core_t *ekf, const baro_measurement_t *baro)
{
    /* predicted altitude from nominal state: alt_pred = -p_ned[2] */
    float alt_pred  = -ekf->state.p_ned[2];
    float innovation = baro->altitude_m - alt_pred;

    /* Innovation gate — reject outliers */
    if (fabsf(innovation) > BARO_INNOV_GATE) {
        return;
    }

    symforce_update_baro(
        ekf->P,
        ekf->state.q,
        ekf->state.v_ned,
        ekf->state.p_ned,
        ekf->state.gyro_bias,
        ekf->state.accel_bias,
        baro->altitude_m,
        BARO_NOISE_VAR,
        1e-6f
    );
}
