#include "gps_fuse.h"
#include "../ekf_math/symforce_wrapper.h"
#include <math.h>
#include <stdbool.h>

/* GPS measurement noise variances */
static const float GPS_POS_VAR[3] = {2.25f, 2.25f, 2.25f};  /* (1.5 m)^2 */
static const float GPS_VEL_VAR[3] = {0.01f, 0.01f, 0.01f};  /* (0.1 m/s)^2 */

/* Innovation gates */
#define GPS_POS_GATE  20.0f  /* metres */
#define GPS_VEL_GATE   5.0f  /* m/s    */

void gps_fuse(ekf_core_t *ekf, const gps_measurement_t *gps)
{
    /* Per-axis innovation gates */
    for (int i = 0; i < 3; i++) {
        float pos_innov = gps->pos_ned[i] - ekf->state.p_ned[i];
        float vel_innov = gps->vel_ned[i] - ekf->state.v_ned[i];

        if (fabsf(pos_innov) > GPS_POS_GATE || fabsf(vel_innov) > GPS_VEL_GATE) {
            return;
        }
    }

    symforce_update_gps(
        ekf->P,
        ekf->state.q,
        ekf->state.v_ned,
        ekf->state.p_ned,
        ekf->state.gyro_bias,
        ekf->state.accel_bias,
        gps->pos_ned,
        gps->vel_ned,
        GPS_POS_VAR,
        GPS_VEL_VAR,
        1e-6f
    );
}
