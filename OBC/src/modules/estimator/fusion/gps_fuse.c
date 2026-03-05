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
    /* ── Kinematic validity guard ──────────────────────────────────────────
     * Reject GPS updates if the EKF's internal state exceeds the receiver's
     * operational tracking limits.  This defends against receivers that
     * continue to output stale or garbage fixes during high-dynamic flight
     * before they assert a "fix lost" flag (e.g. num_sats, fix_type).
     *
     * Operational limits (u-blox / CoCom export control):
     *   Velocity  ≤ 500 m/s   (>500 m/s → tracking loops break)
     *   Altitude  ≤ 80,000 m  (CoCom hard ceiling)
     *   Dynamics  ≤ 4 g       (enforced in simulation; gated here on vel/alt
     *                          since platform acceleration is not stored in
     *                          the EKF state vector)
     *
     * When either limit is exceeded the EKF dead-reckons on IMU + Baro only
     * and GPS is re-enabled automatically once the state drops back below
     * threshold (the re-acquisition dead-band is handled in the simulator).
     */
    float v0 = ekf->delayed_state.v_ned[0];
    float v1 = ekf->delayed_state.v_ned[1];
    float v2 = ekf->delayed_state.v_ned[2];
    float ekf_vel_mag  = sqrtf(v0*v0 + v1*v1 + v2*v2);
    float ekf_altitude = -ekf->delayed_state.p_ned[2];   /* NED Z-down → positive-up */

    if (ekf_vel_mag > 500.0f || ekf_altitude > 80000.0f) {
        return;   /* Reject: receiver tracking limits exceeded */
    }

    /* Per-axis innovation gates */
    for (int i = 0; i < 3; i++) {
        float pos_innov = gps->pos_ned[i] - ekf->delayed_state.p_ned[i];
        float vel_innov = gps->vel_ned[i] - ekf->delayed_state.v_ned[i];

        if (fabsf(pos_innov) > GPS_POS_GATE || fabsf(vel_innov) > GPS_VEL_GATE) {
            return;
        }
    }

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
