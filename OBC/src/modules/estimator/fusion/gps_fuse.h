#ifndef GPS_FUSE_H
#define GPS_FUSE_H

#include "../ekf_core.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief GPS measurement for EKF fusion.
 * All values are in the NED frame.
 * pos_ned: metres. vel_ned: m/s.
 */
typedef struct {
    float pos_ned[3];
    float vel_ned[3];
} gps_measurement_t;

/**
 * @brief Fuse a 6-DOF GPS pos+vel measurement into the EKF.
 *
 * Per-axis innovation gates are applied before the Kalman update:
 *   position:  ±20 m
 *   velocity:  ±5 m/s
 *
 * Noise model:
 *   pos_var = {2.25, 2.25, 2.25} m²  (~1.5 m 1-sigma horizontal/vertical)
 *   vel_var = {0.01, 0.01, 0.01} m²/s²  (~0.1 m/s 1-sigma)
 *
 * @param ekf Pointer to the live EKF state.
 * @param gps Incoming GPS measurement.
 */
void gps_fuse(ekf_core_t *ekf, const gps_measurement_t *gps);

#ifdef __cplusplus
}
#endif

#endif // GPS_FUSE_H
