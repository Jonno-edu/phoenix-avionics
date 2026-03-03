#ifndef BARO_FUSE_H
#define BARO_FUSE_H

#include "../ekf_core.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Barometric measurement for EKF fusion.
 * altitude_m is in metres, positive up (i.e. -p_ned[2]).
 */
typedef struct {
    float altitude_m;
} baro_measurement_t;

/**
 * @brief Fuse a barometric altitude measurement into the EKF.
 *
 * Applies a 5 m innovation gate before calling the SymForce Kalman update.
 * The gate protects against outliers and early-flight initialisation spikes.
 *
 * Noise model: baro_var = 0.25 m² (~0.5 m 1-sigma).
 *
 * @param ekf  Pointer to the live EKF state.
 * @param baro Incoming barometric measurement.
 */
void baro_fuse(ekf_core_t *ekf, const baro_measurement_t *baro);

#ifdef __cplusplus
}
#endif

#endif // BARO_FUSE_H
