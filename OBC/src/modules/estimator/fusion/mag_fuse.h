/**
 * @file mag_fuse.h
 * @brief 3-axis magnetometer fusion update for the error-state EKF.
 *
 * Observation model:  h(q) = q⁻¹ ⊗ mag_ref_ned ⊗ q
 *                           (reference NED field rotated into body frame)
 * Innovation:         y = mag_body - h(q)
 * H matrix:           [3×15], non-zero only in attitude columns 0–2.
 *
 * The update corrects all three attitude axes, but yaw (heading) is the
 * primary observable since GPS/Baro/IMU cannot observe it.
 *
 * Gate:               Angular gate — rejects if angle between measured
 *                     and predicted field > MAG_GATE_ANGLE_DEG (30°).
 *                     This tolerates calibration offsets and EMI spikes
 *                     while rejecting catastrophically wrong readings.
 *
 * Typical use:
 *   - Pad alignment:  mag_var = MAG_DEFAULT_VAR (~0.01)
 *   - Motor burn/flight: mag_var = MAG_FLIGHT_VAR (~100.0) to de-weight.
 */

#ifndef MAG_FUSE_H
#define MAG_FUSE_H

#ifdef __cplusplus
extern "C" {
#endif

#include "ekf_core.h"
#include <stdbool.h>

/* ── Constants ──────────────────────────────────────────────────────────────── */

/** Per-axis noise variance for a calibrated, low-interference environment (Gauss²). */
#define MAG_DEFAULT_VAR    0.01f

/** Per-axis noise variance during high-dynamic flight (essentially de-weights mag). */
#define MAG_FLIGHT_VAR   100.0f

/** Angular gate threshold: cos(30°) = 0.866. */
#define MAG_GATE_COS      0.8660254f

/* ── Types ──────────────────────────────────────────────────────────────────── */

typedef struct {
    float field_gauss[3];  /**< Body-frame magnetic field measurement (Gauss). */
} mag_measurement_t;

/* ── API ────────────────────────────────────────────────────────────────────── */

/**
 * @brief Apply a 3-axis magnetometer update to the EKF.
 *
 * @param ekf          Pointer to the EKF core state (modified in-place).
 * @param mag          Body-frame magnetometer measurement.
 * @param mag_ref_ned  Reference NED magnetic field vector from WMM (Gauss, float[3]).
 *                     For Stellenbosch (~32°S, 19°E): approximately [0.15, -0.05, -0.33].
 * @param mag_var      Per-axis measurement noise variance (Gauss²).
 *                     Use MAG_DEFAULT_VAR on the pad; MAG_FLIGHT_VAR during motor burn.
 * @return             true if the update was accepted (gate passed).
 *                     false if the gate fired (measurement discarded).
 */
bool mag_fuse(ekf_core_t           *ekf,
              const mag_measurement_t *mag,
              const float           mag_ref_ned[3],
              float                 mag_var);

#ifdef __cplusplus
}
#endif

#endif /* MAG_FUSE_H */
