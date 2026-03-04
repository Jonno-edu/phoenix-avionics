/**
 * @file ekf_state.h
 * @brief EKF 5-stage warmup / flight state machine.
 *
 * Owns the transitions between flight modes for the EKF core.
 * The mathematical fusion (ZVU, mag, GPS, baro) still happens in the dedicated
 * fuse modules; this module only drives the *when* and *how* of that fusion.
 *
 * ┌─────────────────────────────────────────────────────────────────────────┐
 * │  State Machine Transitions                                              │
 * │                                                                         │
 * │  UNINITIALIZED ──→ LEVELING ──→ HEADING_ALIGN ──→ ZVU_CALIBRATING      │
 * │                                                         │               │
 * │                                             +2g liftoff ↓               │
 * │                                                     FLIGHT              │
 * └─────────────────────────────────────────────────────────────────────────┘
 *
 * Transition conditions:
 *   UNINITIALIZED  → LEVELING:       First IMU sample received.
 *   LEVELING       → HEADING_ALIGN:  After LEVELING_DURATION_S seconds of
 *                                    static IMU data. Pitch/roll set from avg gravity.
 *   HEADING_ALIGN  → ZVU_CALIBRATING: First valid magnetometer sample received.
 *                                    Yaw set from tilt-compensated heading.
 *   ZVU_CALIBRATING → FLIGHT:        Accel magnitude > 2g (liftoff trigger).
 *
 * Note: ekf_state_is_flight_ready() returns true from ZVU_CALIBRATING onwards
 * once biases have converged, allowing the commander to gate arming.
 */

#ifndef EKF_STATE_H
#define EKF_STATE_H

#ifdef __cplusplus
extern "C" {
#endif

#include "ekf_core.h"
#include <stdbool.h>
#include <stdint.h>

/* ── Tuning constants ────────────────────────────────────────────────────────── */

/** Duration of the leveling phase (seconds). */
#define LEVELING_DURATION_S       2.0f

/** Gyro bias change rate below which biases are considered converged (rad/s per second). */
#define ZVU_BIAS_RATE_THRESHOLD   1e-4f

/** Duration that bias change rate must remain below threshold to declare convergence (s). */
#define ZVU_CONVERGED_DURATION_S  5.0f

/** Liftoff detection threshold (m/s²). 2g. */
#define LIFTOFF_ACCEL_THRESHOLD   (2.0f * 9.80665f)

/**
 * Magnetometer noise variance used on the pad (Gauss²).
 * De-coupled from mag_fuse.h so ekf_state can override per flight phase.
 */
#define EKF_STATE_MAG_VAR_PAD     0.01f

/**
 * Magnetometer noise variance during flight (practically de-weights the update).
 */
#define EKF_STATE_MAG_VAR_FLIGHT  100.0f

/* ── Context struct ──────────────────────────────────────────────────────────── */

/**
 * @brief EKF state machine context.
 *
 * Holds all mutable state for the warmup sequence.
 * Should be allocated as a static or global object alongside ekf_core_t.
 * Initialise with ekf_state_init() before calling ekf_state_update().
 */
typedef struct {
    ekf_flight_mode_t mode;            /**< Current flight mode (mirrors ekf->flight_mode). */

    /* -- LEVELING phase bookkeeping -- */
    float  g_accum[3];         /**< Accumulated accelerometer sum for gravity estimate. */
    int    leveling_count;     /**< Number of IMU samples accumulated so far. */
    int    leveling_needed;    /**< Total samples required (computed from dt at init). */

    /* -- ZVU_CALIBRATING phase bookkeeping -- */
    float  bg_prev[3];             /**< Gyro bias from the previous step. */
    float  zvu_settled_time_s;     /**< Accumulated seconds below the convergence threshold. */
    bool   biases_converged;       /**< True once bias change rate drops below threshold. */

    /* -- Reference data -- */
    float  mag_ref_ned[3];         /**< WMM NED reference field (Gauss). Set at init. */
    float  current_mag_var;        /**< Active magnetometer variance. */

    /* -- Debug / telemetry -- */
    int    step;                   /**< Total update steps since init. */
} ekf_state_ctx_t;

/* ── API ─────────────────────────────────────────────────────────────────────── */

/**
 * @brief Initialise the state machine context.
 *
 * Must be called once before ekf_state_update().
 *
 * @param ctx         State machine context to initialise.
 * @param mag_ref_ned WMM reference field in NED frame (Gauss, float[3]).
 *                    For Stellenbosch (~32°S, 19°E): [0.15f, -0.05f, -0.33f].
 * @param dt          Sample period (seconds). Typically 0.004 for 250 Hz.
 */
void ekf_state_init(ekf_state_ctx_t *ctx,
                    const float      mag_ref_ned[3],
                    float            dt);

/**
 * @brief Run one step of the EKF state machine.
 *
 * Call this every 250 Hz IMU step, after calling imu_predict() / symforce_predict_covariance().
 * This function applies the appropriate fusion updates (ZVU, mag) based on the current
 * mode, and handles mode transitions.
 *
 * @param ctx       State machine context.
 * @param ekf       EKF core state (modified in-place).
 * @param accel_meas  Body-frame accelerometer measurement (m/s², float[3]).
 * @param gyro_raw    Body-frame raw gyro measurement (rad/s, float[3]).
 * @param mag_body    Body-frame magnetometer measurement (Gauss, float[3]).
 * @param mag_valid   true if mag_body contains a fresh, validated sample.
 * @param dt          Time step (seconds). Typically 0.004 for 250 Hz.
 */
void ekf_state_update(ekf_state_ctx_t *ctx,
                      ekf_core_t      *ekf,
                      const float      accel_meas[3],
                      const float      gyro_raw[3],
                      const float      mag_body[3],
                      bool             mag_valid,
                      float            dt);

/**
 * @brief Get the current flight mode.
 */
ekf_flight_mode_t ekf_state_get_mode(const ekf_state_ctx_t *ctx);

/**
 * @brief Returns true when the EKF is ready to support flight.
 *
 * True once the state machine has reached ZVU_CALIBRATING AND gyro/accel biases
 * have converged, OR the vehicle is already in FLIGHT mode.
 * Use this as the EKF health gate in the commander arming check.
 */
bool ekf_state_is_flight_ready(const ekf_state_ctx_t *ctx);

/**
 * @brief Hard-initialise attitude from known launch-rail geometry.
 *
 * Converts the given Z-Y-X Euler angles (azimuth/elevation/zero-roll) directly
 * into the EKF attitude quaternion and fast-forwards the state machine into
 * EKF_MODE_ZVU_CALIBRATING, bypassing the LEVELING and HEADING_ALIGN phases.
 *
 * Call this immediately after ekf_state_init() when the vehicle is known to be
 * fixed to a launch rail (GSE constraint), or in unit tests to skip the gravity
 * leveling checks.
 *
 * @param ctx           State machine context (mode will be overwritten).
 * @param ekf           EKF core state (q and flight_mode are modified in-place).
 * @param elevation_rad Launch rail elevation: 0 = horizontal, π/2 = vertical nose-up.
 * @param azimuth_rad   Launch rail azimuth from True North (0 = North, π/2 = East).
 */
void ekf_state_set_launch_rail(ekf_state_ctx_t *ctx,
                               ekf_core_t      *ekf,
                               float            elevation_rad,
                               float            azimuth_rad);

#ifdef __cplusplus
}
#endif

#endif /* EKF_STATE_H */
