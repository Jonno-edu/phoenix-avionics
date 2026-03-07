#ifndef EKF_MATH_H
#define EKF_MATH_H

/**
 * @file ekf_math.h
 * @brief Lightweight quaternion / vector / DCM helpers for the Phoenix EKF.
 *
 * All quaternions use the [w, x, y, z] convention (scalar first).
 * All rotations are body→NED unless the function name says otherwise.
 * No dynamic allocation; no FreeRTOS dependencies; safe for HOST unit tests.
 */

#include <math.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ── Vector 3 Operations ──────────────────────────────────────────────────── */

/** out = a × b (right-hand cross product) */
void vec3_cross(const float a[3], const float b[3], float out[3]);

/** Returns the Euclidean norm of v. */
float vec3_norm(const float v[3]);

/** Returns the dot product a · b. */
float vec3_dot(const float a[3], const float b[3]);

/* ── Quaternion Operations [w, x, y, z] ──────────────────────────────────── */

/** Normalise q in-place. No-op if the norm is below 1e-6. */
void quat_normalize(float q[4]);

/** Build a unit quaternion from Euler angles (ZYX, intrinsic). */
void quat_from_euler(float roll, float pitch, float yaw, float q[4]);

/**
 * @brief Apply a yaw rotation psi (radians) on top of q_current.
 *
 * Equivalent to: q_out = q_yaw(psi) ⊗ q_current, normalised.
 * Used during heading alignment to initialise yaw without disturbing
 * the levelled pitch/roll already stored in q_current.
 */
void quat_apply_yaw(float q_current[4], float psi, float q_out[4]);

/* ── Kinematic Integration ────────────────────────────────────────────────── */

/**
 * @brief Small-angle quaternion integration.
 *
 * Updates q in-place using the first-order approximation:
 *   q_new = q ⊗ [1, ½ω·dt]
 * Valid when |ω·dt| ≪ 1 (guaranteed at 250 Hz with any reasonable rate).
 *
 * @param q               Quaternion to update [w,x,y,z] (in-place).
 * @param gyro_corrected  Bias-corrected angular rate (rad/s), body frame.
 * @param dt              Integration interval (seconds).
 */
void quat_integrate_small_angle(float q[4], const float gyro_corrected[3], float dt);

/**
 * @brief Exact quaternion integration using trigonometric exponential map.
 *
 * Computes dq = exp(0.5 * ω·dt) exactly via sin/cos, then applies
 *   q_new = q ⊗ dq
 * Safe for high spin rates where the small-angle approximation breaks down.
 * Falls back to the small-angle polynomial when |ω·dt| < 1e-4 rad to
 * avoid division by zero.
 *
 * @param q               Quaternion to update [w,x,y,z] (in-place).
 * @param gyro_corrected  Bias-corrected angular rate (rad/s), body frame.
 * @param dt              Integration interval (seconds).
 */
void quat_integrate_exact(float q[4], const float gyro_corrected[3], float dt);

/* ── Rotations and Direction Cosine Matrices ──────────────────────────────── */

/**
 * @brief Build the 3×3 Direction Cosine Matrix (DCM) from a quaternion.
 *
 * R transforms a body-frame vector to NED:  v_ned = R · v_body.
 *
 * @param q Quaternion [w,x,y,z].
 * @param R Output 3×3 DCM (row-major).
 */
void dcm_from_quat(const float q[4], float R[3][3]);

/** Rotate a body-frame vector to NED using R(q). */
void vec3_rotate_body_to_ned(const float q[4], const float v_body[3], float v_ned[3]);

/** Rotate a NED vector to body frame using Rᵀ(q). */
void vec3_rotate_ned_to_body(const float q[4], const float v_ned[3], float v_body[3]);

#ifdef __cplusplus
}
#endif

#endif /* EKF_MATH_H */
