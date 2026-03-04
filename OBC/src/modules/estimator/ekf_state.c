/**
 * @file ekf_state.c
 * @brief EKF 5-stage warmup / flight state machine implementation.
 *
 * Drives the transitions between EKF_MODE_* states and applies the
 * appropriate sensor fusion updates (ZVU, magnetometer) at each stage.
 *
 * GPS and barometer fusion are applied externally (by ekf.c / baro_fuse.c /
 * gps_fuse.c) and are not gated by this state machine directly — the caller
 * simply should not attempt to fuse GPS/baro before flight_mode >= ZVU_CALIBRATING.
 *
 * Coordinate conventions:
 *  - Quaternion q[4] = [w, x, y, z], body-to-NED.
 *  - Gravity in NED: g_ned = [0, 0, +9.80665] m/s² (Z is down in NED).
 *  - Accelerometer measures specific force: a_meas ≈ R^T * g_ned when stationary.
 *  - For a level body (q = identity): a_meas ≈ [0, 0, g].
 *  - Standard AHRS pitch/roll extraction:
 *      pitch = atan2(-ax, sqrt(ay² + az²))
 *      roll  = atan2(ay, az)
 */

#include "ekf_state.h"
#include "ekf_math/symforce_wrapper.h"
#include "fusion/mag_fuse.h"

#include <math.h>
#include <string.h>

/* ── Internal helpers ────────────────────────────────────────────────────────── */

/** Gravity constant (m/s²). */
#define GRAVITY_MS2 9.80665f

/**
 * Set pitch/roll from the averaged gravitational acceleration, leaving yaw at 0.
 * g_avg is the average accelerometer output in body frame (m/s²).
 * Populates ekf->state.q with the estimated body-to-NED quaternion.
 */
static void _set_attitude_from_gravity(ekf_core_t *ekf, const float g_avg[3])
{
    // The accelerometer measures specific force. When stationary, the pad pushes UP,
    // so the measurement is -g in the Z axis. We must flip the vector to align 
    // it with NED gravity (which acts downwards).
    float ax = -g_avg[0], ay = -g_avg[1], az = -g_avg[2];
    
    float norm = sqrtf(ax*ax + ay*ay + az*az);
    if (norm < 1.0f) return;   /* degenerate — no update */
    ax /= norm; ay /= norm; az /= norm;

    /* Standard AHRS extraction — valid for body Z = down convention */
    float pitch = atan2f(-ax, sqrtf(ay*ay + az*az));
    float roll  = atan2f(ay, az);

    /* Build quaternion q = Ry(pitch) ⊗ Rx(roll)  [yaw = 0]
     * This is the body-to-NED rotation for pitch and roll only. */
    float cp = cosf(pitch * 0.5f), sp = sinf(pitch * 0.5f);
    float cr = cosf(roll  * 0.5f), sr = sinf(roll  * 0.5f);

    float qw = cp * cr;
    float qx = cp * sr;
    float qy = sp * cr;
    float qz = -sp * sr;

    /* Normalise (shouldn't be needed but guards floating-point drift) */
    float n = sqrtf(qw*qw + qx*qx + qy*qy + qz*qz);
    if (n < 1e-6f) return;

    ekf->state.q[0] = qw / n;
    ekf->state.q[1] = qx / n;
    ekf->state.q[2] = qy / n;
    ekf->state.q[3] = qz / n;
}

/**
 * Tilt-compensated heading from a mag measurement, given the current attitude.
 *
 * Rotates mag_body into NED using the current quaternion, then extracts the
 * magnetic heading from the horizontal components. Subtracts the WMM declination
 * (atan2 of the horizontal reference field) to deliver a true heading offset.
 *
 * Returns the heading correction angle (radians) that must be composed
 * with the current attitude quaternion as a Rz rotation.
 */
static float _heading_from_mag(const float q[4],
                                const float mag_body[3],
                                const float mag_ref_ned[3])
{
    /* Rotate body mag into NED using R (body-to-NED): mag_ned = R * mag_body */
    const float w = q[0], x = q[1], y = q[2], z = q[3];

    /* Rodrigues rotation: v' = R * v */
    float t0 = 2.0f * (y*mag_body[2] - z*mag_body[1]);
    float t1 = 2.0f * (z*mag_body[0] - x*mag_body[2]);
    float t2 = 2.0f * (x*mag_body[1] - y*mag_body[0]);

    float mag_ned[3] = {
        mag_body[0] + w*t0 + y*t2 - z*t1,
        mag_body[1] + w*t1 + z*t0 - x*t2,
        mag_body[2] + w*t2 + x*t1 - y*t0
    };

    /* Measured heading in NED: atan2(east_component, north_component) */
    float heading_meas = atan2f(mag_ned[1], mag_ned[0]);

    /* Reference declination: angle of the WMM horizontal field from NED North */
    float heading_ref  = atan2f(mag_ref_ned[1], mag_ref_ned[0]);

    /* Heading correction = true bearing of vehicle */
    return heading_meas - heading_ref;
}

/**
 * Apply a yaw rotation psi to the current attitude quaternion.
 * q_new = Rz(psi) ⊗ q_current
 *
 * In [w,x,y,z] form, the yaw quaternion is q_z = [cos(psi/2), 0, 0, sin(psi/2)].
 */
static void _apply_yaw(ekf_core_t *ekf, float psi)
{
    float cy = cosf(psi * 0.5f), sy = sinf(psi * 0.5f);
    const float *qc = ekf->state.q;

    /* q_z ⊗ q_current:
     *   w' = cy*qw - sy*qz
     *   x' = cy*qx - sy*qy
     *   y' = cy*qy + sy*qx
     *   z' = cy*qz + sy*qw  */
    float q_new[4] = {
        cy*qc[0] - sy*qc[3],
        cy*qc[1] - sy*qc[2],
        cy*qc[2] + sy*qc[1],
        cy*qc[3] + sy*qc[0]
    };

    /* Normalise */
    float n = sqrtf(q_new[0]*q_new[0] + q_new[1]*q_new[1] +
                    q_new[2]*q_new[2] + q_new[3]*q_new[3]);
    if (n < 1e-6f) return;
    for (int i = 0; i < 4; i++) ekf->state.q[i] = q_new[i] / n;
}

/**
 * Returns the magnitude of a 3-vector.
 */
static float _vec3_norm(const float v[3])
{
    return sqrtf(v[0]*v[0] + v[1]*v[1] + v[2]*v[2]);
}

/* ── Public API ──────────────────────────────────────────────────────────────── */

void ekf_state_init(ekf_state_ctx_t *ctx,
                    const float      mag_ref_ned[3],
                    float            dt)
{
    memset(ctx, 0, sizeof(ekf_state_ctx_t));
    ctx->mode           = EKF_MODE_UNINITIALIZED;
    ctx->current_mag_var = EKF_STATE_MAG_VAR_PAD;

    for (int i = 0; i < 3; i++) ctx->mag_ref_ned[i] = mag_ref_ned[i];

    /* Number of IMU samples needed for the leveling phase */
    ctx->leveling_needed = (int)(LEVELING_DURATION_S / dt + 0.5f);
    if (ctx->leveling_needed < 10) ctx->leveling_needed = 10;
}

void ekf_state_update(ekf_state_ctx_t *ctx,
                      ekf_core_t      *ekf,
                      const float      accel_meas[3],
                      const float      gyro_raw[3],
                      const float      mag_body[3],
                      bool             mag_valid,
                      float            dt)
{
    ctx->step++;

    switch (ctx->mode) {

    /* ── UNINITIALIZED: first IMU sample kicks off leveling ─────────────────── */
    case EKF_MODE_UNINITIALIZED:
        ctx->mode = EKF_MODE_LEVELING;
        ekf->flight_mode = EKF_MODE_LEVELING;
        /* Fall through to start accumulating immediately */
        /* FALLTHROUGH */

    /* ── LEVELING: accumulate gravity, set pitch/roll ────────────────────────── */
    case EKF_MODE_LEVELING: {
        /* Apply ZVU to begin converging biases while waiting */
        const float vel_var[3]  = {1e-6f, 1e-6f, 1e-6f};
        const float gyro_var[3] = {1e-6f, 1e-6f, 1e-6f};
        symforce_update_stationary(
            ekf->P, ekf->state.q, ekf->state.v_ned, ekf->state.p_ned,
            ekf->state.gyro_bias, ekf->state.accel_bias,
            gyro_raw, vel_var, gyro_var
        );

        /* Accumulate accelerometer samples for gravity estimation */
        ctx->g_accum[0] += accel_meas[0];
        ctx->g_accum[1] += accel_meas[1];
        ctx->g_accum[2] += accel_meas[2];
        ctx->leveling_count++;

        if (ctx->leveling_count >= ctx->leveling_needed) {
            /* Compute mean gravity and set attitude */
            float g_avg[3] = {
                ctx->g_accum[0] / (float)ctx->leveling_count,
                ctx->g_accum[1] / (float)ctx->leveling_count,
                ctx->g_accum[2] / (float)ctx->leveling_count
            };
            _set_attitude_from_gravity(ekf, g_avg);

            /* Transition to HEADING_ALIGN — wait for next valid mag sample */
            ctx->mode = EKF_MODE_HEADING_ALIGN;
            ekf->flight_mode = EKF_MODE_HEADING_ALIGN;
        }
        break;
    }

    /* ── HEADING_ALIGN: set yaw on first valid mag, then immediately advance ── */
    case EKF_MODE_HEADING_ALIGN: {
        /* Keep applying ZVU while waiting for mag */
        const float vel_var[3]  = {1e-6f, 1e-6f, 1e-6f};
        const float gyro_var[3] = {1e-6f, 1e-6f, 1e-6f};
        symforce_update_stationary(
            ekf->P, ekf->state.q, ekf->state.v_ned, ekf->state.p_ned,
            ekf->state.gyro_bias, ekf->state.accel_bias,
            gyro_raw, vel_var, gyro_var
        );

        if (mag_valid) {
            float psi = _heading_from_mag(ekf->state.q, mag_body, ctx->mag_ref_ned);
            _apply_yaw(ekf, psi);

            /* Initialise previous bias for convergence monitoring */
            for (int i = 0; i < 3; i++) ctx->bg_prev[i] = ekf->state.gyro_bias[i];

            ctx->mode = EKF_MODE_ZVU_CALIBRATING;
            ekf->flight_mode = EKF_MODE_ZVU_CALIBRATING;
        }
        break;
    }

    /* ── ZVU_CALIBRATING: run ZVU, fuse mag, monitor convergence, detect 2g ── */
    case EKF_MODE_ZVU_CALIBRATING: {
        /* ZVU every step */
        const float vel_var[3]  = {1e-6f, 1e-6f, 1e-6f};
        const float gyro_var[3] = {1e-6f, 1e-6f, 1e-6f};
        symforce_update_stationary(
            ekf->P, ekf->state.q, ekf->state.v_ned, ekf->state.p_ned,
            ekf->state.gyro_bias, ekf->state.accel_bias,
            gyro_raw, vel_var, gyro_var
        );

        /* Mag fusion at pad variance */
        if (mag_valid) {
            mag_measurement_t mag_m;
            for (int i = 0; i < 3; i++) mag_m.field_gauss[i] = mag_body[i];
            mag_fuse(ekf, &mag_m, ctx->mag_ref_ned, ctx->current_mag_var);
        }

        /* Bias convergence check */
        float bg_change_norm = _vec3_norm((float[3]){
            (ekf->state.gyro_bias[0] - ctx->bg_prev[0]) / dt,
            (ekf->state.gyro_bias[1] - ctx->bg_prev[1]) / dt,
            (ekf->state.gyro_bias[2] - ctx->bg_prev[2]) / dt
        });
        for (int i = 0; i < 3; i++) ctx->bg_prev[i] = ekf->state.gyro_bias[i];

        if (bg_change_norm < ZVU_BIAS_RATE_THRESHOLD) {
            ctx->zvu_settled_time_s += dt;
        } else {
            ctx->zvu_settled_time_s = 0.0f;  /* Reset if biases jump */
        }

        if (ctx->zvu_settled_time_s >= ZVU_CONVERGED_DURATION_S) {
            ctx->biases_converged = true;
        }

        /* Liftoff detection: if total specific force > 2g → flight */
        if (_vec3_norm(accel_meas) > LIFTOFF_ACCEL_THRESHOLD) {
            ctx->mode = EKF_MODE_FLIGHT;
            ekf->flight_mode = EKF_MODE_FLIGHT;
            ctx->current_mag_var = EKF_STATE_MAG_VAR_FLIGHT;
        }
        break;
    }

    /* ── FLIGHT: ZVU disabled, mag de-weighted ───────────────────────────────── */
    case EKF_MODE_FLIGHT: {
        /* Mag fusion with inflated variance (de-weighted, not disabled) */
        if (mag_valid) {
            mag_measurement_t mag_m;
            for (int i = 0; i < 3; i++) mag_m.field_gauss[i] = mag_body[i];
            mag_fuse(ekf, &mag_m, ctx->mag_ref_ned, ctx->current_mag_var);
        }
        break;
    }

    default:
        break;
    }
}

ekf_flight_mode_t ekf_state_get_mode(const ekf_state_ctx_t *ctx)
{
    return ctx->mode;
}

bool ekf_state_is_flight_ready(const ekf_state_ctx_t *ctx)
{
    if (ctx->mode == EKF_MODE_FLIGHT) return true;
    if (ctx->mode == EKF_MODE_ZVU_CALIBRATING && ctx->biases_converged) return true;
    return false;
}
