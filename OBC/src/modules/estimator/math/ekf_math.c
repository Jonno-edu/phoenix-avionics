#include "ekf_math.h"

/* ── Vector 3 Operations ──────────────────────────────────────────────────── */

void vec3_cross(const float a[3], const float b[3], float out[3]) {
    out[0] = a[1]*b[2] - a[2]*b[1];
    out[1] = a[2]*b[0] - a[0]*b[2];
    out[2] = a[0]*b[1] - a[1]*b[0];
}

float vec3_norm(const float v[3]) {
    return sqrtf(v[0]*v[0] + v[1]*v[1] + v[2]*v[2]);
}

float vec3_dot(const float a[3], const float b[3]) {
    return a[0]*b[0] + a[1]*b[1] + a[2]*b[2];
}

/* ── Quaternion Operations ────────────────────────────────────────────────── */

void quat_normalize(float q[4]) {
    float n = sqrtf(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);
    if (n > 1e-6f) {
        q[0] /= n; q[1] /= n; q[2] /= n; q[3] /= n;
    }
}

void quat_from_euler(float roll, float pitch, float yaw, float q[4]) {
    float cy = cosf(yaw   * 0.5f), sy = sinf(yaw   * 0.5f);
    float cp = cosf(pitch * 0.5f), sp = sinf(pitch * 0.5f);
    float cr = cosf(roll  * 0.5f), sr = sinf(roll  * 0.5f);

    q[0] = cy * cp * cr + sy * sp * sr;  /* w */
    q[1] = cy * sp * cr - sy * cp * sr;  /* x ( = roll rotation ) */
    q[2] = cy * cp * sr + sy * sp * cr;  /* y ( = pitch rotation ) */
    q[3] = sy * cp * cr - cy * sp * sr;  /* z ( = yaw rotation ) */
    quat_normalize(q);
}

void quat_apply_yaw(float q_current[4], float psi, float q_out[4]) {
    float cy = cosf(psi * 0.5f), sy = sinf(psi * 0.5f);
    /* q_yaw = [cy, 0, 0, sy].  q_out = q_yaw ⊗ q_current. */
    q_out[0] = cy*q_current[0] - sy*q_current[3];
    q_out[1] = cy*q_current[1] - sy*q_current[2];
    q_out[2] = cy*q_current[2] + sy*q_current[1];
    q_out[3] = cy*q_current[3] + sy*q_current[0];
    quat_normalize(q_out);
}

/* ── Kinematic Integration ────────────────────────────────────────────────── */

void quat_integrate_small_angle(float q[4], const float gyro_corrected[3], float dt) {
    const float half_angle[3] = {
        0.5f * gyro_corrected[0] * dt,
        0.5f * gyro_corrected[1] * dt,
        0.5f * gyro_corrected[2] * dt
    };

    const float q_new[4] = {
        q[0] - q[1]*half_angle[0] - q[2]*half_angle[1] - q[3]*half_angle[2],
        q[1] + q[0]*half_angle[0] + q[2]*half_angle[2] - q[3]*half_angle[1],
        q[2] + q[0]*half_angle[1] + q[3]*half_angle[0] - q[1]*half_angle[2],
        q[3] + q[0]*half_angle[2] + q[1]*half_angle[1] - q[2]*half_angle[0]
    };

    q[0] = q_new[0]; q[1] = q_new[1]; q[2] = q_new[2]; q[3] = q_new[3];
    quat_normalize(q);
}

/* ── Rotations and DCMs ───────────────────────────────────────────────────── */

void dcm_from_quat(const float q[4], float R[3][3]) {
    /* q = [w, x, y, z] */
    R[0][0] = 1.0f - 2.0f * (q[2]*q[2] + q[3]*q[3]);
    R[0][1] = 2.0f * (q[1]*q[2] - q[0]*q[3]);
    R[0][2] = 2.0f * (q[1]*q[3] + q[0]*q[2]);
    R[1][0] = 2.0f * (q[1]*q[2] + q[0]*q[3]);
    R[1][1] = 1.0f - 2.0f * (q[1]*q[1] + q[3]*q[3]);
    R[1][2] = 2.0f * (q[2]*q[3] - q[0]*q[1]);
    R[2][0] = 2.0f * (q[1]*q[3] - q[0]*q[2]);
    R[2][1] = 2.0f * (q[2]*q[3] + q[0]*q[1]);
    R[2][2] = 1.0f - 2.0f * (q[1]*q[1] + q[2]*q[2]);
}

void vec3_rotate_body_to_ned(const float q[4], const float v_body[3], float v_ned[3]) {
    float R[3][3];
    dcm_from_quat(q, R);
    v_ned[0] = R[0][0]*v_body[0] + R[0][1]*v_body[1] + R[0][2]*v_body[2];
    v_ned[1] = R[1][0]*v_body[0] + R[1][1]*v_body[1] + R[1][2]*v_body[2];
    v_ned[2] = R[2][0]*v_body[0] + R[2][1]*v_body[1] + R[2][2]*v_body[2];
}

void vec3_rotate_ned_to_body(const float q[4], const float v_ned[3], float v_body[3]) {
    /* Use the transpose of R (Rᵀ rotates NED → body). */
    float R[3][3];
    dcm_from_quat(q, R);
    v_body[0] = R[0][0]*v_ned[0] + R[1][0]*v_ned[1] + R[2][0]*v_ned[2];
    v_body[1] = R[0][1]*v_ned[0] + R[1][1]*v_ned[1] + R[2][1]*v_ned[2];
    v_body[2] = R[0][2]*v_ned[0] + R[1][2]*v_ned[1] + R[2][2]*v_ned[2];
}
