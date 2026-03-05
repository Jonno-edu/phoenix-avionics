/**
 * @file test_utils.c
 * @brief Shared test globals and helper implementations extracted from the
 *        original monolithic test_ekf_predict.c.
 */

#include "test_utils.h"

/* ── Global test counters ───────────────────────────────────────────────────── */

int g_tests_run    = 0;
int g_tests_passed = 0;

/* ── Noise generator ────────────────────────────────────────────────────────── */

float generate_gaussian_noise(float mu, float sigma)
{
    static const float epsilon = 1e-6f;
    static const float two_pi  = 2.0f * 3.14159265358979323846f;

    float u1, u2;
    do {
        u1 = (float)rand() / (float)RAND_MAX;
        u2 = (float)rand() / (float)RAND_MAX;
    } while (u1 <= epsilon);

    float z0 = sqrtf(-2.0f * logf(u1)) * cosf(two_pi * u2);
    return z0 * sigma + mu;
}

/* ── Nominal state propagator ───────────────────────────────────────────────── */

void propagate_nominal_state(ekf_core_t *ekf,
                             const float gyro_meas[3],
                             const float accel_meas[3],
                             float dt)
{
    float *q     = ekf->delayed_state.q;
    float *v_ned = ekf->delayed_state.v_ned;
    float *p_ned = ekf->delayed_state.p_ned;

    float gc[3], ac[3];
    for (int i = 0; i < 3; i++) {
        gc[i] = gyro_meas[i]  - ekf->delayed_state.gyro_bias[i];
        ac[i] = accel_meas[i] - ekf->delayed_state.accel_bias[i];
    }

    /* Small-angle quaternion update */
    float ha[3] = { 0.5f*gc[0]*dt, 0.5f*gc[1]*dt, 0.5f*gc[2]*dt };
    float q_new[4];
    q_new[0] = q[0] - q[1]*ha[0] - q[2]*ha[1] - q[3]*ha[2];
    q_new[1] = q[1] + q[0]*ha[0] + q[2]*ha[2] - q[3]*ha[1];
    q_new[2] = q[2] + q[0]*ha[1] + q[3]*ha[0] - q[1]*ha[2];
    q_new[3] = q[3] + q[0]*ha[2] + q[1]*ha[1] - q[2]*ha[0];
    float norm = sqrtf(q_new[0]*q_new[0] + q_new[1]*q_new[1]
                     + q_new[2]*q_new[2] + q_new[3]*q_new[3]);
    if (norm > 0.0f) {
        q[0] = q_new[0]/norm; q[1] = q_new[1]/norm;
        q[2] = q_new[2]/norm; q[3] = q_new[3]/norm;
    }

    /* Rotation matrix body → NED */
    float R[3][3];
    R[0][0] = 1.0f - 2.0f*(q[2]*q[2] + q[3]*q[3]);
    R[0][1] = 2.0f*(q[1]*q[2] - q[0]*q[3]);
    R[0][2] = 2.0f*(q[1]*q[3] + q[0]*q[2]);
    R[1][0] = 2.0f*(q[1]*q[2] + q[0]*q[3]);
    R[1][1] = 1.0f - 2.0f*(q[1]*q[1] + q[3]*q[3]);
    R[1][2] = 2.0f*(q[2]*q[3] - q[0]*q[1]);
    R[2][0] = 2.0f*(q[1]*q[3] - q[0]*q[2]);
    R[2][1] = 2.0f*(q[2]*q[3] + q[0]*q[1]);
    R[2][2] = 1.0f - 2.0f*(q[1]*q[1] + q[2]*q[2]);

    float acc_n[3];
    acc_n[0] = R[0][0]*ac[0] + R[0][1]*ac[1] + R[0][2]*ac[2];
    acc_n[1] = R[1][0]*ac[0] + R[1][1]*ac[1] + R[1][2]*ac[2];
    acc_n[2] = R[2][0]*ac[0] + R[2][1]*ac[1] + R[2][2]*ac[2];
    acc_n[2] += 9.80665f;  /* NED gravity (down = positive) */

    for (int i = 0; i < 3; i++) v_ned[i] += acc_n[i] * dt;
    for (int i = 0; i < 3; i++) p_ned[i] += v_ned[i] * dt;
}

/* ── Mock sensor generators ─────────────────────────────────────────────────── */

mock_baro_t generate_mock_baro(float true_z_pos)
{
    mock_baro_t b;
    b.altitude_m = -true_z_pos + generate_gaussian_noise(0.0f, 0.5f);
    return b;
}

mock_gps_t generate_mock_gps(const float true_pos[3], const float true_vel[3])
{
    mock_gps_t g;
    for (int i = 0; i < 3; i++) {
        g.pos_ned[i] = true_pos[i] + generate_gaussian_noise(0.0f, 1.5f);
        g.vel_ned[i] = true_vel[i] + generate_gaussian_noise(0.0f, 0.1f);
    }
    return g;
}

/* ── Math helpers ───────────────────────────────────────────────────────────── */

float matrix_trace(const float *P, int n)
{
    float t = 0.0f;
    for (int i = 0; i < n; ++i) t += P[i * n + i];
    return t;
}

float quat_norm(const float q[4])
{
    return sqrtf(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);
}

bool all_finite(const float *arr, int n)
{
    for (int i = 0; i < n; ++i) {
        if (!isfinite(arr[i])) return false;
    }
    return true;
}
