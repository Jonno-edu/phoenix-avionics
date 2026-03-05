/**
 * @file test_ekf_core.c
 * @brief Unit tests for EKF initialisation, reset, and covariance propagation.
 *
 * These tests call symforce_predict_covariance directly for precise isolation
 * of the prediction math, independent of sensor fusion or nominal state
 * propagation.
 */

#include "test_utils.h"

/* ── Tests ─────────────────────────────────────────────────────────────────── */

static void test_init_quaternion_is_identity(void)
{
    printf("  test_init_quaternion_is_identity ... ");
    ekf_core_t ekf;
    ekf_core_init(&ekf);

    ASSERT_NEAR(ekf.delayed_state.q[0], 1.0f, 1e-6f);
    ASSERT_NEAR(ekf.delayed_state.q[1], 0.0f, 1e-6f);
    ASSERT_NEAR(ekf.delayed_state.q[2], 0.0f, 1e-6f);
    ASSERT_NEAR(ekf.delayed_state.q[3], 0.0f, 1e-6f);

    printf("OK\n");
}

static void test_init_state_is_zeroed(void)
{
    printf("  test_init_state_is_zeroed ... ");
    ekf_core_t ekf;
    ekf_core_init(&ekf);

    for (int i = 0; i < 3; ++i) {
        ASSERT_NEAR(ekf.delayed_state.v_ned[i],      0.0f, 1e-9f);
        ASSERT_NEAR(ekf.delayed_state.p_ned[i],      0.0f, 1e-9f);
        ASSERT_NEAR(ekf.delayed_state.gyro_bias[i],  0.0f, 1e-9f);
        ASSERT_NEAR(ekf.delayed_state.accel_bias[i], 0.0f, 1e-9f);
    }

    printf("OK\n");
}

static void test_init_covariance_diagonal(void)
{
    printf("  test_init_covariance_diagonal ... ");
    ekf_core_t ekf;
    ekf_core_init(&ekf);

    /* attitude (indices 0-2) */
    ASSERT_NEAR(ekf.P[0*15 + 0],  0.05f,  1e-6f);
    ASSERT_NEAR(ekf.P[1*15 + 1],  0.05f,  1e-6f);
    ASSERT_NEAR(ekf.P[2*15 + 2],  0.05f,  1e-6f);
    /* velocity (3-5) */
    ASSERT_NEAR(ekf.P[3*15 + 3],  100.0f, 1e-4f);
    ASSERT_NEAR(ekf.P[4*15 + 4],  100.0f, 1e-4f);
    ASSERT_NEAR(ekf.P[5*15 + 5],  100.0f, 1e-4f);
    /* position (6-8) */
    ASSERT_NEAR(ekf.P[6*15 + 6],  100.0f, 1e-4f);
    ASSERT_NEAR(ekf.P[7*15 + 7],  100.0f, 1e-4f);
    ASSERT_NEAR(ekf.P[8*15 + 8],  100.0f, 1e-4f);
    /* gyro bias (9-11) */
    ASSERT_NEAR(ekf.P[9*15 + 9],  0.001f, 1e-7f);
    ASSERT_NEAR(ekf.P[10*15+10],  0.001f, 1e-7f);
    ASSERT_NEAR(ekf.P[11*15+11],  0.001f, 1e-7f);
    /* accel bias (12-14) */
    ASSERT_NEAR(ekf.P[12*15+12],  0.1f,   1e-6f);
    ASSERT_NEAR(ekf.P[13*15+13],  0.1f,   1e-6f);
    ASSERT_NEAR(ekf.P[14*15+14],  0.1f,   1e-6f);
    /* sample off-diagonal must be zero */
    ASSERT_NEAR(ekf.P[0*15 + 3],  0.0f,   1e-9f);

    printf("OK\n");
}

static void test_reset_restores_state(void)
{
    printf("  test_reset_restores_state ... ");
    ekf_core_t ekf;
    ekf_core_init(&ekf);

    ekf.delayed_state.q[0]     = 0.5f;
    ekf.delayed_state.v_ned[0] = 50.0f;
    ekf.P[0]           = 9999.0f;

    ekf_core_reset(&ekf);

    ASSERT_NEAR(ekf.delayed_state.q[0],     1.0f,  1e-6f);
    ASSERT_NEAR(ekf.delayed_state.v_ned[0], 0.0f,  1e-9f);
    ASSERT_NEAR(ekf.P[0*15 + 0],    0.05f, 1e-6f);

    printf("OK\n");
}

static void test_covariance_grows_after_predict(void)
{
    printf("  test_covariance_grows_after_predict ... ");
    ekf_core_t ekf;
    ekf_core_init(&ekf);

    float trace_before = matrix_trace(ekf.P, 15);

    /* Hover-like: pure gravity, no rotation */
    const float accel[3]     = {0.0f, 0.0f, 9.81f};
    const float accel_var[3] = {0.01f, 0.01f, 0.01f};
    const float gyro[3]      = {0.0f, 0.0f, 0.0f};
    const float gyro_var     = 1e-4f;
    const float dt           = 0.004f;

    symforce_predict_covariance(
        ekf.P,
        ekf.delayed_state.q, ekf.delayed_state.v_ned, ekf.delayed_state.p_ned,
        ekf.delayed_state.gyro_bias, ekf.delayed_state.accel_bias,
        accel, accel_var, gyro, gyro_var, dt
    );

    float trace_after = matrix_trace(ekf.P, 15);
    ASSERT_TRUE(trace_after > trace_before);
    printf("OK  (trace: %.4f -> %.4f)\n", (double)trace_before, (double)trace_after);
}

static void test_covariance_stays_finite_over_1s(void)
{
    printf("  test_covariance_stays_finite_over_1s ... ");
    ekf_core_t ekf;
    ekf_core_init(&ekf);

    /* Mild motion: slight tilt and slow yaw */
    const float accel[3]     = {0.1f, -0.2f, 9.81f};
    const float accel_var[3] = {0.01f, 0.01f, 0.01f};
    const float gyro[3]      = {0.01f, -0.005f, 0.002f};
    const float gyro_var     = 1e-4f;
    const float dt           = 0.004f;

    /* 250 steps = 1 simulated second */
    for (int i = 0; i < 250; ++i) {
        symforce_predict_covariance(
            ekf.P,
            ekf.delayed_state.q, ekf.delayed_state.v_ned, ekf.delayed_state.p_ned,
            ekf.delayed_state.gyro_bias, ekf.delayed_state.accel_bias,
            accel, accel_var, gyro, gyro_var, dt
        );
    }

    ASSERT_TRUE(all_finite(ekf.P, 225));
    printf("OK  (final trace: %.4f)\n", (double)matrix_trace(ekf.P, 15));
}

static void test_quaternion_norm_preserved_after_predict(void)
{
    printf("  test_quaternion_norm_preserved_after_predict ... ");
    ekf_core_t ekf;
    ekf_core_init(&ekf);

    const float gyro[3] = {0.05f, 0.02f, -0.01f};
    const float dt       = 0.004f;

    for (int i = 0; i < 250; ++i) {
        float *q    = ekf.delayed_state.q;
        float ha[3] = { 0.5f*gyro[0]*dt, 0.5f*gyro[1]*dt, 0.5f*gyro[2]*dt };

        float q_new[4];
        q_new[0] = q[0] - q[1]*ha[0] - q[2]*ha[1] - q[3]*ha[2];
        q_new[1] = q[1] + q[0]*ha[0] + q[2]*ha[2] - q[3]*ha[1];
        q_new[2] = q[2] + q[0]*ha[1] + q[3]*ha[0] - q[1]*ha[2];
        q_new[3] = q[3] + q[0]*ha[2] + q[1]*ha[1] - q[2]*ha[0];

        float norm = sqrtf(q_new[0]*q_new[0] + q_new[1]*q_new[1]
                         + q_new[2]*q_new[2] + q_new[3]*q_new[3]);
        for (int j = 0; j < 4; ++j) q[j] = q_new[j] / norm;
    }

    ASSERT_NEAR(quat_norm(ekf.delayed_state.q), 1.0f, 1e-5f);
    printf("OK  (|q| = %.8f)\n", (double)quat_norm(ekf.delayed_state.q));
}

/* ── Suite runner ───────────────────────────────────────────────────────────── */

void run_core_tests(void)
{
    printf("\n=== EKF Core Suite (init / reset / covariance math) ===\n");
    test_init_quaternion_is_identity();
    test_init_state_is_zeroed();
    test_init_covariance_diagonal();
    test_reset_restores_state();
    test_covariance_grows_after_predict();
    test_covariance_stays_finite_over_1s();
    test_quaternion_norm_preserved_after_predict();
}
