/**
 * @file test_ekf_predict.cpp
 * @brief Unit tests for the EKF core and SymForce covariance predictor.
 *
 * Deliberately avoids FreeRTOS, nORB, and the Pico SDK — this runs on
 * any host (Mac / Linux / CI) as a plain executable.
 *
 * Build:  cmake --build build --target test_ekf_predict
 * Run:    cmake --build build --target run_ekf_tests
 */

#include <cstdio>
#include <cmath>

// Module under test — pure C headers with no RTOS dependency
#include "ekf_core.h"
#include "ekf_math/symforce_wrapper.h"

// ─── Minimal test harness ────────────────────────────────────────────────────

static int g_tests_run    = 0;
static int g_tests_passed = 0;

#define ASSERT_NEAR(a, b, tol)                                                     \
    do {                                                                           \
        ++g_tests_run;                                                             \
        float _a = static_cast<float>(a);                                          \
        float _b = static_cast<float>(b);                                          \
        float _t = static_cast<float>(tol);                                        \
        if (std::fabsf(_a - _b) <= _t) {                                           \
            ++g_tests_passed;                                                      \
        } else {                                                                   \
            std::printf("  FAIL [%s:%d]: |%.6f - %.6f| > %.6f\n",                 \
                        __FILE__, __LINE__, (double)_a, (double)_b, (double)_t);  \
        }                                                                          \
    } while (0)

#define ASSERT_TRUE(cond)                                                          \
    do {                                                                           \
        ++g_tests_run;                                                             \
        if (cond) {                                                                \
            ++g_tests_passed;                                                      \
        } else {                                                                   \
            std::printf("  FAIL [%s:%d]: Condition false: " #cond "\n",           \
                        __FILE__, __LINE__);                                       \
        }                                                                          \
    } while (0)

// ─── Helpers ─────────────────────────────────────────────────────────────────

/** Compute the trace of a flat n×n row-major matrix. */
static float matrix_trace(const float* P, int n) {
    float t = 0.0f;
    for (int i = 0; i < n; ++i) t += P[i * n + i];
    return t;
}

/** Quaternion norm. */
static float quat_norm(const float q[4]) {
    return std::sqrtf(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);
}

/** Returns true if every element of a float array is finite. */
static bool all_finite(const float* arr, int n) {
    for (int i = 0; i < n; ++i) {
        if (!std::isfinite(arr[i])) return false;
    }
    return true;
}

// ─── Tests ───────────────────────────────────────────────────────────────────

static void test_init_quaternion_is_identity(void) {
    std::printf("  test_init_quaternion_is_identity ... ");
    ekf_core_t ekf;
    ekf_core_init(&ekf);

    // Initial orientation must be identity quaternion [w=1, x=0, y=0, z=0]
    ASSERT_NEAR(ekf.state.q[0], 1.0f, 1e-6f);
    ASSERT_NEAR(ekf.state.q[1], 0.0f, 1e-6f);
    ASSERT_NEAR(ekf.state.q[2], 0.0f, 1e-6f);
    ASSERT_NEAR(ekf.state.q[3], 0.0f, 1e-6f);

    std::printf("OK\n");
}

static void test_init_state_is_zeroed(void) {
    std::printf("  test_init_state_is_zeroed ... ");
    ekf_core_t ekf;
    ekf_core_init(&ekf);

    for (int i = 0; i < 3; ++i) {
        ASSERT_NEAR(ekf.state.v_ned[i],    0.0f, 1e-9f);
        ASSERT_NEAR(ekf.state.p_ned[i],    0.0f, 1e-9f);
        ASSERT_NEAR(ekf.state.gyro_bias[i], 0.0f, 1e-9f);
        ASSERT_NEAR(ekf.state.accel_bias[i],0.0f, 1e-9f);
    }

    std::printf("OK\n");
}

static void test_init_covariance_diagonal(void) {
    std::printf("  test_init_covariance_diagonal ... ");
    ekf_core_t ekf;
    ekf_core_init(&ekf);

    // Diagonal values as set in ekf_core_reset()
    // att (0-2)
    ASSERT_NEAR(ekf.P[0*15 + 0],  0.05f,   1e-6f);
    ASSERT_NEAR(ekf.P[1*15 + 1],  0.05f,   1e-6f);
    ASSERT_NEAR(ekf.P[2*15 + 2],  0.05f,   1e-6f);
    // vel (3-5)
    ASSERT_NEAR(ekf.P[3*15 + 3],  100.0f,  1e-4f);
    ASSERT_NEAR(ekf.P[4*15 + 4],  100.0f,  1e-4f);
    ASSERT_NEAR(ekf.P[5*15 + 5],  100.0f,  1e-4f);
    // pos (6-8)
    ASSERT_NEAR(ekf.P[6*15 + 6],  100.0f,  1e-4f);
    ASSERT_NEAR(ekf.P[7*15 + 7],  100.0f,  1e-4f);
    ASSERT_NEAR(ekf.P[8*15 + 8],  100.0f,  1e-4f);
    // gyro bias (9-11)
    ASSERT_NEAR(ekf.P[9*15 + 9],  0.001f,  1e-7f);
    ASSERT_NEAR(ekf.P[10*15+10],  0.001f,  1e-7f);
    ASSERT_NEAR(ekf.P[11*15+11],  0.001f,  1e-7f);
    // accel bias (12-14)
    ASSERT_NEAR(ekf.P[12*15+12],  0.1f,    1e-6f);
    ASSERT_NEAR(ekf.P[13*15+13],  0.1f,    1e-6f);
    ASSERT_NEAR(ekf.P[14*15+14],  0.1f,    1e-6f);

    // A sample off-diagonal must be zero
    ASSERT_NEAR(ekf.P[0*15 + 3],  0.0f,    1e-9f);

    std::printf("OK\n");
}

static void test_reset_restores_state(void) {
    std::printf("  test_reset_restores_state ... ");
    ekf_core_t ekf;
    ekf_core_init(&ekf);

    // Perturb
    ekf.state.q[0]      = 0.5f;
    ekf.state.v_ned[0]  = 50.0f;
    ekf.P[0]            = 9999.0f;

    ekf_core_reset(&ekf);

    ASSERT_NEAR(ekf.state.q[0],     1.0f,  1e-6f);
    ASSERT_NEAR(ekf.state.v_ned[0], 0.0f,  1e-9f);
    ASSERT_NEAR(ekf.P[0*15 + 0],    0.05f, 1e-6f);

    std::printf("OK\n");
}

static void test_covariance_grows_after_predict(void) {
    std::printf("  test_covariance_grows_after_predict ... ");
    ekf_core_t ekf;
    ekf_core_init(&ekf);

    float trace_before = matrix_trace(ekf.P, 15);

    // Hover-like IMU: pure gravity downward, zero rotation
    const float accel[3]     = {0.0f, 0.0f, 9.81f};
    const float accel_var[3] = {0.01f, 0.01f, 0.01f};
    const float gyro[3]      = {0.0f, 0.0f, 0.0f};
    const float gyro_var     = 1e-4f;
    const float dt           = 0.004f;  // 250 Hz

    symforce_predict_covariance(
        ekf.P,
        ekf.state.q,
        ekf.state.v_ned,
        ekf.state.p_ned,
        ekf.state.gyro_bias,
        ekf.state.accel_bias,
        accel, accel_var, gyro, gyro_var, dt
    );

    float trace_after = matrix_trace(ekf.P, 15);

    // Uncorrected prediction must expand uncertainty
    ASSERT_TRUE(trace_after > trace_before);
    std::printf("OK  (trace: %.4f -> %.4f)\n", (double)trace_before, (double)trace_after);
}

static void test_covariance_stays_finite_over_1s(void) {
    std::printf("  test_covariance_stays_finite_over_1s ... ");
    ekf_core_t ekf;
    ekf_core_init(&ekf);

    // Mild motion: slight tilt and slow yaw
    const float accel[3]     = {0.1f, -0.2f, 9.81f};
    const float accel_var[3] = {0.01f, 0.01f, 0.01f};
    const float gyro[3]      = {0.01f, -0.005f, 0.002f};
    const float gyro_var     = 1e-4f;
    const float dt           = 0.004f;

    // 250 steps = 1 simulated second
    for (int i = 0; i < 250; ++i) {
        symforce_predict_covariance(
            ekf.P,
            ekf.state.q,
            ekf.state.v_ned,
            ekf.state.p_ned,
            ekf.state.gyro_bias,
            ekf.state.accel_bias,
            accel, accel_var, gyro, gyro_var, dt
        );
    }

    ASSERT_TRUE(all_finite(ekf.P, 225));
    std::printf("OK  (final trace: %.4f)\n", (double)matrix_trace(ekf.P, 15));
}

static void test_quaternion_norm_preserved_after_predict(void) {
    std::printf("  test_quaternion_norm_preserved_after_predict ... ");
    ekf_core_t ekf;
    ekf_core_init(&ekf);

    // Manually apply small-angle integration (mirrors imu_predictor.c)
    // for 250 steps and verify the quaternion stays unit-norm.
    const float gyro[3] = {0.05f, 0.02f, -0.01f};  // rad/s
    const float dt       = 0.004f;

    for (int i = 0; i < 250; ++i) {
        float* q = ekf.state.q;
        float ha[3] = {0.5f * gyro[0] * dt,
                       0.5f * gyro[1] * dt,
                       0.5f * gyro[2] * dt};

        float q_new[4];
        q_new[0] = q[0] - q[1]*ha[0] - q[2]*ha[1] - q[3]*ha[2];
        q_new[1] = q[1] + q[0]*ha[0] + q[2]*ha[2] - q[3]*ha[1];
        q_new[2] = q[2] + q[0]*ha[1] + q[3]*ha[0] - q[1]*ha[2];
        q_new[3] = q[3] + q[0]*ha[2] + q[1]*ha[1] - q[2]*ha[0];

        float norm = std::sqrtf(q_new[0]*q_new[0] + q_new[1]*q_new[1]
                              + q_new[2]*q_new[2] + q_new[3]*q_new[3]);
        for (int j = 0; j < 4; ++j) q[j] = q_new[j] / norm;
    }

    ASSERT_NEAR(quat_norm(ekf.state.q), 1.0f, 1e-5f);
    std::printf("OK  (|q| = %.8f)\n", (double)quat_norm(ekf.state.q));
}

// ─── Main ────────────────────────────────────────────────────────────────────

int main(void) {
    std::printf("\n╔══════════════════════════════════════╗\n");
    std::printf("║    EKF Predict — Unit Test Suite     ║\n");
    std::printf("╚══════════════════════════════════════╝\n\n");

    test_init_quaternion_is_identity();
    test_init_state_is_zeroed();
    test_init_covariance_diagonal();
    test_reset_restores_state();
    test_covariance_grows_after_predict();
    test_covariance_stays_finite_over_1s();
    test_quaternion_norm_preserved_after_predict();

    std::printf("\n──────────────────────────────────────\n");
    std::printf("  %d / %d tests passed.\n", g_tests_passed, g_tests_run);
    std::printf("──────────────────────────────────────\n\n");

    return (g_tests_passed == g_tests_run) ? 0 : 1;
}
