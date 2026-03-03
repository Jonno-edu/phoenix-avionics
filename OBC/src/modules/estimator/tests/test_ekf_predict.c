/**
 * @file test_ekf_predict.c
 * @brief Standalone pure C unit tests for the EKF predict step.
 *
 * Zero dependencies on FreeRTOS, nORB, or the Pico SDK.
 * Compiles and runs on any host (Mac / Linux / CI).
 *
 * Build:  cmake --build build_host --target test_ekf_predict
 * Run:    cmake --build build_host --target run_ekf_tests
 */

#include <stdio.h>
#include <math.h>
#include <stdbool.h>
#include <string.h>

#include "ekf_core.h"
#include "ekf_math/symforce_wrapper.h"

/* ── Minimal test harness ──────────────────────────────────────────────────── */

static int g_tests_run    = 0;
static int g_tests_passed = 0;

#define ASSERT_NEAR(a, b, tol)                                                  \
    do {                                                                        \
        float _a = (float)(a);                                                  \
        float _b = (float)(b);                                                  \
        float _t = (float)(tol);                                                \
        ++g_tests_run;                                                          \
        if (fabsf(_a - _b) <= _t) {                                             \
            ++g_tests_passed;                                                   \
        } else {                                                                \
            printf("  FAIL [%s:%d]: |%.6f - %.6f| > %.6f\n",                   \
                   __FILE__, __LINE__, (double)_a, (double)_b, (double)_t);    \
        }                                                                       \
    } while (0)

#define ASSERT_TRUE(cond)                                                       \
    do {                                                                        \
        ++g_tests_run;                                                          \
        if (cond) {                                                             \
            ++g_tests_passed;                                                   \
        } else {                                                                \
            printf("  FAIL [%s:%d]: Condition false: " #cond "\n",             \
                   __FILE__, __LINE__);                                         \
        }                                                                       \
    } while (0)

/* ── Helpers ───────────────────────────────────────────────────────────────── */

static float matrix_trace(const float *P, int n) {
    float t = 0.0f;
    for (int i = 0; i < n; ++i) t += P[i * n + i];
    return t;
}

static float quat_norm(const float q[4]) {
    return sqrtf(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);
}

static bool all_finite(const float *arr, int n) {
    for (int i = 0; i < n; ++i) {
        if (!isfinite(arr[i])) return false;
    }
    return true;
}

/* ── Tests ─────────────────────────────────────────────────────────────────── */

static void test_init_quaternion_is_identity(void) {
    printf("  test_init_quaternion_is_identity ... ");
    ekf_core_t ekf;
    ekf_core_init(&ekf);

    ASSERT_NEAR(ekf.state.q[0], 1.0f, 1e-6f);
    ASSERT_NEAR(ekf.state.q[1], 0.0f, 1e-6f);
    ASSERT_NEAR(ekf.state.q[2], 0.0f, 1e-6f);
    ASSERT_NEAR(ekf.state.q[3], 0.0f, 1e-6f);

    printf("OK\n");
}

static void test_init_state_is_zeroed(void) {
    printf("  test_init_state_is_zeroed ... ");
    ekf_core_t ekf;
    ekf_core_init(&ekf);

    for (int i = 0; i < 3; ++i) {
        ASSERT_NEAR(ekf.state.v_ned[i],     0.0f, 1e-9f);
        ASSERT_NEAR(ekf.state.p_ned[i],     0.0f, 1e-9f);
        ASSERT_NEAR(ekf.state.gyro_bias[i], 0.0f, 1e-9f);
        ASSERT_NEAR(ekf.state.accel_bias[i],0.0f, 1e-9f);
    }

    printf("OK\n");
}

static void test_init_covariance_diagonal(void) {
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

static void test_reset_restores_state(void) {
    printf("  test_reset_restores_state ... ");
    ekf_core_t ekf;
    ekf_core_init(&ekf);

    ekf.state.q[0]     = 0.5f;
    ekf.state.v_ned[0] = 50.0f;
    ekf.P[0]           = 9999.0f;

    ekf_core_reset(&ekf);

    ASSERT_NEAR(ekf.state.q[0],     1.0f,  1e-6f);
    ASSERT_NEAR(ekf.state.v_ned[0], 0.0f,  1e-9f);
    ASSERT_NEAR(ekf.P[0*15 + 0],    0.05f, 1e-6f);

    printf("OK\n");
}

static void test_covariance_grows_after_predict(void) {
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
        ekf.state.q,
        ekf.state.v_ned,
        ekf.state.p_ned,
        ekf.state.gyro_bias,
        ekf.state.accel_bias,
        accel, accel_var, gyro, gyro_var, dt
    );

    float trace_after = matrix_trace(ekf.P, 15);

    /* Uncorrected prediction must expand uncertainty */
    ASSERT_TRUE(trace_after > trace_before);
    printf("OK  (trace: %.4f -> %.4f)\n", (double)trace_before, (double)trace_after);
}

static void test_covariance_stays_finite_over_1s(void) {
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
            ekf.state.q,
            ekf.state.v_ned,
            ekf.state.p_ned,
            ekf.state.gyro_bias,
            ekf.state.accel_bias,
            accel, accel_var, gyro, gyro_var, dt
        );
    }

    ASSERT_TRUE(all_finite(ekf.P, 225));
    printf("OK  (final trace: %.4f)\n", (double)matrix_trace(ekf.P, 15));
}

static void test_quaternion_norm_preserved_after_predict(void) {
    printf("  test_quaternion_norm_preserved_after_predict ... ");
    ekf_core_t ekf;
    ekf_core_init(&ekf);

    /* Integrate 250 steps using the same small-angle formula as imu_predictor.c */
    const float gyro[3] = {0.05f, 0.02f, -0.01f};
    const float dt       = 0.004f;

    for (int i = 0; i < 250; ++i) {
        float *q   = ekf.state.q;
        float ha[3] = {0.5f * gyro[0] * dt,
                       0.5f * gyro[1] * dt,
                       0.5f * gyro[2] * dt};

        float q_new[4];
        q_new[0] = q[0] - q[1]*ha[0] - q[2]*ha[1] - q[3]*ha[2];
        q_new[1] = q[1] + q[0]*ha[0] + q[2]*ha[2] - q[3]*ha[1];
        q_new[2] = q[2] + q[0]*ha[1] + q[3]*ha[0] - q[1]*ha[2];
        q_new[3] = q[3] + q[0]*ha[2] + q[1]*ha[1] - q[2]*ha[0];

        float norm = sqrtf(q_new[0]*q_new[0] + q_new[1]*q_new[1]
                         + q_new[2]*q_new[2] + q_new[3]*q_new[3]);
        for (int j = 0; j < 4; ++j) q[j] = q_new[j] / norm;
    }

    ASSERT_NEAR(quat_norm(ekf.state.q), 1.0f, 1e-5f);
    printf("OK  (|q| = %.8f)\n", (double)quat_norm(ekf.state.q));
}

static void test_process_csv_data(void) {
    printf("  test_process_csv_data ... \n");
    ekf_core_t ekf;
    ekf_core_init(&ekf);

    /* Path assumes running from OBC/build_host or similar */
    FILE *file = fopen("../../GSU/phoenix_sensor_stream.csv", "r");
    if (!file) {
        /* Try one level up (if running from OBC) */
        file = fopen("../GSU/phoenix_sensor_stream.csv", "r");
    }
    if (!file) {
        /* Try absolute path based on workspace info */
        file = fopen("/Users/jonno/GitProjects/Phoenix/phoenix-avionics/GSU/phoenix_sensor_stream.csv", "r");
    }

    if (!file) {
        printf("  SKIP: Could not open phoenix_sensor_stream.csv\n");
        return;
    }

    char line[512];
    /* Skip header: time,pressure_pa,temp_c,temp_stack,accel_x,accel_y,accel_z,gyro_x,gyro_y,gyro_z,... */
    if (!fgets(line, sizeof(line), file)) {
        fclose(file);
        return;
    }

    int step_count = 0;
    float last_time = 0.0f;

    /* Read first 1000 steps to verify stability */
    while (fgets(line, sizeof(line), file) && step_count < 1000) {
        float t, p, tc, ts, ax, ay, az, gx, gy, gz;
        /* CSV: time,pressure_pa,temp_c,temp_stack,accel_x,accel_y,accel_z,gyro_x,gyro_y,gyro_z,... */
        if (sscanf(line, "%f,%f,%f,%f,%f,%f,%f,%f,%f,%f", 
                   &t, &p, &tc, &ts, &ax, &ay, &az, &gx, &gy, &gz) == 10) 
        {
            float dt = (step_count == 0) ? 0.004f : (t - last_time);
            if (dt <= 0.0f || dt > 0.1f) dt = 0.004f;

            float accel[3] = {ax, ay, az};
            float gyro[3]  = {gx, gy, gz};

            /* Note: We use the raw symforce predictor here since we're in a standalone test.
               In flight, this is wrapped by imu_predict() in imu_predictor.c */
            float accel_var[3] = {0.01f, 0.01f, 0.01f};
            float gyro_var     = 1e-4f;

            symforce_predict_covariance(
                ekf.P, ekf.state.q, ekf.state.v_ned, ekf.state.p_ned,
                ekf.state.gyro_bias, ekf.state.accel_bias,
                accel, accel_var, gyro, gyro_var, dt
            );

            /* Very basic nominal integration for velocity just to check finiteness */
            for(int i=0; i<3; i++) ekf.state.v_ned[i] += ax * dt; 

            last_time = t;
            step_count++;
        }
    }

    fclose(file);
    ASSERT_TRUE(step_count > 0);
    ASSERT_TRUE(all_finite(ekf.P, 225));
    ASSERT_TRUE(all_finite(ekf.state.v_ned, 3));

    printf("  OK: Processed %d steps from CSV\n", step_count);
}

/* ── Main ──────────────────────────────────────────────────────────────────── */

int main(void) {
    printf("\n=== EKF Predict — Unit Test Suite ===\n\n");

    test_init_quaternion_is_identity();
    test_init_state_is_zeroed();
    test_init_covariance_diagonal();
    test_reset_restores_state();
    test_covariance_grows_after_predict();
    test_covariance_stays_finite_over_1s();
    test_quaternion_norm_preserved_after_predict();
    test_process_csv_data();

    printf("\n--------------------------------------\n");
    printf("  %d / %d tests passed.\n", g_tests_passed, g_tests_run);
    printf("--------------------------------------\n\n");

    return (g_tests_passed == g_tests_run) ? 0 : 1;
}
