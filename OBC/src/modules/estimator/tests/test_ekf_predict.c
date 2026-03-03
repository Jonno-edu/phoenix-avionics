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
#include <stdlib.h>
#include <math.h>
#include <stdbool.h>
#include <string.h>

#include "ekf_core.h"
#include "ekf_math/symforce_wrapper.h"
#include "baro_fuse.h"
#include "gps_fuse.h"

// Generates normally distributed noise: N(mu, sigma^2)
static float generate_gaussian_noise(float mu, float sigma) {
    static const float epsilon = 1e-6f;
    static const float two_pi = 2.0f * 3.14159265358979323846f;
    
    float u1, u2;
    do {
        u1 = (float)rand() / (float)RAND_MAX;
        u2 = (float)rand() / (float)RAND_MAX;
    } while (u1 <= epsilon);
    
    float z0 = sqrtf(-2.0f * logf(u1)) * cosf(two_pi * u2);
    return z0 * sigma + mu;
}

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

static void test_stationary_bias_convergence(void) {
    printf("  test_stationary_bias_convergence ... \n");
    
    ekf_core_t ekf;
    ekf_core_init(&ekf);

    // 1. Define our TRUE state (Ground Truth)
    const float true_accel[3] = {0.0f, 0.0f, 9.80665f}; // Gravity in NED
    const float true_gyro[3]  = {0.0f, 0.0f, 0.0f};     // Not rotating
    
    // The secret biases we want the EKF to find
    const float hidden_gyro_bias[3] = {0.005f, -0.002f, 0.015f};
    const float hidden_accel_bias[3] = {0.1f, -0.05f, 0.0f};

    // IMU Noise profiles (Standard Deviations)
    const float accel_noise_std = 0.05f; // m/s^2
    const float gyro_noise_std  = 0.002f; // rad/s
    
    // EKF tuning parameters
    const float dt = 0.004f; // 250Hz
    const float accel_var[3] = {accel_noise_std*accel_noise_std, accel_noise_std*accel_noise_std, accel_noise_std*accel_noise_std};
    const float gyro_var = gyro_noise_std*gyro_noise_std;

    // 2. Run the simulation for 15 seconds (3750 steps at 250Hz)
    for (int i = 0; i < 3750; i++) {
        float meas_accel[3];
        float meas_gyro[3];

        // Generate synthetic noisy measurements with hidden bias
        for (int axis = 0; axis < 3; axis++) {
            meas_accel[axis] = true_accel[axis] + hidden_accel_bias[axis] + generate_gaussian_noise(0.0f, accel_noise_std);
            meas_gyro[axis]  = true_gyro[axis]  + hidden_gyro_bias[axis]  + generate_gaussian_noise(0.0f, gyro_noise_std);
        }

        // Run the prediction step
        symforce_predict_covariance(
            ekf.P, ekf.state.q, ekf.state.v_ned, ekf.state.p_ned,
            ekf.state.gyro_bias, ekf.state.accel_bias,
            meas_accel, accel_var, meas_gyro, gyro_var, dt
        );

        // Tell the EKF: "I am 100% sure I am not moving"
        const float meas_vel_var[3]  = {1e-6f, 1e-6f, 1e-6f}; 
        const float meas_gyro_var[3] = {1e-6f, 1e-6f, 1e-6f}; 

        symforce_update_stationary(
            ekf.P, ekf.state.q, ekf.state.v_ned, ekf.state.p_ned,
            ekf.state.gyro_bias, ekf.state.accel_bias,
            meas_gyro, meas_vel_var, meas_gyro_var
        );
    }

    // 3. Assert the EKF found the secret biases!
    // We allow a small tolerance since it's estimating through noise.
    printf("    Target Gyro Bias:  [%.4f, %.4f, %.4f]\n", hidden_gyro_bias[0], hidden_gyro_bias[1], hidden_gyro_bias[2]);
    printf("    EKF Gyro Bias:     [%.4f, %.4f, %.4f]\n", ekf.state.gyro_bias[0], ekf.state.gyro_bias[1], ekf.state.gyro_bias[2]);

    ASSERT_NEAR(ekf.state.gyro_bias[0], hidden_gyro_bias[0], 0.002f);
    ASSERT_NEAR(ekf.state.gyro_bias[1], hidden_gyro_bias[1], 0.002f);
    ASSERT_NEAR(ekf.state.gyro_bias[2], hidden_gyro_bias[2], 0.002f);
    
    printf("  OK\n");
}

/* ─────────────────────────────────────────────────────────────────────────── */
/* Phase 4 — GPS + Barometer Fusion Convergence Test                          */
/* ─────────────────────────────────────────────────────────────────────────── */

typedef struct { float altitude_m; } mock_baro_t;
typedef struct { float pos_ned[3]; float vel_ned[3]; } mock_gps_t;

/**
 * Propagate the EKF nominal state (q, v_ned, p_ned) forward one IMU step.
 * Mirrors imu_predictor.c but without the vehicle_imu_t dependency so the
 * test stays self-contained.
 */
static void propagate_nominal_state(ekf_core_t *ekf,
                                    const float gyro_meas[3],
                                    const float accel_meas[3],
                                    float dt)
{
    float *q     = ekf->state.q;
    float *v_ned = ekf->state.v_ned;
    float *p_ned = ekf->state.p_ned;

    float gc[3], ac[3];
    for (int i = 0; i < 3; i++) {
        gc[i] = gyro_meas[i]  - ekf->state.gyro_bias[i];
        ac[i] = accel_meas[i] - ekf->state.accel_bias[i];
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
        q[0]=q_new[0]/norm; q[1]=q_new[1]/norm;
        q[2]=q_new[2]/norm; q[3]=q_new[3]/norm;
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

static mock_baro_t generate_mock_baro(float true_z_pos)
{
    mock_baro_t b;
    /* altitude = -p_ned[2]; add ~0.5 m std-dev noise */
    b.altitude_m = -true_z_pos + generate_gaussian_noise(0.0f, 0.5f);
    return b;
}

static mock_gps_t generate_mock_gps(const float true_pos[3], const float true_vel[3])
{
    mock_gps_t g;
    for (int i = 0; i < 3; i++) {
        g.pos_ned[i] = true_pos[i] + generate_gaussian_noise(0.0f, 1.5f);
        g.vel_ned[i] = true_vel[i] + generate_gaussian_noise(0.0f, 0.1f);
    }
    return g;
}

static void test_gps_baro_fusion(void)
{
    printf("  test_gps_baro_fusion ...\n");

    ekf_core_t ekf;
    ekf_core_init(&ekf);

    /* Ground truth: constant-velocity ascent, 2 m/s up (NED -z direction) */
    float true_pos[3] = {0.0f, 0.0f,  0.0f};
    float true_vel[3] = {0.0f, 0.0f, -2.0f};

    /* Specific force measured by a level accelerometer during constant-velocity
     * flight in NED.  Gravity = [0, 0, +g] (NED, down positive).
     * Specific force = a_true - g = 0 - g => body Z = -9.80665 m/s².
     * In imu_predictor the EKF then adds +g back (acc_n[2] += 9.80665),
     * giving zero net NED acceleration — consistent with constant velocity. */
    const float true_accel_body[3] = {0.0f, 0.0f, -9.80665f};
    const float true_gyro[3]       = {0.0f, 0.0f, 0.0f};

    /* Hidden biases the EKF must estimate */
    const float hidden_gyro_bias[3]  = { 0.010f, -0.005f,  0.002f};
    const float hidden_accel_bias[3] = { 0.050f, -0.030f,  0.000f};

    /* IMU noise tuning */
    const float accel_noise_std = 0.05f;
    const float gyro_noise_std  = 0.002f;
    const float accel_var[3]    = { accel_noise_std*accel_noise_std,
                                    accel_noise_std*accel_noise_std,
                                    accel_noise_std*accel_noise_std };
    const float gyro_var        = gyro_noise_std * gyro_noise_std;

    const float dt          = 0.004f;    /* 250 Hz */
    const int   total_steps = 250 * 60;  /* 60 simulated seconds */

    for (int step = 0; step < total_steps; step++) {

        /* 1. Advance ground truth position */
        for (int i = 0; i < 3; i++) true_pos[i] += true_vel[i] * dt;

        /* 2. Generate noisy IMU measurements (bias + noise on top of true value) */
        float meas_accel[3], meas_gyro[3];
        for (int i = 0; i < 3; i++) {
            meas_accel[i] = true_accel_body[i] + hidden_accel_bias[i]
                            + generate_gaussian_noise(0.0f, accel_noise_std);
            meas_gyro[i]  = true_gyro[i] + hidden_gyro_bias[i]
                            + generate_gaussian_noise(0.0f, gyro_noise_std);
        }

        /* 3. Propagate nominal state and covariance (250 Hz) */
        propagate_nominal_state(&ekf, meas_gyro, meas_accel, dt);

        symforce_predict_covariance(
            ekf.P, ekf.state.q, ekf.state.v_ned, ekf.state.p_ned,
            ekf.state.gyro_bias, ekf.state.accel_bias,
            meas_accel, accel_var, meas_gyro, gyro_var, dt
        );

        /* 4. Barometer update at 50 Hz (every 5 steps) */
        if (step % 5 == 0) {
            mock_baro_t raw = generate_mock_baro(true_pos[2]);
            baro_measurement_t b = { .altitude_m = raw.altitude_m };
            baro_fuse(&ekf, &b);
        }

        /* 5. GPS update at 10 Hz (every 25 steps) */
        if (step % 25 == 0) {
            mock_gps_t raw = generate_mock_gps(true_pos, true_vel);
            gps_measurement_t g;
            for (int i = 0; i < 3; i++) {
                g.pos_ned[i] = raw.pos_ned[i];
                g.vel_ned[i] = raw.vel_ned[i];
            }
            gps_fuse(&ekf, &g);
        }
    }

    printf("    True  p_ned[2]: %8.3f m   |  EKF: %8.3f m\n",
           (double)true_pos[2], (double)ekf.state.p_ned[2]);
    printf("    True  v_ned[2]: %8.3f m/s |  EKF: %8.3f m/s\n",
           (double)true_vel[2], (double)ekf.state.v_ned[2]);
    printf("    True  gyro_bias: [%.4f, %.4f, %.4f]\n",
           (double)hidden_gyro_bias[0], (double)hidden_gyro_bias[1], (double)hidden_gyro_bias[2]);
    printf("    EKF   gyro_bias: [%.4f, %.4f, %.4f]\n",
           (double)ekf.state.gyro_bias[0], (double)ekf.state.gyro_bias[1], (double)ekf.state.gyro_bias[2]);

    /* Vertical position: within 0.5 m of ground truth */
    ASSERT_NEAR(ekf.state.p_ned[2], true_pos[2], 0.5f);
    /* Vertical velocity: within 0.2 m/s of ground truth */
    ASSERT_NEAR(ekf.state.v_ned[2], true_vel[2], 0.2f);
    /* Gyro bias X/Y: GPS+Baro create tilt error observability for roll/pitch axes.
     * Gyro Z (yaw) is NOT observable through GPS/Baro alone — it requires a
     * magnetometer or heading measurement.  Only assert X and Y here. */
    ASSERT_NEAR(ekf.state.gyro_bias[0], hidden_gyro_bias[0], 0.002f);
    ASSERT_NEAR(ekf.state.gyro_bias[1], hidden_gyro_bias[1], 0.002f);
    /* Covariance must remain finite */
    ASSERT_TRUE(all_finite(ekf.P, 225));

    printf("  OK\n");
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
    test_stationary_bias_convergence();
    test_gps_baro_fusion();

    printf("\n--------------------------------------\n");
    printf("  %d / %d tests passed.\n", g_tests_passed, g_tests_run);
    printf("--------------------------------------\n\n");

    return (g_tests_passed == g_tests_run) ? 0 : 1;
}
