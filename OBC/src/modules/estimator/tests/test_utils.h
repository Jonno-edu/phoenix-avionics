/**
 * @file test_utils.h
 * @brief Shared assertions, mock generators, and math helpers for the EKF test
 *        suite.  Included by every test_ekf_*.c file.
 *
 * Zero dependencies on FreeRTOS, nORB runtime, or the Pico SDK.
 */

#ifndef TEST_UTILS_H
#define TEST_UTILS_H

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <stdbool.h>
#include <string.h>

#include "ekf_core.h"
#include "ekf_math/symforce_wrapper.h"
#include "baro_fuse.h"
#include "gps_fuse.h"
#include "mag_fuse.h"
#include "imu_predictor.h"
#include "ekf_state.h"

/* ── Global test counters (defined in test_utils.c) ────────────────────────── */

extern int g_tests_run;
extern int g_tests_passed;

/* ── Assertion macros ───────────────────────────────────────────────────────── */

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

/* ── Mock measurement types ─────────────────────────────────────────────────── */

typedef struct { float altitude_m; } mock_baro_t;
typedef struct { float pos_ned[3]; float vel_ned[3]; } mock_gps_t;

/* ── Shared math helpers (defined in test_utils.c) ──────────────────────────── */

float generate_gaussian_noise(float mu, float sigma);

/**
 * Propagate the EKF nominal state (q, v_ned, p_ned) forward one IMU step.
 * Mirrors imu_predictor.c but without the vehicle_imu_t dependency so core
 * and fusion tests remain self-contained.
 */
void propagate_nominal_state(ekf_core_t *ekf,
                             const float gyro_meas[3],
                             const float accel_meas[3],
                             float dt);

mock_baro_t generate_mock_baro(float true_z_pos);
mock_gps_t  generate_mock_gps(const float true_pos[3], const float true_vel[3]);

float matrix_trace(const float *P, int n);
float quat_norm(const float q[4]);
bool  all_finite(const float *arr, int n);

/* ── Stellenbosch reference magnetic field (launch site: ~32°S, 19°E) ─────── */
/* Values from WMM-2025 for Stellenbosch, elevation 100m, epoch 2026.2.        */
#define MAG_REF_STELLENBOSCH_N   0.1484f   /* Gauss — North component */
#define MAG_REF_STELLENBOSCH_E  -0.0509f   /* Gauss — East component  */
#define MAG_REF_STELLENBOSCH_D  -0.3289f   /* Gauss — Down component  */

/* ── Suite entry points (each defined in their respective test_ekf_*.c) ─────── */

void run_core_tests(void);
void run_fusion_tests(void);
void run_dynamic_tests(void);
void run_fault_tests(void);
void run_state_tests(void);
void run_mission_tests(void);

#endif /* TEST_UTILS_H */
