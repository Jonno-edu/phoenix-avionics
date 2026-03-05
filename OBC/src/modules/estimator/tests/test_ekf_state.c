/**
 * @file test_ekf_state.c
 * @brief Unit tests for the 5-stage EKF warmup state machine.
 *
 * Each test drives ekf_state_update() with deterministic synthetic sensor data
 * and verifies the correct mode transition fires at the expected time.
 *
 * These tests do NOT test the filter math in detail (that's test_ekf_fusion.c).
 * They test the *state machine logic*: does the right mode activate at the
 * right time given the right sensor stimuli?
 */

#include "test_utils.h"
#include "ekf_state.h"
#include "fusion/imu_predictor.h"

/* Stellenbosch WMM reference field (NED, Gauss) */
static const float STELLENBOSCH_MAG_NED[3] = { 0.15f, -0.05f, -0.33f };

/* ── Helpers ─────────────────────────────────────────────────────────────────── */

/**
 * Drive the state machine with a static body-level IMU (gravity on Z axis).
 * Feeds accel = [0, 0, g] and gyro = [0, 0, 0] (both noise-free for determinism).
 * No mag fed (mag_valid = false).
 * Returns number of steps executed.
 */
static int drive_static_imu_no_mag(ekf_state_ctx_t *ctx,
                                   ekf_core_t      *ekf,
                                   float            dt,
                                   int              steps)
{
    const float accel[3] = { 0.0f, 0.0f, -9.80665f };
    const float gyro[3]  = { 0.0f, 0.0f, 0.0f };
    const float mag[3]   = { 0.0f, 0.0f, 0.0f };

    imu_history_t imu = {0};
    imu.dt_s = dt;

    for (int i = 0; i < steps; i++) {
        for (int axis = 0; axis < 3; axis++) {
            imu.delta_angle[axis]    = gyro[axis]  * dt;
            imu.delta_velocity[axis] = accel[axis] * dt;
        }
        imu_predict(ekf, &imu);
        ekf_state_update(ctx, ekf, accel, gyro, mag, false, dt);
    }
    return steps;
}

/**
 * Drive state machine sending a static mag sample every step.
 */
static void drive_with_mag(ekf_state_ctx_t *ctx,
                            ekf_core_t      *ekf,
                            float            dt,
                            int              steps)
{
    const float accel[3] = { 0.0f, 0.0f, 9.80665f };
    const float gyro[3]  = { 0.0f, 0.0f, 0.0f };
    /* Simulate a near-ideal static mag reading — same as the reference field
     * since q is near-identity after leveling (roll≈0, pitch≈0, yaw=0). */
    const float mag_body[3] = {
        STELLENBOSCH_MAG_NED[0],
        STELLENBOSCH_MAG_NED[1],
        STELLENBOSCH_MAG_NED[2]
    };

    imu_history_t imu = {0};
    imu.dt_s = dt;

    for (int i = 0; i < steps; i++) {
        for (int axis = 0; axis < 3; axis++) {
            imu.delta_angle[axis]    = gyro[axis]  * dt;
            imu.delta_velocity[axis] = accel[axis] * dt;
        }
        imu_predict(ekf, &imu);
        ekf_state_update(ctx, ekf, accel, gyro, mag_body, true, dt);
    }
}

/* ── Tests ─────────────────────────────────────────────────────────────────── */

/**
 * After init, the first IMU sample must kick the machine from
 * UNINITIALIZED → LEVELING.
 */
static void test_state_init_to_leveling(void)
{
    printf("  test_state_init_to_leveling ...\n");

    ekf_core_t  ekf;  ekf_core_init(&ekf);
    ekf_state_ctx_t ctx;
    ekf_state_init(&ctx, STELLENBOSCH_MAG_NED, 0.004f);

    ASSERT_TRUE(ekf_state_get_mode(&ctx) == EKF_MODE_UNINITIALIZED);

    /* Feed one IMU step */
    drive_static_imu_no_mag(&ctx, &ekf, 0.004f, 1);

    ASSERT_TRUE(ekf_state_get_mode(&ctx) == EKF_MODE_LEVELING);

    printf("  OK\n");
}

/**
 * After the required leveling duration (LEVELING_DURATION_S = 2 s at 250 Hz),
 * the machine must transition LEVELING → HEADING_ALIGN.
 *
 * Also verifies that the attitude pitch/roll are set from gravity
 * (since accel = [0,0,g], pitch = 0 and roll = 0, q must be identity).
 */
static void test_leveling_to_heading_align(void)
{
    printf("  test_leveling_to_heading_align ...\n");

    const float dt   = 0.004f;         /* 250 Hz */
    const int   need = (int)(LEVELING_DURATION_S / dt + 0.5f);

    ekf_core_t  ekf;  ekf_core_init(&ekf);
    ekf_state_ctx_t ctx;
    ekf_state_init(&ctx, STELLENBOSCH_MAG_NED, dt);

    /* Feed one step fewer than needed — must still be LEVELING */
    drive_static_imu_no_mag(&ctx, &ekf, dt, need - 1);
    ASSERT_TRUE(ekf_state_get_mode(&ctx) == EKF_MODE_LEVELING);

    /* Feed the final step that crosses the threshold */
    drive_static_imu_no_mag(&ctx, &ekf, dt, 1);
    ASSERT_TRUE(ekf_state_get_mode(&ctx) == EKF_MODE_HEADING_ALIGN);

    /* Attitude should be near-identity with level input (q[0] ≈ 1.0) */
    ASSERT_NEAR(ekf.delayed_state.q[0], 1.0f, 0.01f);
    ASSERT_NEAR(ekf.delayed_state.q[1], 0.0f, 0.01f);
    ASSERT_NEAR(ekf.delayed_state.q[2], 0.0f, 0.01f);
    ASSERT_NEAR(ekf.delayed_state.q[3], 0.0f, 0.01f);

    printf("  OK\n");
}

/**
 * The HEADING_ALIGN → ZVU_CALIBRATING transition fires on the first valid
 * magnetometer sample. With no mag, the machine stays in HEADING_ALIGN.
 */
static void test_heading_align_waits_for_mag(void)
{
    printf("  test_heading_align_waits_for_mag ...\n");

    const float dt   = 0.004f;
    const int   need = (int)(LEVELING_DURATION_S / dt + 0.5f);

    ekf_core_t  ekf;  ekf_core_init(&ekf);
    ekf_state_ctx_t ctx;
    ekf_state_init(&ctx, STELLENBOSCH_MAG_NED, dt);

    /* Clear leveling phase */
    drive_static_imu_no_mag(&ctx, &ekf, dt, need + 1);
    ASSERT_TRUE(ekf_state_get_mode(&ctx) == EKF_MODE_HEADING_ALIGN);

    /* 50 more steps with no mag — should stay in HEADING_ALIGN */
    drive_static_imu_no_mag(&ctx, &ekf, dt, 50);
    ASSERT_TRUE(ekf_state_get_mode(&ctx) == EKF_MODE_HEADING_ALIGN);

    /* Now send one valid mag sample — must transition immediately */
    drive_with_mag(&ctx, &ekf, dt, 1);
    ASSERT_TRUE(ekf_state_get_mode(&ctx) == EKF_MODE_ZVU_CALIBRATING);

    printf("  OK\n");
}

/**
 * Once in ZVU_CALIBRATING, a 2g+ acceleration burst must trigger FLIGHT mode.
 * Uses a single high-accel IMU pulse.
 */
static void test_liftoff_detection(void)
{
    printf("  test_liftoff_detection ...\n");

    const float dt   = 0.004f;
    const int   need = (int)(LEVELING_DURATION_S / dt + 0.5f);

    ekf_core_t  ekf;  ekf_core_init(&ekf);
    ekf_state_ctx_t ctx;
    ekf_state_init(&ctx, STELLENBOSCH_MAG_NED, dt);

    /* Run through leveling + heading align */
    drive_static_imu_no_mag(&ctx, &ekf, dt, need + 1);
    drive_with_mag(&ctx, &ekf, dt, 1);
    ASSERT_TRUE(ekf_state_get_mode(&ctx) == EKF_MODE_ZVU_CALIBRATING);

    /* Settle for 1 second */
    drive_static_imu_no_mag(&ctx, &ekf, dt, 250);
    ASSERT_TRUE(ekf_state_get_mode(&ctx) == EKF_MODE_ZVU_CALIBRATING);

    /* Fire a liftoff pulse: ~4g net accel_meas (> 2g threshold = 19.61 m/s²) */
    const float launch_accel[3] = { 0.0f, 0.0f, 39.2266f };  /* 4g */
    const float gyro[3]         = { 0.0f, 0.0f, 0.0f };
    const float mag_body[3]     = { 0.0f, 0.0f, 0.0f };

    imu_history_t imu = {0};
    imu.dt_s = dt;
    for (int axis = 0; axis < 3; axis++) {
        imu.delta_angle[axis]    = gyro[axis]         * dt;
        imu.delta_velocity[axis] = launch_accel[axis] * dt;
    }
    imu_predict(&ekf, &imu);
    ekf_state_update(&ctx, &ekf, launch_accel, gyro, mag_body, false, dt);

    ASSERT_TRUE(ekf_state_get_mode(&ctx) == EKF_MODE_FLIGHT);
    ASSERT_TRUE(ekf.flight_mode          == EKF_MODE_FLIGHT);

    printf("  OK\n");
}

/**
 * ekf_state_is_flight_ready() must return false during LEVELING and HEADING_ALIGN,
 * and true once in FLIGHT mode.
 */
static void test_flight_ready_gate(void)
{
    printf("  test_flight_ready_gate ...\n");

    const float dt   = 0.004f;
    const int   need = (int)(LEVELING_DURATION_S / dt + 0.5f);

    ekf_core_t  ekf;  ekf_core_init(&ekf);
    ekf_state_ctx_t ctx;
    ekf_state_init(&ctx, STELLENBOSCH_MAG_NED, dt);

    ASSERT_TRUE(!ekf_state_is_flight_ready(&ctx));

    drive_static_imu_no_mag(&ctx, &ekf, dt, need + 1);
    ASSERT_TRUE(!ekf_state_is_flight_ready(&ctx));  /* Still HEADING_ALIGN */

    drive_with_mag(&ctx, &ekf, dt, 1);
    /* ZVU_CALIBRATING but biases not yet converged: still false */
    ASSERT_TRUE(!ekf_state_is_flight_ready(&ctx));

    /* Trigger liftoff → FLIGHT */
    const float launch_accel[3] = { 0.0f, 0.0f, 39.2266f };
    const float gyro[3]         = { 0.0f, 0.0f, 0.0f };
    const float mag_body[3]     = { 0.0f, 0.0f, 0.0f };
    imu_history_t imu = {0};
    imu.dt_s = dt;
    for (int axis = 0; axis < 3; axis++) {
        imu.delta_angle[axis]    = gyro[axis]         * dt;
        imu.delta_velocity[axis] = launch_accel[axis] * dt;
    }
    imu_predict(&ekf, &imu);
    ekf_state_update(&ctx, &ekf, launch_accel, gyro, mag_body, false, dt);

    ASSERT_TRUE(ekf_state_is_flight_ready(&ctx));   /* FLIGHT → must be true */

    printf("  OK\n");
}

/**
 * The EKF's ekf->flight_mode field must always mirror the context mode.
 * Verifies synchronisation across every transition in a full sequence.
 */
static void test_mode_mirrors_ekf_core(void)
{
    printf("  test_mode_mirrors_ekf_core ...\n");

    const float dt   = 0.004f;
    const int   need = (int)(LEVELING_DURATION_S / dt + 0.5f);

    ekf_core_t  ekf;  ekf_core_init(&ekf);
    ekf_state_ctx_t ctx;
    ekf_state_init(&ctx, STELLENBOSCH_MAG_NED, dt);

    /* UNINITIALIZED: ekf->flight_mode not set by init */
    ASSERT_TRUE(ekf_state_get_mode(&ctx) == EKF_MODE_UNINITIALIZED);

    /* → LEVELING */
    drive_static_imu_no_mag(&ctx, &ekf, dt, 1);
    ASSERT_TRUE(ekf.flight_mode == EKF_MODE_LEVELING);

    /* → HEADING_ALIGN */
    drive_static_imu_no_mag(&ctx, &ekf, dt, need);
    ASSERT_TRUE(ekf.flight_mode == EKF_MODE_HEADING_ALIGN);

    /* → ZVU_CALIBRATING */
    drive_with_mag(&ctx, &ekf, dt, 1);
    ASSERT_TRUE(ekf.flight_mode == EKF_MODE_ZVU_CALIBRATING);

    printf("  OK\n");
}

/* ── Suite entry point ──────────────────────────────────────────────────────── */

void run_state_tests(void)
{
    printf("\n=== EKF State Machine Suite ===\n");
    test_state_init_to_leveling();
    test_leveling_to_heading_align();
    test_heading_align_waits_for_mag();
    test_liftoff_detection();
    test_flight_ready_gate();
    test_mode_mirrors_ekf_core();
}
