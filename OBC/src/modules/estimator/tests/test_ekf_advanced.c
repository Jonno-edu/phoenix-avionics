/**
 * @file test_ekf_advanced.c
 * @brief Advanced EKF tests: latency, jitter, unmodeled forces.
 *
 * These tests validate robustness under real-world conditions:
 * - GPS latency exposes the need for observation buffers
 * - IMU jitter/drops test dynamic dt handling
 * - Crosswind drag proves accelerometer tracks unmodeled dynamics
 */

#include "test_utils.h"
#include "ekf_state.h"
#include "fusion/imu_predictor.h"
#include "fusion/baro_fuse.h"
#include "fusion/gps_fuse.h"

#define G_MS2 9.80665f

/* ── Test 1: GPS Latency Degradation ─────────────────────────────────────────
 * Simulates a 200ms delay in GPS delivery. Currently, this will fuse old data 
 * against the current state. This test establishes a baseline of the resulting 
 * error, proving the need for a delayed-fusion observation buffer.
 */
static void test_gps_latency(void)
{
    printf("  test_gps_latency (exposing the need for an observation buffer)...\n");

    ekf_core_t ekf;
    ekf_core_init(&ekf);

    float true_pos[3] = {0.0f, 0.0f,  0.0f};
    float true_vel[3] = {10.0f, 0.0f, 0.0f}; /* Moving 10 m/s North */

    const float dt = 0.004f;
    const int steps = 250 * 10; /* 10 seconds */
    const int delay_steps = 50; /* 200ms latency = 50 IMU samples */

    /* Ring buffers to hold historical truth data for delayed GPS delivery */
    float pos_history[256][3] = {0};
    float vel_history[256][3] = {0};

    for (int step = 0; step < steps; step++) {
        for (int i = 0; i < 3; i++) {
            true_pos[i] += true_vel[i] * dt;
            pos_history[step % 256][i] = true_pos[i];
            vel_history[step % 256][i] = true_vel[i];
        }

        imu_history_t imu = { .dt_s = dt };
        imu.delta_velocity[2] = -G_MS2 * dt; /* Gravity */
        imu_predict(&ekf, &imu);

        /* Deliver GPS at 10Hz, but pull the data from 200ms ago */
        if (step >= delay_steps && step % 25 == 0) {
            int delayed_idx = (step - delay_steps) % 256;
            gps_measurement_t g;
            for (int i = 0; i < 3; i++) {
                g.pos_ned[i] = pos_history[delayed_idx][i] + generate_gaussian_noise(0.0f, 1.0f);
                g.vel_ned[i] = vel_history[delayed_idx][i] + generate_gaussian_noise(0.0f, 0.1f);
            }
            gps_fuse(&ekf, &g);
        }
    }

    printf("    True Pos North: %.2f m | EKF Pos North: %.2f m\n", 
           (double)true_pos[0], (double)ekf.delayed_state.p_ned[0]);
    
    /* With 200ms latency at 10m/s, we expect ~2 meters of systemic lag. 
     * Once the delay buffer is implemented, this tolerance can be tightened. */
    ASSERT_NEAR(ekf.delayed_state.p_ned[0], true_pos[0], 2.5f);
    printf("  OK\n");
}

/* ── Test 2: IMU Jitter and Dropped Samples ──────────────────────────────────
 * Simulates FreeRTOS scheduling jitter and SPI bus drops.
 * The EKF must dynamically handle varying dt and accumulated delta angles/vels.
 */
static void test_imu_jitter_and_drops(void)
{
    printf("  test_imu_jitter_and_drops...\n");

    ekf_core_t ekf;
    ekf_core_init(&ekf);

    float true_pos[3] = {0.0f, 0.0f, 0.0f};
    float true_vel[3] = {0.0f, 0.0f, -5.0f}; /* Moving Up */

    float accum_dt = 0.0f;
    
    for (int step = 0; step < 250 * 5; step++) {
        /* Base dt is 4ms, add jitter between -1ms and +1ms */
        float jitter = ((float)rand() / RAND_MAX - 0.5f) * 0.002f;
        float current_dt = 0.004f + jitter;
        
        accum_dt += current_dt;
        for (int i = 0; i < 3; i++) true_pos[i] += true_vel[i] * current_dt;

        /* Simulate a 5% chance of dropping an IMU sample (SPI glitch) */
        bool drop_sample = ((float)rand() / RAND_MAX) < 0.05f;

        if (!drop_sample) {
            imu_history_t imu = { .dt_s = accum_dt };
            imu.delta_velocity[2] = (-G_MS2 - 5.0f) * accum_dt; /* 5m/s^2 specific force + gravity */
            
            imu_predict(&ekf, &imu);
            accum_dt = 0.0f; /* Reset accumulator after successful read */
        }
    }

    /* Verify stability despite the chaotic timing */
    ASSERT_TRUE(all_finite(ekf.P, 225));
    ASSERT_NEAR(quat_norm(ekf.delayed_state.q), 1.0f, 0.001f);
    printf("  OK\n");
}

/* ── Test 3: Unmodeled Crosswind / Drag ──────────────────────────────────────
 * Simulates a constant lateral force (wind shear) pushing the vehicle East.
 * Proves the accelerometer correctly tracks unmodeled dynamic forces.
 */
static void test_crosswind_drag(void)
{
    printf("  test_crosswind_drag...\n");

    ekf_core_t ekf;
    ekf_core_init(&ekf);

    float true_pos[3] = {0.0f, 0.0f,  0.0f};
    float true_vel[3] = {0.0f, 0.0f, -50.0f}; /* Coasting upwards */

    const float dt = 0.004f;
    const float wind_accel_east = 1.5f; /* 1.5 m/s^2 lateral push */

    for (int step = 0; step < 250 * 10; step++) {
        /* Physics integration */
        true_vel[1] += wind_accel_east * dt; /* East acceleration */
        true_vel[2] += G_MS2 * dt;           /* Gravity decelerating the climb */
        
        for (int i = 0; i < 3; i++) true_pos[i] += true_vel[i] * dt;

        /* Accelerometer measures the wind drag (specific force) and gravity */
        imu_history_t imu = { .dt_s = dt };
        imu.delta_velocity[1] = wind_accel_east * dt;
        imu.delta_velocity[2] = 0.0f; /* Freefall in Z, accelerometer reads 0 */

        imu_predict(&ekf, &imu);

        /* Occasional GPS to bind integration drift */
        if (step % 25 == 0) {
            gps_measurement_t g;
            for (int i = 0; i < 3; i++) {
                g.pos_ned[i] = true_pos[i] + generate_gaussian_noise(0.0f, 1.0f);
                g.vel_ned[i] = true_vel[i] + generate_gaussian_noise(0.0f, 0.1f);
            }
            gps_fuse(&ekf, &g);
        }
    }

    printf("    True East Pos: %.2f m | EKF East Pos: %.2f m\n", 
           (double)true_pos[1], (double)ekf.delayed_state.p_ned[1]);
    printf("    True East Vel: %.2f m/s | EKF East Vel: %.2f m/s\n", 
           (double)true_vel[1], (double)ekf.delayed_state.v_ned[1]);

    /* Ensure filter tracked the wind deviation accurately */
    ASSERT_NEAR(ekf.delayed_state.p_ned[1], true_pos[1], 2.0f);
    ASSERT_NEAR(ekf.delayed_state.v_ned[1], true_vel[1], 0.5f);
    printf("  OK\n");
}

/* ── Suite runner ───────────────────────────────────────────────────────────── */

void run_advanced_tests(void)
{
    printf("\n=== EKF Advanced Suite (latency / jitter / unmodeled forces) ===\n");
    test_gps_latency();
    test_imu_jitter_and_drops();
    test_crosswind_drag();
}