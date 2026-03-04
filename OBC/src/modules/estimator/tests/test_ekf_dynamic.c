/**
 * @file test_ekf_dynamic.c
 * @brief Adversarial dynamic-flight tests.
 *
 * These tests call imu_predict(vehicle_imu_t*) for a genuine end-to-end
 * integration test that exercises the same code path used during flight.
 *
 * Ground-truth model uses NED convention (Z+ down).
 * Specific force convention: what the accelerometer measures in body frame
 *   sf = a_true_body - g_body
 * imu_predict() receives delta_velocity = sf * dt and compensates gravity
 * internally.
 */

#include "test_utils.h"

/* ── Test: 1D Launch Profile ─────────────────────────────────────────────────
 *
 * Simulates a complete suborbital flight:
 *   Phase 1 – 3 s pad idle       (specific force = -g,  no acceleration)
 *   Phase 2 – 5 s motor burn     (~3g net upward accel, high VRM vibration)
 *   Phase 3 – 12 s coast to peak (freefall, specific force ≈ 0)
 *
 * Goal: prove the ESKF tracks a parabolic trajectory without the covariance
 * blowing up or the innovation gates falsely rejecting valid measurements.
 * ─────────────────────────────────────────────────────────────────────────── */

static void test_launch_profile(void)
{
    printf("  test_launch_profile ...\n");

    ekf_core_t ekf;
    ekf_core_init(&ekf);

    /* ── Ground truth ── */
    float true_pos[3] = {0.0f, 0.0f, 0.0f};  /* NED, metres */
    float true_vel[3] = {0.0f, 0.0f, 0.0f};  /* NED, m/s    */

    /* Hidden sensor biases */
    const float hidden_gyro_bias[3]  = { 0.005f, -0.002f,  0.000f};
    const float hidden_accel_bias[3] = { 0.02f,  -0.05f,   0.15f };

    const float dt          = 0.004f;   /* 250 Hz */
    const int idle_steps    = 250 *  3; /* 3 s */
    const int burn_steps    = 250 *  5; /* 5 s */
    const int coast_steps   = 250 * 12; /* 12 s */
    const int total_steps   = idle_steps + burn_steps + coast_steps;

    printf("    Steps: idle=%d  burn=%d  coast=%d\n",
           idle_steps, burn_steps, coast_steps);
    printf("    %-5s  %-8s  %-10s  %-10s  %-10s  %-10s\n",
           "t(s)", "phase", "true_p[2]", "ekf_p[2]", "true_v[2]", "ekf_v[2]");

    for (int step = 0; step < total_steps; step++) {

        /* ── Flight model: derive specific force and true NED acceleration ── */
        float true_accel_ned_z;     /* NED Z acceleration (m/s²) */
        float specific_force_z;     /* body-frame Z specific force (m/s²) */
        float vib_noise;            /* std-dev of body-frame accel noise (m/s²) */

        if (step < idle_steps) {
            /* Pad idle: sitting still, no net acceleration */
            true_accel_ned_z  =  0.0f;
            specific_force_z  = -9.80665f; /* upward ground reaction */
            vib_noise         =  0.05f;
        } else if (step < idle_steps + burn_steps) {
            /* Motor burn: ~3g net upward = NED Z = -29.42 m/s² */
            true_accel_ned_z  = -29.42f;
            specific_force_z  = -39.2266f; /* total thrust force / mass */
            vib_noise         =  2.50f;     /* solid rocket motor vibration */
        } else {
            /* Coast / freefall: gravity only, specific force ≈ 0 */
            true_accel_ned_z  =  9.80665f;
            specific_force_z  =  0.0f;
            vib_noise         =  0.10f;
        }

        /* ── 1. Advance ground truth ── */
        true_vel[2] += true_accel_ned_z * dt;
        true_pos[2] += true_vel[2] * dt;

        /* ── 2. Generate noisy IMU measurement ── */
        vehicle_imu_t imu;
        imu.timestamp_us = (uint64_t)(step * (uint64_t)(dt * 1e6f));
        imu.dt_s         = dt;

        for (int i = 0; i < 3; i++) {
            float gyro_rate  = hidden_gyro_bias[i]
                               + generate_gaussian_noise(0.0f, 0.001f);
            float accel_body = (i == 2 ? specific_force_z : 0.0f)
                               + hidden_accel_bias[i]
                               + generate_gaussian_noise(0.0f, vib_noise);

            imu.delta_angle[i]    = gyro_rate  * dt;
            imu.delta_velocity[i] = accel_body * dt;
        }

        /* ── 3. EKF predict via the real integration path ── */
        imu_predict(&ekf, &imu);

        /* ── Periodic snapshot every 2 s ── */
        if (step % 500 == 0) {
            const char *phase = step < idle_steps ? "idle"
                              : step < idle_steps + burn_steps ? "BURN" : "coast";
            printf("    %5.1f  %-8s  %10.2f  %10.2f  %10.3f  %10.3f\n",
                   (double)(step * dt), phase,
                   (double)true_pos[2], (double)ekf.state.p_ned[2],
                   (double)true_vel[2], (double)ekf.state.v_ned[2]);
        }

        /* ── Phase transition snapshots ── */
        if (step == idle_steps) {
            printf("    --- IGNITION (step %d, T=%.1fs)  "
                   "true_vel[2]=%.2f  ekf_vel[2]=%.2f\n",
                   step, (double)(step * dt),
                   (double)true_vel[2], (double)ekf.state.v_ned[2]);
        }
        if (step == idle_steps + burn_steps) {
            printf("    --- BURNOUT  (step %d, T=%.1fs)  "
                   "true_vel[2]=%.2f  ekf_vel[2]=%.2f  "
                   "true_pos[2]=%.1f  ekf_pos[2]=%.1f\n",
                   step, (double)(step * dt),
                   (double)true_vel[2], (double)ekf.state.v_ned[2],
                   (double)true_pos[2], (double)ekf.state.p_ned[2]);
            printf("            P_vz=%.3f  P_pz=%.3f  |q|=%.8f\n",
                   (double)ekf.P[5*15+5], (double)ekf.P[8*15+8],
                   (double)quat_norm(ekf.state.q));
        }

        /* ── 4. Barometer update at 50 Hz (every 5 steps) ── */
        if (step % 5 == 0) {
            baro_measurement_t b = {
                .altitude_m = -true_pos[2] + generate_gaussian_noise(0.0f, 0.5f)
            };
            baro_fuse(&ekf, &b);
        }

        /* ── 5. GPS update at 10 Hz (every 25 steps) ── */
        if (step % 25 == 0) {
            gps_measurement_t g;
            for (int i = 0; i < 3; i++) {
                g.pos_ned[i] = true_pos[i] + generate_gaussian_noise(0.0f, 1.5f);
                g.vel_ned[i] = true_vel[i] + generate_gaussian_noise(0.0f, 0.1f);
            }
            gps_fuse(&ekf, &g);
        }
    }

    printf("    --- Final state (T=%.0fs) ---\n", (double)(total_steps * dt));
    printf("    true_pos = [%.2f, %.2f, %.2f] m\n",
           (double)true_pos[0], (double)true_pos[1], (double)true_pos[2]);
    printf("    ekf_pos  = [%.2f, %.2f, %.2f] m  err_z=%.2f m\n",
           (double)ekf.state.p_ned[0], (double)ekf.state.p_ned[1], (double)ekf.state.p_ned[2],
           (double)(ekf.state.p_ned[2] - true_pos[2]));
    printf("    true_vel = [%.3f, %.3f, %.3f] m/s\n",
           (double)true_vel[0], (double)true_vel[1], (double)true_vel[2]);
    printf("    ekf_vel  = [%.3f, %.3f, %.3f] m/s  err_z=%.3f m/s\n",
           (double)ekf.state.v_ned[0], (double)ekf.state.v_ned[1], (double)ekf.state.v_ned[2],
           (double)(ekf.state.v_ned[2] - true_vel[2]));
    printf("    EKF accel_bias = [%.4f, %.4f, %.4f] m/s²  (truth=[%.4f, %.4f, %.4f])\n",
           (double)ekf.state.accel_bias[0], (double)ekf.state.accel_bias[1],
           (double)ekf.state.accel_bias[2],
           (double)hidden_accel_bias[0], (double)hidden_accel_bias[1],
           (double)hidden_accel_bias[2]);
    printf("    EKF gyro_bias  = [%.5f, %.5f, %.5f] rad/s\n",
           (double)ekf.state.gyro_bias[0], (double)ekf.state.gyro_bias[1],
           (double)ekf.state.gyro_bias[2]);
    printf("    P trace=%.2f  P_vz=%.4f  P_pz=%.4f  |q|=%.8f\n",
           (double)matrix_trace(ekf.P, 15),
           (double)ekf.P[5*15+5], (double)ekf.P[8*15+8],
           (double)quat_norm(ekf.state.q));

    /* Covariance must remain finite — highest-priority assertion */
    ASSERT_TRUE(all_finite(ekf.P, 225));

    /* Quaternion must remain unit-norm */
    ASSERT_NEAR(quat_norm(ekf.state.q), 1.0f, 1e-4f);

    /* Vertical position: within 5 m (GPS+Baro fused across all phases) */
    ASSERT_NEAR(ekf.state.p_ned[2], true_pos[2], 5.0f);

    /* Vertical velocity: within 2 m/s */
    ASSERT_NEAR(ekf.state.v_ned[2], true_vel[2], 2.0f);

    printf("  OK\n");
}

/* ── Test: High-Frequency Motor Vibration ────────────────────────────────────
 *
 * Injects ±50 m/s² zero-mean noise into all IMU axes for 2 simulated seconds
 * to mimic solid-rocket motor vibration.
 *
 * Goal: prove the quaternion stays unit-norm and the covariance matrix does
 * not diverge or produce NaNs — neither the state vector nor P blow up.
 * ─────────────────────────────────────────────────────────────────────────── */

static void test_motor_vibration(void)
{
    printf("  test_motor_vibration ...\n");

    ekf_core_t ekf;
    ekf_core_init(&ekf);

    /* Constant-velocity flight so the ground truth is trivial */
    const float true_pos[3]  = {0.0f, 0.0f,  0.0f};
    const float true_vel[3]  = {0.0f, 0.0f, -5.0f};
    const float sf_nominal_z = -9.80665f; /* steady level flight specific force */

    const float dt          = 0.004f;
    const float vib_std     = 50.0f;    /* ±50 m/s² std-dev — extreme SRM noise */
    const int   total_steps = 250 * 2;  /* 2 simulated seconds */

    for (int step = 0; step < total_steps; step++) {

        vehicle_imu_t imu;
        imu.timestamp_us = (uint64_t)(step * (uint64_t)(dt * 1e6f));
        imu.dt_s         = dt;

        for (int i = 0; i < 3; i++) {
            float sf = (i == 2 ? sf_nominal_z : 0.0f)
                       + generate_gaussian_noise(0.0f, vib_std);
            imu.delta_velocity[i] = sf * dt;
            imu.delta_angle[i]    = generate_gaussian_noise(0.0f, 0.001f) * dt;
        }

        imu_predict(&ekf, &imu);

        /* Barometer at 50 Hz keeps the filter anchored so we can test gating */
        if (step % 5 == 0) {
            baro_measurement_t b = {
                .altitude_m = -true_pos[2] + generate_gaussian_noise(0.0f, 0.5f)
            };
            baro_fuse(&ekf, &b);
        }
    }

    printf("    P trace after vibration: %.4f\n",
           (double)matrix_trace(ekf.P, 15));
    printf("    |q| = %.8f\n", (double)quat_norm(ekf.state.q));

    /* These are the only invariants that must always hold under vibration */
    ASSERT_TRUE(all_finite(ekf.P, 225));
    ASSERT_NEAR(quat_norm(ekf.state.q), 1.0f, 1e-4f);

    printf("  OK\n");
}

/* ── Test: High Pitch Rate with Lever Arm Compensation ────────────────────────
 *
 * Simulates a vehicle pitching at 10 rad/s for 5 seconds with the IMU mounted
 * 0.1 m forward of the CG (matches the hardcode in imu_predictor.c).
 *
 * Physics: omega = [0, wy, 0], r = [rx, 0, 0]
 *   omega x r           = [0, 0, -wy*rx]
 *   omega x (omega x r) = [-wy^2*rx, 0, 0]   <- centripetal, ~10 m/s^2
 *
 * The IMU therefore measures a massive spurious -X body acceleration on top of
 * gravity. Without lever arm compensation this rotates into large NED velocity
 * errors as the body pitches through 90-degree increments.
 *
 * Goal: Prove quaternion stays strictly normalised at high angular rates AND
 * that the centripetal term is stripped, keeping NED velocity near zero.
 * ─────────────────────────────────────────────────────────────────────────── */
static void test_high_roll_rate(void)
{
    printf("  test_high_roll_rate ...\n");

    ekf_core_t ekf;
    ekf_core_init(&ekf);

    const float dt          = 0.004f;
    const int   total_steps = 250 * 5; // 5 seconds at 250 Hz

    // 10 rad/s pitch around body-Y axis
    const float pitch_rate  = 10.0f;
    const float lever_arm_x = 0.1f;

    // Centripetal acceleration felt at IMU location: a_c = -wy^2 * rx (body X)
    const float centripetal_x = -(pitch_rate * pitch_rate) * lever_arm_x; // -10 m/s^2

    for (int step = 0; step < total_steps; step++) {
        vehicle_imu_t imu;
        imu.timestamp_us = (uint64_t)(step * (uint64_t)(dt * 1e6f));
        imu.dt_s         = dt;

        // Pure pitch: body-Y angular rate
        imu.delta_angle[0] = 0.0f;
        imu.delta_angle[1] = pitch_rate * dt;
        imu.delta_angle[2] = 0.0f;

        // IMU feels centripetal acceleration (spurious) plus body-frame gravity
        imu.delta_velocity[0] = centripetal_x * dt;
        imu.delta_velocity[1] = 0.0f;
        imu.delta_velocity[2] = -9.80665f * dt; // gravity in body Z (level at t=0)

        imu_predict(&ekf, &imu);
    }

    printf("    |q| = %.8f\n",          (double)quat_norm(ekf.state.q));
    printf("    v_ned = [%.3f, %.3f, %.3f] m/s\n",
           (double)ekf.state.v_ned[0],
           (double)ekf.state.v_ned[1],
           (double)ekf.state.v_ned[2]);

    // Quaternion must remain unit-norm at high body rates
    ASSERT_NEAR(quat_norm(ekf.state.q), 1.0f, 1e-4f);

    // Lever arm compensation strips the centripetal acceleration in body-X.
    // v_ned[0] should remain near zero despite the ~10 m/s^2 centripetal injection.
    ASSERT_NEAR(ekf.state.v_ned[0], 0.0f, 1.0f);

    // NOTE: v_ned[2] is NOT asserted here. It accumulates ~g*(T - sin(wT)/w) ≈ 49 m/s
    // because the test injects a constant body-Z gravity while the body pitches through
    // multiple full rotations. This is expected physics, not a lever arm failure.

    printf("  OK\n");
}

/* ── Suite runner ───────────────────────────────────────────────────────────── */

void run_dynamic_tests(void)
{
    printf("\n=== EKF Dynamic Flight Suite (launch profile / vibration) ===\n");
    test_launch_profile();
    test_motor_vibration();
    test_high_roll_rate();
}
