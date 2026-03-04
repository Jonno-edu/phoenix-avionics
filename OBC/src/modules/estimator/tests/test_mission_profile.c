/**
 * @file test_mission_profile.c
 * @brief Scripted 60-second pad-to-apogee mission profile test.
 *
 * This is the highest-level integration test in the EKF suite. It scripts a
 * physically realistic flight timeline through all 5 modes of the state machine
 * and verifies that the filter remains locked to ground-truth kinematics across
 * the full dynamic range: static pad → 5g motor burn → freefall coast.
 *
 * Timeline (250 Hz / 4 ms steps)
 * ─────────────────────────────────────────────────────────────────────────────
 *  T -15 s  (3750 steps)   PAD:  Static. State machine completes leveling,
 *                                heading align, and ZVU calibration.
 *  T  +0 s  (1250 steps)   BURN: ~5g net upward accel. State machine trips to
 *                                FLIGHT mode. IMU + mag (de-weighted) only.
 *  T  +5 s  (10000 steps)  COAST: Freefall. IMU + GPS (10Hz) + Baro (50Hz).
 *                                 Nominal tracking should be maintained.
 * ─────────────────────────────────────────────────────────────────────────────
 *
 * Pass criteria:
 *   - State machine enters ZVU_CALIBRATING before liftoff.
 *   - State machine enters FLIGHT mode within 1 step of the 2g trigger.
 *   - Covariance matrix remains finite through the entire profile.
 *   - Quaternion unit-norm is preserved.
 *   - Z position tracks ground truth to within 15 m after 40s coast.
 *   - Z velocity tracks ground truth to within 3 m/s after 40s coast.
 *
 * Conventions:
 *   - NED frame: Z is positive DOWN.
 *   - Upward acceleration  → p_ned[2] decreasing (more negative = higher alt).
 *   - Gravity in body (level, Z-down): accel_meas = [0, 0, +9.80665].
 *   - In freefall: specific_force_body ≈ 0  (accelerometer reads ~0).
 *   - During burn (5g net upward): specific_force_body[2] ≈ -(5g + g) = -6g,
 *     BUT since body Z is DOWN and thrust is UP: specific_force_body[2] ≈ -6g
 *     Wait — thrust UP on Z-down body means specific force = -6g in body Z.
 *     Net NED accel: a_ned[2] = -5g (upward = negative Z-down = negative).
 */

#include "test_utils.h"
#include "ekf_state.h"
#include "fusion/imu_predictor.h"
#include "fusion/baro_fuse.h"
#include "fusion/gps_fuse.h"

/* Stellenbosch WMM reference field (NED, Gauss) — ~32°S, 19°E */
static const float SB_MAG_NED[3] = { 0.15f, -0.05f, -0.33f };

/* Physics constants */
#define G_MS2 9.80665f

/* ── Test ─────────────────────────────────────────────────────────────────── */

static void test_full_mission_60s(void)
{
    printf("  test_full_mission_60s ...\n");

    /* ----- Setup --------------------------------------------------------- */
    ekf_core_t      ekf;
    ekf_state_ctx_t ctx;
    ekf_core_init(&ekf);
    ekf_state_init(&ctx, SB_MAG_NED, 0.004f);

    /* Bypass LEVELING and HEADING_ALIGN by injecting the known launch-rail
     * orientation directly.  elevation=0 (level body) matches the test physics
     * which use specific_force=[0,0,-g] on the pad.  On the real flight
     * computer use LAUNCH_RAIL_ELEVATION_RAD / LAUNCH_RAIL_AZIMUTH_RAD from
     * sensor_config.h instead. */
    ekf_state_set_launch_rail(&ctx, &ekf, 0.0f, 0.0f);

    const float dt = 0.004f;   /* 250 Hz */

    /* Residual turn-on biases that remain *after* factory bench calibration.
     * These are deliberately small — the factory NVM constants (sensor_config.h)
     * would have already removed the large, static portion.  The ZVU phase
     * estimates and corrects for these small run-to-run offsets. */
    const float gyro_bias_true[3]  = { 0.003f, -0.001f,  0.005f };  /* rad/s */
    const float accel_bias_true[3] = { 0.02f,  -0.03f,   0.08f  };  /* m/s² */

    /* Ground truth kinematics in NED (Z down). Start at origin. */
    float true_pos[3] = { 0.0f, 0.0f,  0.0f };
    float true_vel[3] = { 0.0f, 0.0f,  0.0f };

    /* Phase durations */
    const int pad_steps   = (int)(15.0f / dt);   /*  3750 — 15 s static pad  */
    const int burn_steps  = (int)( 5.0f / dt);   /*  1250 — 5 s motor burn   */
    const int coast_steps = (int)(40.0f / dt);   /* 10000 — 40 s freefall    */
    const int total_steps = pad_steps + burn_steps + coast_steps;

    bool flight_mode_triggered = false;
    bool calibrating_before_launch = false;

    printf("    Steps: pad=%d  burn=%d  coast=%d  total=%d\n",
           pad_steps, burn_steps, coast_steps, total_steps);
    printf("    Hidden biases — gyro=[%.4f, %.4f, %.4f] rad/s  "
           "accel=[%.3f, %.3f, %.3f] m/s²\n",
           (double)gyro_bias_true[0],  (double)gyro_bias_true[1],  (double)gyro_bias_true[2],
           (double)accel_bias_true[0], (double)accel_bias_true[1], (double)accel_bias_true[2]);
    printf("    %-6s  %-5s  %-10s  %-10s  %-10s  %-10s  %-8s  %s\n",
           "t(s)", "mode", "true_p[2]", "ekf_p[2]", "true_v[2]", "ekf_v[2]", "|q|", "P_trace");

    for (int step = 0; step < total_steps; step++) {

        /* ── Ground-truth physics ──────────────────────────────────────── */
        float true_accel_ned[3]  = { 0.0f, 0.0f, 0.0f };  /* NED accel (m/s²) */
        float specific_force[3]  = { 0.0f, 0.0f, 0.0f };  /* Body-frame accel  */
        float vib_noise_std      = 0.05f;
        bool  send_mag           = false;
        bool  send_gps           = false;
        bool  send_baro          = false;

        if (step < pad_steps) {
            /* PAD: stationary on the launch rail.
             * Gravity acts on accelerometer (body Z down): a_meas = [0,0,-g]. */
            true_accel_ned[0] = 0.0f;
            true_accel_ned[1] = 0.0f;
            true_accel_ned[2] = 0.0f;                 /* No kinematic accel */
            specific_force[0] = 0.0f;
            specific_force[1] = 0.0f;
            specific_force[2] = -G_MS2;               /* Gravity reaction pushes UP (negative Z) */
            vib_noise_std     = 0.05f;
            send_mag          = (step % 50 == 0);     /* Mag at 5 Hz */
        }
        else if (step < pad_steps + burn_steps) {
            /* BURN: 5g net upward acceleration.
             *   a_ned[2] = -5g (upward = negative Z-down)
             *   specific_force[2] = -(a_ned[2]) + g_body[2]
             *                     = -(-5g) + g = 6g? No...
             *
             * ESKF accel model: a_ned = R * (specific_force - accel_bias) + g_ned
             * where g_ned = [0, 0, +g] in NED.
             * So: specific_force = R^T * (a_ned - g_ned) + accel_bias
             * For level R = I, no bias:
             *   specific_force[2] = a_ned[2] - g_ned[2] = -5g - g = -6g
             *
             * But what the accelerometer *reads* (raw) is specific_force + bias.
             * The delta_velocity fed to imu_predict = specific_force * dt.
             */
            true_accel_ned[0] = 0.0f;
            true_accel_ned[1] = 0.0f;
            true_accel_ned[2] = -5.0f * G_MS2;        /* 5g upward */
            specific_force[0] = 0.0f;
            specific_force[1] = 0.0f;
            specific_force[2] = -6.0f * G_MS2;        /* Thrust reaction (body Z down, thrust up) */
            vib_noise_std     = 2.5f;                  /* SRM vibration */
            send_mag          = false;                 /* GPS/Baro off during burn */
        }
        else {
            /* COAST: freefall (drag neglected for simplicity).
             *   a_ned[2] = +g (gravity dominant, Z-down)
             *   specific_force = 0 (free-fall, accelerometer reads ~0) */
            true_accel_ned[0] = 0.0f;
            true_accel_ned[1] = 0.0f;
            true_accel_ned[2] = +G_MS2;
            specific_force[0] = 0.0f;
            specific_force[1] = 0.0f;
            specific_force[2] = 0.0f;
            vib_noise_std     = 0.1f;
            send_gps          = (step % 25 == 0);      /* GPS at 10 Hz */
            send_baro         = (step % 5  == 0);      /* Baro at 50 Hz */
        }

        /* Integrate ground truth in all phases */
        for (int ax = 0; ax < 3; ax++) {
            true_vel[ax] += true_accel_ned[ax] * dt;
            true_pos[ax] += true_vel[ax]        * dt;
        }

        /* ── IMU measurement (specific force + biases + noise) ─────────── */
        float gyro_meas[3], accel_meas[3];
        for (int ax = 0; ax < 3; ax++) {
            gyro_meas[ax]  = gyro_bias_true[ax]
                             + generate_gaussian_noise(0.0f, 0.001f);
            accel_meas[ax] = specific_force[ax]
                             + accel_bias_true[ax]
                             + generate_gaussian_noise(0.0f, vib_noise_std);
        }

        /* Build vehicle_imu_t and predict */
        vehicle_imu_t imu = {0};
        imu.dt_s = dt;
        for (int ax = 0; ax < 3; ax++) {
            imu.delta_angle[ax]    = gyro_meas[ax]  * dt;
            imu.delta_velocity[ax] = accel_meas[ax] * dt;
        }
        imu_predict(&ekf, &imu);

        /* ── State machine update ──────────────────────────────────────── */
        float mag_body[3] = { 0.0f, 0.0f, 0.0f };
        bool  mag_valid   = false;

        if (send_mag) {
            /* Ideal body-frame field = NED ref (q ≈ identity on the pad) + noise */
            for (int ax = 0; ax < 3; ax++) {
                mag_body[ax] = SB_MAG_NED[ax]
                               + generate_gaussian_noise(0.0f, 0.005f);
            }
            mag_valid = true;
        }
        ekf_state_update(&ctx, &ekf, accel_meas, gyro_meas, mag_body, mag_valid, dt);

        /* ── Periodic debug snapshot (every 5 s = 1250 steps) ────────── */
        if (step % 1250 == 0 || step == pad_steps - 1 || step == pad_steps
                             || step == pad_steps + burn_steps - 1
                             || step == pad_steps + burn_steps) {
            float t_s   = (float)step * dt;
            int   mode  = (int)ekf_state_get_mode(&ctx);
            float ptrace = matrix_trace(ekf.P, 15);
            float qn    = quat_norm(ekf.state.q);
            printf("    %6.1f  %-5d  %10.2f  %10.2f  %10.3f  %10.3f  %8.6f  %.2f\n",
                   (double)t_s, mode,
                   (double)true_pos[2], (double)ekf.state.p_ned[2],
                   (double)true_vel[2], (double)ekf.state.v_ned[2],
                   (double)qn, (double)ptrace);
        }

        /* ── Snapshot checks at phase boundaries ───────────────────────── */
        if (step == pad_steps - 1) {
            calibrating_before_launch =
                (ekf_state_get_mode(&ctx) == EKF_MODE_ZVU_CALIBRATING);
            printf("    --- End of PAD phase (T-0): mode=%d  "
                   "ZVU_CALIBRATING=%s  biases_converged see above\n",
                   (int)ekf_state_get_mode(&ctx),
                   calibrating_before_launch ? "YES" : "NO");
            printf("        EKF gyro_bias=[%.5f, %.5f, %.5f]  "
                   "accel_bias=[%.4f, %.4f, %.4f]\n",
                   (double)ekf.state.gyro_bias[0],  (double)ekf.state.gyro_bias[1],
                   (double)ekf.state.gyro_bias[2],
                   (double)ekf.state.accel_bias[0], (double)ekf.state.accel_bias[1],
                   (double)ekf.state.accel_bias[2]);
            printf("        P diagonal: att=[%.4f,%.4f,%.4f] vel=[%.2f,%.2f,%.2f] "
                   "pos=[%.2f,%.2f,%.2f]\n",
                   (double)ekf.P[0*15+0],  (double)ekf.P[1*15+1],  (double)ekf.P[2*15+2],
                   (double)ekf.P[3*15+3],  (double)ekf.P[4*15+4],  (double)ekf.P[5*15+5],
                   (double)ekf.P[6*15+6],  (double)ekf.P[7*15+7],  (double)ekf.P[8*15+8]);
        }
        if (step == pad_steps) {
            flight_mode_triggered =
                (ekf_state_get_mode(&ctx) == EKF_MODE_FLIGHT);
            printf("    --- Start of BURN phase (T+0): mode=%d  "
                   "FLIGHT=%s\n",
                   (int)ekf_state_get_mode(&ctx),
                   flight_mode_triggered ? "YES" : "NO");
            printf("        imu: specific_force[2]=%.2f  true_accel_ned[2]=%.2f\n",
                   (double)specific_force[2], (double)true_accel_ned[2]);
        }
        if (step == pad_steps + burn_steps - 1) {
            printf("    --- End of BURN phase: true_vel[2]=%.2f  ekf_vel[2]=%.2f  "
                   "err=%.2f m/s\n",
                   (double)true_vel[2], (double)ekf.state.v_ned[2],
                   (double)(ekf.state.v_ned[2] - true_vel[2]));
            printf("        true_pos[2]=%.2f  ekf_pos[2]=%.2f  err=%.2f m\n",
                   (double)true_pos[2], (double)ekf.state.p_ned[2],
                   (double)(ekf.state.p_ned[2] - true_pos[2]));
            printf("        P_vz=%.3f  P_pz=%.3f  P_trace=%.1f\n",
                   (double)ekf.P[5*15+5], (double)ekf.P[8*15+8],
                   (double)matrix_trace(ekf.P, 15));
        }
        if (step == pad_steps + burn_steps) {
            printf("    --- Start of COAST phase: GPS+Baro fusion resumes\n");
        }

        /* ── External sensor fusion (coast only) ───────────────────────── */
        if (send_baro) {
            baro_measurement_t baro;
            /* altitude_m = positive up = -p_ned[2] (NED Z-down) */
            baro.altitude_m = -true_pos[2]
                              + generate_gaussian_noise(0.0f, 0.5f);
            baro_fuse(&ekf, &baro);
        }

        if (send_gps) {
            gps_measurement_t gps;
            for (int ax = 0; ax < 3; ax++) {
                gps.pos_ned[ax] = true_pos[ax]
                                  + generate_gaussian_noise(0.0f, 1.5f);
                gps.vel_ned[ax] = true_vel[ax]
                                  + generate_gaussian_noise(0.0f, 0.1f);
            }
            gps_fuse(&ekf, &gps);
        }
    }

    /* ── Final assertions ─────────────────────────────────────────────────── */

    printf("\n    --- Final state at T+%.0f s ---\n",
           (double)(total_steps * dt - 15.0f));
    printf("    State machine mode:   %d (4=FLIGHT expected)\n",
           (int)ekf_state_get_mode(&ctx));
    printf("    ZVU_CALIBRATING before launch: %s\n",
           calibrating_before_launch ? "YES" : "NO");
    printf("    FLIGHT triggered at liftoff:   %s\n",
           flight_mode_triggered     ? "YES" : "NO");
    printf("    Covariance finite: %s  |q|=%.8f\n",
           all_finite(ekf.P, 225)  ? "YES" : "NO",
           (double)quat_norm(ekf.state.q));
    printf("    EKF gyro_bias  = [%.5f, %.5f, %.5f] rad/s\n",
           (double)ekf.state.gyro_bias[0], (double)ekf.state.gyro_bias[1],
           (double)ekf.state.gyro_bias[2]);
    printf("    EKF accel_bias = [%.4f, %.4f, %.4f] m/s²  (truth=[%.4f,%.4f,%.4f])\n",
           (double)ekf.state.accel_bias[0], (double)ekf.state.accel_bias[1],
           (double)ekf.state.accel_bias[2],
           (double)accel_bias_true[0], (double)accel_bias_true[1],
           (double)accel_bias_true[2]);
    printf("    P diagonal (last 3 att, vel_z, pos_z):  att_z=%.5f  vel_z=%.4f  pos_z=%.3f\n",
           (double)ekf.P[2*15+2], (double)ekf.P[5*15+5], (double)ekf.P[8*15+8]);
    printf("    P trace = %.2f\n", (double)matrix_trace(ekf.P, 15));

    /* State machine transitions */
    ASSERT_TRUE(calibrating_before_launch);   /* ZVU_CALIBRATING reached on pad */
    ASSERT_TRUE(flight_mode_triggered);       /* FLIGHT triggered at liftoff     */
    ASSERT_TRUE(ekf_state_get_mode(&ctx) == EKF_MODE_FLIGHT);

    /* Math stability — the critical flight-software guarantee */
    ASSERT_TRUE(all_finite(ekf.P, 225));
    ASSERT_NEAR(quat_norm(ekf.state.q), 1.0f, 0.001f);

    /* Kinematic tracking (at end of 40 s coast) */
    printf("\n    Kinematic tracking error:\n");
    printf("    true_pos = [%.2f, %.2f, %.2f] m\n",
           (double)true_pos[0], (double)true_pos[1], (double)true_pos[2]);
    printf("    ekf_pos  = [%.2f, %.2f, %.2f] m\n",
           (double)ekf.state.p_ned[0], (double)ekf.state.p_ned[1], (double)ekf.state.p_ned[2]);
    printf("    err_pos  = [%.2f, %.2f, %.2f] m\n",
           (double)(ekf.state.p_ned[0]-true_pos[0]),
           (double)(ekf.state.p_ned[1]-true_pos[1]),
           (double)(ekf.state.p_ned[2]-true_pos[2]));
    printf("    true_vel = [%.3f, %.3f, %.3f] m/s\n",
           (double)true_vel[0], (double)true_vel[1], (double)true_vel[2]);
    printf("    ekf_vel  = [%.3f, %.3f, %.3f] m/s\n",
           (double)ekf.state.v_ned[0], (double)ekf.state.v_ned[1], (double)ekf.state.v_ned[2]);
    printf("    err_vel  = [%.3f, %.3f, %.3f] m/s\n",
           (double)(ekf.state.v_ned[0]-true_vel[0]),
           (double)(ekf.state.v_ned[1]-true_vel[1]),
           (double)(ekf.state.v_ned[2]-true_vel[2]));

    ASSERT_NEAR(ekf.state.p_ned[2], true_pos[2], 15.0f);   /* pos within 15 m  */
    ASSERT_NEAR(ekf.state.v_ned[2], true_vel[2],  3.0f);   /* vel within 3 m/s */

    printf("  OK\n");
}

/* ── Suite entry point ──────────────────────────────────────────────────────── */

void run_mission_tests(void)
{
    printf("\n=== EKF Mission Profile Suite ===\n");
    test_full_mission_60s();
}
