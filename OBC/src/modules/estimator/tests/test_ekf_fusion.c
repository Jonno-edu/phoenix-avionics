/**
 * @file test_ekf_fusion.c
 * @brief Unit tests for stationary bias convergence and GPS + Baro fusion.
 *
 * These tests call symforce_update_* and baro_fuse / gps_fuse directly.
 * Nominal state propagation is handled by propagate_nominal_state() so that
 * the fusion update functions can be tested in isolation from imu_predict().
 */

#include "test_utils.h"

/* ── Tests ─────────────────────────────────────────────────────────────────── */

static void test_stationary_bias_convergence(void)
{
    printf("  test_stationary_bias_convergence ...\n");

    ekf_core_t ekf;
    ekf_core_init(&ekf);

    const float true_accel[3]       = {0.0f, 0.0f, 9.80665f};
    const float true_gyro[3]        = {0.0f, 0.0f, 0.0f};
    const float hidden_gyro_bias[3]  = {0.005f, -0.002f, 0.015f};
    const float hidden_accel_bias[3] = {0.1f, -0.05f, 0.0f};

    const float accel_noise_std = 0.05f;
    const float gyro_noise_std  = 0.002f;
    const float accel_var[3]    = { accel_noise_std*accel_noise_std,
                                    accel_noise_std*accel_noise_std,
                                    accel_noise_std*accel_noise_std };
    const float gyro_var        = gyro_noise_std * gyro_noise_std;
    const float dt              = 0.004f;   /* 250 Hz */

    /* 15 simulated seconds */
    for (int i = 0; i < 3750; i++) {
        float meas_accel[3], meas_gyro[3];
        for (int axis = 0; axis < 3; axis++) {
            meas_accel[axis] = true_accel[axis] + hidden_accel_bias[axis]
                               + generate_gaussian_noise(0.0f, accel_noise_std);
            meas_gyro[axis]  = true_gyro[axis]  + hidden_gyro_bias[axis]
                               + generate_gaussian_noise(0.0f, gyro_noise_std);
        }

        symforce_predict_covariance(
            ekf.P, ekf.state.q, ekf.state.v_ned, ekf.state.p_ned,
            ekf.state.gyro_bias, ekf.state.accel_bias,
            meas_accel, accel_var, meas_gyro, gyro_var, dt
        );

        const float meas_vel_var[3]  = {1e-6f, 1e-6f, 1e-6f};
        const float meas_gyro_var[3] = {1e-6f, 1e-6f, 1e-6f};

        symforce_update_stationary(
            ekf.P, ekf.state.q, ekf.state.v_ned, ekf.state.p_ned,
            ekf.state.gyro_bias, ekf.state.accel_bias,
            meas_gyro, meas_vel_var, meas_gyro_var
        );
    }

    printf("    Target Gyro Bias:  [%.4f, %.4f, %.4f]\n",
           (double)hidden_gyro_bias[0],
           (double)hidden_gyro_bias[1],
           (double)hidden_gyro_bias[2]);
    printf("    EKF Gyro Bias:     [%.4f, %.4f, %.4f]\n",
           (double)ekf.state.gyro_bias[0],
           (double)ekf.state.gyro_bias[1],
           (double)ekf.state.gyro_bias[2]);

    ASSERT_NEAR(ekf.state.gyro_bias[0], hidden_gyro_bias[0], 0.002f);
    ASSERT_NEAR(ekf.state.gyro_bias[1], hidden_gyro_bias[1], 0.002f);
    ASSERT_NEAR(ekf.state.gyro_bias[2], hidden_gyro_bias[2], 0.002f);

    printf("  OK\n");
}

static void test_gps_baro_fusion(void)
{
    printf("  test_gps_baro_fusion ...\n");

    ekf_core_t ekf;
    ekf_core_init(&ekf);

    /* Ground truth: constant-velocity ascent, 2 m/s up (NED -z direction) */
    float true_pos[3] = {0.0f, 0.0f,  0.0f};
    float true_vel[3] = {0.0f, 0.0f, -2.0f};

    /* Specific force for a level accelerometer during constant-velocity flight.
     * Body Z = -g; imu_predictor adds +g back giving zero net NED acceleration. */
    const float true_accel_body[3]  = {0.0f, 0.0f, -9.80665f};
    const float true_gyro[3]        = {0.0f, 0.0f, 0.0f};
    const float hidden_gyro_bias[3]  = { 0.010f, -0.005f,  0.002f};
    const float hidden_accel_bias[3] = { 0.050f, -0.030f,  0.000f};

    const float accel_noise_std = 0.05f;
    const float gyro_noise_std  = 0.002f;
    const float accel_var[3]    = { accel_noise_std*accel_noise_std,
                                    accel_noise_std*accel_noise_std,
                                    accel_noise_std*accel_noise_std };
    const float gyro_var        = gyro_noise_std * gyro_noise_std;
    const float dt              = 0.004f;
    const int   total_steps     = 250 * 60; /* 60 simulated seconds */

    for (int step = 0; step < total_steps; step++) {

        for (int i = 0; i < 3; i++) true_pos[i] += true_vel[i] * dt;

        float meas_accel[3], meas_gyro[3];
        for (int i = 0; i < 3; i++) {
            meas_accel[i] = true_accel_body[i] + hidden_accel_bias[i]
                            + generate_gaussian_noise(0.0f, accel_noise_std);
            meas_gyro[i]  = true_gyro[i] + hidden_gyro_bias[i]
                            + generate_gaussian_noise(0.0f, gyro_noise_std);
        }

        propagate_nominal_state(&ekf, meas_gyro, meas_accel, dt);

        symforce_predict_covariance(
            ekf.P, ekf.state.q, ekf.state.v_ned, ekf.state.p_ned,
            ekf.state.gyro_bias, ekf.state.accel_bias,
            meas_accel, accel_var, meas_gyro, gyro_var, dt
        );

        if (step % 5 == 0) {
            mock_baro_t raw = generate_mock_baro(true_pos[2]);
            baro_measurement_t b = { .altitude_m = raw.altitude_m };
            baro_fuse(&ekf, &b);
        }

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
           (double)hidden_gyro_bias[0],
           (double)hidden_gyro_bias[1],
           (double)hidden_gyro_bias[2]);
    printf("    EKF   gyro_bias: [%.4f, %.4f, %.4f]\n",
           (double)ekf.state.gyro_bias[0],
           (double)ekf.state.gyro_bias[1],
           (double)ekf.state.gyro_bias[2]);

    /* Vertical position: within 0.5 m */
    ASSERT_NEAR(ekf.state.p_ned[2], true_pos[2], 0.5f);
    /* Vertical velocity: within 0.2 m/s */
    ASSERT_NEAR(ekf.state.v_ned[2], true_vel[2], 0.2f);
    /* Gyro bias X/Y observable through GPS+Baro (Z/yaw requires magnetometer) */
    ASSERT_NEAR(ekf.state.gyro_bias[0], hidden_gyro_bias[0], 0.002f);
    ASSERT_NEAR(ekf.state.gyro_bias[1], hidden_gyro_bias[1], 0.002f);
    /* Covariance must remain finite */
    ASSERT_TRUE(all_finite(ekf.P, 225));

    printf("  OK\n");
}

/* ── Suite runner ───────────────────────────────────────────────────────────── */

/* ── Suite runner ───────────────────────────────────────────────────────────── */

static void test_mag_fusion(void)
{
    printf("  test_mag_fusion ...\n");
    printf("    [Stellenbosch NED ref: N=%.4f E=%.4f D=%.4f Gauss]\n",
           (double)MAG_REF_STELLENBOSCH_N,
           (double)MAG_REF_STELLENBOSCH_E,
           (double)MAG_REF_STELLENBOSCH_D);

    ekf_core_t ekf;
    ekf_core_init(&ekf);

    /* Reference field for Stellenbosch launch site */
    const float mag_ref_ned[3] = {
        MAG_REF_STELLENBOSCH_N,
        MAG_REF_STELLENBOSCH_E,
        MAG_REF_STELLENBOSCH_D
    };

    /* True hidden gyro bias including a Z-axis component (yaw — previously unobservable) */
    const float hidden_gyro_bias[3]  = { 0.005f, -0.002f, 0.008f };
    const float hidden_accel_bias[3] = { 0.05f, -0.03f, 0.0f };

    const float accel_noise_std = 0.05f;
    const float gyro_noise_std  = 0.002f;
    const float mag_noise_std   = 0.015f;   /* ~15 mGauss noise per axis */
    const float dt              = 0.004f;   /* 250 Hz */

    const float true_accel[3] = {0.0f, 0.0f, 9.80665f};  /* level, stationary */
    const float true_gyro[3]  = {0.0f, 0.0f, 0.0f};

    const float accel_var[3] = { accel_noise_std*accel_noise_std,
                                  accel_noise_std*accel_noise_std,
                                  accel_noise_std*accel_noise_std };
    const float gyro_var     = gyro_noise_std * gyro_noise_std;

    /* 30 simulated seconds: ZVU + magnetometer fusion at 5 Hz */
    int mag_hz_ratio = (int)(1.0f / (5.0f * dt));  /* every 50 steps = 5 Hz */

    for (int i = 0; i < 7500; i++) {
        float meas_accel[3], meas_gyro[3];
        for (int axis = 0; axis < 3; axis++) {
            meas_accel[axis] = true_accel[axis] + hidden_accel_bias[axis]
                               + generate_gaussian_noise(0.0f, accel_noise_std);
            meas_gyro[axis]  = true_gyro[axis]  + hidden_gyro_bias[axis]
                               + generate_gaussian_noise(0.0f, gyro_noise_std);
        }

        symforce_predict_covariance(
            ekf.P, ekf.state.q, ekf.state.v_ned, ekf.state.p_ned,
            ekf.state.gyro_bias, ekf.state.accel_bias,
            meas_accel, accel_var, meas_gyro, gyro_var, dt
        );

        /* Stationary ZVU every step */
        const float vel_var3[3]  = {1e-6f, 1e-6f, 1e-6f};
        const float gyro_var3[3] = {1e-6f, 1e-6f, 1e-6f};
        symforce_update_stationary(
            ekf.P, ekf.state.q, ekf.state.v_ned, ekf.state.p_ned,
            ekf.state.gyro_bias, ekf.state.accel_bias,
            meas_gyro, vel_var3, gyro_var3
        );

        /* Magnetometer fusion at 5 Hz */
        if (i % mag_hz_ratio == 0) {
            /* Simulate the true body-frame field: rotate NED ref into body.
             * q = identity (level, zero-yaw) at init — body ≈ NED, so mag_body ≈ mag_ref.
             * Add noise. */
            mag_measurement_t mag_meas;
            for (int axis = 0; axis < 3; axis++) {
                mag_meas.field_gauss[axis] = mag_ref_ned[axis]
                    + generate_gaussian_noise(0.0f, mag_noise_std);
            }
            mag_fuse(&ekf, &mag_meas, mag_ref_ned, MAG_DEFAULT_VAR);
        }
    }

    printf("    After 30s ZVU+Mag: gyro_bias [%.4f, %.4f, %.4f]  (truth: [%.4f, %.4f, %.4f])\n",
           (double)ekf.state.gyro_bias[0], (double)ekf.state.gyro_bias[1],
           (double)ekf.state.gyro_bias[2],
           (double)hidden_gyro_bias[0], (double)hidden_gyro_bias[1],
           (double)hidden_gyro_bias[2]);
    printf("    P[2,2] (yaw cov): %.6f   <- should decrease with mag fusion\n",
           (double)ekf.P[2*15 + 2]);

    /* X/Y gyro bias convergence (same as before) */
    ASSERT_NEAR(ekf.state.gyro_bias[0], hidden_gyro_bias[0], 0.002f);
    ASSERT_NEAR(ekf.state.gyro_bias[1], hidden_gyro_bias[1], 0.002f);
    /* Z gyro bias (yaw) — now observable with magnetometer */
    ASSERT_NEAR(ekf.state.gyro_bias[2], hidden_gyro_bias[2], 0.003f);
    /* Yaw covariance must have reduced significantly from initial 0.05 */
    ASSERT_TRUE(ekf.P[2*15 + 2] < 0.04f);
    /* Full covariance must remain finite */
    ASSERT_TRUE(all_finite(ekf.P, 225));

    printf("  OK\n");
}

void run_fusion_tests(void)
{
    printf("\n=== EKF Fusion Suite (stationary / GPS / Baro / Mag) ===\n");
    test_stationary_bias_convergence();
    test_gps_baro_fusion();
    test_mag_fusion();
}
