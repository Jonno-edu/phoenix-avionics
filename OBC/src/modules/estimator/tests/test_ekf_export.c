/**
 * @file test_ekf_export.c
 * @brief Mission-profile CSV exporter for EKF visualisation and tuning.
 *
 * This is a standalone program (its own main) that runs the same scripted
 * 60-second pad-to-apogee mission as test_mission_profile.c but writes every
 * timestep to a CSV instead of running pass/fail assertions.
 *
 * Build target:   cmake --build build_host --target run_ekf_export
 * Output:         build_host/ekf_mission_data.csv
 * Visualise with: python3 scripts/plot_ekf_mission.py
 *
 * CSV columns  (16 total)
 * ───────────────────────────────────────────────────────────────────────────
 *  time_s    — simulation time (s)
 *  mode      — EKF flight mode (0=UNINIT … 4=FLIGHT)
 *  true_pz   — ground-truth NED Z position, down+ (m)
 *  ekf_pz    — EKF NED Z position estimate (m)
 *  cov_pz    — P[8,8] — Z-position variance (m²)
 *  true_vz   — ground-truth NED Z velocity (m/s)
 *  ekf_vz    — EKF NED Z velocity estimate (m/s)
 *  cov_vz    — P[5,5] — Z-velocity variance (m²/s²)
 *  true_abx  — true accel bias X (m/s²)
 *  ekf_abx   — EKF accel bias X estimate (m/s²)
 *  true_gbx  — true gyro bias X (rad/s)
 *  ekf_gbx   — EKF gyro bias X estimate (rad/s)
 *  raw_baro  — raw baro altitude (m, positive-up); NaN on non-baro steps
 *             includes 200 m fault injected at T=30–31 s
 *  baro_chi2 — baro χ² = ν²/S; NaN on non-baro steps
 *  gps_chi2  — GPS Z-pos χ² = ν²/S; NaN on non-GPS steps (includes lock-loss
 *             gap during BURN + 5 s re-acquisition window at start of COAST)
 *  gps_lock  — 1 = receiver has satellite lock, 0 = tracking loops lost
 *             (models >4 g lock-loss and 5 s hot-start re-acquisition)
 */

#include <stdio.h>
#include <math.h>
#include <stdbool.h>
#include <string.h>

#include "test_utils.h"
#include "ekf_state.h"
#include "fusion/imu_predictor.h"
#include "fusion/baro_fuse.h"
#include "fusion/gps_fuse.h"

/* ── Must mirror the private constants in baro_fuse.c / gps_fuse.c ─────────── */
#define EXPORT_BARO_NOISE_VAR   0.25f    /* (0.5 m)²  — must match baro_fuse.c */
#define EXPORT_BARO_INNOV_GATE  5.0f     /* metres    — must match baro_fuse.c */
#define EXPORT_GPS_POS_VAR_Z    2.25f    /* (1.5 m)²  — must match gps_fuse.c  */
#define EXPORT_GPS_POS_GATE     20.0f    /* metres    — must match gps_fuse.c   */
#define EXPORT_GPS_VEL_GATE      5.0f    /* m/s       — must match gps_fuse.c   */
#define EXPORT_GPS_VEL_VAR_Z     0.01f   /* (0.1 m/s)² — must match gps_fuse.c */

/* ── Stellenbosch WMM reference field (NED, Gauss) ─────────────────────────── */
static const float SB_MAG_NED[3] = { 0.15f, -0.05f, -0.33f };

#define G_MS2 9.80665f

/* EKF 15-state index layout (row-major P[i*15+j])
 *   0..2   — attitude error (δθ)
 *   3..5   — velocity NED (vx, vy, vz)   ← vz at index 5
 *   6..8   — position NED (px, py, pz)   ← pz at index 8
 *   9..11  — gyro bias
 *  12..14  — accel bias                  ← bx=12, by=13, bz=14
 */
#define IDX_VZ   5
#define IDX_PZ   8
#define IDX_GBX  9   /* gyro bias X  */
#define IDX_BX  12
#define IDX_BY  13
#define IDX_BZ  14

/* Convenience macro for covariance diagonal element */
#define P_DIAG(ekf, i)  ((ekf).P[(i)*15 + (i)])

/* NaN sentinel written to CSV columns when no measurement arrived this step */
#define CSV_NAN "nan"

/* ── Main export function ───────────────────────────────────────────────────── */

static void run_ekf_export(void)
{
    /* ----- EKF initialisation -------------------------------------------- */
    ekf_core_t      ekf;
    ekf_state_ctx_t ctx;
    ekf_core_init(&ekf);
    ekf_state_init(&ctx, SB_MAG_NED, 0.004f);
    ekf_state_set_launch_rail(&ctx, &ekf, 0.0f, 0.0f);

    const float dt = 0.004f;   /* 250 Hz */

    /* Residual run-to-run biases the EKF must estimate */
    const float gyro_bias_true[3]  = { 0.003f, -0.001f,  0.005f };  /* rad/s */
    const float accel_bias_true[3] = { 0.02f,  -0.03f,   0.08f  };  /* m/s² */

    /* Ground truth kinematics in NED (Z down) */
    float true_pos[3] = { 0.0f, 0.0f, 0.0f };
    float true_vel[3] = { 0.0f, 0.0f, 0.0f };

    /* GPS state machine — models receiver tracking-loop dynamics.
     * Limits (u-blox / CoCom): ≤ 4 g dynamics, ≤ 500 m/s, ≤ 80,000 m alt. */
    bool  gps_has_lock         = true;
    float time_since_lock_lost = 0.0f;

    /* Phase durations (match test_mission_profile.c exactly) */
    const int pad_steps   = (int)(15.0f / dt);   /* 3750  — 15 s static pad */
    const int burn_steps  = (int)( 5.0f / dt);   /* 1250  — 5 s motor burn  */
    const int coast_steps = (int)(40.0f / dt);   /* 10000 — 40 s freefall   */
    const int total_steps = pad_steps + burn_steps + coast_steps;

    /* ----- Open CSV ------------------------------------------------------ */
    const char *csv_path = "ekf_mission_data.csv";
    FILE *csv = fopen(csv_path, "w");
    if (!csv) {
        fprintf(stderr, "[export] ERROR: cannot open %s for writing\n", csv_path);
        return;
    }

    fprintf(csv,
            "time_s,mode,"
            "true_pz,ekf_pz,cov_pz,"
            "true_vz,ekf_vz,cov_vz,"
            "true_abx,ekf_abx,"
            "true_gbx,ekf_gbx,"
            "raw_baro,baro_chi2,gps_chi2,gps_lock\n");

    printf("[export] Starting EKF mission export (%d steps, %.0f s)...\n",
           total_steps, (double)(total_steps * dt));

    /* ----- Main simulation loop ------------------------------------------ */
    for (int step = 0; step < total_steps; step++) {

        /* Time at the start of this step — needed for fault injection below */
        float t_s = (float)step * dt;

        /* ── Ground-truth physics ──────────────────────────────────────── */
        float true_accel_ned[3] = { 0.0f, 0.0f, 0.0f };
        float specific_force[3] = { 0.0f, 0.0f, 0.0f };
        float vib_noise_std     = 0.05f;
        bool  send_mag          = false;
        bool  send_gps          = false;
        bool  send_baro         = false;

        if (step < pad_steps) {
            /* PAD: stationary — gravity reaction (specific force = -g body-Z) */
            specific_force[2] = -G_MS2;
            vib_noise_std     = 0.05f;
            send_mag          = (step % 50 == 0);     /* 5 Hz */
        }
        else if (step < pad_steps + burn_steps) {
            /* BURN: 5g net upward acceleration */
            true_accel_ned[2] = -5.0f * G_MS2;       /* NED Z: upward = negative */
            specific_force[2] = -6.0f * G_MS2;       /* Thrust reaction in body  */
            vib_noise_std     = 2.5f;                 /* SRM vibration            */
        }
        else {
            /* COAST: free-fall (specific force ≈ 0, gravity dominates NED) */
            true_accel_ned[2] = +G_MS2;
            specific_force[2] = 0.0f;
            vib_noise_std     = 0.1f;
            send_gps          = (step % 25 == 0);    /* 10 Hz (gated below) */
            send_baro         = (step % 5  == 0);    /* 50 Hz */
        }

        /* ── GPS tracking-loop state machine ──────────────────────────────
         * Models: >4 g causes loss of lock; re-acquisition requires dynamics
         * to fall below threshold AND a 5-second tracking-loop recovery delay.
         * CoCom altitude ceiling (80 km) and velocity ceiling (500 m/s) are
         * enforced as hard lock-loss conditions as well.
         */
        {
            float true_accel_g = sqrtf(
                true_accel_ned[0]*true_accel_ned[0] +
                true_accel_ned[1]*true_accel_ned[1] +
                true_accel_ned[2]*true_accel_ned[2]) / G_MS2;
            float true_vel_mag = sqrtf(
                true_vel[0]*true_vel[0] +
                true_vel[1]*true_vel[1] +
                true_vel[2]*true_vel[2]);
            float true_altitude = -true_pos[2];   /* NED Z-down → positive-up */

            if (gps_has_lock) {
                if (true_accel_g > 4.0f ||
                    true_vel_mag  > 500.0f ||
                    true_altitude > 80000.0f) {
                    gps_has_lock         = false;
                    time_since_lock_lost = 0.0f;
                }
            } else {
                /* Re-acquire once dynamics and velocity drop below thresholds */
                if (true_accel_g  <= 4.0f &&
                    true_vel_mag   < 505.0f &&
                    true_altitude  < 80000.0f) {
                    time_since_lock_lost += dt;
                    if (time_since_lock_lost > 5.0f) {
                        gps_has_lock = true;
                    }
                }
            }
        }
        /* Gate GPS transmission on lock status */
        send_gps = send_gps && gps_has_lock;

        /* Integrate true kinematics */
        for (int ax = 0; ax < 3; ax++) {
            true_vel[ax] += true_accel_ned[ax] * dt;
            true_pos[ax] += true_vel[ax]        * dt;
        }

        /* ── IMU measurement ───────────────────────────────────────────── */
        float gyro_meas[3], accel_meas[3];
        for (int ax = 0; ax < 3; ax++) {
            gyro_meas[ax]  = gyro_bias_true[ax]
                             + generate_gaussian_noise(0.0f, 0.001f);
            accel_meas[ax] = specific_force[ax]
                             + accel_bias_true[ax]
                             + generate_gaussian_noise(0.0f, vib_noise_std);
        }

        imu_history_t imu = {0};
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
            for (int ax = 0; ax < 3; ax++) {
                mag_body[ax] = SB_MAG_NED[ax]
                               + generate_gaussian_noise(0.0f, 0.005f);
            }
            mag_valid = true;
        }
        ekf_state_update(&ctx, &ekf, accel_meas, gyro_meas, mag_body, mag_valid, dt);

        /* ── Sensor fusion + innovation logging ────────────────────────── *
         * Innovations are computed BEFORE the Kalman update so they reflect
         * the pre-update residual (the standard definition of "innovation").
         * After the update the state would be partially corrected toward the
         * measurement, making the residual smaller and less informative.
         */

        /* --- Baro --- */
        /* raw_baro_alt is only sampled on send_baro steps (50 Hz) so that the
         * Gaussian noise process matches what the physical sensor would produce.
         * Non-baro steps write NaN to the CSV; the fault is injected on any
         * step that falls within the 30–31 s fault window.                  */
        float raw_baro_alt = 0.0f;   /* set below if send_baro */
        float baro_chi2    = 0.0f;
        bool  baro_valid   = false;

        if (send_baro) {
            raw_baro_alt = -true_pos[2] + generate_gaussian_noise(0.0f, 0.5f);

            /* INJECT FAULT: 200 m altitude drop at T=30–31 s.
             * NED Z is down+, so altitude = -p_ned[2].
             * Subtracting 200 m from reported altitude makes the sensor appear
             * to see the rocket suddenly 200 m lower than reality.          */
            if (t_s >= 30.0f && t_s < 31.0f) {
                raw_baro_alt -= 200.0f;
            }

            baro_measurement_t baro;
            baro.altitude_m = raw_baro_alt;

            /* Innovation: predicted alt = -p_ned[2] */
            float alt_pred   = -ekf.delayed_state.p_ned[2];
            float baro_innov = baro.altitude_m - alt_pred;

            /* χ² = ν² / S  where S = H·P·Hᵀ + R
             * H selects altitude = -p_ned[2], so H[IDX_PZ] = -1
             * H·P·Hᵀ = (-1)·P[IDX_PZ,IDX_PZ]·(-1) = P[IDX_PZ,IDX_PZ]    */
            float baro_S = P_DIAG(ekf, IDX_PZ) + EXPORT_BARO_NOISE_VAR;
            baro_chi2    = (baro_innov * baro_innov) / baro_S;
            baro_valid   = true;

            /* Gate check mirrors baro_fuse.c — only fuse if within gate */
            if (fabsf(baro_innov) <= EXPORT_BARO_INNOV_GATE) {
                baro_fuse(&ekf, &baro);
            }
        }

        /* --- GPS --- */
        float gps_chi2  = 0.0f;
        bool  gps_valid = false;

        if (send_gps) {
            gps_measurement_t gps;
            for (int ax = 0; ax < 3; ax++) {
                gps.pos_ned[ax] = true_pos[ax]
                                  + generate_gaussian_noise(0.0f, 1.5f);
                gps.vel_ned[ax] = true_vel[ax]
                                  + generate_gaussian_noise(0.0f, 0.1f);
            }

            /* Z-position innovation before update */
            float gps_innov_pz = gps.pos_ned[2] - ekf.delayed_state.p_ned[2];

            /* S = P[IDX_PZ,IDX_PZ] + GPS_POS_VAR[Z] */
            float gps_S_pz = P_DIAG(ekf, IDX_PZ) + EXPORT_GPS_POS_VAR_Z;
            gps_chi2  = (gps_innov_pz * gps_innov_pz) / gps_S_pz;
            gps_valid = true;

            /* Let gps_fuse handle its own full gate check and Kalman update */
            gps_fuse(&ekf, &gps);
        }

        /* ── Write CSV row ─────────────────────────────────────────────── */
        int mode = (int)ekf_state_get_mode(&ctx);

        fprintf(csv, "%.4f,%d,"
                "%.3f,%.3f,%.6f,"  /* true_pz, ekf_pz, cov_pz */
                "%.3f,%.3f,%.6f,"  /* true_vz, ekf_vz, cov_vz */
                "%.5f,%.5f,"       /* true_abx, ekf_abx */
                "%.5f,%.5f,",      /* true_gbx, ekf_gbx */
                (double)t_s, mode,
                (double)true_pos[2],        (double)ekf.delayed_state.p_ned[2],      (double)P_DIAG(ekf, IDX_PZ),
                (double)true_vel[2],        (double)ekf.delayed_state.v_ned[2],      (double)P_DIAG(ekf, IDX_VZ),
                (double)accel_bias_true[0], (double)ekf.delayed_state.accel_bias[0],
                (double)gyro_bias_true[0],  (double)ekf.delayed_state.gyro_bias[0]);

        /* raw_baro — NaN on non-baro steps */
        if (baro_valid) {
            fprintf(csv, "%.2f,", (double)raw_baro_alt);
        } else {
            fprintf(csv, "%s,", CSV_NAN);
        }

        /* baro_chi2 — NaN on non-baro steps */
        if (baro_valid) {
            fprintf(csv, "%.4f,", (double)baro_chi2);
        } else {
            fprintf(csv, "%s,", CSV_NAN);
        }

        /* gps_chi2 — NaN on non-GPS steps */
        if (gps_valid) {
            fprintf(csv, "%.4f,", (double)gps_chi2);
        } else {
            fprintf(csv, "%s,", CSV_NAN);
        }

        /* gps_lock — 1 while receiver has satellite lock, 0 during loss */
        fprintf(csv, "%d\n", gps_has_lock ? 1 : 0);
    }

    fclose(csv);

    printf("[export] Done. CSV written to: %s\n", csv_path);
    printf("[export] Rows: %d  Columns: 16  (200 m baro fault @ T=30-31s; GPS lock-loss model active)\n", total_steps);
    printf("[export] Visualise with: python3 scripts/plot_ekf_mission.py\n");
}

/* ── Entry point ─────────────────────────────────────────────────────────────── */

int main(void)
{
    run_ekf_export();
    return 0;
}
