/**
 * @file test_csv_runner.c
 * @brief EKF Software-In-the-Loop (SIL) runner against real sensor CSV data.
 *
 * Reads a real sensor stream CSV, drives the full EKF pipeline, and writes
 * ekf_output.csv to the tests/ directory for analysis with plot_csv_results.py.
 *
 * Features:
 *  - PX4-style trapezoidal IMU pre-integration (averages prev+curr sample × dt)
 *    to minimise aliasing/drift during high-vibration engine burns.
 *  - Sensor isolation flags: toggle GPS, baro, and mag independently to
 *    isolate divergence sources without modifying any EKF code.
 *
 * Build target:   cmake --build build_host --target run_csv_runner
 * Output:         src/modules/estimator/tests/ekf_output.csv
 * Visualise with: python3 src/modules/estimator/tests/plot_csv_results.py
 *    (run from the OBC/ workspace root)
 *
 * Input CSV path is relative to the tests/ directory where this binary runs.
 * Adjust SENSOR_CSV_PATH if your GSU data lives elsewhere.
 *
 * CSV columns expected (22 fields):
 *   time, pressure_pa, temp_c, temp_stack,
 *   accel_x, accel_y, accel_z,
 *   gyro_x, gyro_y, gyro_z,
 *   mag_x, mag_y, mag_z,
 *   lat, lon, alt, vel_n, vel_e, vel_d,
 *   itow, week, flags
 */

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <stdbool.h>

#include "../ekf_core.h"
#include "../ekf_state.h"
#include "../fusion/baro_fuse.h"
#include "../fusion/gps_fuse.h"
#include "../fusion/mag_fuse.h"

// Explicitly define for the linter if it's missing them despite stdint.h
#ifndef uint32_t
typedef unsigned int uint32_t;
#endif
#ifndef uint64_t
typedef unsigned long long uint64_t;
#endif

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// ============================================================================
// EKF SIL SIMULATION CONFIGURATION
// Toggle these flags to isolate sensors and test pure dead-reckoning.
// ============================================================================
// Fixed linting: ensured stdint.h is at the very top.
#define FUSE_GPS  true   // Set false to dead-reckon position/velocity
#define FUSE_BARO false  // Set false to ignore pressure altitude
#define FUSE_MAG  false // Set false to bypass pad heading alignment
// ============================================================================

// Path to the input sensor stream CSV, relative to the tests/ working directory.
// Binary runs from OBC/src/modules/estimator/tests/, and you moved the CSV there.
#define SENSOR_CSV_PATH "phoenix_sensor_stream.csv"
#define OUTPUT_CSV_PATH "ekf_output.csv"
#define OUTPUT_LOG_PATH  "plots/csv_runner_log.txt"

// ── Stellenbosch WMM Magnetic Reference (Gauss, NED) ─────────────────────────
// X: North, Y: East, Z: Down. Update via WMM calculator for your launch site.
static const float MAG_REF_NED[3] = { 0.2337f, -0.0456f, 0.0551f };

// ── Launch rail geometry ──────────────────────────────────────────────────────
// Used when FUSE_MAG=false to bypass heading alignment. 80° elevation, True North.
#define LAUNCH_RAIL_ELEVATION_RAD 1.39626f   // 80 degrees
#define LAUNCH_RAIL_AZIMUTH_RAD   0.0f       // True North

// ── EKF step rate ─────────────────────────────────────────────────────────────
// Run the EKF predict step when accumulated dt exceeds this threshold (≈250 Hz).
#define EKF_STEP_DT_S  0.0039f

// ── GPS measurement rate ──────────────────────────────────────────────────────
#define GPS_RATE_S  0.1f   // 10 Hz

// ─────────────────────────────────────────────────────────────────────────────

static float pressure_to_altitude(float pressure_pa) {
    return 44330.0f * (1.0f - powf(pressure_pa / 101325.0f, 0.190295f));
}

static void quat_to_euler(const float q[4], float *roll, float *pitch, float *yaw) {
    *roll  = atan2f(2.0f * (q[0]*q[1] + q[2]*q[3]), 1.0f - 2.0f * (q[1]*q[1] + q[2]*q[2]));
    *pitch = asinf( 2.0f * (q[0]*q[2] - q[3]*q[1]));
    *yaw   = atan2f(2.0f * (q[0]*q[3] + q[1]*q[2]), 1.0f - 2.0f * (q[2]*q[2] + q[3]*q[3]));
}

int main(void) {
    ekf_core_t      ekf;
    ekf_state_ctx_t state_ctx;

    ekf_core_init(&ekf);
    ekf_state_init(&state_ctx, MAG_REF_NED, 0.004f);

    if (!FUSE_MAG) {
        // Without the magnetometer we cannot find North via HEADING_ALIGN.
        // Bypass that phase by hard-locking the attitude to the known launch rail
        // geometry; the state machine is fast-forwarded to ZVU_CALIBRATING so
        // stationary-update bias estimation still runs normally.
        printf("Mag disabled: locking launch rail to %.1f deg elevation.\n",
               LAUNCH_RAIL_ELEVATION_RAD * (180.0f / (float)M_PI));
        ekf_state_set_launch_rail(&state_ctx, &ekf,
                                  LAUNCH_RAIL_ELEVATION_RAD,
                                  LAUNCH_RAIL_AZIMUTH_RAD);
    }

    FILE *fin = fopen(SENSOR_CSV_PATH, "r");
    if (!fin) {
        fprintf(stderr, "Error: could not open '%s'.\n"
                        "       Adjust SENSOR_CSV_PATH in test_csv_runner.c\n"
                        "       or ensure the GSU data file is present.\n",
                SENSOR_CSV_PATH);
        return 1;
    }

    FILE *fout = fopen(OUTPUT_CSV_PATH, "w");
    if (!fout) {
        fprintf(stderr, "Error: could not open output '%s' for writing.\n", OUTPUT_CSV_PATH);
        fclose(fin);
        return 1;
    }

    FILE *flog = fopen(OUTPUT_LOG_PATH, "w");
    if (!flog) {
        fprintf(stderr, "Warning: could not open log '%s' — terminal output will not be saved.\n",
                OUTPUT_LOG_PATH);
    }

    fprintf(fout, "time,flight_mode,"
                  "p_N,p_E,p_D,v_N,v_E,v_D,"
                  "roll,pitch,yaw,bg_x,bg_y,bg_z,ba_x,ba_y,ba_z,"
                  "std_pN,std_pE,std_pD,std_vD,std_yaw,"
                  "baro_innov,baro_ratio,baro_rej,"
                  "gps_innov_pD,gps_ratio_pD,gps_rej\n");

    char line[512];
    fgets(line, sizeof(line), fin); // Skip header row

    float prev_time    = -1.0f;
    float last_gps_time = -1.0f;

    // ── GPS Tracking Variables for Logging ──
    bool   home_set     = false;
    double home_lat_rad = 0.0;
    double home_lon_rad = 0.0;
    double home_lat_deg = 0.0;
    double home_lon_deg = 0.0;
    float  home_alt_m   = 0.0f;

    float  latest_gps_ned[3] = {0.0f, 0.0f, 0.0f};
    double latest_lat = 0.0;
    double latest_lon = 0.0;
    float  latest_alt = 0.0f;

    // --- Relax GPS parameters for this specific noisy receiver ---
    for (int i = 0; i < 3; i++) {
        ekf.params.gps_pos_var[i] = 25.0f; // Expect 5m std deviation (5^2 = 25)
        ekf.params.gps_vel_var[i] = 10.0f; // Expect 3.1m/s std deviation
    }
    // Widen the Chi-Squared gate from the default 3-sigma to 5-sigma
    ekf.params.gps_pos_gate = 5.0f;
    ekf.params.gps_vel_gate = 5.0f;

    // ── PX4-style trapezoidal integrator state ────────────────────────────────
    // 1000 Hz raw samples are averaged (prev + curr) × dt before accumulation,
    // matching the PX4 SensorIntegrator approach for aliasing reduction.
    imu_history_t imu_accum    = {0};
    float         prev_gyro[3] = {0};
    float         prev_accel[3]= {0};   // stored as specific force (sign-inverted)
    bool          first_sample = true;
    uint32_t      step_count   = 0;     // counts 250 Hz EKF steps; used to throttle terminal output

    while (fgets(line, sizeof(line), fin)) {
        float    time, pressure_pa, temp_c, temp_stack;
        float    accel[3], gyro[3], mag[3];
        double   lat, lon;
        float    alt, vel_n, vel_e, vel_d;
        uint32_t itow, week, flags;

        int parsed = sscanf(line,
               "%f,%f,%f,%f,"
               "%f,%f,%f,"
               "%f,%f,%f,"
               "%f,%f,%f,"
               "%lf,%lf,%f,%f,%f,%f,"
               "%u,%u,%u",
               &time, &pressure_pa, &temp_c, &temp_stack,
               &accel[0], &accel[1], &accel[2],
               &gyro[0],  &gyro[1],  &gyro[2],
               &mag[0],   &mag[1],   &mag[2],
               &lat, &lon, &alt, &vel_n, &vel_e, &vel_d,
               &itow, &week, &flags);

        if (parsed < 22) continue;

        if (first_sample) {
            for (int i = 0; i < 3; i++) {
                prev_gyro[i]  = gyro[i];
                prev_accel[i] = -accel[i]; // Accelerometer sign: negate for specific force
            }
            prev_time     = time;
            last_gps_time = time;
            first_sample  = false;
            continue;
        }

        float dt = time - prev_time;
        if (dt <= 0.0f) continue;

        // ── 1. Trapezoidal IMU pre-integration (1000 Hz → 250 Hz) ────────────
        // Accumulate using the trapezoidal rule: area = 0.5 * (prev + curr) * dt.
        // This exactly halves the aliasing error vs. a simple rectangular (Euler)
        // integration and mirrors how PX4's SensorIntegrator works.
        imu_accum.dt_s += dt;

        for (int i = 0; i < 3; i++) {
            float sf = -accel[i]; // Specific force: negate to remove gravity convention

            imu_accum.delta_angle[i]    += 0.5f * (prev_gyro[i]  + gyro[i]) * dt;
            imu_accum.delta_velocity[i] += 0.5f * (prev_accel[i] + sf)      * dt;

            prev_gyro[i]  = gyro[i];
            prev_accel[i] = sf;
        }

        // ── 2. Run EKF predict + update at ≈250 Hz ───────────────────────────
        if (imu_accum.dt_s >= EKF_STEP_DT_S) {
            imu_accum.timestamp_us = (uint64_t)(time * 1e6);

            // Magnetometer in Gauss (CSV stores in µT → ×0.01 → Gauss)
            float mag_gauss[3] = {
                mag[0] / 100.0f,
                mag[1] / 100.0f,
                mag[2] / 100.0f,
            };

            // Advance the state machine (handles LEVELING / HEADING_ALIGN / ZVU).
            // Pass FUSE_MAG as mag_valid — suppresses mag fusion when disabled.
            ekf_state_update(&state_ctx, &ekf, accel, gyro, mag_gauss,
                             FUSE_MAG, imu_accum.dt_s);

            // ── ROCKET DYNAMICS FIX: Bias Locking & Boost Noise ───────────────
            static bool flight_biases_locked = false;
            
            if (state_ctx.mode == 4) { // FLIGHT MODE
                // 1. Lock biases at liftoff so scale-factor isn't absorbed as bias
                if (!flight_biases_locked) {
                    flight_biases_locked = true;
                    // Mathematically decouple biases from the covariance matrix
                    for (int i = 0; i < 3; i++) {
                        for (int j = 0; j < 15; j++) {
                            ekf.P[(9+i)*15 + j]  = 0.0f; // Gyro bias row
                            ekf.P[j*15 + (9+i)]  = 0.0f; // Gyro bias col
                            ekf.P[(12+i)*15 + j] = 0.0f; // Accel bias row
                            ekf.P[j*15 + (12+i)] = 0.0f; // Accel bias col
                        }
                        // Set tiny variance to keep matrix invertible but practically static
                        ekf.P[(9+i)*15 + (9+i)]   = 1e-8f;  
                        ekf.P[(12+i)*15 + (12+i)] = 1e-8f;
                    }
                    printf("\n>>> LIFTOFF DETECTED: IMU Biases Locked for Flight <<<\n\n");
                }

                // 2. Inflate accel noise during boost to encompass scale-factor error.
                // This ensures the 3-sigma bounds grow wide enough to accept the GPS at burnout.
                float acc_mag = sqrtf(accel[0]*accel[1] + accel[1]*accel[1] + accel[2]*accel[2]);
                if (acc_mag > 25.0f) {
                    ekf.params.accel_noise_var[0] = 5.0f; 
                    ekf.params.accel_noise_var[1] = 5.0f;
                    ekf.params.accel_noise_var[2] = 5.0f;
                } else {
                    ekf.params.accel_noise_var[0] = 0.05f; // Normal coast phase noise
                    ekf.params.accel_noise_var[1] = 0.05f;
                    ekf.params.accel_noise_var[2] = 0.05f;
                }
            }
            // ──────────────────────────────────────────────────────────────────

            // Propagate EKF with the accumulated delta-angle / delta-velocity.
            ekf_core_step(&ekf, &imu_accum);

            // ── 3. GPS measurement update at 10 Hz ───────────────────────────
            float acc_mag = sqrtf(accel[0]*accel[0] + accel[1]*accel[1] + accel[2]*accel[2]);
            bool is_boosting = (acc_mag > 15.0f); // Mask GPS/Baro if > 2.5 G's

            if (FUSE_GPS && (time - last_gps_time >= GPS_RATE_S) && !is_boosting) {
                const double R_EARTH = 6378137.0; // WGS-84 equatorial radius in meters
                const double DEG2RAD = M_PI / 180.0;

                // GUARD: Only latch home if the GPS has a valid fix.
                if (!home_set && lat != 0.0 && lon != 0.0) {
                    home_lat_deg = lat;
                    home_lon_deg = lon;
                    home_lat_rad = lat * DEG2RAD;
                    home_lon_rad = lon * DEG2RAD;
                    home_alt_m   = alt;
                    home_set = true;
                    
                    // Print a massive banner so it's obvious exactly where and what latched
                    char home_msg[512];
                    snprintf(home_msg, sizeof(home_msg), 
                        "\n================================================================================\n"
                        ">>> GPS HOME LATCHED: Lat %.6f, Lon %.6f, Alt %.1fm\n"
                        "================================================================================\n\n", 
                        lat, lon, alt);
                    printf("%s", home_msg);
                    if (flog) fprintf(flog, "%s", home_msg);
                }

                if (home_set) {
                    double lat_rad = lat * DEG2RAD;
                    double lon_rad = lon * DEG2RAD;

                    // Flat-earth approximation
                    float pN = (float)((lat_rad - home_lat_rad) * R_EARTH);
                    float pE = (float)((lon_rad - home_lon_rad) * R_EARTH * cos(home_lat_rad));
                    float rel_alt = alt - home_alt_m;

                    // Save to latest for the logger
                    latest_gps_ned[0] = pN;
                    latest_gps_ned[1] = pE;
                    latest_gps_ned[2] = -rel_alt;
                    latest_lat = lat;
                    latest_lon = lon;
                    latest_alt = alt;

                    float pos_ned[3] = { pN, pE, -rel_alt };
                    float vel_ned[3] = { vel_n, vel_e, vel_d };
                    
                    uint64_t tail_time = ekf.imu_buffer[ekf.tail_idx].timestamp_us;
                    ekf_core_push_gps(&ekf, tail_time, pos_ned, vel_ned);
                }
                last_gps_time = time;
            }

            // ── 4. Barometer measurement update ──────────────────────────────
            if (FUSE_BARO && ekf_state_is_flight_ready(&state_ctx) && !is_boosting) {
                baro_measurement_t baro_m;
                baro_m.altitude_m = pressure_to_altitude(pressure_pa);
                baro_fuse(&ekf, &baro_m);
            }

            // ── 5. Log output row ─────────────────────────────────────────────
            float roll, pitch, yaw;
            quat_to_euler(ekf.head_state.q, &roll, &pitch, &yaw);

            /* Extract σ from P diagonal (row-major: P[row*15 + col]).
             * Index map: att=0-2, vel=3-5, pos=6-8, gyro_bias=9-11, accel_bias=12-14 */
            float std_pN  = sqrtf(ekf.P[6*15 + 6]);
            float std_pE  = sqrtf(ekf.P[7*15 + 7]);
            float std_pD  = sqrtf(ekf.P[8*15 + 8]);
            float std_vD  = sqrtf(ekf.P[5*15 + 5]);
            float std_yaw = sqrtf(ekf.P[2*15 + 2]);

            fprintf(fout, "%.3f,%d,"
                          "%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,"
                          "%.3f,%.3f,%.3f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,"
                          "%.4f,%.4f,%.4f,%.4f,%.4f,"
                          "%.3f,%.3f,%d,"
                          "%.3f,%.3f,%d\n",
                    time, state_ctx.mode,
                    ekf.head_state.p_ned[0], ekf.head_state.p_ned[1], ekf.head_state.p_ned[2],
                    ekf.head_state.v_ned[0], ekf.head_state.v_ned[1], ekf.head_state.v_ned[2],
                    roll, pitch, yaw,
                    ekf.head_state.gyro_bias[0], ekf.head_state.gyro_bias[1], ekf.head_state.gyro_bias[2],
                    ekf.head_state.accel_bias[0], ekf.head_state.accel_bias[1], ekf.head_state.accel_bias[2],
                    std_pN, std_pE, std_pD, std_vD, std_yaw,
                    ekf.debug.baro_innov, ekf.debug.baro_test_ratio, (int)ekf.debug.baro_rejected,
                    ekf.debug.gps_innov_pos[2], ekf.debug.gps_test_ratio_pos[2], (int)ekf.debug.gps_rejected);

            // ── Throttled Terminal Readout (0.5 Hz) ──────────────────────────
            static float last_print_time = -2.0f;
            if (time - last_print_time >= 2.0f) {
                last_print_time = time;

                const char *mode_str =
                    (state_ctx.mode == 0) ? "UNINIT" :
                    (state_ctx.mode == 1) ? "LEVEL"  :
                    (state_ctx.mode == 2) ? "ALIGN"  :
                    (state_ctx.mode == 3) ? "ZVU"    : "FLIGHT";

                char log_buffer[2048];
                snprintf(log_buffer, sizeof(log_buffer),
                    "[%7.2f] MODE: %-6s\n"
                    "  RAW SENS: ACC:%7.3f %7.3f %7.3f  GYR:%7.4f %7.4f %7.4f  BARO: %8.1f Pa\n"
                    "  GPS RAW : Lat: %10.6f  Lon: %10.6f  Alt: %6.1f m  (HOME: %10.6f, %10.6f, %6.1f m)\n"
                    "  GPS NED : N: %8.2f m  E: %8.2f m  D: %8.2f m\n"
                    "  EKF NED : N: %8.2f m  E: %8.2f m  D: %8.2f m  (ALT: %.1f m)\n"
                    "  EKF VEL : VN: %7.2f m/s  VE: %7.2f m/s  VD: %7.2f m/s  YAW: %6.1f deg\n"
                    "  EKF BIAS: GYR X:%7.4f Y:%7.4f Z:%7.4f  ACC X:%7.3f Y:%7.3f Z:%7.3f\n"
                    "  UNCERT  : sPN: %5.2f  sPE: %5.2f  sPD: %5.2f  sVD: %5.2f  sYAW: %5.2f\n"
                    "  INNOV   : POS N: %6.2fm (TR: %5.2f)  E: %6.2fm (TR: %5.2f)  D: %6.2fm (TR: %5.2f)\n"
                    "  VEL_TR  : VN_TR: %5.2f  VE_TR: %5.2f  VD_TR: %5.2f\n"
                    "  REJECTS : %s%s%s\n"
                    "--------------------------------------------------------------------------------\n",
                    time, mode_str,
                    accel[0], accel[1], accel[2], gyro[0], gyro[1], gyro[2], pressure_pa,
                    latest_lat, latest_lon, latest_alt, home_lat_deg, home_lon_deg, home_alt_m,
                    latest_gps_ned[0], latest_gps_ned[1], latest_gps_ned[2],
                    ekf.head_state.p_ned[0], ekf.head_state.p_ned[1], ekf.head_state.p_ned[2], -ekf.head_state.p_ned[2],
                    ekf.head_state.v_ned[0], ekf.head_state.v_ned[1], ekf.head_state.v_ned[2], yaw * 57.2958f,
                    ekf.head_state.gyro_bias[0], ekf.head_state.gyro_bias[1], ekf.head_state.gyro_bias[2],
                    ekf.head_state.accel_bias[0], ekf.head_state.accel_bias[1], ekf.head_state.accel_bias[2],
                    std_pN, std_pE, std_pD, std_vD, std_yaw,
                    ekf.debug.gps_innov_pos[0], ekf.debug.gps_test_ratio_pos[0],
                    ekf.debug.gps_innov_pos[1], ekf.debug.gps_test_ratio_pos[1],
                    ekf.debug.gps_innov_pos[2], ekf.debug.gps_test_ratio_pos[2],
                    ekf.debug.gps_test_ratio_vel[0], ekf.debug.gps_test_ratio_vel[1], ekf.debug.gps_test_ratio_vel[2],
                    ekf.debug.gps_rejected ? "G" : "-",
                    ekf.debug.baro_rejected ? "B" : "-",
                    ekf.debug.mag_rejected ? "M" : "-");

                printf("%s", log_buffer);
                if (flog) {
                    fprintf(flog, "%s", log_buffer);
                }
            }
            step_count++;

            // Reset accumulator for next EKF predict window
            imu_accum.dt_s = 0.0f;
            for (int i = 0; i < 3; i++) {
                imu_accum.delta_angle[i]    = 0.0f;
                imu_accum.delta_velocity[i] = 0.0f;
            }
        }

        prev_time = time;
    }

    fclose(fin);
    fclose(fout);

    printf("EKF CSV SIL complete.  Results → %s\n", OUTPUT_CSV_PATH);
    printf("  FUSE_GPS=%s  FUSE_BARO=%s  FUSE_MAG=%s\n",
           FUSE_GPS  ? "true" : "false",
           FUSE_BARO ? "true" : "false",
           FUSE_MAG  ? "true" : "false");

    if (flog) {
        fprintf(flog, "EKF CSV SIL complete.  Results → %s\n", OUTPUT_CSV_PATH);
        fprintf(flog, "  FUSE_GPS=%s  FUSE_BARO=%s  FUSE_MAG=%s\n",
                FUSE_GPS  ? "true" : "false",
                FUSE_BARO ? "true" : "false",
                FUSE_MAG  ? "true" : "false");
        fclose(flog);
        printf("[log] Saved → %s\n", OUTPUT_LOG_PATH);
    }
    return 0;
}
