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
                  "roll,pitch,yaw,bg_x,bg_y,bg_z,"
                  "std_pN,std_pE,std_pD,std_vD,std_yaw,"
                  "baro_innov,baro_ratio,baro_rej,"
                  "gps_innov_pD,gps_ratio_pD,gps_rej\n");

    char line[512];
    fgets(line, sizeof(line), fin); // Skip header row

    float prev_time    = -1.0f;
    float last_gps_time = -1.0f;

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

            // Propagate EKF with the accumulated delta-angle / delta-velocity.
            ekf_core_step(&ekf, &imu_accum);

            // ── 3. GPS measurement update at 10 Hz ───────────────────────────
            if (FUSE_GPS && (time - last_gps_time >= GPS_RATE_S)) {
                if (ekf_state_is_flight_ready(&state_ctx)) {
                    
                    // --- Convert Lat/Lon to local NED ---
                    static bool home_set = false;
                    static double home_lat_rad = 0.0;
                    static double home_lon_rad = 0.0;
                    const double R_EARTH = 6378137.0; // WGS-84 equatorial radius in meters
                    const double DEG2RAD = M_PI / 180.0;

                    // Latch the first valid coordinate as the launch pad origin
                    if (!home_set) {
                        home_lat_rad = lat * DEG2RAD;
                        home_lon_rad = lon * DEG2RAD;
                        home_set = true;
                        printf("\n>>> GPS Home origin latched. Fusing incoming fixes...\n\n");
                    }

                    double lat_rad = lat * DEG2RAD;
                    double lon_rad = lon * DEG2RAD;

                    // Flat-earth approximation (valid for local flights < 50km)
                    float pN = (float)((lat_rad - home_lat_rad) * R_EARTH);
                    float pE = (float)((lon_rad - home_lon_rad) * R_EARTH * cos(home_lat_rad));
                    // ------------------------------------

                    float pos_ned[3] = { pN, pE, -alt };
                    float vel_ned[3] = { vel_n, vel_e, vel_d };
                    
                    // Fetch the tail timestamp to bypass the 1-element queue limitation
                    uint64_t tail_time = ekf.imu_buffer[ekf.tail_idx].timestamp_us;
                    
                    ekf_core_push_gps(&ekf, tail_time, pos_ned, vel_ned);
                }
                last_gps_time = time;
            }

            // ── 4. Barometer measurement update ──────────────────────────────
            if (FUSE_BARO && ekf_state_is_flight_ready(&state_ctx)) {
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
                          "%.3f,%.3f,%.3f,%.5f,%.5f,%.5f,"
                          "%.4f,%.4f,%.4f,%.4f,%.4f,"
                          "%.3f,%.3f,%d,"
                          "%.3f,%.3f,%d\n",
                    time, state_ctx.mode,
                    ekf.head_state.p_ned[0], ekf.head_state.p_ned[1], ekf.head_state.p_ned[2],
                    ekf.head_state.v_ned[0], ekf.head_state.v_ned[1], ekf.head_state.v_ned[2],
                    roll, pitch, yaw,
                    ekf.head_state.gyro_bias[0], ekf.head_state.gyro_bias[1], ekf.head_state.gyro_bias[2],
                    std_pN, std_pE, std_pD, std_vD, std_yaw,
                    ekf.debug.baro_innov, ekf.debug.baro_test_ratio, (int)ekf.debug.baro_rejected,
                    ekf.debug.gps_innov_pos[2], ekf.debug.gps_test_ratio_pos[2], (int)ekf.debug.gps_rejected);

            // ── Throttled Terminal Readout (1Hz) ──────────────────────────
            static float last_print_time = -1.0f;
            if (time - last_print_time >= 1.0f) {
                last_print_time = time;

                const char *mode_str =
                    (state_ctx.mode == 0) ? "UNINIT" :
                    (state_ctx.mode == 1) ? "LEVEL"  :
                    (state_ctx.mode == 2) ? "ALIGN"  :
                    (state_ctx.mode == 3) ? "ZVU"    : "FLIGHT";

                // Buffer the formatted string to print to both console and log
                char log_buffer[1024];
                snprintf(log_buffer, sizeof(log_buffer),
                    "[%7.2f] MODE:%-6s dt:%.4fs\n"
                    "  SENSORS | ACC:%7.2f %7.2f %7.2f m/s2  GYR:%7.3f %7.3f %7.3f rad/s  BARO:%8.1f Pa\n"
                    "  STATE   | ALT:%7.1f m  VD:%6.2f m/s  YAW:%6.1f deg  REJ:%s%s%s\n"
                    "  UNCERT  | σ_PN:%.2f σ_PE:%.2f σ_PD:%.2f σ_VD:%.2f σ_YAW:%.2f\n"
                    "--------------------------------------------------------------------------------\n",
                    time, mode_str, imu_accum.dt_s,
                    accel[0], accel[1], accel[2], gyro[0], gyro[1], gyro[2], pressure_pa,
                    -ekf.head_state.p_ned[2], ekf.head_state.v_ned[2], yaw * 57.2958f,
                    ekf.debug.gps_rejected  ? "G" : "-",
                    ekf.debug.baro_rejected ? "B" : "-",
                    ekf.debug.mag_rejected  ? "M" : "-",
                    std_pN, std_pE, std_pD, std_vD, std_yaw);

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
