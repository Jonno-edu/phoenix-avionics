// rocket_data.c

#include "core/rocket_data.h"
#include "telemetry.h"     // Defines SCALE_ macros and TrackingBeacon_t
#include <string.h>
#include <math.h>          // Required for lroundf()

// RP2040/RP2350 SDK Includes for Spinlocks
#if PICO_BUILD
#include "pico/stdlib.h"
#include "hardware/sync.h"
#endif

// The Master Copy of the Data
static TrackingBeacon_t _rocket_data;

#if PICO_BUILD
// Hardware Spinlock Handle
static spin_lock_t *s_rocket_data_lock = NULL;

// Macros for thread safety
#define LOCK_DATA()   uint32_t flags = spin_lock_blocking(s_rocket_data_lock)
#define UNLOCK_DATA() spin_unlock(s_rocket_data_lock, flags)

#else
// Host/Mock Build: No-op or simple mutex if needed
#define LOCK_DATA()
#define UNLOCK_DATA()
#endif

void rocket_data_init(void) {
    memset(&_rocket_data, 0, sizeof(_rocket_data));

#if PICO_BUILD
    // Claim a free hardware spinlock index
    // Note: This relies on the spinlock API being initialized by runtime
    int lock_num = spin_lock_claim_unused(true); 
    s_rocket_data_lock = spin_lock_init(lock_num);
#endif
}

// ============================================================================
// SETTERS (Input: SI Units -> Output: Telemetry Encoded Integers)
// Safe to call from any core, any task.
// ============================================================================

void rocket_data_update_gps(double lat_deg, double lon_deg, float alt_m, 
                            float vel_n_ms, float vel_e_ms, float vel_d_ms,
                            uint32_t itow, uint16_t week) {
    LOCK_DATA();
    // Position: Degrees / 1e-7 = Degrees * 10,000,000
    _rocket_data.gps_lat = (int32_t)(lat_deg / SCALE_POS_LATLON);
    _rocket_data.gps_lon = (int32_t)(lon_deg / SCALE_POS_LATLON);
    
    // Altitude: m / 0.001 = mm
    _rocket_data.gps_alt = (int32_t)lroundf(alt_m / SCALE_ALTITUDE);
    
    // Velocity: m/s / 0.1 = decimeters/s
    _rocket_data.gps_vel_n = (int16_t)lroundf(vel_n_ms / SCALE_VELOCITY);
    _rocket_data.gps_vel_e = (int16_t)lroundf(vel_e_ms / SCALE_VELOCITY);
    _rocket_data.gps_vel_d = (int16_t)lroundf(vel_d_ms / SCALE_VELOCITY);
    
    _rocket_data.itow = itow;     // Raw from GPS
    _rocket_data.gps_week = week; // Raw from GPS
    UNLOCK_DATA();
}

void rocket_data_update_accel(float ax_ms2, float ay_ms2, float az_ms2) {
    LOCK_DATA();
    _rocket_data.accel_x = (int16_t)lroundf(ax_ms2 / SCALE_ACCEL);
    _rocket_data.accel_y = (int16_t)lroundf(ay_ms2 / SCALE_ACCEL);
    _rocket_data.accel_z = (int16_t)lroundf(az_ms2 / SCALE_ACCEL);
    UNLOCK_DATA();
}

void rocket_data_update_gyro(float gx_rads, float gy_rads, float gz_rads) {
    LOCK_DATA();
    _rocket_data.gyro_x = (int16_t)lroundf(gx_rads / SCALE_GYRO);
    _rocket_data.gyro_y = (int16_t)lroundf(gy_rads / SCALE_GYRO);
    _rocket_data.gyro_z = (int16_t)lroundf(gz_rads / SCALE_GYRO);
    UNLOCK_DATA();
}

void rocket_data_update_mag(float mx_ut, float my_ut, float mz_ut) {
    LOCK_DATA();
    _rocket_data.mag_x = (int16_t)lroundf(mx_ut / SCALE_MAG);
    _rocket_data.mag_y = (int16_t)lroundf(my_ut / SCALE_MAG);
    _rocket_data.mag_z = (int16_t)lroundf(mz_ut / SCALE_MAG);
    UNLOCK_DATA();
}

void rocket_data_update_est_state(double lat_deg, double lon_deg, float alt_m,
                                  float vn_ms, float ve_ms, float vd_ms) {
    LOCK_DATA();
    _rocket_data.est_lat = (int32_t)(lat_deg / SCALE_POS_LATLON);
    _rocket_data.est_lon = (int32_t)(lon_deg / SCALE_POS_LATLON);
    _rocket_data.est_alt = (int32_t)lroundf(alt_m / SCALE_ALTITUDE);
    
    _rocket_data.est_vel_n = (int16_t)lroundf(vn_ms / SCALE_VELOCITY);
    _rocket_data.est_vel_e = (int16_t)lroundf(ve_ms / SCALE_VELOCITY);
    _rocket_data.est_vel_d = (int16_t)lroundf(vd_ms / SCALE_VELOCITY);
    UNLOCK_DATA();
}

void rocket_data_update_battery(float current_ma) {
    LOCK_DATA();
    // Simple cast, or lroundf if you want precision
    _rocket_data.battery_current = (uint16_t)current_ma; 
    UNLOCK_DATA();
}

void rocket_data_update_state(uint8_t state, uint8_t apogee_count) {
    LOCK_DATA();
    _rocket_data.avionics_state = state;
    _rocket_data.apogee_counter = apogee_count;
    UNLOCK_DATA();
}

void rocket_data_update_baro(float pressure_pa, float temp_c) {
    LOCK_DATA();
    // 101325 Pa / 0.015625 = 6,484,800 LSBs
    _rocket_data.baro_press = (uint32_t)lroundf(pressure_pa / SCALE_BARO_PRESS);
    // 25.00 C / 0.01 = 2500 LSBs
    _rocket_data.baro_temp = (int16_t)lroundf(temp_c / SCALE_BARO_TEMP);
    UNLOCK_DATA();
}

void rocket_data_update_temp_stack(float temp_c) {
    LOCK_DATA();
    _rocket_data.temp_stack = (int16_t)lroundf(temp_c / SCALE_TEMP_STACK);
    UNLOCK_DATA();
}

void rocket_data_update_runtime(uint16_t seconds, uint16_t milliseconds) {
    LOCK_DATA();
    _rocket_data.runtime_sec = seconds;
    _rocket_data.runtime_ms = milliseconds;
    UNLOCK_DATA();
}

void rocket_data_update_time_flags(uint8_t flags_in) {
    LOCK_DATA();
    _rocket_data.time_valid_flags = flags_in;
    UNLOCK_DATA();
}

void rocket_data_update_apogee_votes(uint8_t votes) {
    LOCK_DATA();
    // Unpack the byte into the bitfields
    _rocket_data.apogee_vote_1 = (votes >> 0) & 0x01;
    _rocket_data.apogee_vote_2 = (votes >> 1) & 0x01;
    _rocket_data.apogee_vote_3 = (votes >> 2) & 0x01;
    UNLOCK_DATA();
}

// ============================================================================
// GETTERS (Raw Struct Access for Telemetry)
// ============================================================================

void getRocketTrackingInfo(TrackingBeacon_t *out_beacon) {
    LOCK_DATA();
    memcpy(out_beacon, &_rocket_data, sizeof(TrackingBeacon_t));
    UNLOCK_DATA();
}

// ============================================================================
// SI UNIT GETTERS (Telemetry Encoded -> SI Output)
// ============================================================================

// IMU
float rocket_data_get_accel_x_si(void) { return _rocket_data.accel_x * SCALE_ACCEL; }
float rocket_data_get_accel_y_si(void) { return _rocket_data.accel_y * SCALE_ACCEL; }
float rocket_data_get_accel_z_si(void) { return _rocket_data.accel_z * SCALE_ACCEL; }

float rocket_data_get_gyro_x_si(void) { return _rocket_data.gyro_x * SCALE_GYRO; }
float rocket_data_get_gyro_y_si(void) { return _rocket_data.gyro_y * SCALE_GYRO; }
float rocket_data_get_gyro_z_si(void) { return _rocket_data.gyro_z * SCALE_GYRO; }

float rocket_data_get_mag_x_si(void) { return _rocket_data.mag_x * SCALE_MAG; }
float rocket_data_get_mag_y_si(void) { return _rocket_data.mag_y * SCALE_MAG; }
float rocket_data_get_mag_z_si(void) { return _rocket_data.mag_z * SCALE_MAG; }

// Baro 
float rocket_data_get_baro_press_si(void) { 
    LOCK_DATA(); 
    float val = (float)_rocket_data.baro_press * SCALE_BARO_PRESS; 
    UNLOCK_DATA();
    return val;
}
float rocket_data_get_baro_temp_si(void) { 
    LOCK_DATA(); 
    float val = (float)_rocket_data.baro_temp * SCALE_BARO_TEMP; 
    UNLOCK_DATA();
    return val;
}

// GPS
double rocket_data_get_gps_lat_si(void) { 
    LOCK_DATA(); 
    double val = _rocket_data.gps_lat * (double)SCALE_POS_LATLON; 
    UNLOCK_DATA();
    return val;
}
double rocket_data_get_gps_lon_si(void) { 
    LOCK_DATA(); 
    double val = _rocket_data.gps_lon * (double)SCALE_POS_LATLON; 
    UNLOCK_DATA();
    return val;
}
float  rocket_data_get_gps_alt_si(void) { 
    LOCK_DATA(); 
    float val = _rocket_data.gps_alt * SCALE_ALTITUDE; 
    UNLOCK_DATA();
    return val;
} 
float  rocket_data_get_gps_vel_n_si(void) { 
    LOCK_DATA(); 
    float val = _rocket_data.gps_vel_n * SCALE_VELOCITY; 
    UNLOCK_DATA();
    return val;
}
float  rocket_data_get_gps_vel_e_si(void) { 
    LOCK_DATA(); 
    float val = _rocket_data.gps_vel_e * SCALE_VELOCITY; 
    UNLOCK_DATA();
    return val;
}
float  rocket_data_get_gps_vel_d_si(void) { 
    LOCK_DATA(); 
    float val = _rocket_data.gps_vel_d * SCALE_VELOCITY; 
    UNLOCK_DATA();
    return val;
}

// Estimation
double rocket_data_get_est_lat_si(void) { 
    LOCK_DATA(); 
    double val = _rocket_data.est_lat * (double)SCALE_POS_LATLON; 
    UNLOCK_DATA();
    return val;
}
double rocket_data_get_est_lon_si(void) { 
    LOCK_DATA(); 
    double val = _rocket_data.est_lon * (double)SCALE_POS_LATLON; 
    UNLOCK_DATA();
    return val;
}
float  rocket_data_get_est_alt_si(void) { 
    LOCK_DATA(); 
    float val = _rocket_data.est_alt * SCALE_ALTITUDE; 
    UNLOCK_DATA();
    return val;
} 
float  rocket_data_get_est_vel_n_si(void) { 
    LOCK_DATA(); 
    float val = _rocket_data.est_vel_n * SCALE_VELOCITY; 
    UNLOCK_DATA();
    return val;
}
float  rocket_data_get_est_vel_e_si(void) { 
    LOCK_DATA(); 
    float val = _rocket_data.est_vel_e * SCALE_VELOCITY; 
    UNLOCK_DATA();
    return val;
}
float  rocket_data_get_est_vel_d_si(void) { 
    LOCK_DATA(); 
    float val = _rocket_data.est_vel_d * SCALE_VELOCITY; 
    UNLOCK_DATA();
    return val;
}

// Internal
float rocket_data_get_temp_stack_si(void) { 
    LOCK_DATA(); 
    float val = _rocket_data.temp_stack * SCALE_TEMP_STACK; 
    UNLOCK_DATA();
    return val;
}

int32_t rocket_data_get_gps_lat(void) { LOCK_DATA(); int32_t val = _rocket_data.gps_lat; UNLOCK_DATA(); return val; }
int32_t rocket_data_get_gps_lon(void) { LOCK_DATA(); int32_t val = _rocket_data.gps_lon; UNLOCK_DATA(); return val; }
int32_t rocket_data_get_gps_alt(void) { LOCK_DATA(); int32_t val = _rocket_data.gps_alt; UNLOCK_DATA(); return val; }

uint8_t rocket_data_get_avionics_state(void) { LOCK_DATA(); uint8_t val = _rocket_data.avionics_state; UNLOCK_DATA(); return val; }

// ============================================================================
// TEST UTILS
// ============================================================================

void rocket_data_fill_test_values(void) {
    
    // -----------------------------------------------------------------
    // 1. Define Physical Reality (SI Units)
    // -----------------------------------------------------------------
    
    // System
    uint16_t test_time_sec   = 123;
    uint16_t test_time_ms    = 456;
    uint8_t  test_state      = 2;     // Enabled
    uint8_t  test_apogee_cnt = 0;
    float    test_batt_ma    = 450.0f;
    uint8_t  test_flags      = 0x07;
    uint8_t  test_votes      = 0x01;  // Vote 1 only

    // Coordinates (Stellenbosch area)
    double   test_lat        = -33.9380;
    double   test_lon        = 18.7600;
    float    test_alt        = 150.5f;

    // Velocity (Downwards)
    float    test_vel_n      = 0.0f;
    float    test_vel_e      = 0.0f;
    float    test_vel_d      = -5.2f;

    // GPS Specific
    uint32_t test_itow       = 345678900;
    uint16_t test_week       = 2260;
    float    test_gps_alt    = 150.0f;
    float    test_gps_vel_d  = -5.0f;

    // Environment
    float    test_press_pa   = 101325.0f;
    float    test_temp_c     = 25.0f;

    // IMU (~1g on Z, some noise on X/Y)
    float    test_acc_x      = 0.047f; // ~0.04 m/s^2
    float    test_acc_y      = -0.095f;
    float    test_acc_z      = 9.81f;  // 1G

    // Gyro (Small drift)
    float    test_gyro_x     = 0.001f; // rad/s
    float    test_gyro_y     = 0.002f;
    float    test_gyro_z     = 0.003f;

    // Magnetometer (Earth field ~45uT)
    float    test_mag_x      = 12.0f; // uT
    float    test_mag_y      = 9.0f;
    float    test_mag_z      = -24.0f;

    // -----------------------------------------------------------------
    // 2. Inject Data via Public API (Tests the Setters too!)
    // -----------------------------------------------------------------
    
    rocket_data_update_runtime(test_time_sec, test_time_ms);
    rocket_data_update_state(test_state, test_apogee_cnt);
    rocket_data_update_battery(test_batt_ma);
    rocket_data_update_time_flags(test_flags);
    rocket_data_update_apogee_votes(test_votes);

    // Update GPS
    rocket_data_update_gps(test_lat, test_lon, test_gps_alt,
                           test_vel_n, test_vel_e, test_gps_vel_d,
                           test_itow, test_week);

    // Update Estimates
    rocket_data_update_est_state(test_lat, test_lon, test_alt,
                                 test_vel_n, test_vel_e, test_vel_d);

    // Update Sensors
    rocket_data_update_baro(test_press_pa, test_temp_c);
    rocket_data_update_temp_stack(test_temp_c);
    
    rocket_data_update_accel(test_acc_x, test_acc_y, test_acc_z);
    rocket_data_update_gyro(test_gyro_x, test_gyro_y, test_gyro_z);
    rocket_data_update_mag(test_mag_x, test_mag_y, test_mag_z);
}
