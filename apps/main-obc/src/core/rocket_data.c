#include "core/rocket_data.h"
#include <string.h>

// Helper macro for thread safety (placeholder)
// TODO: Replace with FreeRTOS mutex when you add RTOS
#define LOCK_DATA()   
#define UNLOCK_DATA() 

// The Master Copy of the Data
static TlmTrackingBeaconPayload_t _rocket_data;

void rocket_data_init(void) {
    memset(&_rocket_data, 0, sizeof(_rocket_data));
}

// ============================================================================
// SETTERS
// ============================================================================

void rocket_data_update_gps(int32_t lat, int32_t lon, int32_t alt, 
                            int32_t vel_n, int32_t vel_e, int32_t vel_d,
                            uint32_t itow, uint16_t week) {
    LOCK_DATA();
    _rocket_data.gps_lat = lat;
    _rocket_data.gps_lon = lon;
    _rocket_data.gps_alt = alt;
    _rocket_data.gps_vel_n = vel_n;
    _rocket_data.gps_vel_e = vel_e;
    _rocket_data.gps_vel_d = vel_d;
    _rocket_data.gps_itow = itow;
    _rocket_data.gps_week = week;
    UNLOCK_DATA();
}

void rocket_data_update_accel(int16_t x, int16_t y, int16_t z) {
    LOCK_DATA();
    _rocket_data.accel_x = x;
    _rocket_data.accel_y = y;
    _rocket_data.accel_z = z;
    UNLOCK_DATA();
}

void rocket_data_update_gyro(int16_t x, int16_t y, int16_t z) {
    LOCK_DATA();
    _rocket_data.gyro_x = x;
    _rocket_data.gyro_y = y;
    _rocket_data.gyro_z = z;
    UNLOCK_DATA();
}

void rocket_data_update_mag(int16_t x, int16_t y, int16_t z) {
    LOCK_DATA();
    _rocket_data.mag_x = x;
    _rocket_data.mag_y = y;
    _rocket_data.mag_z = z;
    UNLOCK_DATA();
}

void rocket_data_update_est_state(int32_t lat, int32_t lon, int32_t alt,
                                  int16_t vel_n, int16_t vel_e, int16_t vel_d) {
    LOCK_DATA();
    _rocket_data.est_lat = lat;
    _rocket_data.est_lon = lon;
    _rocket_data.est_alt = alt;
    _rocket_data.est_vel_n = vel_n;
    _rocket_data.est_vel_e = vel_e;
    _rocket_data.est_vel_d = vel_d;
    UNLOCK_DATA();
}

void rocket_data_update_battery(uint16_t current_ma) {
    LOCK_DATA();
    _rocket_data.battery_current = current_ma;
    UNLOCK_DATA();
}

void rocket_data_update_state(uint8_t state, uint8_t apogee_count) {
    LOCK_DATA();
    _rocket_data.avionics_state = state;
    _rocket_data.apogee_counter = apogee_count;
    UNLOCK_DATA();
}

void rocket_data_update_baro(uint32_t pressure_pa, int32_t temp_c_x100) {
    LOCK_DATA();
    _rocket_data.baro_pressure = pressure_pa;
    _rocket_data.baro_temp = temp_c_x100;
    UNLOCK_DATA();
}

void rocket_data_update_temp_stack(int16_t temp_c_x128) {
    LOCK_DATA();
    _rocket_data.temp_stack = temp_c_x128;
    UNLOCK_DATA();
}

void rocket_data_update_runtime(uint16_t seconds, uint16_t milliseconds) {
    LOCK_DATA();
    _rocket_data.runtime_seconds = seconds;
    _rocket_data.runtime_milliseconds = milliseconds;
    UNLOCK_DATA();
}

void rocket_data_update_time_flags(uint8_t flags) {
    LOCK_DATA();
    _rocket_data.time_valid_flags = flags;
    UNLOCK_DATA();
}

void rocket_data_update_apogee_votes(uint8_t votes) {
    LOCK_DATA();
    _rocket_data.apogee_votes = votes;
    UNLOCK_DATA();
}

// ============================================================================
// GETTERS
// ============================================================================

void getRocketTrackingInfo(TlmTrackingBeaconPayload_t *out_beacon) {
    LOCK_DATA();
    memcpy(out_beacon, &_rocket_data, sizeof(TlmTrackingBeaconPayload_t));
    UNLOCK_DATA();
}

// ============================================================================
// TEST UTILS
// ============================================================================

void rocket_data_fill_test_values(void) {
    LOCK_DATA();
    
    // System
    _rocket_data.runtime_seconds = 123;
    _rocket_data.runtime_milliseconds = 456;
    _rocket_data.avionics_state = 2; // Enabled
    _rocket_data.apogee_counter = 0;
    _rocket_data.battery_current = 450; // 450mA
    
    // GPS (Stellenbosch area)
    _rocket_data.gps_lat = -339380000;  // -33.9380° (Stellenbosch)
    _rocket_data.gps_lon = 187600000;   // 18.7600°
    _rocket_data.gps_alt = 150000;      // 150m MSL
    _rocket_data.gps_vel_n = 0;
    _rocket_data.gps_vel_e = 0;
    _rocket_data.gps_vel_d = -50;       // 5.0 m/s up
    _rocket_data.gps_itow = 345678900;  // GPS time of week
    _rocket_data.gps_week = 2260;
    _rocket_data.time_valid_flags = 0x07; // All time fields valid

    // Estimation (slightly different from raw GPS)
    _rocket_data.est_lat = -339380000;
    _rocket_data.est_lon = 187600000;
    _rocket_data.est_alt = 150500;      // 150.5m (mm)
    _rocket_data.est_vel_n = 0;
    _rocket_data.est_vel_e = 0;
    _rocket_data.est_vel_d = -52;       // -5.2 m/s (m/s * 10)

    // Barometer
    _rocket_data.baro_pressure = 101325; // 1 atm in Pa
    _rocket_data.baro_temp = 2500;       // 25.00°C (°C * 100)
    
    // Temperature sensor
    _rocket_data.temp_stack = 0;         // Initialize to 0, wait for sensor update
    
    // IMU - Accel (on pad, ~1g down)
    _rocket_data.accel_x = 10;
    _rocket_data.accel_y = -20;
    _rocket_data.accel_z = 16384;        // ~1g (depends on scale factor A)
    
    // IMU - Gyro (stationary)
    _rocket_data.gyro_x = 1;
    _rocket_data.gyro_y = 2;
    _rocket_data.gyro_z = 3;
    
    // Magnetometer (South African field)
    _rocket_data.mag_x = 200;
    _rocket_data.mag_y = 150;
    _rocket_data.mag_z = -400;
    
    // Apogee detection
    _rocket_data.apogee_votes = 0b000;   // No votes yet

    UNLOCK_DATA();
}
