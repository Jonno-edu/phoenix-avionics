#pragma once
#include <stdint.h>

#if defined(__GNUC__)
    #define PACKED_STRUCT __attribute__((packed))
#else
    #define PACKED_STRUCT
#endif

// ID: ID_TLM_IDENTIFICATION (used by all nodes)
typedef struct {
    uint8_t  node_type;
    uint8_t  interface_version;
    uint8_t  firmware_major;
    uint8_t  firmware_minor;
    uint16_t uptime_seconds;
    uint16_t uptime_milliseconds;
    uint8_t  status_flags;
} PACKED_STRUCT TlmIdentificationPayload_t;

// ID: ID_TLM_TRACKING_BEACON
typedef struct {
    uint16_t runtime_seconds;
    uint16_t runtime_milliseconds;
    uint8_t  avionics_state;
    uint8_t  apogee_counter;
    uint16_t battery_current;
    int32_t  est_lat;
    int32_t  est_lon;
    int32_t  est_alt;
    int16_t  est_vel_n;
    int16_t  est_vel_e;
    int16_t  est_vel_d;
    uint32_t baro_pressure;
    int32_t  baro_temp;
    int16_t  temp_stack;
    int16_t  accel_x;
    int16_t  accel_y;
    int16_t  accel_z;
    int16_t  gyro_x;
    int16_t  gyro_y;
    int16_t  gyro_z;
    int16_t  mag_x;
    int16_t  mag_y;
    int16_t  mag_z;
    int32_t  gps_lat;
    int32_t  gps_lon;
    int32_t  gps_alt;
    int32_t  gps_vel_n;
    int32_t  gps_vel_e;
    int32_t  gps_vel_d;
    uint32_t gps_itow;
    uint16_t gps_week;
    uint8_t  time_valid_flags;
    uint8_t  apogee_votes;
} PACKED_STRUCT TlmTrackingBeaconPayload_t;
