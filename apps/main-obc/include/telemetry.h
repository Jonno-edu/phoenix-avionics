#ifndef TELEMETRY_H
#define TELEMETRY_H

#include <stdint.h>

/* =========================================================================
 * SCALING FACTORS (Multiply Raw Integer by this to get Real Value)
 * ========================================================================= */

// Position & Altitude
#define SCALE_POS_LATLON     0.0000001f    // 1e-7
#define SCALE_ALTITUDE       0.001f        // 1 LSB = 1mm (0.001m)

// Velocity
#define SCALE_VELOCITY       0.1f          // 1 LSB = 0.1 m/s

// Barometer
#define SCALE_BARO_PRESS     0.015625f     // 1 LSB = 1/64 Pa
#define SCALE_BARO_TEMP      0.01f         // 1 LSB = 0.01 C

// Internal Sensors
#define SCALE_TEMP_STACK     0.0078125f    // 1 LSB = 1/128 C
#define SCALE_BATTERY_MA     1.0f          // 1:1 (mA)

// IMU (Inertial)
#define SCALE_ACCEL          0.0047884f    // m/s^2 per LSB
#define SCALE_GYRO           0.0010653f    // rad/s per LSB
#define SCALE_MAG            0.0610352f    // uT per LSB

// Time
#define SCALE_TIME_MS        1.0f          // 1:1 (Milliseconds)

/* =========================================================================
 * PACKET STRUCTURE
 * Target Size: 74 Bytes
 * ========================================================================= */

// Ensure strict byte packing (no compiler padding between fields)
#pragma pack(push, 1)

typedef struct {
    // -- Header --
    uint16_t runtime_sec;           // 0-16:   Seconds
    uint16_t runtime_ms;            // 16-32:  Milliseconds
    uint8_t  avionics_state;        // 32-40:  Enum
    uint8_t  apogee_counter;        // 40-48:  Enum
    uint16_t battery_current;       // 48-64:  mA
    
    // -- Position Estimates --
    int32_t  est_lat;               // 64-96:  deg * 1e7
    int32_t  est_lon;               // 96-128: deg * 1e7
    int32_t  est_alt;               // 128-160: mm (Raw integer)
    
    // -- Velocity Estimates (Scaled 0.1) --
    int16_t  est_vel_n;             // 160-176: m/s * 10
    int16_t  est_vel_e;             // 176-192: m/s * 10
    int16_t  est_vel_d;             // 192-208: m/s * 10
    
    // -- Raw Sensors --
    uint32_t baro_press;            // 208-240: Pa * 64 (Updated to 32-bit)
    int16_t  baro_temp;             // 240-256: degC * 100 (Updated to 16-bit)
    
    int16_t  temp_stack;            // 256-272: Scaled
    int16_t  accel_x;               // 272-288: Scaled
    int16_t  accel_y;               // 288-304: Scaled
    int16_t  accel_z;               // 304-320: Scaled
    int16_t  gyro_x;                // 320-336: Scaled
    int16_t  gyro_y;                // 336-352: Scaled
    int16_t  gyro_z;                // 352-368: Scaled
    int16_t  mag_x;                 // 368-384: Scaled
    int16_t  mag_y;                 // 384-400: Scaled
    int16_t  mag_z;                 // 400-416: Scaled
    
    // -- GPS Data --
    int32_t  gps_lat;               // 416-448: deg * 1e7
    int32_t  gps_lon;               // 448-480: deg * 1e7
    int32_t  gps_alt;               // 480-512: mm (Raw integer)
    
    int16_t  gps_vel_n;             // 512-528: m/s * 10
    int16_t  gps_vel_e;             // 528-544: m/s * 10
    int16_t  gps_vel_d;             // 544-560: m/s * 10
    
    uint32_t itow;                  // 560-592: ms
    uint16_t gps_week;              // 592-608: week count
    
    // -- Flags --
    uint8_t  time_valid_flags;      // 608-616: Bitmask
    
    // Bitfields for the last byte (Offset 616-624)
    // Warning: Bitfield ordering can be compiler specific. 
    // LSB (Vote 1) first is common on Little Endian (ARM/RP2040).
    uint8_t  apogee_vote_1 : 1;     // 616
    uint8_t  apogee_vote_2 : 1;     // 617
    uint8_t  apogee_vote_3 : 1;     // 618
    uint8_t  _reserved     : 5;     // Padding to finish the byte
    
} TrackingBeacon_t;

#pragma pack(pop)

// Sanity Check Size
_Static_assert(sizeof(TrackingBeacon_t) == 78, "TrackingBeacon_t size mismatch!");

#endif // TELEMETRY_H
