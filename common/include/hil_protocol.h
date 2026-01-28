#ifndef HIL_PROTOCOL_H
#define HIL_PROTOCOL_H

#include <stdint.h>

/**
 * @file hil_protocol.h
 * @brief Shared protocol definitions for I2C communication between OBC and HIL node
 * 
 * This header defines the multi-rate sensor data packets exchanged over I2C:
 * - FAST (1kHz): IMU data (accel/gyro)
 * - MEDIUM (100Hz): Barometer and temperature
 * - SLOW (10Hz): GPS data
 * 
 * MUST be kept in sync between OBC master and HIL node slave.
 */

// --- I2C CONFIGURATION ---
#define HIL_I2C_SLAVE_ADDR    0x55
#define HIL_I2C_BAUDRATE      1000000  // 1 MHz

// Register addresses for multi-rate data access
#define HIL_REG_FAST          0x00     // 1kHz data (14 bytes)
#define HIL_REG_MEDIUM        0x01     // 100Hz data (7 bytes)
#define HIL_REG_SLOW          0x02     // 10Hz data (33 bytes)

// --- PACKET STRUCTURES ---
// Note: Packed to ensure consistent layout across platforms
#pragma pack(push, 1)

/**
 * @brief Fast packet - Updated at 1kHz
 * Register: 0x00
 * Size: 14 bytes
 */
typedef struct {
    int16_t  accel[3];       // Accelerometer X, Y, Z (raw ADC or m/s^2 * scale)
    int16_t  gyro[3];        // Gyroscope X, Y, Z (raw ADC or deg/s * scale)
    uint16_t sequence_id;    // Incrementing counter for detecting missed reads
} fast_packet_t;

/**
 * @brief Medium packet - Updated at 100Hz
 * Register: 0x01
 * Size: 7 bytes
 */
typedef struct {
    uint32_t baro_pres;      // Barometric pressure (Pa)
    int8_t   baro_temp;      // Barometer temperature (°C)
    int16_t  temp_stack;     // Stack/board temperature (°C * 128)
} medium_packet_t;

/**
 * @brief Slow packet - Updated at 10Hz
 * Register: 0x02
 * Size: 31 bytes
 */
typedef struct {
    int32_t  lat;            // Latitude (deg * 1e7)
    int32_t  lon;            // Longitude (deg * 1e7)
    int32_t  alt;            // Altitude (mm above MSL)
    int32_t  vel_n;          // North velocity (mm/s)
    int32_t  vel_e;          // East velocity (mm/s)
    int32_t  vel_d;          // Down velocity (mm/s)
    uint32_t i_tow;          // GPS time of week (ms)
    uint16_t gps_week;       // GPS week number
    uint8_t  flags;          // Status flags
} slow_packet_t;

#pragma pack(pop)

// Compile-time size checks
_Static_assert(sizeof(fast_packet_t) == 14, "fast_packet_t must be 14 bytes");
_Static_assert(sizeof(medium_packet_t) == 7, "medium_packet_t must be 7 bytes");
_Static_assert(sizeof(slow_packet_t) == 31, "slow_packet_t must be 31 bytes");

#endif // HIL_PROTOCOL_H
