#ifndef HIL_PROTOCOL_H
#define HIL_PROTOCOL_H

#include <stdint.h>

#define HIL_SLAVE_ADDR      0x55
#define HIL_I2C_BAUDRATE    400000 

// --- VIRTUAL REGISTERS (Mapped to sensors.json) ---
#define HIL_REG_WHOAMI      0x00
#define HIL_REG_IMU         0x10  // TDK IIM-46230 (1000Hz)
#define HIL_REG_BARO1       0x20  // BMP581 (50Hz)
#define HIL_REG_BARO2       0x21  // MS5607 (50Hz)
#define HIL_REG_GPS         0x30  // NEO-M9N (40Hz)
#define HIL_REG_MAG         0x40  // BMM350 (100Hz)
#define HIL_REG_TEMP        0x50  // TMP117 (1Hz)

#pragma pack(push, 1)

// 1. IMU (6-DOF)
typedef struct { 
    uint16_t seq; 
    uint32_t time_ms; 
    int16_t  accel[3]; // X, Y, Z
    int16_t  gyro[3];  // X, Y, Z
} packet_imu_t;

// 2. Barometer (Used for both BMP581 and MS5607)
typedef struct { 
    uint16_t seq; 
    uint32_t time_ms; 
    int32_t  pressure_pa; 
    int16_t  temp_c;      // Most baros give temp too
} packet_baro_t;

// 3. GPS (Standard PVT)
typedef struct { 
    uint16_t seq; 
    uint32_t time_ms; 
    int32_t  lat; 
    int32_t  lon; 
    int32_t  alt; 
    int32_t  vel_n;
    int32_t  vel_e;
    int32_t  vel_d;
} packet_gps_t;

// 4. Magnetometer (3-Axis)
typedef struct { 
    uint16_t seq; 
    uint32_t time_ms; 
    int16_t  mag[3];   // X, Y, Z
} packet_mag_t;

// 5. Temperature Sensor
typedef struct { 
    uint16_t seq; 
    uint32_t time_ms; 
    int16_t  temp_c; 
} packet_temp_t;

#pragma pack(pop)

#endif // HIL_PROTOCOL_H