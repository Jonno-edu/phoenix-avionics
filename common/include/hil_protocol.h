#ifndef HIL_PROTOCOL_H
#define HIL_PROTOCOL_H

#include <stdint.h>

// --- I2C CONFIGURATION ---
#define HIL_SLAVE_ADDR      0x55
// UPDATED: Set to 400kHz to match HIL Node settings
#define HIL_I2C_BAUDRATE    400000 

// --- REGISTER MAP ---
// UPDATED: Matches the new HIL Node logic
#define HIL_REG_FAST        0x01   // IMU Packet (1kHz)
#define HIL_REG_MEDIUM      0x02   // Baro Packet (100Hz)
#define HIL_REG_SLOW        0x03   // GPS Packet (10Hz)

// --- PACKET STRUCTURES ---
// These match the HIL Node exactly.
#pragma pack(push, 1)

typedef struct { 
    uint16_t seq; 
    uint32_t time_ms; 
    int16_t  accel_x; 
} packet_imu_t;

typedef struct { 
    uint16_t seq; 
    uint32_t time_ms; 
    int32_t  pressure; 
} packet_baro_t;

typedef struct { 
    uint16_t seq; 
    uint32_t time_ms; 
    int32_t  lat; 
} packet_gps_t;

#pragma pack(pop)

#endif // HIL_PROTOCOL_H