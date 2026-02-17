#ifndef TELEMETRY_DEFS_H
#define TELEMETRY_DEFS_H

#include <stdint.h>

// Ensure struct packing across different architectures
#if defined(__GNUC__)
#define PACKED_STRUCT __attribute__((packed))
#else
#define PACKED_STRUCT
#endif

// ============================================================================
// DEVICE ADDRESSES
// ============================================================================
#define ADDR_OBC             0x01
#define ADDR_EPS             0x02
#define ADDR_TRACKING_RADIO  0x03 
#define ADDR_GSE             0xF0

// ============================================================================
// COMMON IDs (Supported by ALL Devices)
// ============================================================================
// TC: Telecommands (Host -> Device)
#define TC_COMMON_RESET         0x00

// TLM: Telemetry Responses (Device -> Host)
#define TLM_COMMON_IDENT        0x00 // Identification Response
#define TLM_COMMON_LOG          0x05 // Log Message (Payload: [Level 1B] + [Text])

// Log Levels (First byte of payload)
#define LOG_LEVEL_ERROR         0
#define LOG_LEVEL_WARN          1
#define LOG_LEVEL_INFO          2
#define LOG_LEVEL_DEBUG         3

// ============================================================================
// EPS SPECIFIC IDs (Target: ADDR_EPS)
// ============================================================================
// TC: Telecommands
#define TC_EPS_POWER            0x01 // Set power lines
#define TC_EPS_FULL_RESET       0x03 // Reset EPS (Simulate power cycle)

// TLM: Telemetry Responses
#define TLM_EPS_POWER           0x01 // Get power line status
#define TLM_EPS_MEASURE         0x02 // Get voltage/current measurements

// ============================================================================
// TRACKING RADIO SPECIFIC IDs (Target: ADDR_TRACKING_RADIO)
// ============================================================================
// TC: Telecommands
#define TC_RADIO_BEACON         0x02 // Send Beacon 
// Note: No TC_RADIO_BEACON_ACK needed; ACKs use the Command ID (0x02)

// TLM: Telemetry Responses
#define TLM_RADIO_DATA          0x03 // Beacon/Sensor Data downlinked
#define TLM_RADIO_STATUS        0x04 // Radio health status

// ============================================================================
// OBC / SYSTEM IDs (Target: ADDR_OBC)
// ============================================================================
// TC: Telecommands (Ground -> OBC)
#define TC_OBC_TELEM_RATE       0x05
#define TC_OBC_LOG_LEVEL        0x06
#define TC_OBC_SIM_MODE         0x08

// Log Levels for TC_OBC_LOG_LEVEL (Deprecated/Redefined above)
// #define LOG_LEVEL_NONE          0
// #define LOG_LEVEL_ERROR         1
// #define LOG_LEVEL_INFO          2
// #define LOG_LEVEL_DEBUG         3

#define TC_OBC_AVIONICS_MODE    0x09
#define TC_OBC_ENABLE_TUNNEL    0x0A // "Please forward all RS485 traffic to USB"
#define TC_OBC_FLIGHT_STATE     0x0B

#define TC_OBC_SYSTEM_REBOOT    0x0C
#define TC_OBC_SYSTEM_SHUTDOWN  0x0D
#define TC_OBC_DATALOG_ENABLE   0x0E


// TLM: Telemetry Responses (OBC -> Ground)
#define TLM_OBC_SENSOR_DATA     0x02 // OBC Internal Sensors 
#define TLM_OBC_HIGH_SPEED_STREAM  0x0B // Packed IMU/Baro struct


// ============================================================================
// TELEMETRY STRUCTURES
// ============================================================================

// ID: TLM_COMMON_IDENT
typedef struct {
    uint8_t node_type;              // 1=OBC, 2=EPS, ...
    uint8_t interface_version;
    uint8_t firmware_major;
    uint8_t firmware_minor;
    uint16_t uptime_seconds;
    uint16_t uptime_milliseconds;         
} PACKED_STRUCT TlmIdentificationPayload_t;

// ID: TLM_OBC_SENSOR_DATA
typedef struct {
    uint32_t time;
    int16_t accel_x;
    int16_t accel_y;
    int16_t accel_z;
} PACKED_STRUCT TlmSensorData_t;

// ID: ??? (System Status)
typedef struct {
    uint32_t time;
    uint8_t mode;
    uint16_t vbat_mv;
    uint8_t cpu_load;
} PACKED_STRUCT TlmSystemStatus_t;




#endif // TELEMETRY_DEFS_H