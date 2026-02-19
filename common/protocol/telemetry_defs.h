#ifndef TELEMETRY_DEFS_H
#define TELEMETRY_DEFS_H

#include <stdint.h>

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
// MESSAGE DESCRIPTOR PACKING
// MSG_DESC byte = (msg_id & 0x1F) | ((msg_type & 0x07) << 5)
//
// Example — EPS identification request:
//   type = MSG_TYPE_TLM_REQ (4 = 0b100), id = TLM_COMMON_IDENT (0x00)
//   MSG_DESC = (0x00 & 0x1F) | (0b100 << 5) = 0x80
//
// Example — EPS power status request:
//   type = MSG_TYPE_TLM_REQ (4 = 0b100), id = TLM_EPS_POWER (0x01)
//   MSG_DESC = (0x01 & 0x1F) | (0b100 << 5) = 0x81
//
// Example — EPS power command:
//   type = MSG_TYPE_TELECOMMAND (2 = 0b010), id = TC_EPS_POWER (0x01)
//   MSG_DESC = (0x01 & 0x1F) | (0b010 << 5) = 0x41
// ============================================================================

// ============================================================================
// COMMON IDs (supported by ALL nodes)
// ============================================================================

// Telecommand IDs
#define TC_COMMON_RESET         0x00  // Reboot the node

// Telemetry IDs
#define TLM_COMMON_IDENT        0x00  // Identification (fw version, uptime, etc.)
#define TLM_COMMON_LOG          0x05  // Log message: [level 1B][text]

// Log level values (first byte of TLM_COMMON_LOG payload)
#define LOG_LEVEL_ERROR         0
#define LOG_LEVEL_WARN          1
#define LOG_LEVEL_INFO          2
#define LOG_LEVEL_DEBUG         3

// ============================================================================
// EPS-SPECIFIC IDs (target: ADDR_EPS = 0x02)
// ============================================================================

// Telecommand IDs
#define TC_EPS_POWER            0x01  // Set power line states (EpsPowerSetCmd_t)

// Telemetry IDs
#define TLM_EPS_POWER           0x01  // Power line on/off status (EpsPowerStatus_t)
#define TLM_EPS_MEASURE         0x02  // Voltage + current measurements (EpsMeasurements_t)

// ============================================================================
// TRACKING RADIO-SPECIFIC IDs (target: ADDR_TRACKING_RADIO = 0x03)
// ============================================================================

// Telecommand IDs
#define TC_RADIO_BEACON         0x02  // Trigger beacon transmission

// Telemetry IDs
#define TLM_RADIO_DATA          0x03  // Beacon / sensor data downlinked
#define TLM_RADIO_STATUS        0x04  // Radio health status

// ============================================================================
// OBC-SPECIFIC IDs (target: ADDR_OBC = 0x01, used for ground commands)
// ============================================================================

// Telecommand IDs (ground → OBC)
#define TC_OBC_TELEM_RATE       0x05  // Set telemetry rate
#define TC_OBC_LOG_LEVEL        0x06  // Set log verbosity
#define TC_OBC_SIM_MODE         0x08  // Enable/disable simulation
#define TC_OBC_AVIONICS_MODE    0x09  // Set avionics operating mode
#define TC_OBC_ENABLE_TUNNEL    0x0A  // Forward all RS485 traffic to USB
#define TC_OBC_FLIGHT_STATE     0x0B  // Override flight state machine

// Telemetry IDs (OBC → ground)
#define TLM_OBC_SENSOR_DATA        0x02  // OBC internal sensor snapshot
#define TLM_OBC_HIGH_SPEED_STREAM  0x0B  // Packed IMU/baro struct

// ============================================================================
// TELEMETRY PAYLOAD STRUCTURES
// ============================================================================

// TLM_COMMON_IDENT — 8 bytes, same struct for every node
// node_type: 1=OBC, 2=EPS, 3=TrackingRadio
typedef struct {
    uint8_t  node_type;
    uint8_t  interface_version;
    uint8_t  firmware_major;
    uint8_t  firmware_minor;
    uint16_t uptime_seconds;
    uint16_t uptime_milliseconds;
} PACKED_STRUCT TlmIdentificationPayload_t;

// TLM_OBC_SENSOR_DATA
typedef struct {
    uint32_t time;
    int16_t  accel_x;
    int16_t  accel_y;
    int16_t  accel_z;
} PACKED_STRUCT TlmSensorData_t;

#endif // TELEMETRY_DEFS_H
