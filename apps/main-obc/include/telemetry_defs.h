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

// ============================================================================
// EPS SPECIFIC IDs (Target: ADDR_EPS)
// ============================================================================
// TC: Telecommands
#define TC_EPS_POWER            0x01 // Set power lines

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
#define TC_OBC_AVIONICS_MODE    0x09

// TLM: Telemetry Responses (OBC -> Ground)
#define TLM_OBC_SENSOR_DATA     0x02 // OBC Internal Sensors 


#endif // TELEMETRY_DEFS_H