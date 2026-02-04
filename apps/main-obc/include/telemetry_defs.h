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
// TELECOMMAND IDs
// ============================================================================
#define TC_ID_RESET             0
#define TC_ID_EPS_POWER         1
// #define TC_ID_AVIONICS_MODE     2
// #define TC_ID_UPDATE_TRIGGER    3

// ============================================================================
// TELEMETRY IDs
// ============================================================================
#define TLM_ID_IDENTIFICATION   0
#define TLM_ID_EPS_POWER        1
#define TLM_ID_EPS_MEASURE      2

//Definitions copied from example in comms library
// #define ID_TLM_IDENTIFICATION   1
// #define ID_TLM_EPS_POWER_STATUS 5
// #define ID_TLM_EPS_MEASUREMENTS 6
#define ID_TLM_SENSOR_DATA      2
#define ID_TLM_TRACKING_BEACON  3
#define ID_TLM_STATUS           4


#endif // TELEMETRY_DEFS_H
