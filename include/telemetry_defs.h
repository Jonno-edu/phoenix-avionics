#ifndef TELEMETRY_DEFS_H
#define TELEMETRY_DEFS_H

#include <stdint.h>

// Ensure struct packing across different architectures
#if defined(__GNUC__)
#define PACKED_STRUCT __attribute__((packed))
#else
#define PACKED_STRUCT
#endif

// Addressing
#define ADDR_OBC             0x01
#define ADDR_EPS             0x02
#define ADDR_TRACKING_RADIO  0x03 
#define ADDR_GSE             0xF0

// --- Telecommand IDs ---
#define ID_CMD_RESET           0b00001 // 1
#define ID_CMD_SLEEP           0b00010 // 2
#define ID_CMD_TRACKING_BEACON 0b00011 // 3
#define ID_CMD_WAKE            0b00100 // 4
#define ID_CMD_SET_TELEM_RATE  0b00101 // 5
#define ID_CMD_SET_LOG_LEVEL   0b00110 // 6

#define ID_CMD_SET_SIM_STATE   0b01000 // 8
#define ID_CMD_GET_PENDING_MSG 0b10000 // 16

// MSG_TYPE_TLM_RESP IDs
#define ID_TLM_IDENTIFICATION  0b00001
#define ID_TLM_SENSOR_DATA     0b00010
#define ID_TLM_TRACKING_BEACON 0b00011
#define ID_TLM_STATUS          0b00100


#endif // TELEMETRY_DEFS_H
