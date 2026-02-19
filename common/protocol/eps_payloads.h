#ifndef EPS_PAYLOADS_H
#define EPS_PAYLOADS_H

#include <stdint.h>
#include "telemetry_defs.h"

// --- EPS Node Payload Structs ---

typedef enum {
    POWER_OFF = 0,
    POWER_ON  = 1
} PowerSelect_t;

typedef enum {
    EPS_LINE_3V3_1 = 0,
    EPS_LINE_3V3_2,
    EPS_LINE_3V3_3,
    EPS_LINE_5V_1,
    EPS_LINE_5V_2,
    EPS_LINE_5V_3,
    EPS_LINE_12V,
    EPS_LINE_COUNT
} EpsLineIndex_t;

// Wire format: 2 bytes, 1 bit per rail.
// Identical layout for both TLM_EPS_POWER response and TC_EPS_POWER command.
typedef struct {
    uint8_t rail_3v3_1 : 1;  // bit 0
    uint8_t rail_3v3_2 : 1;  // bit 1
    uint8_t rail_3v3_3 : 1;  // bit 2
    uint8_t rail_5v_1  : 1;  // bit 3
    uint8_t rail_5v_2  : 1;  // bit 4
    uint8_t rail_5v_3  : 1;  // bit 5
    uint8_t rail_12v   : 1;  // bit 6
    uint8_t reserved   : 1;  // bit 7
    uint8_t fault_flags;      // byte 1: 0x00 = all OK
} PACKED_STRUCT EpsPowerStatus_t;

typedef EpsPowerStatus_t EpsPowerSetCmd_t;

// Wire format: 5 × uint16_t = 10 bytes
typedef struct {
    uint16_t batt_voltage_mv;     // Battery voltage (units TBD — 0 on bench)
    uint16_t batt_current_ma;     // Battery current
    uint16_t current_3v3_1_ma;    // 3V3 rail 1 current
    uint16_t current_5v_1_ma;     // 5V rail 1 current
    uint16_t current_12v_ma;      // 12V rail current
} PACKED_STRUCT EpsMeasurements_t;

#endif // EPS_PAYLOADS_H
