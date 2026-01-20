// system_data.h
#ifndef SYSTEM_DATA_H
#define SYSTEM_DATA_H

#include <stdint.h>
#include <stdbool.h>
#include "telemetry_defs.h"

#define TLM_ID_IDENTIFICATION 128

// System identification data structure
typedef TlmIdentificationPayload_t SystemData_t;

typedef struct {
    uint8_t log_level;      // 0=None, 1=Error, 2=Info, 3=Debug
    bool sim_mode_enabled;  // true = Use HIL/Simulated data
    uint8_t telem_rate_hz;  // Frequency of telemetry broadcasts
} SystemConfig_t;

void system_data_get(SystemData_t *data);
void system_data_init(void);
void system_data_pack(const SystemData_t *data, uint8_t *buffer);

// Configuration accessors
void system_config_init(void);
void system_config_set_log_level(uint8_t level);
void system_config_set_sim_mode(bool enabled);
void system_config_set_telem_rate(uint8_t rate_hz);

uint8_t system_config_get_log_level(void);
bool system_config_get_sim_mode(void);
uint8_t system_config_get_telem_rate(void);

#endif // SYSTEM_DATA_H
