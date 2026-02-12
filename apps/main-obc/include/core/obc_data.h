// obc_data.h
#ifndef OBC_DATA_H
#define OBC_DATA_H

#include <stdint.h>
#include <stdbool.h>
#include "telemetry_defs.h"
#include "rs485_protocol.h"
#include "../../src/tasks/queue_manager.h" // For InterfaceID_t

// System identification data structure
typedef TlmIdentificationPayload_t SystemData_t;

typedef struct {
    uint8_t log_level;      // 0=None, 1=Error, 2=Info, 3=Debug
    bool sim_mode_enabled;  // true = Use HIL/Simulated data
    uint8_t telem_rate_hz;  // Frequency of telemetry broadcasts
    uint8_t avionics_mode;  // 0=Debug, 1=Release (Flight)
} SystemConfig_t;

void getSystemIdentInfo(SystemData_t *data);
void system_data_init(void);
void system_data_pack(const SystemData_t *data, uint8_t *buffer);

// Configuration accessors
void system_config_init(void);
void system_config_set_log_level(uint8_t level);
void system_config_set_sim_mode(bool enabled);
void system_config_set_telem_rate(uint8_t rate_hz);
void system_config_set_avionics_mode(uint8_t mode);

uint8_t system_config_get_log_level(void);
bool system_config_get_sim_mode(void);
uint8_t system_config_get_telem_rate(void);
uint8_t system_config_get_avionics_mode(void);

/**
 * @brief Handle a Telemetry Request (for OBC data)
 * @param pkt The request packet
 */
void obc_handle_telemetry_request(InterfaceID_t src_id, RS485_packet_t *pkt);

#endif // OBC_DATA_H
