// system_data.h
#ifndef SYSTEM_DATA_H
#define SYSTEM_DATA_H

#include <stdint.h>
#include "telemetry_defs.h"

#define TLM_ID_IDENTIFICATION 128

// System identification data structure
typedef TlmIdentificationPayload_t SystemData_t;

void system_data_get(SystemData_t *data);
void system_data_init(void);
void system_data_pack(const SystemData_t *data, uint8_t *buffer);

#endif // SYSTEM_DATA_H
