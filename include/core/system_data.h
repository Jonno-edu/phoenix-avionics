// system_data.h
#ifndef SYSTEM_DATA_H
#define SYSTEM_DATA_H

#include <stdint.h>

#define TLM_ID_IDENTIFICATION 128
#define SYSTEM_DATA_FRAME_LENGTH 9

// System identification data structure
typedef struct {
    uint8_t node_type;
    uint8_t interface_version;
    uint8_t firmware_major_version;
    uint8_t firmware_minor_version;
    uint16_t runtime_seconds;
    uint16_t runtime_milliseconds;
    uint8_t status_flags;  // <--- NEW: Status Flags (e.g. Pending Command)
} SystemData_t;

void system_data_get(SystemData_t *data);
void system_data_init(void);
void system_data_pack(const SystemData_t *data, uint8_t *buffer);

#endif // SYSTEM_DATA_H
