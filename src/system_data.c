// system_data.c
#include "system_data.h"
#include <string.h>

#if PICO_BUILD
    #include <pico/stdlib.h>
#else
    #include <time.h>
#endif

// Static configuration data
static struct {
    uint8_t node_type;
    uint8_t interface_version;
    uint8_t firmware_major_version;
    uint8_t firmware_minor_version;
} system_config = {
    .node_type = 1,
    .interface_version = 1,
    .firmware_major_version = 0,
    .firmware_minor_version = 1
};

// Boot time reference
static uint32_t boot_time_ms = 0;

#if !PICO_BUILD
// Host boot time reference (milliseconds)
static uint64_t host_boot_time_ms = 0;

static uint64_t host_time_ms(void) {
    // Use a monotonic clock so system time changes don't affect uptime.
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return ((uint64_t)ts.tv_sec * 1000ULL) + ((uint64_t)ts.tv_nsec / 1000000ULL);
}
#endif

void system_data_init(void) {
#if PICO_BUILD
    // Capture boot time
    boot_time_ms = to_ms_since_boot(get_absolute_time());
#else
    // Capture host boot time reference
    host_boot_time_ms = host_time_ms();
#endif
}

void system_data_get(SystemData_t *data) {
    if (data == NULL) {
        return;
    }
    
    // Copy static configuration
    data->node_type = system_config.node_type;
    data->interface_version = system_config.interface_version;
    data->firmware_major_version = system_config.firmware_major_version;
    data->firmware_minor_version = system_config.firmware_minor_version;
    
    // Calculate runtime on-demand
#if PICO_BUILD
    uint32_t current_ms = to_ms_since_boot(get_absolute_time());
    uint32_t uptime_ms = current_ms - boot_time_ms;
    
    data->runtime_seconds = uptime_ms / 1000;
    data->runtime_milliseconds = uptime_ms % 1000;
#else
    // Host build: monotonic uptime since system_data_init()
    uint64_t current_ms = host_time_ms();
    uint64_t uptime_ms = (current_ms >= host_boot_time_ms) ? (current_ms - host_boot_time_ms) : 0;

    // SystemData_t uses uint16_t fields, so clamp to prevent overflow (~65s max)
    if (uptime_ms > 65535ULL * 1000ULL + 999ULL) {
        uptime_ms = 65535ULL * 1000ULL + 999ULL;
    }

    data->runtime_seconds = (uint16_t)(uptime_ms / 1000ULL);
    data->runtime_milliseconds = (uint16_t)(uptime_ms % 1000ULL);
#endif
    
    // Status flags are populated by the caller (OBC logic) if needed, 
    // but initialized to 0 here by default.
    data->status_flags = 0;
}

void system_data_pack(const SystemData_t *data, uint8_t *buffer) {
    if (data == NULL || buffer == NULL) {
        return;
    }
    
    buffer[0] = data->node_type;
    buffer[1] = data->interface_version;
    buffer[2] = data->firmware_major_version;
    buffer[3] = data->firmware_minor_version;
    buffer[4] = (data->runtime_seconds >> 8) & 0xFF;
    buffer[5] = data->runtime_seconds & 0xFF;
    buffer[6] = (data->runtime_milliseconds >> 8) & 0xFF;
    buffer[7] = data->runtime_milliseconds & 0xFF;
    buffer[8] = data->status_flags;
}
