// obc_data.c
#include "modules/data_store/obc_data.h"
#include <string.h>
#include "logging.h"
#include "rs485_protocol.h"

static const char *TAG = "OBC_DATA";

#if PICO_BUILD
    #include <pico/stdlib.h>
#else
    #include <time.h>
#endif

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

static volatile SystemConfig_t sys_config = {
    .log_level = 3,
    .sim_mode_enabled = false,
    .telem_rate_hz = 1,
    .avionics_mode = 0
};

static uint32_t boot_time_ms = 0;

#if !PICO_BUILD
static uint64_t host_boot_time_ms = 0;
static uint64_t host_time_ms(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return ((uint64_t)ts.tv_sec * 1000ULL) + ((uint64_t)ts.tv_nsec / 1000000ULL);
}
#endif

void system_data_init(void) {
#if PICO_BUILD
    boot_time_ms = to_ms_since_boot(get_absolute_time());
#else
    host_boot_time_ms = host_time_ms();
#endif
}

void getSystemIdentInfo(SystemData_t *data) {
    if (data == NULL) return;
    data->node_type = system_config.node_type;
    data->interface_version = system_config.interface_version;
    data->firmware_major = system_config.firmware_major_version;
    data->firmware_minor = system_config.firmware_minor_version;
#if PICO_BUILD
    uint32_t current_ms = to_ms_since_boot(get_absolute_time());
    uint32_t uptime_ms = current_ms - boot_time_ms;
    data->uptime_seconds = uptime_ms / 1000;
    data->uptime_milliseconds = uptime_ms % 1000;
#else
    uint64_t current_ms = host_time_ms();
    uint64_t uptime_ms = (current_ms >= host_boot_time_ms) ? (current_ms - host_boot_time_ms) : 0;
    if (uptime_ms > 65535ULL * 1000ULL + 999ULL) {
        uptime_ms = 65535ULL * 1000ULL + 999ULL;
    }
    data->uptime_seconds = (uint16_t)(uptime_ms / 1000ULL);
    data->uptime_milliseconds = (uint16_t)(uptime_ms % 1000ULL);
#endif
}

void system_data_pack(const SystemData_t *data, uint8_t *buffer) {
    if (data == NULL || buffer == NULL) return;
    memcpy(buffer, data, sizeof(SystemData_t));
}

void system_config_init(void) {}
void system_config_set_log_level(uint8_t level) { if(level <= 3) sys_config.log_level = level; }
void system_config_set_sim_mode(bool enabled) { sys_config.sim_mode_enabled = enabled; }
void system_config_set_telem_rate(uint8_t rate_hz) { if(rate_hz > 0) sys_config.telem_rate_hz = rate_hz; }
void system_config_set_avionics_mode(uint8_t mode) { sys_config.avionics_mode = mode; }
uint8_t system_config_get_log_level(void) { return sys_config.log_level; }
bool system_config_get_sim_mode(void) { return sys_config.sim_mode_enabled; }
uint8_t system_config_get_telem_rate(void) { return sys_config.telem_rate_hz; }
uint8_t system_config_get_avionics_mode(void) { return sys_config.avionics_mode; }

#include "tctlm.h"

void obc_handle_telemetry_request(InterfaceID_t src_id, RS485_packet_t *pkt) {
    if (pkt == NULL) return;
    uint8_t id = pkt->msg_desc.id;
    switch (id) {
        case TLM_COMMON_IDENT: {
            SystemData_t sys_data;
            uint8_t buffer[sizeof(SystemData_t)];
            getSystemIdentInfo(&sys_data);
            system_data_pack(&sys_data, buffer);
            RS485_packet_t reply;
            reply.dest_addr = pkt->src_addr;
            reply.src_addr = ADDR_OBC;
            reply.msg_desc.type = MSG_TYPE_TLM_RESP;
            reply.msg_desc.id = id;
            reply.length = sizeof(SystemData_t);
            memcpy(reply.data, buffer, sizeof(SystemData_t));
            reply.crc = 0;
            tctlm_send_reply(src_id, &reply);
            ESP_LOGD(TAG, "Sent identification to 0x%02X", pkt->src_addr);
            break;
        }
        default:
            break;
    }
}
