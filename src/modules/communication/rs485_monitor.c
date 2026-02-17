#include "rs485_monitor.h"
#include "logging.h"
#include <stdio.h>

static const char *TAG = "RS485_MON";
static bool monitor_logging_enabled = true;

void rs485_monitor_init(void) {
    ESP_LOGI(TAG, "RS485 Monitor Initialized");
}

void rs485_monitor_log_rx(const uint8_t *data, uint16_t len) {
    if (monitor_logging_enabled && data && len >= 6) {
        // Skip framing if present (0x1F 0x7F)
        uint16_t offset = (data[0] == 0x1F && data[1] == 0x7F) ? 2 : 0;
        uint8_t pkt_len = data[offset + 0];
        uint8_t dest     = data[offset + 1];
        uint8_t src      = data[offset + 2];
        uint8_t desc     = data[offset + 3];
        uint8_t id       = desc & 0x1F;

        ESP_LOGD(TAG, "RX: [0x%02X] From: 0x%02X ID: 0x%02X Len: %u", 
                 dest, src, id, pkt_len);
    }
}

void rs485_monitor_log_tx(const uint8_t *data, uint16_t len) {
    if (monitor_logging_enabled && data && len >= 6) {
        // Skip framing if present (0x1F 0x7F)
        uint16_t offset = (data[0] == 0x1F && data[1] == 0x7F) ? 2 : 0;
        uint8_t pkt_len = data[offset + 0];
        uint8_t dest     = data[offset + 1];
        uint8_t src      = data[offset + 2];
        uint8_t desc     = data[offset + 3];
        uint8_t id       = desc & 0x1F;

        ESP_LOGD(TAG, "TX: [0x%02X] From: 0x%02X ID: 0x%02X Len: %u", 
                 dest, src, id, pkt_len);
    }
}

void rs485_monitor_log_byte(uint8_t byte, bool is_tx) {
    (void)byte;
    (void)is_tx;
    // For extreme low-level hex debugging if needed
    // printf("%02X ", byte);
}

void rs485_monitor_set_enable(bool enabled) {
    monitor_logging_enabled = enabled;
}

bool rs485_monitor_is_enabled(void) {
    return monitor_logging_enabled;
}
