#include "modules/communication/obc_commands.h"
#include "logging.h"
#include "rs485_protocol.h"
#include <string.h>

static const char *TAG = "OBC_CMD";

void obc_handle_telecommand(InterfaceID_t src_id, RS485_packet_t *pkt) {
    if (pkt == NULL) return;
    
    ESP_LOGI(TAG, "Telecommand from %u received (ID: 0x%02X)", src_id, pkt->msg_desc.id);
    
    switch (pkt->msg_desc.id) {
        // Example command IDs
        case TC_OBC_SYSTEM_REBOOT:
            ESP_LOGW(TAG, "System restart requested");
            // Implement reboot logic
            break;
            
        case TC_OBC_SYSTEM_SHUTDOWN:
            ESP_LOGW(TAG, "System shutdown requested");
            // Implement shutdown logic
            break;
            
        case TC_OBC_DATALOG_ENABLE:
            if (pkt->length >= 1) {
                bool enable = pkt->data[0] != 0;
                ESP_LOGI(TAG, "Datalogging set to %s", enable ? "ENABLED" : "DISABLED");
            }
            break;
            
        default:
            ESP_LOGE(TAG, "Unknown telecommand ID: 0x%02X", pkt->msg_desc.id);
            break;
    }
}
