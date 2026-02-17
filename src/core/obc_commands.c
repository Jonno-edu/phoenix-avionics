#include "core/obc_commands.h"
#include "core/obc_data.h"
#include "core/logging.h"
#include "core/rs485_monitor.h"
#include "telemetry_defs.h"
#include "tctlm.h"
#include <string.h>
#include <FreeRTOS.h>

static const char *TAG = "OBC_CMD";

// Helper to send ACK
static void send_ack(InterfaceID_t dest, uint8_t dest_addr, uint8_t cmd_id) {
    RS485_packet_t ack_pkt;
    ack_pkt.dest_addr = dest_addr;
    ack_pkt.src_addr = ADDR_OBC;
    ack_pkt.msg_desc.type = MSG_TYPE_TC_ACK;
    ack_pkt.msg_desc.id = cmd_id;
    ack_pkt.length = 0;
    ack_pkt.crc = 0; // Driver will calculate
    
    tctlm_send_reply(dest, &ack_pkt);
}

void obc_handle_telecommand(InterfaceID_t src_id, RS485_packet_t *pkt) {
    if (pkt == NULL) return;
    uint8_t id = pkt->msg_desc.id;
    
    // Always acknowledge commands directed to us
    send_ack(src_id, pkt->src_addr, id);

    switch (id) {
        case TC_COMMON_RESET: 
            if (pkt->length >= 1 && pkt->data[0] == 0x85) {
                ESP_LOGE(TAG, "RESET COMMAND RECEIVED! Rebooting...");
                // In a real system: platform_reset();
            } else {
                ESP_LOGW(TAG, "Reset command with invalid confirmation byte");
            }
            break;

        case TC_OBC_LOG_LEVEL:
             if (pkt->length >= 1) {
                system_config_set_log_level(pkt->data[0]);
                ESP_LOGI(TAG, "Log Level set to %d", pkt->data[0]);
             }
             break;

        case TC_OBC_TELEM_RATE:
             if (pkt->length >= 1) {
                system_config_set_telem_rate(pkt->data[0]);
                ESP_LOGI(TAG, "Telemetry Rate set to %dHz", pkt->data[0]);
             }
             break;
             
        case TC_OBC_SIM_MODE:
             if (pkt->length >= 1) {
                bool enable = (pkt->data[0] != 0);
                system_config_set_sim_mode(enable);
                ESP_LOGI(TAG, "Simulation Mode set to %s", enable ? "ENABLED (HIL Sensors)" : "DISABLED (Real Sensors)");
             }
             break;

        case TC_OBC_AVIONICS_MODE:
             if (pkt->length >= 1) {
                uint8_t mode = pkt->data[0];
                system_config_set_avionics_mode(mode);
                ESP_LOGI(TAG, "Avionics Mode set to %d (%s)", mode, mode == 0 ? "DEBUG" : "FLIGHT");
             }
             break;

        default:
            ESP_LOGW(TAG, "Unknown TC ID: %d", id);
            break;
    }
}
