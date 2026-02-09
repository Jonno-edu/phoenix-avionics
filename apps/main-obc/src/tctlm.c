#include "tctlm.h"
#include "rs485_protocol.h"
#include "core/obc_data.h"
#include "core/rocket_data.h"
#include "core/eps_node.h"
#include "core/tracking_radio_node.h"
#include "core/logging.h"
#include "core/rs485_monitor.h"
#include <stdint.h>
#include <string.h>

static const char *TAG = "TCTLM";

// ============================================================================
// RS485 LOG HELPER FUNCTION
// ============================================================================
static void log_rx_packet_to_monitor(RS485_packet_t *pkt) {
    if (!rs485_monitor_is_enabled()) return;
    
    // Reconstruct the raw packet for display
    // Format: [ESC SOM] SIZE DEST SRC DESC [PAYLOAD] CRC16 [ESC EOM]
    uint8_t display_buf[270];  // Max payload + framing
    uint16_t idx = 0;
    
    display_buf[idx++] = 0x1F;  // ESC
    display_buf[idx++] = 0x7F;  // SOM
    display_buf[idx++] = pkt->length;
    display_buf[idx++] = pkt->dest_addr;
    display_buf[idx++] = pkt->src_addr;
    display_buf[idx++] = pkt->msg_desc.raw;
    
    // Copy payload
    for (uint8_t i = 0; i < pkt->length && i < 256; i++) {
        display_buf[idx++] = pkt->data[i];
    }
    
    // CRC (use actual CRC from packet)
    display_buf[idx++] = (pkt->crc >> 8) & 0xFF;
    display_buf[idx++] = pkt->crc & 0xFF;
    display_buf[idx++] = 0x1F;  // ESC
    display_buf[idx++] = 0xFF;  // EOM
    
    rs485_monitor_log_rx(display_buf, idx);
}

// ============================================================================
// EVENT HANDLER
// ============================================================================

void TCTLM_processEvent(RS485_packet_t *pkt) {
    log_rx_packet_to_monitor(pkt);
    ESP_LOGI(TAG, "Event received from %02X", pkt->src_addr);
}

// ============================================================================
// TELECOMMAND HANDLER (commands received by OBC)
// ============================================================================

void TCTLM_processTelecommand(RS485_packet_t *pkt) {
    log_rx_packet_to_monitor(pkt);
    uint8_t id = pkt->msg_desc.id;
    // ESP_LOGI(TAG, "Telecommand %d received from %02X", id, pkt->src_addr);

    // Send ACK for the received command
    rs485_send_packet(pkt->src_addr, MSG_TYPE_TC_ACK, id, NULL, 0);

    switch (id) {
        case TC_ID_RESET: 
            if (pkt->length >= 1 && pkt->data[0] == 0x85) {
                ESP_LOGE(TAG, "RESET COMMAND RECEIVED! Rebooting...");
                // TODO: Trigger actual reset
            } else {
                ESP_LOGW(TAG, "Reset command with invalid confirmation byte");
            }
            break;

        // Shouldn't get a telecommand from any other device to the OBC?      

        default:
            ESP_LOGW(TAG, "Unknown TC ID: %d", id);
            break;
    }
}

// ============================================================================
// TELECOMMAND ACK HANDLER (ACKs for command OBC sent)
// ============================================================================

void TCTLM_processTelecommandAck(RS485_packet_t *pkt) {
    log_rx_packet_to_monitor(pkt);
    // ESP_LOGI(TAG, "TC ACK received from %02X for ID %d", pkt->src_addr, pkt->msg_desc.id);

    uint8_t src_address = pkt->src_addr;

    switch (src_address) {
        case ADDR_EPS:
            eps_handle_telecommand_ack(pkt);
            break;
        
        case ADDR_TRACKING_RADIO:
            tracking_radio_handle_telecommand_ack(pkt);
            break;

        default:
            ESP_LOGD(TAG, "TC ACK received: ID=%d from 0x%02X", pkt->msg_desc.id, pkt->src_addr);
            break;
    }
}

// ============================================================================
// TELEMETRY REQUEST HANDLER (when someone requests data from OBC)
// ============================================================================

void TCTLM_processTelemetryRequest(RS485_packet_t *pkt) {
    log_rx_packet_to_monitor(pkt);
    uint8_t id = pkt->msg_desc.id;
    // ESP_LOGI(TAG, "Telemetry Request %d from %02X", id, pkt->src_addr);

    switch (id) {
        case TLM_ID_IDENTIFICATION:
            obc_handle_telemetry_request(pkt);
            break;

        default:
            // ESP_LOGW(TAG, "Unknown TLM Req ID: %d from 0x%02X", id, pkt->src_addr);
            break;
    }
}

// ============================================================================
// TELEMETRY RESPONSE HANDLER (responses to OBC's requests)
// ============================================================================

void TCTLM_processTelemetryResponse(RS485_packet_t *pkt) {
    log_rx_packet_to_monitor(pkt);
    // ESP_LOGD(TAG, "TLM Response: ID=%d, Src=%02X, Len=%d", pkt->msg_desc.id, pkt->src_addr, pkt->length);
    
    switch (pkt->src_addr) {
        case ADDR_EPS:
            eps_handle_telemetry_response(pkt);
            break;

        case ADDR_TRACKING_RADIO:
            tracking_radio_handle_telemetry_response(pkt);
            break;

        default:
            ESP_LOGD(TAG, "Unknown Source for TLM Response: %02X (ID=%d)", pkt->src_addr, pkt->msg_desc.id);
            break;
    }
}

// ============================================================================
// BULK TRANSFER HANDLER
// ============================================================================

void TCTLM_processBulkTransfer(RS485_packet_t *pkt) {
    log_rx_packet_to_monitor(pkt);
    ESP_LOGI(TAG, "Bulk transfer from %02X", pkt->src_addr);
}

// ============================================================================
// UNKNOWN MESSAGE HANDLER 
// ============================================================================

void TCTLM_processUnknownMessage(RS485_packet_t *pkt) {
    log_rx_packet_to_monitor(pkt);
    ESP_LOGW(TAG, "Unknown message type %d from %02X", pkt->msg_desc.type, pkt->src_addr);
}

