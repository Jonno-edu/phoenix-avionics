#include "tctlm.h"
#include "rs485_protocol.h"
#include "core/obc_data.h"
#include "core/obc_commands.h"
#include "core/rocket_data.h"
#include "core/eps_node.h"
#include "core/tracking_radio_node.h"
#include "core/logging.h"
#include "core/rs485_monitor.h"
#include "tasks/queue_manager.h" // For queues and InterfaceID_t
#include <stdint.h>
#include <string.h>

static const char *TAG = "TCTLM";

// ============================================================================
// ROUTING HELPER
// ============================================================================
void tctlm_send_reply(InterfaceID_t dest, RS485_packet_t *pkt) {
    if (pkt == NULL) return;

    if (dest == IF_USB) {
        if (q_usb_out != NULL) {
            if (xQueueSend(q_usb_out, pkt, 0) != pdTRUE) {
                // Drop packet if full (High speed stream)
            }
        }
    } 
    else if (dest == IF_RS485) {
        if (q_rs485_out != NULL) {
            if (xQueueSend(q_rs485_out, pkt, 0) != pdTRUE) {
                ESP_LOGW(TAG, "RS485 TX Queue Full - Dropped Packet");
            }
        }
    }
}

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

void TCTLM_processEvent(InterfaceID_t src_id, RS485_packet_t *pkt) {
    log_rx_packet_to_monitor(pkt);
    ESP_LOGI(TAG, "Event received from %02X", pkt->src_addr);
}

// ============================================================================
// TELECOMMAND HANDLER (commands received by OBC)
// ============================================================================

void TCTLM_processTelecommand(InterfaceID_t src_id, RS485_packet_t *pkt) {
    log_rx_packet_to_monitor(pkt);
    // uint8_t id = pkt->msg_desc.id;
    // ESP_LOGI(TAG, "Telecommand %d received from %02X", id, pkt->src_addr);
    
    // Delegate to OBC Data Handler (Logic Node)
    obc_handle_telecommand(src_id, pkt);
}

// ============================================================================
// TELECOMMAND ACK HANDLER (ACKs for command OBC sent)
// ============================================================================

void TCTLM_processTelecommandAck(InterfaceID_t src_id, RS485_packet_t *pkt) {
    log_rx_packet_to_monitor(pkt);
    // ESP_LOGI(TAG, "TC ACK received from %02X for ID %d", pkt->src_addr, pkt->msg_desc.id);

    uint8_t src_address = pkt->src_addr;
    
    // Acknowledgements are responses to OUR commands, so we don't reply to them.
    // We just forward them to the relevant logic module.

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

void TCTLM_processTelemetryRequest(InterfaceID_t src_id, RS485_packet_t *pkt) {
    log_rx_packet_to_monitor(pkt);
    uint8_t id = pkt->msg_desc.id;
    // ESP_LOGI(TAG, "Telemetry Request %d from %02X", id, pkt->src_addr);

    switch (id) {
        case TLM_ID_IDENTIFICATION:
            obc_handle_telemetry_request(src_id, pkt);
            break;

        default:
            // ESP_LOGW(TAG, "Unknown TLM Req ID: %d from 0x%02X", id, pkt->src_addr);
            break;
    }
}

// ============================================================================
// TELEMETRY RESPONSE HANDLER (responses to OBC's requests)
// ============================================================================

void TCTLM_processTelemetryResponse(InterfaceID_t src_id, RS485_packet_t *pkt) {
    log_rx_packet_to_monitor(pkt);
    // ESP_LOGD(TAG, "TLM Response: ID=%d, Src=%02X, Len=%d", pkt->msg_desc.id, pkt->src_addr, pkt->length);
    
    // If we received telemetry (e.g. from EPS), we might want to FORWARD it to the Ground Station (USB)
    // if the GSU asked for it, or if we are streaming.
    // For now, if it came from RS485, we likely want to mirror it to USB? 
    // User Requirement: "Pipe A (GSU Link) ... Pipe B (Bus Link)"
    // Typically GSU wants to see everything.
    
    // Forwarding Logic:
    if (src_id == IF_RS485) {
        tctlm_send_reply(IF_USB, pkt);
    }
    
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

void TCTLM_processBulkTransfer(InterfaceID_t src_id, RS485_packet_t *pkt) {
    log_rx_packet_to_monitor(pkt);
    ESP_LOGI(TAG, "Bulk transfer from %02X", pkt->src_addr);
}

// ============================================================================
// UNKNOWN MESSAGE HANDLER 
// ============================================================================

void TCTLM_processUnknownMessage(InterfaceID_t src_id, RS485_packet_t *pkt) {
    log_rx_packet_to_monitor(pkt);
    ESP_LOGW(TAG, "Unknown message type %d from %02X", pkt->msg_desc.type, pkt->src_addr);
}

