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
            eps_handle_response(pkt);
            break;
        
        case ADDR_TRACKING_RADIO:
            tracking_radio_handle_response(pkt);
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
        case TLM_ID_IDENTIFICATION: {
            SystemData_t sys_data;
            uint8_t buffer[sizeof(SystemData_t)];
            getSystemIdentInfo(&sys_data);
            system_data_pack(&sys_data, buffer);
            rs485_send_packet(pkt->src_addr, MSG_TYPE_TLM_RESP, id, buffer, sizeof(SystemData_t));
            ESP_LOGI(TAG, "Sent identification to 0x%02X", pkt->src_addr);
            break;
        }

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
    uint8_t id = pkt->msg_desc.id;
    ESP_LOGD(TAG, "TLM Response: ID=%d, Src=%02X, Len=%d", id, pkt->src_addr, pkt->length);
    
    switch (id) {
        case 0:
            printf("Received TLM_ID_IDENTIFICATION from 0x%02X\n", pkt->src_addr);
            if (pkt->length >= sizeof(TlmIdentificationPayload_t)) {
                TlmIdentificationPayload_t tlm;
                memcpy(&tlm, pkt->data, sizeof(TlmIdentificationPayload_t));
                
                ESP_LOGI(TAG, "Node 0x%02X: Type=%d, Ver=%d): FW=%d.%d, Uptime=%us", 
                         pkt->src_addr, tlm.node_type, tlm.interface_version,
                         tlm.firmware_major, tlm.firmware_minor, 
                         tlm.uptime_seconds);

                if (pkt->src_addr == ADDR_TRACKING_RADIO) {
                    tracking_radio_node_store_status((TrackingRadioStatus_t*)&tlm);
                }
            } else {
                ESP_LOGW(TAG, "Identification response too short: %d bytes, ", pkt->length);
            }
            break;
        
        case TLM_ID_EPS_POWER:
            if (pkt->src_addr != ADDR_EPS) {
                ESP_LOGW(TAG, "EPS Power response from unexpected source: 0x%02X", pkt->src_addr);
                break;
            }

            if (pkt->length >= sizeof(EpsPowerStatus_t)) {
                EpsPowerStatus_t power_status;
                memcpy(&power_status, pkt->data, sizeof(EpsPowerStatus_t));

                eps_node_store_power_status(&power_status);
            } else {
                ESP_LOGW(TAG, "EPS Power response too short: %d bytes (expected %d)", pkt->length, sizeof(EpsPowerStatus_t));
            }
            break;

        case TLM_ID_EPS_MEASURE: 
            if (pkt->src_addr != ADDR_EPS) {
                ESP_LOGW(TAG, "EPS Measurements response from unexpected source: 0x%02X", pkt->src_addr);
                break;
            }

            if (pkt->length >= sizeof(EpsMeasurements_t)) {
                EpsMeasurements_t measurements;
                memcpy(&measurements, pkt->data, sizeof(EpsMeasurements_t));

                eps_node_store_measurements(&measurements);
            } else {
                ESP_LOGW(TAG, "EPS Measurements response too short: %d bytes (expected %d)", pkt->length, sizeof(EpsMeasurements_t));
            }
            break;

        default:
            ESP_LOGD(TAG, "Unknown TLM Response ID: %d from 0x%02X", id, pkt->src_addr);
            break;

        }

    // // Log full raw packet: [Len Dest Src Desc] [Payload...]
    // char raw_buf[256] = {0};
    // int offset = 0;
    
    // // Header
    // offset += snprintf(raw_buf + offset, sizeof(raw_buf) - offset, "%02X %02X %02X %02X ",
    //                    pkt->length, pkt->dest_addr, pkt->src_addr, pkt->msg_desc.raw);
                       
    // // Payload
    // for(int i=0; i<pkt->length && offset < (sizeof(raw_buf) - 4); i++) {
    //     offset += snprintf(raw_buf + offset, sizeof(raw_buf) - offset, "%02X ", pkt->data[i]);
    // }
    // // ESP_LOGI(TAG, "RX Raw: [%s]", raw_buf);

    // if (id == ID_TLM_IDENTIFICATION) {
    //     if (pkt->src_addr == 0x03) {
    //         ESP_LOGI(TAG, "!!! RECEIVED IDENTIFICATION FROM TARGET 0x03 !!!");
    //     }
    //     // Accept 8 bytes (legacy/EPS) or more (upto full struct size)
    //     if (pkt->length >= 8 && pkt->length <= sizeof(TlmIdentificationPayload_t)) {
    //         TlmIdentificationPayload_t tlm = {0};
    //         // Safely copy available data
    //         memcpy(&tlm, pkt->data, pkt->length);
            
    //         // ESP_LOGI(TAG, "Node %02X (Type %d, Ver %d): fw %d.%d, uptime %us.%03u", 
    //         //          pkt->src_addr, tlm.node_type, tlm.interface_version,
    //         //          tlm.firmware_major, tlm.firmware_minor, 
    //         //          tlm.uptime_seconds, tlm.uptime_milliseconds);
            
    //         if (pkt->length >= 9) {
    //             ESP_LOGI(TAG, "Flags 0x%02X", tlm.status_flags);
    //             if (tlm.status_flags & 0x01) { // STATUS_FLAG_CMD_PENDING
    //                 ESP_LOGI(TAG, "Node %02X indicates CMD_PENDING", pkt->src_addr);
    //             }
    //         }
    //     } else {
    //          // ESP_LOGW(TAG, "ID_TLM_IDENTIFICATION: Unexpected length %d", pkt->length);
    //     }
    // }
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

