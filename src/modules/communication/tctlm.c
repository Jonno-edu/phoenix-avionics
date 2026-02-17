#include "tctlm.h"
#include "rs485_protocol.h"
#include "core/obc_data.h"
#include "core/obc_commands.h"
#include "core/rocket_data.h"
#include "core/eps_node.h"
#include "core/tracking_radio_node.h"
#include "core/logging.h"
#include "core/logging_shim.h"
#include "core/rs485_monitor.h"
#include "../tasks/queue_manager.h"
#include <stdint.h>
#include <string.h>

static const char *TAG = "TCTLM";

void tctlm_send_reply(CommsInterfaceId_t dest, RS485_packet_t *pkt) {
    if (pkt == NULL) return;

    if (dest == IF_USB) {
        if (q_usb_out != NULL) {
            if (xQueueSend(q_usb_out, pkt, 0) != pdTRUE) {
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

static void log_rx_packet_to_monitor(RS485_packet_t *pkt) {
    if (!rs485_monitor_is_enabled()) return;
    uint8_t display_buf[270];
    uint16_t idx = 0;
    display_buf[idx++] = 0x1F;  
    display_buf[idx++] = 0x7F;  
    display_buf[idx++] = pkt->length;
    display_buf[idx++] = pkt->dest_addr;
    display_buf[idx++] = pkt->src_addr;
    display_buf[idx++] = pkt->msg_desc.raw;
    for (uint8_t i = 0; i < pkt->length && i < 256; i++) {
        display_buf[idx++] = pkt->data[i];
    }
    display_buf[idx++] = (pkt->crc >> 8) & 0xFF;
    display_buf[idx++] = pkt->crc & 0xFF;
    display_buf[idx++] = 0x1F;  
    display_buf[idx++] = 0xFF;  
    rs485_monitor_log_rx(display_buf, idx);
}

void TCTLM_processEvent(CommsInterfaceId_t src_id, RS485_packet_t *pkt) {
    log_rx_packet_to_monitor(pkt);
    LOGI(TAG, "Event received from %02X", pkt->src_addr);
}

void TCTLM_processTelecommand(CommsInterfaceId_t src_id, RS485_packet_t *pkt) {
    log_rx_packet_to_monitor(pkt);
    obc_handle_telecommand((InterfaceID_t)src_id, pkt);
}

void TCTLM_processTelecommandAck(CommsInterfaceId_t src_id, RS485_packet_t *pkt) {
    log_rx_packet_to_monitor(pkt);
    uint8_t src_address = pkt->src_addr;
    switch (src_address) {
        case ADDR_EPS:
            eps_handle_telecommand_ack(pkt);
            break;
        case ADDR_TRACKING_RADIO:
            tracking_radio_handle_telecommand_ack(pkt);
            break;
        default:
            LOGD(TAG, "TC ACK received: ID=%d from 0x%02X", pkt->msg_desc.id, pkt->src_addr);
            break;
    }
}

void TCTLM_processTelemetryRequest(CommsInterfaceId_t src_id, RS485_packet_t *pkt) {
    log_rx_packet_to_monitor(pkt);
    uint8_t id = pkt->msg_desc.id;
    LOGI(TAG, "Telemetry Request %d from %02X", id, pkt->src_addr);
    switch (id) {
        case TLM_COMMON_IDENT:
            obc_handle_telemetry_request((InterfaceID_t)src_id, pkt);
            break;
        default:
            LOGW(TAG, "Unknown TLM Req ID: %d from 0x%02X", id, pkt->src_addr);
            break;
    }
}

void TCTLM_processTelemetryResponse(CommsInterfaceId_t src_id, RS485_packet_t *pkt) {
    log_rx_packet_to_monitor(pkt);
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

void TCTLM_processBulkTransfer(CommsInterfaceId_t src_id, RS485_packet_t *pkt) {
    log_rx_packet_to_monitor(pkt);
    ESP_LOGI(TAG, "Bulk transfer from %02X", pkt->src_addr);
}

void TCTLM_processUnknownMessage(CommsInterfaceId_t src_id, RS485_packet_t *pkt) {
    log_rx_packet_to_monitor(pkt);
    ESP_LOGW(TAG, "Unknown message type %d from %02X", pkt->msg_desc.type, pkt->src_addr);
}
