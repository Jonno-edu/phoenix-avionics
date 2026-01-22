#include "tctlm.h"
#include "rs485_protocol.h"
#include "core/obc_data.h"
#include "core/rocket_data.h"
#include "core/logging.h"
#include <stdint.h>
#include <string.h>

static const char *TAG = "TCTLM";

void TCTLM_processEvent(RS485_packet_t *pkt) {
    ESP_LOGI(TAG, "Event received from %02X", pkt->src_addr);
}

void TCTLM_processTelecommand(RS485_packet_t *pkt) {
    uint8_t id = pkt->msg_desc.id;
    ESP_LOGI(TAG, "Telecommand %d received from %02X", id, pkt->src_addr);

    // Echo back an ACK
    rs485_send_packet(pkt->src_addr, MSG_TYPE_TC_ACK, id, NULL, 0);

    switch (id) {
        case ID_CMD_RESET:
            ESP_LOGE(TAG, "RESET COMMAND RECEIVED! Rebooting...");
            // In a real app, you'd trigger a reboot here.
            break;
        case ID_CMD_SET_SIM_STATE:
            if (pkt->length >= 1) {
                bool enable = (pkt->data[0] == 1);
                system_config_set_sim_mode(enable);
                ESP_LOGI(TAG, "CMD: Set Sim Mode = %d", enable);
            }
            break;
        case ID_CMD_SET_LOG_LEVEL:
            if (pkt->length >= 1) {
                system_config_set_log_level(pkt->data[0]);
                ESP_LOGI(TAG, "CMD: Set Log Level = %d", pkt->data[0]);
            }
            break;
        case ID_CMD_SET_TELEM_RATE:
            if (pkt->length >= 1) {
                system_config_set_telem_rate(pkt->data[0]);
                ESP_LOGI(TAG, "CMD: Set Telem Rate = %d Hz", pkt->data[0]);
            }
            break;
        default:
            ESP_LOGW(TAG, "Unknown TC ID: %d", id);
            break;
    }
}

void TCTLM_processTelecommandAck(RS485_packet_t *pkt) {
    ESP_LOGI(TAG, "TC ACK received from %02X for ID %d", pkt->src_addr, pkt->msg_desc.id);
}

void TCTLM_processTelemetryRequest(RS485_packet_t *pkt) {
    uint8_t id = pkt->msg_desc.id;
    ESP_LOGI(TAG, "Telemetry Request %d from %02X", id, pkt->src_addr);

    switch (id) {
        case ID_TLM_IDENTIFICATION: {
            SystemData_t sys_data;
            uint8_t buffer[sizeof(SystemData_t)];
            getSystemIdentInfo(&sys_data);
            system_data_pack(&sys_data, buffer);
            rs485_send_packet(pkt->src_addr, MSG_TYPE_TLM_RESP, id, buffer, sizeof(SystemData_t));
            break;
        }
        default:
            ESP_LOGW(TAG, "Unknown TLM Req ID: %d", id);
            break;
    }
}

void TCTLM_processTelemetryResponse(RS485_packet_t *pkt) {
    uint8_t id = pkt->msg_desc.id;
    ESP_LOGI(TAG, "Telemetry Response %d from %02X", id, pkt->src_addr);
    
    if (id == ID_TLM_IDENTIFICATION) {
        if (pkt->length == sizeof(TlmIdentificationPayload_t)) {
            TlmIdentificationPayload_t *tlm = (TlmIdentificationPayload_t *)pkt->data;
            ESP_LOGI(TAG, "Node %02X: fw %d.%d, uptime %us", 
                     pkt->src_addr, tlm->firmware_major, tlm->firmware_minor, tlm->uptime_seconds);
            
            if (tlm->status_flags & 0x01) { // STATUS_FLAG_CMD_PENDING
                ESP_LOGI(TAG, "Node %02X indicates CMD_PENDING", pkt->src_addr);
                // Maybe request it
            }
        }
    }
}

void TCTLM_processBulkTransfer(RS485_packet_t *pkt) {
    ESP_LOGI(TAG, "Bulk transfer from %02X", pkt->src_addr);
}

void TCTLM_processUnknownMessage(RS485_packet_t *pkt) {
    ESP_LOGW(TAG, "Unknown message type %d from %02X", pkt->msg_desc.type, pkt->src_addr);
}
