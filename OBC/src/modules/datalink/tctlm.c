#include "tctlm.h"
#include "datalink.h"
#include <stddef.h>
#include "FreeRTOS.h"
#include "queue.h"

static QueueHandle_t g_resp_queue = NULL;

void tctlm_init(void) {
    if (!g_resp_queue) {
        g_resp_queue = xQueueCreate(1, sizeof(RS485_packet_t));
    }
}

// --- Project Implementation of TCTLM high-level API ---

rs485_status_t tctlm_send_telemetry_request(
    rs485_instance_t *inst,
    uint8_t target_addr,
    uint8_t tm_id,
    RS485_packet_t *out_resp,
    uint32_t timeout_ms,
    tctlm_get_time_ms_t get_time_ms,
    tctlm_poll_t poll_cb
) {
    if (!inst || !g_resp_queue) return RS485_ERR_NO_PACKET;

    // Clear any stale responses
    xQueueReset(g_resp_queue);

    // 1. Send the request
    rs485_send_packet(inst, target_addr, MSG_TYPE_TLM_REQ, tm_id, NULL, 0);

    // 2. Wait for response on queue
    RS485_packet_t pkt;
    if (xQueueReceive(g_resp_queue, &pkt, pdMS_TO_TICKS(timeout_ms)) == pdTRUE) {
        if (pkt.src_addr == target_addr && 
            pkt.msg_desc.type == MSG_TYPE_TLM_RESP && 
            pkt.msg_desc.id == tm_id) {
            if (out_resp) *out_resp = pkt;
            return RS485_OK;
        }
    }
    return RS485_ERR_TIMEOUT;
}

rs485_status_t tctlm_send_telecommand(
    rs485_instance_t *inst,
    uint8_t target_addr,
    uint8_t cmd_id,
    uint8_t *payload,
    uint8_t len,
    RS485_packet_t *out_ack,
    uint32_t timeout_ms,
    tctlm_get_time_ms_t get_time_ms,
    tctlm_poll_t poll_cb
) {
    if (!inst || !g_resp_queue) return RS485_ERR_NO_PACKET;

    // Clear any stale responses
    xQueueReset(g_resp_queue);

    // 1. Send the telecommand
    rs485_send_packet(inst, target_addr, MSG_TYPE_TELECOMMAND, cmd_id, payload, len);

    // 2. Wait for ACK on queue
    RS485_packet_t pkt;
    if (xQueueReceive(g_resp_queue, &pkt, pdMS_TO_TICKS(timeout_ms)) == pdTRUE) {
        if (pkt.src_addr == target_addr && 
            pkt.msg_desc.type == MSG_TYPE_TC_ACK && 
            pkt.msg_desc.id == cmd_id) {
            if (out_ack) *out_ack = pkt;
            return RS485_OK;
        }
    }
    return RS485_ERR_TIMEOUT;
}

// --- Inbound handlers ---

void TCTLM_processEvent(RS485_packet_t *pkt) {
    (void)pkt;
}

void TCTLM_processTelecommand(RS485_packet_t *pkt) {
    (void)pkt;
}

void TCTLM_processTelecommandAck(RS485_packet_t *pkt) {
    if (g_resp_queue) {
        xQueueOverwrite(g_resp_queue, pkt);
    }
}

void TCTLM_processTelemetryRequest(RS485_packet_t *pkt) {
    (void)pkt;
}

void TCTLM_processTelemetryResponse(RS485_packet_t *pkt) {
    if (g_resp_queue) {
        xQueueOverwrite(g_resp_queue, pkt);
    }
}

void TCTLM_processBulkTransfer(RS485_packet_t *pkt) {
    (void)pkt;
}

void TCTLM_processUnknownMessage(RS485_packet_t *pkt) {
    (void)pkt;
}
