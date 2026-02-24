#include "tctlm.h"
#include "modules/datalink/datalink.h"
#include <stddef.h>

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
    if (!inst || !get_time_ms) return RS485_ERR_NO_PACKET;

    // 1. Send the request
    rs485_send_packet(inst, target_addr, MSG_TYPE_TLM_REQ, tm_id, NULL, 0);

    // 2. Wait for response
    uint32_t start_time = get_time_ms();
    while ((get_time_ms() - start_time) < timeout_ms) {
        if (poll_cb) poll_cb();

        RS485_packet_t pkt;
        if (rs485_get_packet(inst, &pkt)) {
            if (pkt.src_addr == target_addr && 
                pkt.msg_desc.type == MSG_TYPE_TLM_RESP && 
                pkt.msg_desc.id == tm_id) {
                if (out_resp) *out_resp = pkt;
                return RS485_OK;
            }
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
    if (!inst || !get_time_ms) return RS485_ERR_NO_PACKET;

    // 1. Send the telecommand
    rs485_send_packet(inst, target_addr, MSG_TYPE_TELECOMMAND, cmd_id, payload, len);

    // 2. Wait for ACK
    uint32_t start_time = get_time_ms();
    while ((get_time_ms() - start_time) < timeout_ms) {
        if (poll_cb) poll_cb();

        RS485_packet_t pkt;
        if (rs485_get_packet(inst, &pkt)) {
            if (pkt.src_addr == target_addr && 
                pkt.msg_desc.type == MSG_TYPE_TC_ACK && 
                pkt.msg_desc.id == cmd_id) {
                if (out_ack) *out_ack = pkt;
                return RS485_OK;
            }
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
    (void)pkt;
}

void TCTLM_processTelemetryRequest(RS485_packet_t *pkt) {
    (void)pkt;
}

void TCTLM_processTelemetryResponse(RS485_packet_t *pkt) {
    (void)pkt;
}

void TCTLM_processBulkTransfer(RS485_packet_t *pkt) {
    (void)pkt;
}

void TCTLM_processUnknownMessage(RS485_packet_t *pkt) {
    (void)pkt;
}
