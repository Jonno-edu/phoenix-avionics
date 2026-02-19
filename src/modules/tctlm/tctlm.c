// OBC implementation of the fixed tctlm.h library interface.
// Implements exactly the functions declared in tctlm.h — no more, no less.

#include "tctlm.h"
#include "modules/datalink/datalink.h"

// Called when OBC needs to send a reply to an inbound request.
// dest is the CommsInterfaceId_t the original request arrived on.
void tctlm_send_reply(CommsInterfaceId_t dest, RS485_packet_t *pkt)
{
    // dest selects the transport — only RS485 for now,
    // USB mirror can be added here when ready.
    (void)dest;
    datalink_send(pkt->dest_addr,
                  pkt->msg_desc.type,
                  pkt->msg_desc.id,
                  pkt->data,
                  pkt->length);
}

// --- Inbound handlers ---
// Note: OBC-initiated request/response exchanges (e.g. tracking_radio_request_ident)
// are handled entirely inside tracking_radio.c via datalink_request_response().
// These handlers are for *unsolicited* inbound packets from other nodes.

void TCTLM_processEvent(CommsInterfaceId_t src_id, RS485_packet_t *pkt) {
    (void)src_id; (void)pkt;
    // TODO
}

void TCTLM_processTelecommand(CommsInterfaceId_t src_id, RS485_packet_t *pkt) {
    (void)src_id; (void)pkt;
    // TODO: e.g. uplink commands from ground via 433 radio
}

void TCTLM_processTelecommandAck(CommsInterfaceId_t src_id, RS485_packet_t *pkt) {
    (void)src_id; (void)pkt;
    // TODO
}

void TCTLM_processTelemetryRequest(CommsInterfaceId_t src_id, RS485_packet_t *pkt) {
    (void)src_id; (void)pkt;
    // TODO: e.g. EPS asks OBC for its identification
}

void TCTLM_processTelemetryResponse(CommsInterfaceId_t src_id, RS485_packet_t *pkt) {
    (void)src_id; (void)pkt;
    // Not used for OBC-initiated requests (handled in tracking_radio.c directly)
}

void TCTLM_processBulkTransfer(CommsInterfaceId_t src_id, RS485_packet_t *pkt) {
    (void)src_id; (void)pkt;
    // TODO
}

void TCTLM_processUnknownMessage(CommsInterfaceId_t src_id, RS485_packet_t *pkt) {
    (void)src_id; (void)pkt;
    // TODO: log unknown message type
}
