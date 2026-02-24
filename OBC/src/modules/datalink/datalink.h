#pragma once
#include <stdint.h>
#include <stdbool.h>
#include "rs485_protocol.h"

typedef enum {
    DATALINK_OK = 0,
    DATALINK_TIMEOUT,
    DATALINK_BAD_RESPONSE,
} datalink_status_t;

// Must be called once before any transactions (from main or a task init)
void datalink_init(void);

// Blocking: send a request, wait for a matching response from target_addr.
// Returns DATALINK_OK and populates out_resp on success.
datalink_status_t datalink_request_response(
    uint8_t          target_addr,
    RS485_msgType_t  req_type,
    uint8_t          req_id,
    const uint8_t   *req_payload,
    uint8_t          req_len,
    RS485_msgType_t  expected_resp_type,
    uint8_t          expected_resp_id,
    RS485_packet_t  *out_resp,
    uint32_t         timeout_ms
);

// Fire-and-forget send (for replies, events, no ACK expected).
// Used by tctlm_send_reply().
void datalink_send(
    uint8_t          target_addr,
    RS485_msgType_t  type,
    uint8_t          id,
    const uint8_t   *payload,
    uint8_t          len
);
