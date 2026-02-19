#include "eps.h"
#include "modules/datalink/datalink.h"
#include "protocol/phoenix_node_addrs.h"
#include "protocol/phoenix_tctlm_ids.h"
#include "common/logging.h"
#include <string.h>

static const char *TAG = "EPS";

bool eps_request_ident(TlmIdentificationPayload_t *out, uint32_t timeout_ms)
{
    RS485_packet_t resp;

    printf("[%s] Requesting identification from EPS (0x%02X)\n", TAG, ADDR_EPS);

    datalink_status_t st = datalink_request_response(
        ADDR_EPS,
        MSG_TYPE_TLM_REQ,
        ID_TLM_IDENTIFICATION,
        NULL, 0,
        MSG_TYPE_TLM_RESP,
        ID_TLM_IDENTIFICATION,
        &resp,
        timeout_ms
    );

    if (st != DATALINK_OK) return false;
    if (resp.length < sizeof(TlmIdentificationPayload_t)) return false;

    memcpy(out, resp.data, sizeof(TlmIdentificationPayload_t));
    return true;
}
