#include "tracking_radio.h"
#include "modules/datalink/datalink.h"
#include "common/include/protocol/phoenix_node_addrs.h"
#include "common/include/protocol/phoenix_tctlm_ids.h"
#include "logging.h"

#include <string.h>
#include <stddef.h> // for NULL

static const char *TAG = "TrackingRadio";

bool tracking_radio_request_ident(TlmIdentificationPayload_t *out,
                                  uint32_t timeout_ms)
{
    RS485_packet_t resp;

    ESP_LOGI(TAG, "Requesting identification from tracking radio (0x%02X)", ADDR_TRACKING_RADIO);

    datalink_status_t st = datalink_request_response(
        ADDR_TRACKING_RADIO,
        MSG_TYPE_TLM_REQ,
        ID_TLM_IDENTIFICATION,
        NULL, 0,                        // no request payload
        MSG_TYPE_TLM_RESP,
        ID_TLM_IDENTIFICATION,
        &resp,
        timeout_ms
    );

    if (st != DATALINK_OK) return false;

    // Basic sanity: response payload must be at least the size we expect
    if (resp.length < sizeof(TlmIdentificationPayload_t)) return false;

    memcpy(out, resp.data, sizeof(TlmIdentificationPayload_t));
    return true;
}

bool tracking_radio_send_beacon(const TlmTrackingBeaconPayload_t *beacon,
                                uint32_t timeout_ms)
{
    (void)beacon;
    (void)timeout_ms;
    // TODO: implement in next step
    //   datalink_request_response(ADDR_TRACKING_RADIO,
    //       MSG_TYPE_TELECOMMAND, ID_CMD_TRACKING_BEACON,
    //       (uint8_t*)beacon, sizeof(*beacon),
    //       MSG_TYPE_TC_ACK, ID_CMD_TRACKING_BEACON,
    //       NULL, timeout_ms);
    return false;
}
