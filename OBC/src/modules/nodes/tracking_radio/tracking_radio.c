#include "tracking_radio.h"
#include "modules/datalink/datalink.h"
#include "phoenix_icd.h"
#include "logging.h"

#include "norb/norb.h"

#include <string.h>
#include <stddef.h> // for NULL

static const char *TAG = "TrackingRadio";

// Generic telemetry request helper
static datalink_status_t tr_request_tlm(uint8_t id,
                                        RS485_packet_t *resp,
                                        uint32_t timeout_ms)
{
    return datalink_request_response(
        ADDR_TRACKING_RADIO,
        MSG_TYPE_TLM_REQ,  id,
        NULL, 0,
        MSG_TYPE_TLM_RESP, id,
        resp, timeout_ms
    );
}

bool tracking_radio_request_ident(TlmIdentificationPayload_t *out,
                                  uint32_t timeout_ms)
{
    RS485_packet_t resp;
    if (tr_request_tlm(TLM_COMMON_IDENT, &resp, timeout_ms) != DATALINK_OK)
        return false;

    // Basic sanity: response payload must be at least the size we expect
    if (resp.length < sizeof(TlmIdentificationPayload_t)) return false;

    memcpy(out, resp.data, sizeof(TlmIdentificationPayload_t));

    ESP_LOGI(TAG, "Ident: type=0x%02X fw=%u.%u uptime=%us",
             out->node_type, out->firmware_major,
             out->firmware_minor, out->uptime_seconds);

    return true;
}

bool tracking_radio_send_beacon(const TlmTrackingBeaconPayload_t *beacon,
                                uint32_t timeout_ms)
{
    printf("[%s] I: Sending beacon (len=%zu): ", TAG, sizeof(*beacon));
    const uint8_t *raw = (const uint8_t *)beacon;
    for (size_t i = 0; i < sizeof(*beacon); i++) {
        printf("%02X ", raw[i]);
    }
    printf("\n");

    RS485_packet_t resp;
    datalink_status_t st = datalink_request_response(
        ADDR_TRACKING_RADIO,
        MSG_TYPE_TELECOMMAND, TC_RADIO_BEACON,
        (const uint8_t*)beacon, sizeof(*beacon),
        MSG_TYPE_TC_ACK, TC_RADIO_BEACON,
        &resp, timeout_ms
    );

    if (st != DATALINK_OK) {
        ESP_LOGW(TAG, "Failed to send beacon to tracking radio");
        return false;
    }

    ESP_LOGI(TAG, "Beacon sent successfully, ACK received (len=%d)", resp.length);
    return true;
}

// Poll helper: perform tracking-radio telemetry requests and publish results
// to NORB. Keeps housekeeping.c small — call from the periodic poll task.
void tracking_radio_poll(uint32_t timeout_ms)
{
    TlmIdentificationPayload_t ident;

    if (tracking_radio_request_ident(&ident, timeout_ms))
        norb_publish(TOPIC_TRACKING_RADIO_IDENT, &ident);

    TlmTrackingBeaconPayload_t beacon;
    memset(&beacon, 0, sizeof(beacon));
    
    // Sample static values for testing
    beacon.runtime_seconds = 1234;
    beacon.runtime_milliseconds = 567;
    beacon.avionics_state = 3; // e.g., FLIGHT
    beacon.apogee_counter = 0;
    beacon.battery_current = 1500; // 1.5A
    beacon.est_lat = 377749000; // 37.7749 N
    beacon.est_lon = -1224194000; // 122.4194 W
    beacon.est_alt = 10000; // 10,000m
    beacon.est_vel_n = 100;
    beacon.est_vel_e = 50;
    beacon.est_vel_d = -20;
    beacon.baro_pressure = 101325; // 1 atm
    beacon.baro_temp = 2500; // 25.00 C
    beacon.temp_stack = 3000; // 30.00 C
    beacon.accel_x = 0;
    beacon.accel_y = 0;
    beacon.accel_z = 981; // 1g
    beacon.gps_lat = 377749000;
    beacon.gps_lon = -1224194000;
    beacon.gps_alt = 10000;
    beacon.gps_itow = 123456789;
    beacon.gps_week = 2250;
    beacon.time_valid_flags = 0x03;
    beacon.apogee_votes = 1;

    tracking_radio_send_beacon(&beacon, timeout_ms);
}
