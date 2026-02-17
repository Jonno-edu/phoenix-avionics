#include "core/tracking_radio_node.h"
#include "core/logging.h"
#include "rs485_protocol.h"
#include "../communication/rs485_task.h"
#include <string.h>
#include <FreeRTOS.h>
#include <semphr.h>
#include <pico/stdlib.h>

#define GET_TIME_MS() to_ms_since_boot(get_absolute_time())

static const char *TAG = "TRACKING_RADIO_NODE";

typedef struct {
    TrackingRadioStatus_t status;
    uint32_t status_age_ms;
    bool status_valid;
} TrackingRadioNodeStore_t;

static TrackingRadioNodeStore_t radio_store;
static SemaphoreHandle_t radio_mutex;
static SemaphoreHandle_t ack_semaphore;

#define TRACKING_RADIO_TIMEOUT_MS     10000 
#define BEACON_ACK_TIMEOUT_MS         200
#define BEACON_RETRIES                3 

void tracking_radio_node_init(void) {
    radio_mutex = xSemaphoreCreateMutex();
    ack_semaphore = xSemaphoreCreateBinary();
    memset(&radio_store, 0, sizeof(TrackingRadioNodeStore_t));
    radio_store.status_valid = false;
    ESP_LOGI(TAG, "Tracking Radio node storage initialized");
}

void tracking_radio_request_health(void) {
    rs485_send_packet(rs485_get_default_instance(), ADDR_TRACKING_RADIO, MSG_TYPE_TLM_REQ, TLM_COMMON_IDENT, NULL, 0);
    ESP_LOGD(TAG, "Sent health request to Tracking Radio");
}

bool tracking_radio_send_beacon(const TrackingBeacon_t *beacon_data) {
    if (beacon_data == NULL) return false;
    ESP_LOGI(TAG, "TX BEACON | PktTime: %u.%03u | GPS: [%.6f, %.6f] Alt: %.1fm | AccZ: %.2f m/s^2", 
             beacon_data->runtime_sec, 
             beacon_data->runtime_ms,
             beacon_data->gps_lat * SCALE_POS_LATLON,
             beacon_data->gps_lon * SCALE_POS_LATLON,
             beacon_data->gps_alt * SCALE_ALTITUDE,
             beacon_data->accel_z * SCALE_ACCEL);

    xSemaphoreTake(ack_semaphore, 0);
    rs485_send_packet(rs485_get_default_instance(), ADDR_TRACKING_RADIO, MSG_TYPE_TELECOMMAND, TC_RADIO_BEACON, (uint8_t*)beacon_data, sizeof(TrackingBeacon_t));

    if (xSemaphoreTake(ack_semaphore, pdMS_TO_TICKS(BEACON_ACK_TIMEOUT_MS)) == pdTRUE) {
        ESP_LOGD(TAG, "Beacon ACK received");
        return true;
    } 
    ESP_LOGW(TAG, "Beacon ACK timeout (Single Attempt - Skipped Retry)");
    return false;
}

void tracking_radio_handle_telecommand_ack(RS485_packet_t *pkt) {
    if (pkt == NULL) return;
    uint8_t message_ID = pkt->msg_desc.id;
    switch (message_ID) {
        case TC_RADIO_BEACON:
            tracking_radio_confirm_beacon_ack();
            break;
        case TC_COMMON_RESET:
            ESP_LOGI(TAG, "Reset command acknowledged by 0x%02X", pkt->src_addr);
            break;
        default:
            ESP_LOGD(TAG, "Tracking Radio TC ACK received: ID=%d", message_ID);
            break;
    }
}

void tracking_radio_handle_telemetry_response(RS485_packet_t *pkt) {
    if (pkt == NULL) return;
    uint8_t message_ID = pkt->msg_desc.id;
    switch (message_ID) {
        case TLM_COMMON_IDENT:
             if (pkt->length >= sizeof(TlmIdentificationPayload_t)) {
                TlmIdentificationPayload_t tlm;
                memcpy(&tlm, pkt->data, sizeof(TlmIdentificationPayload_t));
                ESP_LOGI(TAG, "Tracking Radio Info: Ver=%d): FW=%d.%d, Uptime=%us", 
                         tlm.interface_version,
                         tlm.firmware_major, tlm.firmware_minor, 
                         tlm.uptime_seconds);

                tracking_radio_node_store_status((TrackingRadioStatus_t*)&tlm);
            } else {
                ESP_LOGW(TAG, "Tracking Radio Identification response too short");
            }
            break;
        default:
            ESP_LOGD(TAG, "Tracking Radio TLM Response received: ID=%d", message_ID);
            break;
    }
}

void tracking_radio_confirm_beacon_ack(void) {
    if (ack_semaphore != NULL) {
        xSemaphoreGive(ack_semaphore);
    }
}

void tracking_radio_node_store_status(const TrackingRadioStatus_t *status) {
    if (status == NULL) return;
    if (xSemaphoreTake(radio_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        memcpy(&radio_store.status, status, sizeof(TrackingRadioStatus_t));
        radio_store.status_age_ms = GET_TIME_MS();
        radio_store.status_valid = true;
        xSemaphoreGive(radio_mutex);
        ESP_LOGI(TAG, "Tracking Radio status stored.");
    } else {
        ESP_LOGW(TAG, "Failed to acquire mutex to store status");
    }
}

bool tracking_radio_node_get_status(TrackingRadioStatus_t *out_status) {
    if (out_status == NULL) return false;
    bool valid = false;
    if (xSemaphoreTake(radio_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        if (radio_store.status_valid) {
            memcpy(out_status, &radio_store.status, sizeof(TrackingRadioStatus_t));
            valid = true;
        }
        xSemaphoreGive(radio_mutex);
    }
    return valid;
}

bool tracking_radio_node_is_status_valid(void) {
    bool valid = false;
    if (xSemaphoreTake(radio_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        if (radio_store.status_valid) {
            uint32_t age = GET_TIME_MS() - radio_store.status_age_ms;
            if (age < TRACKING_RADIO_TIMEOUT_MS) {
                valid = true;
            }
        }
        xSemaphoreGive(radio_mutex);
    }
    return valid;
}

uint32_t tracking_radio_node_get_status_age_ms(void) {
    uint32_t age = 0xFFFFFFFF;
    if (xSemaphoreTake(radio_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        if (radio_store.status_valid) {
            age = GET_TIME_MS() - radio_store.status_age_ms;
        }
        xSemaphoreGive(radio_mutex);
    }
    return age;
}
