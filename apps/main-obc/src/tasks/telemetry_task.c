#include "telemetry_task.h"
#include "core/rocket_data.h"
#include "core/logging.h"
#include "rs485_protocol.h"
#include <FreeRTOS.h>
#include <task.h>

static const char *TAG = "TELEMETRY";

// 1Hz Telemetry Rate
#define TELEMETRY_RATE_MS 1000

static void vTelemetryTask(void *pvParameters) {
    (void)pvParameters;
    TrackingBeacon_t tx_beacon;
    
    // Initialize wake time for precise periodic execution
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(TELEMETRY_RATE_MS);

    while (true) {
        // Wait for the next cycle
        vTaskDelayUntil(&xLastWakeTime, xFrequency);

        // 1. Get snapshot of the latest tracking data (Already formatted for telemetry)
        getRocketTrackingInfo(&tx_beacon);
        
        // 2. Broadcast packet via RS485
        // Addressed to 0x03
        ESP_LOGI(TAG, "Sending Tracking Beacon (%zu bytes) to 0x03", sizeof(TrackingBeacon_t));
        ESP_LOG_BUFFER_HEX(TAG, &tx_beacon, sizeof(TrackingBeacon_t));

        // Detailed field logging (Debug level)
        ESP_LOGD(TAG, "  Runtime: %u.%03u s | State: %u", 
                tx_beacon.runtime_sec, tx_beacon.runtime_ms, tx_beacon.avionics_state);
        ESP_LOGD(TAG, "  Est Pos: Lat=%.7f, Lon=%.7f, Alt=%d m", 
                tx_beacon.est_lat/1e7, tx_beacon.est_lon/1e7, tx_beacon.est_alt);
        ESP_LOGD(TAG, "  Stack Temp: %.2f C", tx_beacon.temp_stack / 128.0f);

        rs485_send_packet(
            0x03,
            MSG_TYPE_TLM_RESP,
            ID_TLM_TRACKING_BEACON,
            (uint8_t*)&tx_beacon,
            sizeof(TrackingBeacon_t)
        );

        // Send status request to Tracking Radio
        rs485_send_packet(ADDR_TRACKING_RADIO, MSG_TYPE_TLM_REQ, ID_TLM_STATUS, NULL, 0);

    }
}

void telemetry_task_init(void) {
    xTaskCreate(
        vTelemetryTask,
        "Telemetry",
        2048,                        // Stack size (Beacon is ~82 bytes, plenty of room)
        NULL,
        tskIDLE_PRIORITY + 2,        // Priority: Moderate (Above IDLE, below hardware IO)
        NULL
    );
}
