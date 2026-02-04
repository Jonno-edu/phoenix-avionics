// telemetry_task.c

#include "telemetry_task.h"
#include "core/rocket_data.h"
#include "core/logging.h"
#include "core/eps_data.h"
#include "telemetry_defs.h"
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

        // ====================================================================
        // 1. BROADCAST ROCKET STATE (To Tracking Radio / Ground)
        // ====================================================================
        getRocketTrackingInfo(&tx_beacon);
        
        ESP_LOGI(TAG, "Sending Tracking Beacon (%zu bytes)", sizeof(TrackingBeacon_t));
        // ESP_LOG_BUFFER_HEX(TAG, &tx_beacon, sizeof(TrackingBeacon_t));

        rs485_send_packet(
            ADDR_TRACKING_RADIO, // 0x03
            MSG_TYPE_TLM_RESP,
            ID_TLM_TRACKING_BEACON,
            (uint8_t*)&tx_beacon,
            sizeof(TrackingBeacon_t)
        );

        // ====================================================================
        // 2. POLL SUBSYSTEMS (Staggered to prevent collisions)
        // ====================================================================

        // A. Tracking Radio Status
        vTaskDelay(pdMS_TO_TICKS(50));
        rs485_send_packet(ADDR_TRACKING_RADIO, MSG_TYPE_TLM_REQ, ID_TLM_STATUS, NULL, 0);

        // B. EPS Power Status
        vTaskDelay(pdMS_TO_TICKS(100)); // Allow time for Radio reply
        eps_request_power_status();

        // C. EPS Measurements
        vTaskDelay(pdMS_TO_TICKS(100)); // Allow time for EPS reply
        eps_request_measurements();
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
