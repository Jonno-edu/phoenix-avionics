// telemetry_task.c

#include "telemetry_task.h"
#include "core/rocket_data.h"
#include "core/logging.h"
#include "core/eps_node.h"
#include "core/tracking_radio_node.h"
#include "telemetry_defs.h"
#include "rs485_protocol.h"
#include <FreeRTOS.h>
#include <task.h>

static const char *TAG = "TELEMETRY";

// 1Hz Telemetry Ratex
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
        
        // Send via Tracking Radio Node (handles valid packet wrapping)
        bool result = tracking_radio_send_beacon(&tx_beacon);
        if (!result) {
            ESP_LOGW(TAG, "Tracking beacon transmission failed");
        }

        // ====================================================================
        // 2. POLL SUBSYSTEMS (Staggered to prevent collisions)
        // ====================================================================

        // A. Tracking Radio Status
        vTaskDelay(pdMS_TO_TICKS(50));
        tracking_radio_request_health();

        // B. EPS Power Status
        vTaskDelay(pdMS_TO_TICKS(100)); // Allow time for Radio reply
        eps_request_power_status();

        // C. EPS Measurements
        vTaskDelay(pdMS_TO_TICKS(100)); // Allow time for EPS reply
        eps_request_measurements();
    }
}

void telemetry_task_init(void) {
    TaskHandle_t xHandle = NULL;

    xTaskCreate(
        vTelemetryTask,
        "Telemetry",
        2048,                        // Stack size (Beacon is ~82 bytes, plenty of room)
        NULL,
        tskIDLE_PRIORITY + 2,        // Priority: Moderate (Above IDLE, below hardware IO)
        &xHandle
    );

    // Pilot & Co-Pilot Model:
    // Core 1 (Co-Pilot): Handles Comms, Logging and Telemetry
    vTaskCoreAffinitySet(xHandle, (1 << 1));
}
