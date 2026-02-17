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

#define TELEMETRY_RATE_MS 1000

static void vTelemetryTask(void *pvParameters) {
    (void)pvParameters;
    TrackingBeacon_t tx_beacon;
    
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(TELEMETRY_RATE_MS);

    while (true) {
        vTaskDelayUntil(&xLastWakeTime, xFrequency);

        getRocketTrackingInfo(&tx_beacon);
        
        bool result = tracking_radio_send_beacon(&tx_beacon);
        if (!result) {
            ESP_LOGW(TAG, "Tracking beacon transmission failed");
        }

        vTaskDelay(pdMS_TO_TICKS(50));
        tracking_radio_request_health();
        vTaskDelay(pdMS_TO_TICKS(100));
        eps_request_power_status();
        vTaskDelay(pdMS_TO_TICKS(100));
        eps_request_measurements();
    }
}

void telemetry_task_init(void) {
    TaskHandle_t xHandle = NULL;

    xTaskCreate(
        vTelemetryTask,
        "Telemetry",
        2048,
        NULL,
        tskIDLE_PRIORITY + 2,
        &xHandle
    );

    vTaskCoreAffinitySet(xHandle, (1 << 1));
}
