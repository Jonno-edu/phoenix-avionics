#include "housekeeping.h"
#include <stdio.h>
#include "hal/rs485_hal.h"
#include "modules/datalink/datalink.h"
#include "modules/tracking_radio/tracking_radio.h"
#include "modules/eps/eps.h"
#include "common/pubsub.h"
#include "common/topics.h"

#include "FreeRTOS.h"
#include "task.h"

#define POLL_PERIOD_MS             1000
#define TRACKING_RADIO_TIMEOUT_MS  100
#define EPS_TIMEOUT_MS             100

void housekeeping_task(void *pvParameters)
{
    (void)pvParameters;

    // datalink_init() owns all hardware setup — call it here once
    datalink_init();
    printf("[HK] datalink init OK\n");

    TickType_t last_wake = xTaskGetTickCount();

    while (1)
    {
        TlmIdentificationPayload_t ident;

        // --- Tracking radio health ---
        if (tracking_radio_request_ident(&ident, TRACKING_RADIO_TIMEOUT_MS)) {
            publish(TOPIC_TRACKING_RADIO_IDENT, &ident);
            printf("[HK] tracking_radio: OK\n");
        } else {
            printf("[HK] tracking_radio: TIMEOUT\n");
        }

        // --- EPS health ---
        if (eps_request_ident(&ident, EPS_TIMEOUT_MS)) {
            publish(TOPIC_EPS_IDENT, &ident);
            printf("[HK] eps: OK\n");
        } else {
            printf("[HK] eps: TIMEOUT\n");
        }

        // Safe: called from task, not ISR
        rs485_hal_print_raw_log();

        // Print stack headroom for housekeeping task
        printf("[STACK] housekeeping: %lu words free\n",
               uxTaskGetStackHighWaterMark(NULL));

        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(POLL_PERIOD_MS));
    }
}
