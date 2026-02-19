#include "housekeeping.h"
#include <stdio.h>
#include "hal/rs485_hal.h"
#include "modules/datalink/datalink.h"
#include "modules/nodes/eps/eps.h"
#include "modules/nodes/tracking_radio/tracking_radio.h"
#include "norb/norb.h"
#include "norb/topics.h"

#include "FreeRTOS.h"
#include "task.h"

#define POLL_PERIOD_MS    1000
#define NODE_TIMEOUT_MS   100

void housekeeping_task(void *pvParameters)
{
    (void)pvParameters;

    // datalink_init() owns all hardware setup — call it here once
    datalink_init();
    printf("[HK] datalink init OK\n");

    TickType_t last_wake = xTaskGetTickCount();

    while (1) {
        // Poll each node; node modules own their telemetry details.
        eps_poll(NODE_TIMEOUT_MS);
        tracking_radio_poll(NODE_TIMEOUT_MS);

        // Safe: called from task, not ISR
        rs485_hal_print_raw_log();

        // Print stack headroom for housekeeping task
        printf("[STACK] housekeeping: %lu words free\n",
               uxTaskGetStackHighWaterMark(NULL));

        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(POLL_PERIOD_MS));
    }
}
