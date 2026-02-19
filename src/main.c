#include <stdio.h>
#include <FreeRTOS.h>
#include <task.h>

#if PICO_BUILD
#include <pico/stdlib.h>
#endif

#include "common/pubsub.h"
#include "modules/sensors/sensors.h"
#include "modules/estimator/estimator.h"
#include "modules/housekeeping/housekeeping.h"

int main(void) {
#if PICO_BUILD
    // Pico SDK hardware init
    stdio_init_all();
#endif

    printf("Phoenix OBC Booting Clean Architecture...\n");

    // 1. Initialize the pub/sub queues first
    pubsub_init();

    // 2. Initialize modules (spawns their FreeRTOS tasks)
    sensors_init();
    estimator_init();

    xTaskCreate(housekeeping_task,
            "housekeeping",
            512,          // stack words — increase if needed
            NULL,
            2,            // priority: lower than sensors, higher than logging
            NULL);

    // 3. Hand control over to FreeRTOS
    vTaskStartScheduler();

    while (1) {
        // Should never reach here if the scheduler starts successfully
    }
    return 0;
}
