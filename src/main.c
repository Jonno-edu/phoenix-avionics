#include <stdio.h>
#include <FreeRTOS.h>
#include <task.h>

#if PICO_BUILD
#include <pico/stdlib.h>
#include <pico/stdio_usb.h>
#endif

#include "common/pubsub.h"
#include "modules/sensors/sensors.h"
#include "modules/estimator/estimator.h"
#include "modules/housekeeping/housekeeping.h"

// Stack overflow hook — called by FreeRTOS when a task overflows
void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName) {
    (void)xTask;
    printf("STACK OVERFLOW in task: %s\n", pcTaskName);
    for (;;); // halt — you will see this in debugger
}

// Heap exhaustion hook
void vApplicationMallocFailedHook(void) {
    printf("MALLOC FAILED - heap exhausted\n");
    for (;;);
}

static void heartbeat_task(void *pvParameters) {
    (void)pvParameters;
    while (1) {
        printf("Heartbeat: System is running...\n");
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

int main(void) {
#if PICO_BUILD
    // Pico SDK hardware init
    stdio_init_all();
    // Wait for USB to enumerate (up to 5s) before first print
    // This prevents losing early boot messages
    uint32_t usb_wait = 0;
    while (!stdio_usb_connected() && usb_wait < 5000) {
        sleep_ms(10);
        usb_wait += 10;
    }
#endif

    printf("Phoenix OBC Booting...\n");

    // 1. Initialize the pub/sub queues first
    pubsub_init();

    // 2. Create the heartbeat task
    xTaskCreate(heartbeat_task, "heartbeat", 256, NULL, 1, NULL);

    // 3. Initialize modules (Optional: Commented out for debugging)
    // sensors_init();
    // estimator_init();

    /*
    xTaskCreate(housekeeping_task,
            "housekeeping",
            2048,          // stack words — rs485_send_packet needs ~1200 bytes alone
            NULL,
            2,            // priority: lower than sensors, higher than logging
            NULL);
    */

    // 4. Hand control over to FreeRTOS
    vTaskStartScheduler();

    while (1) {
        // Should never reach here if the scheduler starts successfully
    }
    return 0;
}
