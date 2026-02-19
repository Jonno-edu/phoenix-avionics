#include <stdio.h>
#include <FreeRTOS.h>
#include <task.h>

#if PICO_BUILD
#include <pico/stdlib.h>
#include <pico/stdio_usb.h>
#endif

#include "norb/norb.h"
#include "modules/sensors/sensors.h"
#include "modules/estimator/estimator.h"
#include "modules/housekeeping/housekeeping.h"
#include "hal/rs485_hal.h"

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

        // Print stack headroom for each task (in words)
        printf("[STACK] heartbeat:    %lu words free\n",
               uxTaskGetStackHighWaterMark(NULL));

        vTaskDelay(pdMS_TO_TICKS(5000)); // every 5s
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

    rs485_hal_set_raw_debug(false); 
    printf("Phoenix OBC Booting...\n");

    // 1. Initialize the nORB queues first
    norb_init();

    // 2. Create the heartbeat task
    xTaskCreate(heartbeat_task, "heartbeat", 512, NULL, 1, NULL);

    // 3. Initialize modules
    sensors_init();
    estimator_init();

    // 4. Create the housekeeping task (Step 3: datalink_init)
    xTaskCreate(housekeeping_task,
            "housekeeping",
            2048,          // stack words — rs485_send_packet needs ~1200 bytes alone
            NULL,
            2,             // priority
            NULL);

    // 5. Hand control over to FreeRTOS
    vTaskStartScheduler();

    while (1) {
        // Should never reach here if the scheduler starts successfully
    }
    return 0;
}
