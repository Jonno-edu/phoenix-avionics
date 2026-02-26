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
#include "bsp_init.h"

#include "modules/datalink/datalink.h"
#include "phoenix_icd.h"

/* Static memory for all application tasks */
static StaticTask_t heartbeat_tcb;
static StackType_t  heartbeat_stack[512];

static StaticTask_t housekeeping_tcb;
static StackType_t  housekeeping_stack[2048];

static void heartbeat_task(void *pvParameters) {
    (void)pvParameters;
    while (1) {
        // printf("Heartbeat: System is running...\n");
        datalink_sendLog(LOG_LEVEL_DEBUG, "Heartbeat: System is running...");

        // Print stack headroom for each task (in words)
        // printf("[STACK] heartbeat:    %lu words free\n",
        //        uxTaskGetStackHighWaterMark(NULL));
        datalink_sendLog(LOG_LEVEL_DEBUG, "[STACK] heartbeat:    %lu words free", 
            uxTaskGetStackHighWaterMark(NULL));

        vTaskDelay(pdMS_TO_TICKS(5000)); // every 5s
    }
}

int main(void) {
#if PICO_BUILD
    // Unified BSP hardware init (includes stdio_init_all and delays)
    bsp_hardware_init();

    // Extra wait for USB to enumerate (up to 5s) before first print
    // This prevents losing early boot messages
    uint32_t usb_wait = 0;
    while (!stdio_usb_connected() && usb_wait < 2000) {
        sleep_ms(10);
        usb_wait += 10;
    }
#endif

    rs485_hal_set_raw_debug(false); 
    // printf("Phoenix OBC Booting...\n");
    datalink_sendLog(LOG_LEVEL_INFO, "Phoenix OBC Booting...");

    // 1. Initialize the nORB queues first
    norb_init();

    // 2. Create the heartbeat task
    xTaskCreateStatic(heartbeat_task, "heartbeat", 512, NULL, 1,
                      heartbeat_stack, &heartbeat_tcb);

    // 3. Initialize modules
    sensors_init();
    estimator_init();

    // 4. Create the housekeeping task (Step 3: datalink_init)
    xTaskCreateStatic(housekeeping_task,
                      "housekeeping",
                      2048,   // stack words — rs485_send_packet needs ~1200 bytes alone
                      NULL,
                      2,      // priority
                      housekeeping_stack,
                      &housekeeping_tcb);

    // 5. Hand control over to FreeRTOS
    vTaskStartScheduler();

    while (1) {
        // Should never reach here if the scheduler starts successfully
    }
    return 0;
}

/* ----------------------------------------------------------------
 * FreeRTOS static-allocation callbacks
 * Required when configSUPPORT_STATIC_ALLOCATION = 1 and
 * configSUPPORT_DYNAMIC_ALLOCATION = 0.
 * ---------------------------------------------------------------- */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer,
                                    StackType_t  **ppxIdleTaskStackBuffer,
                                    uint32_t      *pulIdleTaskStackSize )
{
    static StaticTask_t xIdleTaskTCB;
    static StackType_t  uxIdleTaskStack[ configMINIMAL_STACK_SIZE ];

    *ppxIdleTaskTCBBuffer   = &xIdleTaskTCB;
    *ppxIdleTaskStackBuffer = uxIdleTaskStack;
    *pulIdleTaskStackSize   = configMINIMAL_STACK_SIZE;
}

void vApplicationGetTimerTaskMemory( StaticTask_t **ppxTimerTaskTCBBuffer,
                                     StackType_t  **ppxTimerTaskStackBuffer,
                                     uint32_t      *pulTimerTaskStackSize )
{
    static StaticTask_t xTimerTaskTCB;
    static StackType_t  uxTimerTaskStack[ configTIMER_TASK_STACK_DEPTH ];

    *ppxTimerTaskTCBBuffer   = &xTimerTaskTCB;
    *ppxTimerTaskStackBuffer = uxTimerTaskStack;
    *pulTimerTaskStackSize   = configTIMER_TASK_STACK_DEPTH;
}


