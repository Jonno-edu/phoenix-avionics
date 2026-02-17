// heartbeat_task.c
#include "heartbeat_task.h"
#include "task_manager.h"
#include "core/logging_shim.h"
#include <FreeRTOS.h>
#include <task.h>
#include <stdio.h>

static void vHeartbeatTask(void *pvParameters) {
    (void)pvParameters;
    while (true) {
        LOGI("HB", "Phoenix Avionics Alive - Uptime: %lu s", 
               (unsigned long)(xTaskGetTickCount() * portTICK_PERIOD_MS / 1000));
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}

void heartbeat_task_init(void) {
    xTaskCreate(
        vHeartbeatTask,
        "Heartbeat",
        512,
        NULL,
        PRIORITY_HEARTBEAT,
        NULL
    );
}
