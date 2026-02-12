#include "task_manager.h"
#include "queue_manager.h"
#include "pilot_tasks.h"
#include "command_handler_task.h"

#include "heartbeat_task.h"
#include "rs485_task.h"
#include "telemetry_task.h" // Keeping for now, but will likely be deprecated/merged
#include "sensors_task.h"
#include "hil_sensor_task.h"
#include "core/eps_node.h"
#include "core/tracking_radio_node.h"
#include <FreeRTOS.h>
#include <task.h>
#include <stdio.h>

#if !PICO_BUILD
    #include "core/usb_console.h"
#endif

// Temporary startup check
static void vStartupCheck(void *pvParameters) {
    vTaskDelay(pdMS_TO_TICKS(100));
    printf("\n=== TASKS STARTED ===\n");
    // TODO: Iterate handles if possible, or just trust the init
    vTaskDelete(NULL);
}

void tasks_create_all(void) {
    queue_manager_init();

    // Core 0: Pilot
    pilot_tasks_init();
    
    // Existing HIL or Sensor Task (Pinned to Core 0)
    hil_sensor_task_init(); 
    // sensors_task_init(); 
    heartbeat_task_init();

    // Core 1: Co-Pilot
    command_handler_task_init();
    
    rs485_task_init(); // Needs refactoring to be RX/TX only
    telemetry_task_init(); // Deprecated by TelemSnapshot + RS485 TX logic? 
                             // Keeping enabled if it handles Comms logic required for now
    
    eps_node_init();
    tracking_radio_node_init();

    xTaskCreate(vStartupCheck, "StartupCheck", 512, NULL, tskIDLE_PRIORITY, NULL);
}
