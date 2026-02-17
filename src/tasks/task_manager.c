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
#include "telemetry_defs.h" // For TC_OBC_LOG_LEVEL
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

// Temporary Test Injection Task
// static void vTestInjectionTask(void *pvParameters) {
//     uint8_t level = 1; // Start with Error (1)
//     const TickType_t xDelay = pdMS_TO_TICKS(5000);

//     // Give system time to boot
//     vTaskDelay(pdMS_TO_TICKS(3000));
//     printf("--- TEST INJECTOR STARTED ---\n");

//     for (;;) {
//         vTaskDelay(xDelay);
        
//         // Toggle Level: 1 (Error) <-> 3 (Info)
//         level = (level == 1) ? 3 : 1;
//         uint8_t payload[1] = { level };

//         printf(">>> INJECTING: Log Level = %d\n", level);
        
//         // Inject command via Loopback Interface
//         cmd_inject_packet(IF_LOOPBACK, MSG_TYPE_TELECOMMAND, TC_OBC_LOG_LEVEL, payload, 1);
//     }
// }

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
    
    // Create Test Injector
    // xTaskCreate(vTestInjectionTask, "TestInject", 1024, NULL, tskIDLE_PRIORITY + 1, NULL);

    xTaskCreate(vStartupCheck, "StartupCheck", 512, NULL, tskIDLE_PRIORITY, NULL);
}
