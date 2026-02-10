#include "command_handler_task.h"
#include "task_manager.h"
#include "core/logging.h"
#include <FreeRTOS.h>
#include <task.h>

static const char *TAG = "CMD_HANDLER";

static void vCommandHandlerTask(void *pvParameters) {
    (void)pvParameters;
    
    // Event driven loop
    while (true) {
        // TODO: Block on xCommandQueue
        vTaskDelay(pdMS_TO_TICKS(100)); // Placeholder
        
        // TODO: Validation & Routing
        // if (is_arm_command) xQueueSend(xPilotCommandQueue, ...);
    }
}

void command_handler_task_init(void) {
    TaskHandle_t hCmd = NULL;

    xTaskCreate(vCommandHandlerTask, "CmdHandler", 2048, NULL, PRIORITY_COPILOT_CMD, &hCmd);
    vTaskCoreAffinitySet(hCmd, (1 << 1)); // Core 1
    
    ESP_LOGI(TAG, "Command Handler Initialized on Core 1");
}
