#include "command_handler_task.h"
#include "task_manager.h"
#include "core/logging.h"
#include "queue_manager.h"
#include "tctlm.h"
#include "rs485_protocol.h"
#include <FreeRTOS.h>
#include <task.h>

static const char *TAG = "DISPATCHER";

static void vDispatcherTask(void *pvParameters) {
    (void)pvParameters;
    
    CommandEvent_t event;
    
    ESP_LOGI(TAG, "Dispatcher Task Started");
    
    while (true) {
        // Block indefinitely until a command arrives
        if (xQueueReceive(q_command_inbox, &event, portMAX_DELAY) == pdTRUE) {
            
            // Check destination (Broadcast or Me)
            // Note: Transport layer should have ideally filtered this, but double check is cheap.
            if (event.packet.dest_addr == ADDR_OBC || event.packet.dest_addr == RS485_ADDR_BROADCAST) {
                
                // Routing Logic (The Brain)
                switch(event.packet.msg_desc.type) {
                    case MSG_TYPE_EVENT:
                        TCTLM_processEvent(event.source_interface, &event.packet);
                        break;
                    case MSG_TYPE_TELECOMMAND:
                        TCTLM_processTelecommand(event.source_interface, &event.packet);
                        break;
                    case MSG_TYPE_TC_ACK:
                        TCTLM_processTelecommandAck(event.source_interface, &event.packet);
                        break;
                    case MSG_TYPE_TLM_REQ:
                        TCTLM_processTelemetryRequest(event.source_interface, &event.packet);
                        break;
                    case MSG_TYPE_TLM_RESP:
                        TCTLM_processTelemetryResponse(event.source_interface, &event.packet);
                        break;
                    case MSG_TYPE_BULK:
                        TCTLM_processBulkTransfer(event.source_interface, &event.packet);
                        break;
                    default:
                        TCTLM_processUnknownMessage(event.source_interface, &event.packet);
                        break;
                }
            }
        }
    }
}

void command_handler_task_init(void) {
    TaskHandle_t hDispatcher = NULL;

    xTaskCreate(vDispatcherTask, "Dispatcher", 2048, NULL, PRIORITY_COPILOT_CMD, &hDispatcher);
    vTaskCoreAffinitySet(hDispatcher, (1 << 1)); // Core 1 (Co-Pilot)
    
    ESP_LOGI(TAG, "Dispatcher Initialized on Core 1");
}
