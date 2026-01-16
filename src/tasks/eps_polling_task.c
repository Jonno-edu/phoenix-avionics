#include "eps_polling_task.h"
#include "task_manager.h"
#include "rs485_protocol.h"
#include <FreeRTOS.h>
#include <task.h>
#include <stdio.h>

static void vEPSPollingTask(void *pvParameters) {
    (void)pvParameters;
    
    printf("Starting EPS Polling Task...\n");
    
    while (true) {
        // Send Status/Identification Request to EPS
        printf("[OBC -> EPS] Polling Status...\n");
        
        uint8_t msg_desc = BUILD_MSG_DESC(MSG_TYPE_TLM_REQ, ID_TLM_IDENTIFICATION);
        rs485_send_packet(ADDR_EPS, msg_desc, NULL, 0);
        
        printf("\n");

        // Wait 5 seconds
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}

void eps_polling_task_init(void) {
    xTaskCreate(
        vEPSPollingTask,
        "EPS_Poll",
        1024,
        NULL,
        PRIORITY_EPS_POLLING,
        NULL
    );
}
