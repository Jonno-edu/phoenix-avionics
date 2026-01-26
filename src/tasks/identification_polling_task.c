#include "identification_polling_task.h"
#include "task_manager.h"
#include "rs485_protocol.h"
#include "telemetry_defs.h"
#include "core/obc_data.h"
#include <FreeRTOS.h>
#include <task.h>
#include <stdio.h>
#include "core/logging.h"

static const char *TAG = "ID_POLL";

static void vIdentificationPollingTask(void *pvParameters) {
    (void)pvParameters;
    
    ESP_LOGI(TAG, "Starting Identification Polling Task...");

    // List of devices to poll for identification/status
    const uint8_t device_addresses[] = {
        ADDR_EPS,
        ADDR_TRACKING_RADIO
    };
    const size_t num_devices = sizeof(device_addresses) / sizeof(device_addresses[0]);
    
    while (true) {
        for (size_t i = 0; i < num_devices; i++) {
            uint8_t target_addr = device_addresses[i];

            // Send Status/Identification Request to Device
            ESP_LOGI(TAG, "[OBC -> 0x%02X] Polling Identification...", target_addr);
            ESP_LOGD(TAG, "Raw bytes: [00 %02X %02X %02X]", target_addr, ADDR_OBC, (ID_TLM_IDENTIFICATION << 3) | MSG_TYPE_TLM_REQ);
            
            rs485_send_packet(target_addr, MSG_TYPE_TLM_REQ, ID_TLM_IDENTIFICATION, NULL, 0);
            
            // Add delay between polling each device to avoid bus collisions
            // This gives the slave time to process and reply before we talk again
            vTaskDelay(pdMS_TO_TICKS(50));
        }
        
        // Use dynamic telemetry rate
        uint8_t rate = system_config_get_telem_rate();
        uint32_t delay_ms = (rate > 0) ? (1000 / rate) : 1000;
        
        vTaskDelay(pdMS_TO_TICKS(delay_ms));
    }
}

void identification_polling_task_init(void) {
    xTaskCreate(
        vIdentificationPollingTask,
        "ID_Poll",
        1024,
        NULL,
        PRIORITY_ID_POLLING,
        NULL
    );
}
