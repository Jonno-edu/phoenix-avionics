// rs485_task.c

#include "core/logging.h"
#include "rs485_task.h"
#include "task_manager.h"
#include "rs485_protocol.h"
#include "hal/rs485_hal.h"
#include "core/usb_console.h"
#include "core/obc_data.h"
#include <FreeRTOS.h>
#include <task.h>
#include <stdio.h>

#if PICO_BUILD
#include <hardware/watchdog.h>
#endif

static const char *TAG = "RS485";

// Callback to bridge protocol lib to HAL
static void rs485_tx_bridge(const uint8_t *data, uint16_t length) {
    // Log internal raw bytes for debugging
    ESP_LOGD(TAG, "TX Raw (%u bytes):", length);
    ESP_LOG_BUFFER_HEX(TAG, data, length);

#if PICO_BUILD
    rs485_hal_send(data, length);
#else
    // SIL: Echo to console for now
    for (uint16_t i = 0; i < length; i++) {
        putchar(data[i]);
    }
    fflush(stdout);
#endif
}

static void vRS485Task(void *pvParameters) {
    (void)pvParameters;
    
    // Initialize the library
    rs485_init(rs485_tx_bridge, ADDR_OBC);

    RS485_packet_t packet;
    
    while (true) {
        // Feed buffered bytes into the protocol parser
#if PICO_BUILD
        // 1. Process bytes from real RS485 Hardware (Bus A)
        while (rs485_hal_bytes_available()) {
            rs485_process_byte(rs485_hal_read_byte());
        }

        // 2. Process bytes from USB Console (Debug/Test)
        while (console_bytes_available()) {
            rs485_process_byte(console_read_byte());
        }
#else
        // SIL: Read from Virtual Serial Port (Console)
        while (console_bytes_available()) {
            rs485_process_byte(console_read_byte());
        }
#endif

        // Use the library's dispatch logic
        rs485_rx_update(&packet);

        vTaskDelay(pdMS_TO_TICKS(5)); // Yield
    }
}

void rs485_task_init(void) {
    xTaskCreate(
        vRS485Task,
        "RS485",
        1024,
        NULL,
        PRIORITY_RS485_PROCESSING,
        NULL
    );
}
