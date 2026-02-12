// rs485_task.c

#include "core/logging.h"
#include "rs485_task.h"
#include "task_manager.h"
#include "rs485_protocol.h"
#include "hal/rs485_hal.h"
#include "core/usb_console.h"
#include "core/obc_data.h"
#include "core/rs485_monitor.h" // Added for monitor logging
#include <FreeRTOS.h>
#include <task.h>
#include <stdio.h>

#if PICO_BUILD
#include <hardware/watchdog.h>
#endif

#include "queue_manager.h"

static const char *TAG = "TRANSPORT";

// Callback to bridge protocol lib to HAL (Egress Bus)
static void rs485_tx_bridge(const uint8_t *data, uint16_t length) {
    // Log internal raw bytes for debugging (Verbose)
    // ESP_LOGD(TAG, "TX BUS (%u):", length);
    // ESP_LOG_BUFFER_HEX(TAG, data, length);

#if PICO_BUILD
    rs485_hal_send(data, length);
#else
    // SIL: Echo to console? No, SIL typically uses console FOR the bus.
    // Ideally SIL should have a separate socket or pty.
    // For now, we assume standard console IS the interface.
    // But wait, if we have USB pipe AND RS485 pipe in SIL, they conflict on stdout.
    // Let's assume SIL only tests the 'Default' output.
    for (uint16_t i = 0; i < length; i++) {
        putchar(data[i]);
    }
    fflush(stdout);
#endif
}

// Callback to bridge protocol lib to USB Console (Egress USB)
static void usb_tx_bridge(const uint8_t *data, uint16_t length) {
    // ALWAYS send packets directed to USB.
    // Even if monitor is in Verbose mode, we need the binary response 
    // to reach the host script (GSE).
    console_send(data, length);
}

// Global instances
static rs485_instance_t main_bus_ctx;  // Pipe B
static rs485_instance_t usb_ctx;       // Pipe A

rs485_instance_t *rs485_get_default_instance(void) {
    return &main_bus_ctx;
}

static void vRS485Task(void *pvParameters) {
    (void)pvParameters;
    
    // Initialize Pipe B (Bus)
    rs485_init(&main_bus_ctx, rs485_tx_bridge, ADDR_OBC);
    
    // Initialize Pipe A (USB)
    rs485_init(&usb_ctx, usb_tx_bridge, ADDR_OBC);

    RS485_packet_t packet;
    CommandEvent_t event;

    ESP_LOGI(TAG, "Transport Task Started (Twin Pipes)");
    
    while (true) {
        // ====================================================================
        // 1. INGRESS: Poll Hardware & Feed Parsers
        // ====================================================================
        
        // Pipe B (RS485 Hardware)
#if PICO_BUILD
        while (rs485_hal_bytes_available()) {
            uint8_t b = rs485_hal_read_byte();
            // Pass to monitor (handles raw streaming or verbose logging)
            rs485_monitor_log_byte(b, false);
            rs485_process_byte(&main_bus_ctx, b);
            
            if (rs485_get_packet(&main_bus_ctx, &packet)) {
                // Wrap and Queue
                event.source_interface = IF_RS485;
                event.packet = packet;
                if (xQueueSend(q_command_inbox, &event, 0) != pdTRUE) {
                    ESP_LOGW(TAG, "Inbox Full - Dropped RS485 Packet");
                }
            }
        }
#endif

        // Pipe A (USB Console)
        while (console_bytes_available()) {
            rs485_process_byte(&usb_ctx, console_read_byte());
            
            if (rs485_get_packet(&usb_ctx, &packet)) {
                // Wrap and Queue
                event.source_interface = IF_USB;
                event.packet = packet;
                if (xQueueSend(q_command_inbox, &event, 0) != pdTRUE) {
                    ESP_LOGW(TAG, "Inbox Full - Dropped USB Packet");
                }
            }
        }

        // ====================================================================
        // 2. EGRESS: Poll Outboxes & Serialize to Wire
        // ====================================================================
        
        // Pipe B (RS485 Bus)
        if (q_rs485_out != NULL && xQueueReceive(q_rs485_out, &packet, 0) == pdTRUE) {
            // Serialize and Send (The Context knows the Callback)
            // Use the data inside the packet to reconstruct the frame
            rs485_send_packet(&main_bus_ctx, 
                              packet.dest_addr, 
                              packet.msg_desc.type, 
                              packet.msg_desc.id, 
                              packet.data, 
                              packet.length);
        }

        // Pipe A (USB Stream)
        // We can process multiple per cycle for higher throughput if needed
        while (q_usb_out != NULL && xQueueReceive(q_usb_out, &packet, 0) == pdTRUE) {
             rs485_send_packet(&usb_ctx, 
                              packet.dest_addr, 
                              packet.msg_desc.type, 
                              packet.msg_desc.id, 
                              packet.data, 
                              packet.length);
        }

        vTaskDelay(pdMS_TO_TICKS(5)); // Yield
    }
}


void rs485_task_init(void) {
    TaskHandle_t xHandle = NULL;

    xTaskCreate(
        vRS485Task,
        "RS485",
        1024,
        NULL,
        PRIORITY_RS485_PROCESSING,
        &xHandle
    );

    // Pilot & Co-Pilot Model:
    // Core 1 (Co-Pilot): Handles I/O, Comms and Logging
    vTaskCoreAffinitySet(xHandle, (1 << 1));
}
