// main.c
#include <stdio.h>
#include <stdbool.h>
#include <stdlib.h>
#include <FreeRTOS.h>
#include <task.h>
#include "usb_console.h"
#include "rs485_protocol.h"
#include "rs485_hal.h"
#include "system_data.h"

#if PICO_BUILD
    #include "i2c.h"
    #include "baroMS5607.h"
#endif

#if !PICO_BUILD
    #include <unistd.h>
#else
    #include <pico/stdlib.h>
#endif

void vApplicationStackOverflowHook(TaskHandle_t xTask, char* pcTaskName) {
#if PICO_BUILD
    panic("Stack overflow. Task: %s\n", pcTaskName);
#else
    exit(1);
#endif
}

void vApplicationMallocFailedHook() {
#if PICO_BUILD
    panic("malloc failed");
#else
    exit(1);
#endif
}

// Send to both RS485 bus and USB Console
void mux_send_byte(uint8_t byte) {
#if PICO_BUILD
    // Send to real RS485 bus
    rs485_hal_send_byte(byte);
#endif
    // Also send to USB console (for debugging/monitoring)
    console_send_byte(byte);
}

// Heartbeat Task - Prints status every 5 seconds
void vHeartbeatTask(void *pvParameters) {
    (void)pvParameters;
    while (true) {
        printf("[HB] Phoenix Avionics Alive - Uptime: %lu s\n", 
               (unsigned long)(xTaskGetTickCount() * portTICK_PERIOD_MS / 1000));
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}


// EPS Polling Task
void vEPSPollingTask(void *pvParameters) {
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

// RS485 Processing Task
void vRS485Task(void *pvParameters) {
    (void)pvParameters;
    
    RS485Packet_t packet;
    
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

        if (rs485_get_packet(&packet)) {
            // Is it for us? (OBC = 1)
            if (packet.dest_addr == ADDR_OBC) {
                // Determine Message Type & ID
                uint8_t type = GET_MSG_TYPE(packet.msg_desc);
                uint8_t id   = GET_MSG_ID(packet.msg_desc);

                if (type == MSG_TYPE_TELECOMMAND) {
                    if (id == ID_CMD_RESET) {
                        printf("RESET COMMAND RECEIVED!\n");
                    }
                    // Echo back an ACK
                    uint8_t ack_desc = BUILD_MSG_DESC(MSG_TYPE_TC_ACK, id);
                    rs485_send_packet(packet.src_addr, ack_desc, NULL, 0);
                }
                else if (type == MSG_TYPE_TLM_REQ) {
                    if (id == ID_TLM_IDENTIFICATION) {
                        SystemData_t sys_data;
                        uint8_t frame_buffer[SYSTEM_DATA_FRAME_LENGTH];
                        
                        system_data_get(&sys_data);
                        system_data_pack(&sys_data, frame_buffer);
                        
                        uint8_t resp_desc = BUILD_MSG_DESC(MSG_TYPE_TLM_RESP, id);
                        rs485_send_packet(packet.src_addr, resp_desc, frame_buffer, SYSTEM_DATA_FRAME_LENGTH);
                    }
                }
            }
        }
        vTaskDelay(pdMS_TO_TICKS(5)); // Yield
    }
}

int main() {
#if PICO_BUILD
    stdio_init_all();
    
    // Give USB a moment to stabilize so we don't miss early messages
    for (int i = 0; i < 5; i++) {
        printf("Booting in %d...\n", 5-i);
        sleep_ms(200);
    }
    
    printf("\n--- Phoenix Avionics Starting ---\n");
    
    console_init();
    system_data_init();

    printf("Initializing RS485 HAL...\n");
    // RS485 Bus A hardware (UART1: GPIO 4/5, DE/RE: GPIO 3)
    rs485_hal_init();

    // Register RS485 protocol TX callback to the Mux (sends to both RS485 and USB)
    rs485_init(mux_send_byte);

    printf("Initializing Sensors...\n");
    // Initialize hardware sensors
    if (!I2C_init()) {
        printf("CRITICAL: I2C initialization failed!\n");
    }

    I2C_scan_bus();

    if (!BAROMS5607_init()) {
        printf("CRITICAL: Barometer initialization failed!\n");
    }
    printf("Init complete, starting scheduler.\n");
    sleep_ms(1000);
#else
    setvbuf(stdout, NULL, _IONBF, 0);
    console_init();
    rs485_init(console_send_byte);
    system_data_init();
#endif

#if !PICO_BUILD
    // SIL: Create serial RX polling task (high priority to simulate interrupt)
    xTaskCreate(
        vConsoleRxTask,
        "ConsoleRx",
        1024,
        NULL,
        tskIDLE_PRIORITY + 3,  // Higher than RS485 task to preempt it
        NULL
    );
#endif

    // RS485 task
    xTaskCreate(
        vRS485Task,
        "RS485",
        1024,
        NULL,
        tskIDLE_PRIORITY + 2,
        NULL
    );

    // EPS Polling task
    xTaskCreate(
        vEPSPollingTask,
        "EPS_Poll",
        1024,
        NULL,
        tskIDLE_PRIORITY + 2,
        NULL
    );

    // Heartbeat task
    xTaskCreate(
        vHeartbeatTask,
        "Heartbeat",
        512,
        NULL,
        tskIDLE_PRIORITY + 1,
        NULL
    );

    vTaskStartScheduler();

    // Should never reach here - scheduler runs forever
    // If we do reach here, something went wrong
#if PICO_BUILD
    panic("Scheduler returned unexpectedly!");
#else
    printf("ERROR: Scheduler returned unexpectedly!\n");
    return 1;
#endif
}
