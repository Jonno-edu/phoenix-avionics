// main.c
#include <stdio.h>
#include <stdbool.h>
#include <stdlib.h>
#include <FreeRTOS.h>
#include <task.h>
#include "usb_console.h"
#include "rs485_protocol.h"
#include "system_data.h"

#if PICO_BUILD
    #include "i2c.h"
    #include "baroMS5607.h"
#endif

#if !PICO_BUILD
    #include <unistd.h>
#else
    #include <pico/stdlib.h>
    #include <hardware/gpio.h>
    #include <hardware/uart.h>
    #include <hardware/irq.h>
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

#if PICO_BUILD
// UART RX Interrupt handler
void on_uart_rx() {
    while (uart_is_readable(uart0)) {
        uint8_t ch = uart_getc(uart0);
        console_buffer_push(ch);
    }
}
#endif

// RS485 Processing Task
void vRS485Task(void *pvParameters) {
    (void)pvParameters;
    
    RS485Packet_t packet;
    
    while (true) {
        // Feed buffered bytes into the protocol parser
        while (console_bytes_available()) {
            rs485_process_byte(console_read_byte());
        }

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
    console_init();
    system_data_init();

    // UART Setup for RS485
    uart_init(uart0, 115200);
    gpio_set_function(0, GPIO_FUNC_UART); // TX
    gpio_set_function(1, GPIO_FUNC_UART); // RX
    
    // Enable UART Interrupts
    uart_set_irq_enables(uart0, true, false);
    irq_set_exclusive_handler(UART0_IRQ, on_uart_rx);
    irq_set_enabled(UART0_IRQ, true);

    // NOW it's safe to register the callback
    rs485_init(console_send_byte);

    // Initialize hardware sensors
    if (!I2C_init()) {
        printf("CRITICAL: I2C initialization failed!\n");
    }

    I2C_scan_bus();

    if (!BAROMS5607_init()) {
        printf("CRITICAL: Barometer initialization failed!\n");
    }
    sleep_ms(2000);
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
