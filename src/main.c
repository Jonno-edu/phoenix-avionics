// main.c
#include <stdio.h>
#include <stdbool.h>
#include <stdlib.h>
#include <FreeRTOS.h>
#include <task.h>
#include "serial.h"
#include "rs485_protocol.h"
#include "system_data.h"
#include "i2c.h"
#include "baroMS5607.h"

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
        serial_buffer_push(ch);
    }
}
#endif

// RS485 Processing Task
void vRS485Task(void *pvParameters) {
    (void)pvParameters;
    
    RS485Packet_t packet;
    
    while (true) {
        // Feed buffered bytes into the protocol parser
        while (serial_bytes_available()) {
            rs485_process_byte(serial_read_byte());
        }

        if (rs485_get_packet(&packet)) {
            // Is it for us? (OBC = 1)
            if (packet.dest_addr == ADDR_OBC) {
                // Determine Message Type & ID
                uint8_t type = GET_MSG_TYPE(packet.msg_desc);
                uint8_t id   = GET_MSG_ID(packet.msg_desc);

                if (type == MSG_TYPE_TELECOMMAND && id == ID_CMD_RESET) {
                     printf("RESET COMMAND RECEIVED!\n");
                     // Handle reset...
                }
                
                // Echo back an ACK to prove it works
                // Send "ACK" (Type 3) with same ID
                uint8_t ack_desc = BUILD_MSG_DESC(MSG_TYPE_TC_ACK, id);
                rs485_send_packet(packet.src_addr, ack_desc, NULL, 0);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(5)); // Yield
    }
}

int main() {
#if PICO_BUILD
    stdio_init_all();
    serial_init();
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
    rs485_init(serial_send_byte);

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
    serial_init();
    rs485_init(serial_send_byte);
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

    while (true) {
        sleep_ms(1000);
    }
    
    return 0;
}
