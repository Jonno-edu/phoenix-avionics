// serial.c
#include "serial.h"
#include <stdio.h>
#include "FreeRTOS.h"
#include "task.h"

#if !PICO_BUILD
    #include <unistd.h>
#else
    #include <pico/stdlib.h>
    #include <hardware/uart.h>
#endif

// Circular buffer
static volatile uint8_t rx_buffer[SERIAL_BUFFER_SIZE];
static volatile uint16_t write_index = 0;
static volatile uint16_t read_index = 0;

#if PICO_BUILD
// USB RX callback - drain ALL available data into buffer
static void on_usb_rx(void *param) {
    (void)param;
    
    int ch;
    while ((ch = getchar_timeout_us(0)) != PICO_ERROR_TIMEOUT) {
        serial_buffer_push((uint8_t)ch);
    }
}
#endif

void serial_init(void) {
#if PICO_BUILD
    stdio_set_chars_available_callback(on_usb_rx, NULL);
#endif
}

void serial_send_byte(uint8_t byte) {
#if PICO_BUILD
    if (uart_is_enabled(uart0)) {
        uart_putc(uart0, byte);
    } 
    // Always also send to USB CDC for testing/monitoring
    putchar_raw(byte);
#else
    putchar(byte);
    fflush(stdout);
#endif
}
// NOTE: This function is designed to be called from ISR context ONLY.
// Do NOT call from task context without adding interrupt disable guards.
void serial_buffer_push(uint8_t byte) {
    rx_buffer[write_index] = byte;
    write_index = (write_index + 1) % SERIAL_BUFFER_SIZE;
    
    // If buffer full, overwrite oldest data (move read_index)
    if (write_index == read_index) {
        read_index = (read_index + 1) % SERIAL_BUFFER_SIZE;
    }
}
    
bool serial_bytes_available(void) {
    taskENTER_CRITICAL();
    bool available = (read_index != write_index);
    taskEXIT_CRITICAL();
    return available;
}

uint8_t serial_read_byte(void) {
    taskENTER_CRITICAL();
    if (read_index == write_index) {
        taskEXIT_CRITICAL();
        return 0;
    }
    
    uint8_t byte = rx_buffer[read_index];
    read_index = (read_index + 1) % SERIAL_BUFFER_SIZE;
    taskEXIT_CRITICAL();
    return byte;
}