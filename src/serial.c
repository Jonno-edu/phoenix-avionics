// serial.c
#include "serial.h"
#include <stdio.h>

#if !PICO_BUILD
    #include <unistd.h>
#else
    #include <pico/stdlib.h>
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
        uint8_t byte = (uint8_t)ch;
        
        rx_buffer[write_index] = byte;
        write_index = (write_index + 1) % SERIAL_BUFFER_SIZE;
        
        // If buffer full, move read pointer (overwrite old data)
        if (write_index == read_index) {
            read_index = (read_index + 1) % SERIAL_BUFFER_SIZE;
        }
    }
}
#endif

void serial_init(void) {
#if PICO_BUILD
    stdio_set_chars_available_callback(on_usb_rx, NULL);
#endif
}

void serial_send_byte(uint8_t byte) {
    putchar(byte);
    fflush(stdout);
}

void serial_process_commands(void) {
    // Scan entire buffer between read_index and write_index
    uint16_t current = read_index;
    
    while (current != write_index) {
        uint8_t byte = rx_buffer[current];
        
        // Found a command!
        if (byte == COMMAND_BYTE) {
            // Execute command
            serial_send_byte(0xAA);
            serial_send_byte('\n');
            
            // Move read pointer to just after this command
            read_index = (current + 1) % SERIAL_BUFFER_SIZE;
            return;  // Process one command per call
        }
        
        // Move to next byte
        current = (current + 1) % SERIAL_BUFFER_SIZE;
    }
    
    // No commands found - discard all scanned garbage
    read_index = write_index;
}
