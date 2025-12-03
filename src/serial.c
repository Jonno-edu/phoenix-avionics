// serial.c
#include "serial.h"
#include <stdio.h>

#if !PICO_BUILD
    #include <unistd.h>
#else
    #include <pico/stdlib.h>
#endif

#if PICO_BUILD
// USB RX callback - triggered when data arrives
static void on_usb_rx(void *param) {
    (void)param;
    
    int ch = getchar_timeout_us(0);
    if (ch != PICO_ERROR_TIMEOUT) {
        // Send 0xAA back immediately
        putchar(0xAA);
    }
}
#endif

void serial_init(void) {
#if PICO_BUILD
    // Register callback for when USB data arrives
    stdio_set_chars_available_callback(on_usb_rx, NULL);
#endif
}

void serial_send_byte(uint8_t byte) {
    putchar(byte);
}
