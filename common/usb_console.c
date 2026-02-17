#include "usb_console.h"
#include <stdio.h>
#include <string.h>

#if PICO_BUILD
    #include <pico/stdlib.h>
#endif

void console_init(void) {
    #if PICO_BUILD
        stdio_init_all();
    #endif
}

void console_send(const uint8_t *data, size_t length) {
    if (data == NULL || length == 0) return;
    
    // stdio_put_string / printf handle raw bytes fine or we can use fwrite
    for (size_t i = 0; i < length; i++) {
        putchar(data[i]);
    }
}

#if PICO_BUILD
static int16_t cached_char = -1;
#endif

bool console_bytes_available(void) {
#if PICO_BUILD
    if (cached_char != -1) return true;
    int c = getchar_timeout_us(0);
    if (c != PICO_ERROR_TIMEOUT) {
        cached_char = (int16_t)c;
        return true;
    }
#endif
    return false;
}

uint8_t console_read_byte(void) {
#if PICO_BUILD
    if (cached_char != -1) {
        uint8_t b = (uint8_t)cached_char;
        cached_char = -1;
        return b;
    }
    int c = getchar_timeout_us(0);
    if (c != PICO_ERROR_TIMEOUT) {
        return (uint8_t)c;
    }
#endif
    return 0;
}

void console_buffer_push(uint8_t byte) {
    (void)byte;
    // Placeholder for SIL/Host builds
}
