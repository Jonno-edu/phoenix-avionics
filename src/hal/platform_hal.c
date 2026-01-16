#include "platform_hal.h"
#include <FreeRTOS.h>
#include <task.h>
#include <stdio.h>
#include <stdlib.h>
#include "usb_console.h"

#if PICO_BUILD
    #include "pico/stdlib.h"
    #include "rs485_hal.h"
#else
    #include <unistd.h>
#endif

// ============================================================================
// FreeRTOS Hooks (Platform Specific Implementation)
// ============================================================================

void vApplicationStackOverflowHook(TaskHandle_t xTask, char* pcTaskName) {
    (void)xTask;
#if PICO_BUILD
    panic("Stack overflow. Task: %s\n", pcTaskName);
#else
    (void)pcTaskName;
    printf("Stack overflow. Task: %s\n", pcTaskName);
    exit(1);
#endif
}

void vApplicationMallocFailedHook(void) {
#if PICO_BUILD
    panic("malloc failed");
#else
    printf("malloc failed\n");
    exit(1);
#endif
}

// ============================================================================
// Platform Abstraction Implementation
// ============================================================================

void platform_panic(const char *msg) {
#if PICO_BUILD
    panic("%s", msg);
#else
    printf("PANIC: %s\n", msg);
    exit(1);
#endif
}

void platform_send_byte_mux(uint8_t byte) {
#if PICO_BUILD
    // Send to real RS485 bus
    rs485_hal_send_byte(byte);
#endif
    // Also send to USB console (for debugging/monitoring)
    console_send_byte(byte);
}
