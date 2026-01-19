#include "hal/platform_hal.h"
#include <FreeRTOS.h>
#include <task.h>
#include <stdio.h>
#include <stdlib.h>
#include "core/usb_console.h"
#include "core/logging.h"

#if PICO_BUILD
    #include "pico/stdlib.h"
    #include "hal/rs485_hal.h"
#else
    #include <unistd.h>
#endif

// ============================================================================
// FreeRTOS Hooks (Platform Specific Implementation)
// ============================================================================

static const char *TAG = "HAL";

void vApplicationStackOverflowHook(TaskHandle_t xTask, char* pcTaskName) {
    (void)xTask;
#if PICO_BUILD
    panic("Stack overflow. Task: %s\n", pcTaskName);
#else
    (void)pcTaskName;
    ESP_LOGE(TAG, "Stack overflow. Task: %s", pcTaskName);
    exit(1);
#endif
}

void vApplicationMallocFailedHook(void) {
#if PICO_BUILD
    panic("malloc failed");
#else
    ESP_LOGE(TAG, "malloc failed");
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
    ESP_LOGE(TAG, "PANIC: %s", msg);
    exit(1);
#endif
}

void platform_send_mux(const uint8_t *data, uint16_t len) {
#if PICO_BUILD
    // Send to real RS485 bus
    rs485_hal_send(data, len);
#endif
    // Also send to USB console (for debugging/monitoring)
    console_send(data, len);
}
