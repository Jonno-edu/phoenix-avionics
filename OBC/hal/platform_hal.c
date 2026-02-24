#include "hal/platform_hal.h"
#include <FreeRTOS.h>
#include <task.h>
#include <stdio.h>
#include <stdlib.h>
#include "logging.h"

#if PICO_BUILD
    #include "hal/rs485_hal.h"
    #include <pico/stdlib.h>
#else
    #include <unistd.h>
#endif

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

void platform_panic(const char *msg) {
#if PICO_BUILD
    panic("%s", msg);
#else
    ESP_LOGE(TAG, "PANIC: %s", msg);
    exit(1);
#endif
}

void console_send(const uint8_t *data, uint16_t len) {
    for (uint16_t i = 0; i < len; i++) {
#if PICO_BUILD
        putchar_raw(data[i]);
#else
        putchar(data[i]);
#endif
    }
#if !PICO_BUILD
    fflush(stdout);
#endif
}

void platform_send_mux(const uint8_t *data, uint16_t len) {
#if PICO_BUILD
    rs485_hal_send(data, len);
#endif
    console_send(data, len);
}
