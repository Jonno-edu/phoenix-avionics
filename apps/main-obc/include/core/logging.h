#ifndef LOGGING_H
#define LOGGING_H

#include <stdio.h>
#include <stdint.h>
#include "FreeRTOS.h"
#include "task.h"
#include "core/obc_data.h"

#if PICO_BUILD
    #include <pico/stdlib.h>
    #define LOG_GET_TIME() (to_ms_since_boot(get_absolute_time()))
#else
    #include <time.h>
    // Fallback for SIL/Host builds
    static inline uint32_t get_host_time_ms(void) {
        struct timespec ts;
        clock_gettime(CLOCK_MONOTONIC, &ts);
        return (uint32_t)((ts.tv_sec * 1000) + (ts.tv_nsec / 1000000));
    }
    #define LOG_GET_TIME() (get_host_time_ms())
#endif

// ANSI Color Codes
#define LOG_COLOR_RESET   "\033[0m"
#define LOG_COLOR_RED     "\033[31m"
#define LOG_COLOR_YELLOW  "\033[33m"
#define LOG_COLOR_GREEN   "\033[32m"
#define LOG_COLOR_CYAN    "\033[36m"
#define LOG_COLOR_BLUE    "\033[34m"

static inline const char* log_get_task_name(void) {
    if (xTaskGetSchedulerState() == taskSCHEDULER_NOT_STARTED) {
        return "init";
    }
    const char *name = pcTaskGetName(NULL);
    return name ? name : "none";
}

// ESP-style Logging Macros
// Format: L (Timestamp) [TASK] [TAG]: Message
#define ESP_LOGE(tag, fmt, ...) \
    do { if (system_config_get_log_level() >= 1) printf(LOG_COLOR_RED "E (%lu) [%s] [%s]: " fmt LOG_COLOR_RESET "\n", (unsigned long)LOG_GET_TIME(), log_get_task_name(), tag, ##__VA_ARGS__); } while(0)

#define ESP_LOGW(tag, fmt, ...) \
    do { if (system_config_get_log_level() >= 2) printf(LOG_COLOR_YELLOW "W (%lu) [%s] [%s]: " fmt LOG_COLOR_RESET "\n", (unsigned long)LOG_GET_TIME(), log_get_task_name(), tag, ##__VA_ARGS__); } while(0)

#define ESP_LOGI(tag, fmt, ...) \
    do { if (system_config_get_log_level() >= 2) printf(LOG_COLOR_GREEN "I (%lu) [%s] [%s]: " fmt LOG_COLOR_RESET "\n", (unsigned long)LOG_GET_TIME(), log_get_task_name(), tag, ##__VA_ARGS__); } while(0)

#define ESP_LOGD(tag, fmt, ...) \
    do { if (system_config_get_log_level() >= 3) printf(LOG_COLOR_CYAN "D (%lu) [%s] [%s]: " fmt LOG_COLOR_RESET "\n", (unsigned long)LOG_GET_TIME(), log_get_task_name(), tag, ##__VA_ARGS__); } while(0)

#define ESP_LOGV(tag, fmt, ...) \
    do { if (system_config_get_log_level() >= 3) printf(LOG_COLOR_BLUE "V (%lu) [%s] [%s]: " fmt LOG_COLOR_RESET "\n", (unsigned long)LOG_GET_TIME(), log_get_task_name(), tag, ##__VA_ARGS__); } while(0)

// Raw hex dump helper
static inline void ESP_LOG_BUFFER_HEX(const char *tag, const void *buffer, uint16_t buff_len) {
    if (system_config_get_log_level() < 2) return;
    if (buff_len == 0) return;
    const uint8_t *ptr = (const uint8_t *)buffer;
    for (uint16_t i = 0; i < buff_len; i++) {
        if (i % 16 == 0) {
            if (i > 0) printf(LOG_COLOR_RESET "\n");
            printf(LOG_COLOR_CYAN "D (%lu) [%s] [%s]: ", (unsigned long)LOG_GET_TIME(), log_get_task_name(), tag);
        }
        printf("%02X ", ptr[i]);
    }
    printf(LOG_COLOR_RESET "\n");
}

#endif // LOGGING_H
