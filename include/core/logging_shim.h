#ifndef LOGGING_SHIM_H
#define LOGGING_SHIM_H

#include <stdint.h>
#include "telemetry_defs.h"

// Core logging function
void GSU_LOG(uint8_t level, const char *format, ...);

// Macros to replace ESP_LOGx
// We map them to our GSU_LOG with appropriate levels
#define LOGE(tag, fmt, ...) GSU_LOG(LOG_LEVEL_ERROR, fmt, ##__VA_ARGS__)
#define LOGW(tag, fmt, ...) GSU_LOG(LOG_LEVEL_WARN,  fmt, ##__VA_ARGS__)
#define LOGI(tag, fmt, ...) GSU_LOG(LOG_LEVEL_INFO,  fmt, ##__VA_ARGS__)
#define LOGD(tag, fmt, ...) GSU_LOG(LOG_LEVEL_DEBUG, fmt, ##__VA_ARGS__)

#endif // LOGGING_SHIM_H
