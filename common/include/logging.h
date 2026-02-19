#ifndef LOGGING_H
#define LOGGING_H

#include <stdio.h>

// Minimal logging macros to satisfy existing code
#define ESP_LOGI(tag, fmt, ...) printf("[%s] I: " fmt "\n", tag, ##__VA_ARGS__)
#define ESP_LOGE(tag, fmt, ...) printf("[%s] E: " fmt "\n", tag, ##__VA_ARGS__)
#define ESP_LOGW(tag, fmt, ...) printf("[%s] W: " fmt "\n", tag, ##__VA_ARGS__)
#define ESP_LOGD(tag, fmt, ...) printf("[%s] D: " fmt "\n", tag, ##__VA_ARGS__)
#define ESP_LOGV(tag, fmt, ...) printf("[%s] V: " fmt "\n", tag, ##__VA_ARGS__)

#endif // LOGGING_H
