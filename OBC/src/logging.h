#ifndef LOGGING_H
#define LOGGING_H

#include <stdio.h>
#include "modules/datalink/datalink.h"
#include "phoenix_icd.h"

// Minimal logging macros to satisfy existing code
// #define LOG_I(tag, fmt, ...) printf("[%s] I: " fmt "\n", tag, ##__VA_ARGS__)
// #define LOG_E(tag, fmt, ...) printf("[%s] E: " fmt "\n", tag, ##__VA_ARGS__)
// #define LOG_W(tag, fmt, ...) printf("[%s] W: " fmt "\n", tag, ##__VA_ARGS__)
// #define LOG_D(tag, fmt, ...) printf("[%s] D: " fmt "\n", tag, ##__VA_ARGS__)
// #define LOG_V(tag, fmt, ...) printf("[%s] V: " fmt "\n", tag, ##__VA_ARGS__)

#define LOG_I(tag, fmt, ...) datalink_sendLog(LOG_LEVEL_INFO, "[%s] I: " fmt, tag, ##__VA_ARGS__)
#define LOG_E(tag, fmt, ...) datalink_sendLog(LOG_LEVEL_ERROR, "[%s] E: " fmt, tag, ##__VA_ARGS__)
#define LOG_W(tag, fmt, ...) datalink_sendLog(LOG_LEVEL_WARN, "[%s] W: " fmt, tag, ##__VA_ARGS__)
#define LOG_D(tag, fmt, ...) datalink_sendLog(LOG_LEVEL_DEBUG, "[%s] D: " fmt, tag, ##__VA_ARGS__)
#define LOG_V(tag, fmt, ...) datalink_sendLog(LOG_LEVEL_DEBUG, "[%s] V: " fmt, tag, ##__VA_ARGS__)

#endif // LOGGING_H
