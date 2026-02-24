#ifndef LOGGING_H
#define LOGGING_H

#include <stdio.h>

// Minimal logging macros to satisfy existing code
#define LOG_I(tag, fmt, ...) printf("[%s] I: " fmt "\n", tag, ##__VA_ARGS__)
#define LOG_E(tag, fmt, ...) printf("[%s] E: " fmt "\n", tag, ##__VA_ARGS__)
#define LOG_W(tag, fmt, ...) printf("[%s] W: " fmt "\n", tag, ##__VA_ARGS__)
#define LOG_D(tag, fmt, ...) printf("[%s] D: " fmt "\n", tag, ##__VA_ARGS__)
#define LOG_V(tag, fmt, ...) printf("[%s] V: " fmt "\n", tag, ##__VA_ARGS__)

#endif // LOGGING_H
