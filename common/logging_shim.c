#include "logging_shim.h"
#include <stdio.h>
#include <stdarg.h>

#if PICO_BUILD
    #include <pico/stdlib.h>
#else
    #include <time.h>
#endif

void GSU_LOG(uint8_t level, const char *format, ...) {
    (void)level; // Suppress unused level for now
    
    va_list args;
    va_start(args, format);

    vprintf(format, args);
    printf("\n");
    
    va_end(args);
}
