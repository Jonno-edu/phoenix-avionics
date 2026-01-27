// main.c
#include <stdio.h>
#include "pico/stdlib.h"

int main() {
    // Initialize USB Stdstdio
    stdio_init_all();
    


    while (true) {
        printf("Hello from Minimal HIL Node!\n");
        sleep_ms(1000);
    }
}


