#include "bsp_init.h"
#include <stdio.h>

#if PICO_BUILD
    #include "pico/stdlib.h"
    #include "rs485_hal.h"
    #include "i2c.h"
    #include "baroMS5607.h"
#else
    #include <unistd.h>
#endif

void bsp_hardware_init(void) {
#if PICO_BUILD
    stdio_init_all();
    
    // Give USB a moment to stabilize so we don't miss early messages
    for (int i = 0; i < 5; i++) {
        printf("Booting in %d...\n", 5-i);
        sleep_ms(200);
    }
    
    printf("\n--- Phoenix Avionics Starting ---\n");

    printf("Initializing RS485 HAL...\n");
    // RS485 Bus A hardware (UART1: GPIO 4/5, DE/RE: GPIO 3)
    rs485_hal_init();
#else
    setvbuf(stdout, NULL, _IONBF, 0);
#endif
}

void bsp_peripheral_init(void) {
#if PICO_BUILD
    printf("Initializing Sensors...\n");
    // Initialize hardware sensors
    if (!I2C_init()) {
        printf("CRITICAL: I2C initialization failed!\n");
    }

    I2C_scan_bus();

    if (!BAROMS5607_init()) {
        printf("CRITICAL: Barometer initialization failed!\n");
    }
    printf("Init complete, starting scheduler.\n");
    sleep_ms(1000); // Give user time to read init logs before tasks spam
#endif
}
