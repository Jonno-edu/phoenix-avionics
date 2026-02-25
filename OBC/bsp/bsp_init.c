#include "bsp_init.h"
#include <stdio.h>
#include "logging.h"

#if PICO_BUILD
    #include "pico/stdlib.h"
    #include "hal/rs485_hal.h"
    #include "i2c.h"
    #include "baroMS5607.h"
#else
    #include <unistd.h>
#endif

static const char *TAG = "BSP";

void bsp_hardware_init(void) {
#if PICO_BUILD
    stdio_init_all();
    sleep_ms(1000);
    LOG_I(TAG, "--- Phoenix Avionics Starting ---");
    LOG_I(TAG, "Initializing RS485 HAL...");
    rs485_hal_init();
#else
    setvbuf(stdout, NULL, _IONBF, 0);
#endif
}

void bsp_peripheral_init(void) {
#if PICO_BUILD
    LOG_I(TAG, "Initializing Sensors...");
    if (!I2C_init()) {
        LOG_E(TAG, "CRITICAL: I2C initialization failed!");
    }

    I2C_scan_bus();

    if (!BAROMS5607_init()) {
        LOG_E(TAG, "CRITICAL: Barometer initialization failed!");
    }
    LOG_I(TAG, "Init complete, starting scheduler.");
#endif
}
