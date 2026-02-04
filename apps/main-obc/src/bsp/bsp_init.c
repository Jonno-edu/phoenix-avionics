#include "bsp_init.h"
#include <stdio.h>
#include "core/logging.h"

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
    
    // Give USB a moment to stabilize
    sleep_ms(1000);
    
    ESP_LOGI(TAG, "--- Phoenix Avionics Starting ---");

    ESP_LOGI(TAG, "Initializing RS485 HAL...");
    // RS485 Bus A hardware (UART1: GPIO 4/5, DE/RE: GPIO 3)
    rs485_hal_init();
#else
    setvbuf(stdout, NULL, _IONBF, 0);
#endif
}

void bsp_peripheral_init(void) {
#if PICO_BUILD
    ESP_LOGI(TAG, "Initializing Sensors...");
    // Initialize hardware sensors
    if (!I2C_init()) {
        ESP_LOGE(TAG, "CRITICAL: I2C initialization failed!");
    }

    I2C_scan_bus();

    if (!BAROMS5607_init()) {
        ESP_LOGE(TAG, "CRITICAL: Barometer initialization failed!");
    }
    ESP_LOGI(TAG, "Init complete, starting scheduler.");
#endif
}
