// main.c
#include <stdio.h>
#include <stdbool.h>
#include <stdlib.h>
#include <FreeRTOS.h>
#include <task.h>
#include "serial.h"
#include "system_data.h"

#if !PICO_BUILD
    #include <unistd.h>
#else
    #include <pico/stdlib.h>
    #include <hardware/gpio.h>
#endif

void vApplicationStackOverflowHook(TaskHandle_t xTask, char* pcTaskName) {
#if PICO_BUILD
    panic("Stack overflow. Task: %s\n", pcTaskName);
#else
    exit(1);
#endif
}

void vApplicationMallocFailedHook() {
#if PICO_BUILD
    panic("malloc failed");
#else
    exit(1);
#endif
}

// Serial command processing task
void vSerialCommandTask(void *pvParameters) {
    (void)pvParameters;
    
    while (true) {
        serial_process_commands();
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

int main() {
#if PICO_BUILD
    stdio_init_all();
    serial_init();
    system_data_init();
    sleep_ms(2000);
#else
    setvbuf(stdout, NULL, _IONBF, 0);
#endif

    // Serial command task
    xTaskCreate(
        vSerialCommandTask,
        "SerialCmd",
        256,
        NULL,
        tskIDLE_PRIORITY + 1,
        NULL
    );

    vTaskStartScheduler();

    while (true) {
        sleep_ms(1000);
    }
    
    return 0;
}
