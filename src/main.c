#include <stdio.h>
#include <stdbool.h>
#include <stdlib.h>
#include <FreeRTOS.h>
#include <task.h>

#if !PICO_BUILD
    // Mac host build - use standard headers
    #include <unistd.h>
#else
    // Pico build - include Pico SDK headers
    #include <pico/stdlib.h>
    #include <hardware/gpio.h>
#endif

// Hook definitions
void vApplicationStackOverflowHook(TaskHandle_t xTask, char* pcTaskName) {
#if PICO_BUILD
    panic("Stack overflow. Task: %s\n", pcTaskName);
#else
    printf("Stack overflow. Task: %s\n", pcTaskName);
    exit(1);
#endif
}

void vApplicationMallocFailedHook() {
#if PICO_BUILD
    panic("malloc failed");
#else
    printf("malloc failed\n");
    exit(1);
#endif
}

// TASK 1: Blinks LED on GPIO 2 every 1 second (Low Priority)
static void ledTask(void* pvParameters) {
    int led_count = 0;

#if PICO_BUILD
    // Initialize GPIO 2 as output
    gpio_init(2);
    gpio_set_dir(2, GPIO_OUT);
#endif

    while (true) {
#if PICO_BUILD
        gpio_put(2, 1);
        printf("[Task 1] LED ON (GPIO 2). Count: %d\n", led_count++);
#else
        printf("[Task 1] LED ON (simulated). Count: %d\n", led_count++);
#endif
        vTaskDelay(pdMS_TO_TICKS(2000));
        
#if PICO_BUILD
        gpio_put(2, 0);
        printf("[Task 1] LED OFF (GPIO 2). Count: %d\n", led_count++);
#else
        printf("[Task 1] LED OFF (simulated). Count: %d\n", led_count++);
#endif
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

// TASK 2: Prints a message every 200ms (Higher Priority)
static void printTask(void* pvParameters) {
    int count = 0;
    while (true) {
        printf("    [Task 2] Hello from second thread! Count: %d\n", count++);
        vTaskDelay(pdMS_TO_TICKS(200));
    }
}

int main() {
#if PICO_BUILD
    stdio_init_all();
    
    // Hard wait for USB connection (optional, but helps catch the very first print)
    sleep_ms(2000); 
#else
    // Disable buffering for host build so output appears immediately in CMake/Ninja logs
    setvbuf(stdout, NULL, _IONBF, 0);
    printf("Running on Mac (host build)\n");
#endif
    
    printf("Starting FreeRTOS Scheduler...\n");

    // Create Task 1 (LED)
    xTaskCreate(
        ledTask,                // Function
        "LED_Task",             // Name
        1024,                   // Stack size (increased for printf)
        NULL,                   // Parameters
        tskIDLE_PRIORITY + 1,   // Priority
        NULL                    // Handle
    );

    // Create Task 2 (Printer)
    xTaskCreate(
        printTask,              // Function
        "Print_Task",           // Name
        1024,                   // Stack size (increased for printf)
        NULL,                   // Parameters
        tskIDLE_PRIORITY + 1,   // Priority (Same as LED task, so they share time)
        NULL                    // Handle
    );

    vTaskStartScheduler();
    
    return 0;
}
