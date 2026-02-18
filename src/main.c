#include <stdio.h>
#include "FreeRTOS.h"
#include "task.h"

#if PICO_BUILD
#include <pico/stdlib.h>
#else
#include <unistd.h>
#endif

void vTask1(void *pvParameters) {
    int count1 = 0;
    for (;;) {
        printf("Task 1 (printf test): %d\n", count1++);
        fflush(stdout);
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void vTask2(void *pvParameters) {
    int count2 = 0;
    for (;;) {
        printf("Task 2: %d\n", count2++);
        fflush(stdout);
        vTaskDelay(pdMS_TO_TICKS(200));
    }
}

void vTask3(void *pvParameters) {
    int count3 = 0;
    for (;;) {
        printf("Task 3: %d\n", count3++);
        fflush(stdout);
        vTaskDelay(pdMS_TO_TICKS(300));
    }
}

int main(void) {
#if PICO_BUILD
    stdio_init_all();
#endif

    xTaskCreate(vTask1, "Task1", 2048, NULL, 1, NULL);
    xTaskCreate(vTask2, "Task2", 2048, NULL, 1, NULL);
    xTaskCreate(vTask3, "Task3", 2048, NULL, 1, NULL);

    vTaskStartScheduler();

    while (1) {
    }
}