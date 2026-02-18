#include "sensors.h"
#include <FreeRTOS.h>
#include <task.h>

extern void imu_task(void *params);

void sensors_init(void) {
    // Create the IMU task (pinned to Core 0 if you are using SMP on the RP2350)
    xTaskCreate(imu_task, "IMU", 1024, NULL, configMAX_PRIORITIES - 1, NULL); 
}
