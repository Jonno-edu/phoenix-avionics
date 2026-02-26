#include "sensors.h"
#include <FreeRTOS.h>
#include <task.h>

extern void imu_task(void *params);

static StaticTask_t imu_tcb;
static StackType_t  imu_stack[1024];

void sensors_init(void) {
    // Create the IMU task (pinned to Core 0 if you are using SMP on the RP2350)
    xTaskCreateStatic(imu_task, "IMU", 1024, NULL, configMAX_PRIORITIES - 1,
                      imu_stack, &imu_tcb);
}
