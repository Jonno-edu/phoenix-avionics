#include "queue_manager.h"

QueueHandle_t xEstimatorQueue = NULL;
QueueHandle_t xPilotCommandQueue = NULL;

void queue_manager_init(void) {
    // Estimator Queue: Holds raw sensor data packets
    // Size: TBD (e.g., 10 items)
    // Item Size: TBD (sizeof(SensorData_t))
    // xEstimatorQueue = xQueueCreate(10, sizeof(SensorData_t));

    // Pilot Command Queue: Holds validated commands for Flight State
    // Size: 5 items (process quickly)
    // Item Size: TBD (sizeof(Command_t))
    // xPilotCommandQueue = xQueueCreate(5, sizeof(Command_t));
}
