#ifndef QUEUE_MANAGER_H
#define QUEUE_MANAGER_H

#include <FreeRTOS.h>
#include <queue.h>

// Queue Handles (Externally accessible)
extern QueueHandle_t xEstimatorQueue;
extern QueueHandle_t xPilotCommandQueue;

// Initialization
void queue_manager_init(void);

#endif // QUEUE_MANAGER_H
