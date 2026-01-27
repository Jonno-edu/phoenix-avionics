#ifndef TASK_MANAGER_H
#define TASK_MANAGER_H

#include <FreeRTOS.h>
#include <task.h>

// Task Priorities
#define PRIORITY_CONSOLE_RX       (tskIDLE_PRIORITY + 3)
#define PRIORITY_RS485_PROCESSING (tskIDLE_PRIORITY + 2)
#define PRIORITY_ID_POLLING       (tskIDLE_PRIORITY + 2)
#define PRIORITY_SENSORS          (tskIDLE_PRIORITY + 1)
#define PRIORITY_HEARTBEAT        (tskIDLE_PRIORITY + 1)

// Wrapper to create all tasks
void tasks_create_all(void);

#endif // TASK_MANAGER_H
