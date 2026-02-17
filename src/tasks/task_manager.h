#ifndef TASK_MANAGER_H
#define TASK_MANAGER_H

#include <FreeRTOS.h>
#include <task.h>

// Task Priorities

// Core 0 (Pilot)
#define PRIORITY_PILOT_IMU        (tskIDLE_PRIORITY + 6)
#define PRIORITY_PILOT_ESTIMATOR  (tskIDLE_PRIORITY + 5)
#define PRIORITY_PILOT_FSM        (tskIDLE_PRIORITY + 4)
#define PRIORITY_SENSORS          (tskIDLE_PRIORITY + 3) // Sensor Poll
#define PRIORITY_HIL_SENSORS      (tskIDLE_PRIORITY + 3) // HIL (Alternative to Sensor Poll)
#define PRIORITY_PILOT_TELEM      (tskIDLE_PRIORITY + 2) // Snapshot
#define PRIORITY_HEARTBEAT        (tskIDLE_PRIORITY + 1)

// Core 1 (Co-Pilot)
#define PRIORITY_COPILOT_CMD      (tskIDLE_PRIORITY + 4)
#define PRIORITY_CONSOLE_RX       (tskIDLE_PRIORITY + 3) // Used by RS485 RX?
#define PRIORITY_RS485_PROCESSING (tskIDLE_PRIORITY + 3) // RX Task
#define PRIORITY_RS485_TX         (tskIDLE_PRIORITY + 2)
#define PRIORITY_ID_POLLING       (tskIDLE_PRIORITY + 2)

// Wrapper to create all tasks
void tasks_create_all(void);

#endif // TASK_MANAGER_H
