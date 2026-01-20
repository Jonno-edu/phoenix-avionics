#ifndef TELEMETRY_TASK_H
#define TELEMETRY_TASK_H

/**
 * @brief Initialize the Telemetry Broadcast Task
 * 
 * This task periodcially (1Hz) collects data from the rocket_data store
 * and broadcasts it via RS485 for the Radio/GSE to pick up.
 */
void telemetry_task_init(void);

#endif // TELEMETRY_TASK_H
