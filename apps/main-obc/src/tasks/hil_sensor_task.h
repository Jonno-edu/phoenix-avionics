#ifndef HIL_SENSOR_TASK_H
#define HIL_SENSOR_TASK_H

/**
 * @file hil_sensor_task.h
 * @brief HIL (Hardware-In-Loop) sensor task for polling simulated sensor data via I2C
 * 
 * This task reads multi-rate sensor data from the HIL node:
 * - Fast (1kHz): IMU data (accel/gyro)
 * - Medium (100Hz): Barometer and temperature
 * - Slow (10Hz): GPS data
 */

/**
 * @brief Initialize and create the HIL sensor polling task
 * 
 * Creates a FreeRTOS task that:
 * - Initializes I2C hardware
 * - Polls HIL node at 1kHz base rate
 * - Updates rocket_data structures with received values
 */
void hil_sensor_task_init(void);

#endif // HIL_SENSOR_TASK_H
