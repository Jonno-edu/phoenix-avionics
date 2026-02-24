#include <FreeRTOS.h>
#include <task.h>
#include "pico/time.h"
#include "norb/norb.h"
#include "norb/topic_defs/sensor_imu.h"

void imu_task(void *params) {
    TickType_t xLastWake = xTaskGetTickCount();
    sensor_imu_t imu_data = {0};

    while (1) {
        vTaskDelayUntil(&xLastWake, pdMS_TO_TICKS(4)); // 250 Hz (4ms)

        // 1. Fake reading from SPI/I2C hardware for testing
        // Use true 64-bit microsecond hardware timer
        imu_data.timestamp_us = time_us_64(); 
        imu_data.accel_ms2[2] = 9.81f; // Fake 1G on Z-axis

        // 2. Publish to the void!
        norb_publish(TOPIC_SENSOR_IMU, &imu_data);
    }
}
