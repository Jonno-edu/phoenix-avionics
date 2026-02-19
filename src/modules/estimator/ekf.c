#include <FreeRTOS.h>
#include <task.h>
#include <stdio.h>
#include "pico/time.h"
#include "common/pubsub.h"
#include "common/topic_defs/sensor_imu.h"

void ekf_task(void *params) {
    TickType_t xLastWake = xTaskGetTickCount();
    sensor_imu_t imu;

    while (1) {
        vTaskDelayUntil(&xLastWake, pdMS_TO_TICKS(4)); // 250 Hz

        // Peek at the absolute newest IMU snapshot
        // if (subscribe_poll(TOPIC_SENSOR_IMU, &imu)) {
            
        //     // Print it out to test!
        //     uint64_t current_time_us = time_us_64();
        //     printf("[EKF] Got IMU Data at %llu us (now: %llu us) | Z-Accel: %.2f\n", 
        //            imu.timestamp_us, current_time_us, imu.accel_ms2[2]);
                   
        // } else {
        //     printf("[EKF] Waiting for IMU data...\n");
        // }
    }
}

void estimator_init(void) {
    xTaskCreate(ekf_task, "EKF", 2048, NULL, configMAX_PRIORITIES - 2, NULL);
}
