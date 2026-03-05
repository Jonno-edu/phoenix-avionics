#include <FreeRTOS.h>
#include <task.h>
#include <stdio.h>
#include "norb/norb.h"
#include "norb/topic_defs/vehicle_imu.h"
#include "ekf_core.h"

static ekf_core_t ekf;

void ekf_task(void *params) {
    TickType_t xLastWake = xTaskGetTickCount();
    vehicle_imu_t imu_data;

    ekf_core_init(&ekf);

    while (1) {
        vTaskDelayUntil(&xLastWake, pdMS_TO_TICKS(4)); // 250 Hz

        // Subscribe to TOPIC_VEHICLE_IMU (Integrated data from integrator module)
        if (norb_subscribe(TOPIC_VEHICLE_IMU, &imu_data, 10)) {

            // Convert nORB message to ring-buffer frame and run the 3-step loop.
            imu_history_t imu_hist;
            imu_hist.timestamp_us      = imu_data.timestamp_us;
            imu_hist.dt_s              = imu_data.dt_s;
            imu_hist.delta_angle[0]    = imu_data.delta_angle[0];
            imu_hist.delta_angle[1]    = imu_data.delta_angle[1];
            imu_hist.delta_angle[2]    = imu_data.delta_angle[2];
            imu_hist.delta_velocity[0] = imu_data.delta_velocity[0];
            imu_hist.delta_velocity[1] = imu_data.delta_velocity[1];
            imu_hist.delta_velocity[2] = imu_data.delta_velocity[2];
            
            // Push to the EKF observation buffer
            ekf_core_step(&ekf, &imu_hist);

            // Debug: Print every ~1 second (250 samples)
            static int count = 0;
            if (++count >= 250) {
                count = 0;
                printf("[EKF] Pos: [%.2f, %.2f, %.2f] | Vel: [%.2f, %.2f, %.2f] | BiasG: [%.4f, %.4f, %.4f]\n",
                       ekf.head_state.p_ned[0], ekf.head_state.p_ned[1], ekf.head_state.p_ned[2],
                       ekf.head_state.v_ned[0], ekf.head_state.v_ned[1], ekf.head_state.v_ned[2],
                       ekf.head_state.gyro_bias[0], ekf.head_state.gyro_bias[1], ekf.head_state.gyro_bias[2]);
            }
        }
    }
}

static StaticTask_t ekf_tcb;
static StackType_t  ekf_stack[2048];

void estimator_init(void) {
    xTaskCreateStatic(ekf_task, "EKF", 2048, NULL, configMAX_PRIORITIES - 2,
                      ekf_stack, &ekf_tcb);
}