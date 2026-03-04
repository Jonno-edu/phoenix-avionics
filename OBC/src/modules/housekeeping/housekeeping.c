#include "housekeeping.h"
#include <stdio.h>
#include "hal/rs485_hal.h"
#include "modules/datalink/datalink.h"
#include "modules/nodes/eps/eps.h"
#include "modules/nodes/tracking_radio/tracking_radio.h"
#include "norb/norb.h"
#include "norb/topics.h"
#include "norb/topic_defs/vehicle_imu.h"
#include "norb/topic_defs/sensor_imu.h"

#include "FreeRTOS.h"
#include "task.h"

#define POLL_PERIOD_MS    1000
#define NODE_TIMEOUT_MS   100

TlmIdentificationPayload_t g_ident_payload = {
    .node_type = 0x01, // OBCs
    .firmware_major = 0,
    .firmware_minor = 1,
    .uptime_seconds = 0,
    .uptime_milliseconds = 0
};

void housekeeping_task(void *pvParameters)
{
    (void)pvParameters;

    // datalink_init() owns all hardware setup — call it here once
    datalink_init();
    // printf("[HK] datalink init OK\n");
    datalink_sendLog(LOG_LEVEL_DEBUG, "[HK] datalink init OK");

    norb_publish(TOPIC_OBC_IDENT, &g_ident_payload);

    TickType_t last_wake = xTaskGetTickCount();

    while (1) {
        // Poll each node; node modules own their telemetry details.
        eps_poll(NODE_TIMEOUT_MS);
        tracking_radio_poll(NODE_TIMEOUT_MS);

        // Safe: called from task, not ISR
        rs485_hal_print_raw_log();

        // Print stack headroom for housekeeping task
        datalink_sendLog(LOG_LEVEL_DEBUG, "[STACK] housekeeping: %lu words free",
            uxTaskGetStackHighWaterMark(NULL));

        /* --- Phase 2 verification: confirm TOPIC_VEHICLE_IMU data --------
         * Expected: delta_velocity[2] ~= 9.80665 * 0.004 = 0.03923 m/s
         *           delta_angle[2]    ~= 0.01    * 0.004 = 0.00004 rad
         * ----------------------------------------------------------------- */
        vehicle_imu_t vimu = {0};
        if (norb_subscribe_poll(TOPIC_VEHICLE_IMU, &vimu)) {
            datalink_sendLog(LOG_LEVEL_INFO,
                "[VERIFY] vehicle_imu: dv[2]=%.5f m/s  da[2]=%.6f rad  dt=%.4f s",
                (double)vimu.delta_velocity[2],
                (double)vimu.delta_angle[2],
                (double)vimu.dt_s);
        } else {
            datalink_sendLog(LOG_LEVEL_WARN, "[VERIFY] vehicle_imu: no data yet");
        }

        sensor_imu_t simu = {0};
        if (norb_subscribe_poll(TOPIC_SENSOR_IMU, &simu)) {
            datalink_sendLog(LOG_LEVEL_INFO,
                "[VERIFY] sensor_imu:  accel[2]=%.5f m/s2  gyro[2]=%.5f rad/s",
                (double)simu.accel_ms2[2],
                (double)simu.gyro_rads[2]);
        }
        
        // User requested: ticks / (HZ * 1000)
        TickType_t now = xTaskGetTickCount();
        g_ident_payload.uptime_seconds = (uint16_t)(now / configTICK_RATE_HZ);
        
        g_ident_payload.uptime_milliseconds = (uint16_t)(now % 1000);

        norb_publish(TOPIC_OBC_IDENT, &g_ident_payload);

        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(POLL_PERIOD_MS));
    }
}
