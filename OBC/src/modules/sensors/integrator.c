#include <FreeRTOS.h>
#include <task.h>
#include "norb/norb.h"
#include "norb/topic_defs/sensor_imu.h"
#include "norb/topic_defs/vehicle_imu.h"

/* -----------------------------------------------------------------------
 * The simulated IMU publishes at 1 kHz (1 ms).  The integrator accumulates
 * N_SAMPLES consecutive raw samples then publishes TOPIC_VEHICLE_IMU at
 * 250 Hz (every 4 ms window).  dt_s reported to the EKF always reflects
 * the full accumulated window.
 * ----------------------------------------------------------------------- */
#define INTEGRATOR_SAMPLE_DT_S  0.001f   /* raw IMU period (seconds)            */
#define INTEGRATOR_SAMPLE_DT_MS 1u       /* raw IMU period (milliseconds)       */
#define INTEGRATOR_N_SAMPLES    4u       /* accumulate 4 × 1 ms = 4 ms window  */
#define INTEGRATOR_OUT_DT_S     (INTEGRATOR_SAMPLE_DT_S * INTEGRATOR_N_SAMPLES)

static StaticTask_t s_integrator_tcb;
static StackType_t  s_integrator_stack[512];

/* -----------------------------------------------------------------------
 * Integrator task — runs at 1 kHz alongside the IMU.
 *
 * Accumulates delta-angle and delta-velocity across N_SAMPLES raw samples,
 * then publishes TOPIC_VEHICLE_IMU once per window and resets accumulators.
 *
 * Because norb uses mailbox semantics (xQueuePeek — value is never consumed),
 * this task time-drives itself with vTaskDelayUntil at the same 1 ms cadence
 * as the IMU task.  Each loop iteration reads the freshest available sample.
 * ----------------------------------------------------------------------- */
static void integrator_task(void *params)
{
    (void)params;
    TickType_t    xLastWake = xTaskGetTickCount();
    sensor_imu_t  imu       = {0};
    vehicle_imu_t out       = {0};
    uint32_t      count     = 0;

    /* Accumulator arrays — zeroed at the start of each output window */
    float da[3] = {0.0f, 0.0f, 0.0f};  /* delta-angle    (rad) */
    float dv[3] = {0.0f, 0.0f, 0.0f};  /* delta-velocity (m/s) */

    while (1) {
        vTaskDelayUntil(&xLastWake, pdMS_TO_TICKS(INTEGRATOR_SAMPLE_DT_MS));

        if (!norb_subscribe_poll(TOPIC_SENSOR_IMU, &imu)) {
            continue; /* IMU has not published yet — skip this sample */
        }

        /* Accumulate this sample */
        for (int i = 0; i < 3; i++) {
            da[i] += imu.gyro_rads[i] * INTEGRATOR_SAMPLE_DT_S;
            dv[i] += imu.accel_ms2[i] * INTEGRATOR_SAMPLE_DT_S;
        }
        count++;

        if (count >= INTEGRATOR_N_SAMPLES) {
            /* Publish the completed integration window */
            for (int i = 0; i < 3; i++) {
                out.delta_angle[i]    = da[i];
                out.delta_velocity[i] = dv[i];
            }
            out.dt_s         = INTEGRATOR_OUT_DT_S;
            out.timestamp_us = imu.timestamp_us;

            norb_publish(TOPIC_VEHICLE_IMU, &out);

            /* Reset accumulators for next window */
            for (int i = 0; i < 3; i++) {
                da[i] = 0.0f;
                dv[i] = 0.0f;
            }
            count = 0;
        }
    }
}

void integrator_init(void)
{
    xTaskCreateStatic(integrator_task, "INTEG", 512, NULL,
                      configMAX_PRIORITIES - 2,
                      s_integrator_stack, &s_integrator_tcb);
}
