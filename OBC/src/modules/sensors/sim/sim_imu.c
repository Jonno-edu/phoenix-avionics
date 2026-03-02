/**
 * @file sim_imu.c
 * @brief Simulated IMU driver — publishes TOPIC_SENSOR_IMU at 1 kHz.
 *
 * Generates physically plausible IMU readings for software-in-the-loop
 * testing without requiring physical hardware.
 *
 * Accelerometer output: gravity-aligned (NED, body at rest, Z-down) plus
 *   configurable Gaussian white noise.
 *
 * Gyroscope output: zero-mean angular rate with a configurable DC bias on
 *   Z-axis (giving the ESKF something real to identify) plus white noise.
 *
 * Replace this module with a real hardware driver when transitioning to
 * flight hardware. The publish interface (TOPIC_SENSOR_IMU) is identical.
 */

#include <FreeRTOS.h>
#include <task.h>
#include "pico/time.h"
#include "norb/norb.h"
#include "norb/topic_defs/sensor_imu.h"
#include "sim_sensors.h"

/* -----------------------------------------------------------------------
 * Simulation parameters — tune these to characterise the test environment.
 * ----------------------------------------------------------------------- */
#define SIM_IMU_RATE_HZ         1000u     /* publish rate                       */
#define SIM_IMU_DT_MS           1u        /* period in milliseconds             */
#define SIM_IMU_GRAVITY_MS2     9.80665f  /* 1G (NED Z-down convention)         */
#define SIM_IMU_ACCEL_NOISE_MS2 0.05f     /* accel white-noise amplitude (m/s²) */
#define SIM_IMU_GYRO_NOISE_RADS 0.002f    /* gyro white-noise amplitude (rad/s) */
#define SIM_IMU_GYRO_BIAS_Z     0.01f     /* simulated DC gyro bias on Z (rad/s)*/
#define SIM_IMU_TEMPERATURE_C   25.0f     /* simulated on-die temperature (°C)  */

/* -----------------------------------------------------------------------
 * Minimal 32-bit LCG (Knuth MMIX constants).
 * Returns a value uniformly distributed in [-1, 1].
 * ----------------------------------------------------------------------- */
static uint32_t s_prng = 0xDEADBEEFu;

static float prng_noise(void)
{
    s_prng = s_prng * 1664525u + 1013904223u;
    return (float)(int32_t)s_prng * (1.0f / 2147483648.0f);
}

/* -----------------------------------------------------------------------
 * Static task storage.
 * ----------------------------------------------------------------------- */
static StaticTask_t s_sim_imu_tcb;
static StackType_t  s_sim_imu_stack[512];

/* -----------------------------------------------------------------------
 * Simulated IMU task — 1 kHz.
 * ----------------------------------------------------------------------- */
static void sim_imu_task(void *params)
{
    (void)params;
    TickType_t xLastWake = xTaskGetTickCount();
    sensor_imu_t imu = {0};

    while (1) {
        vTaskDelayUntil(&xLastWake, pdMS_TO_TICKS(SIM_IMU_DT_MS));

        imu.timestamp_us = time_us_64();

        /* Accelerometer: 1G on Z (NED, body at rest) + white noise */
        imu.accel_ms2[0] =  SIM_IMU_ACCEL_NOISE_MS2 * prng_noise();
        imu.accel_ms2[1] =  SIM_IMU_ACCEL_NOISE_MS2 * prng_noise();
        imu.accel_ms2[2] =  SIM_IMU_GRAVITY_MS2
                          + SIM_IMU_ACCEL_NOISE_MS2 * prng_noise();

        /* Gyroscope: zero mean + known DC bias on Z + white noise */
        imu.gyro_rads[0] = SIM_IMU_GYRO_NOISE_RADS * prng_noise();
        imu.gyro_rads[1] = SIM_IMU_GYRO_NOISE_RADS * prng_noise();
        imu.gyro_rads[2] = SIM_IMU_GYRO_BIAS_Z
                         + SIM_IMU_GYRO_NOISE_RADS * prng_noise();

        imu.temperature_c = SIM_IMU_TEMPERATURE_C;

        norb_publish(TOPIC_SENSOR_IMU, &imu);
    }
}

/* -----------------------------------------------------------------------
 * Public init — called from sim_sensors_init().
 * ----------------------------------------------------------------------- */
static void sim_imu_init(void)
{
    xTaskCreateStatic(sim_imu_task, "SIM_IMU", 512, NULL,
                      configMAX_PRIORITIES - 1,
                      s_sim_imu_stack, &s_sim_imu_tcb);
}

/* -----------------------------------------------------------------------
 * sim_sensors_init — entry point for the whole sim module.
 * ----------------------------------------------------------------------- */
void sim_sensors_init(void)
{
    sim_imu_init();
}
