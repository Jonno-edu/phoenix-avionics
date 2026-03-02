#include "sensors.h"
#include <FreeRTOS.h>
#include <task.h>
#include "sim/sim_sensors.h"

extern void integrator_init(void);

void sensors_init(void)
{
    /* Simulated sensor drivers — replace with hardware drivers for flight */
    sim_sensors_init();

    /* Integrator — accumulates raw IMU samples into TOPIC_VEHICLE_IMU */
    integrator_init();
}
