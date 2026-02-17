// flight_control.c - top-level glue for flight control module
#include "flight_control.h"
#include "pilot_tasks.h"

void flight_control_init(void) {
    // Initialize pilot tasks (IMU, EKF, FSM, Telem snapshot)
    pilot_tasks_init();
}
