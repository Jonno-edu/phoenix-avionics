#include "task_manager.h"
#include "heartbeat_task.h"
#include "rs485_task.h"
#include "telemetry_task.h"
#include "sensors_task.h"
#include "hil_sensor_task.h"
#include "core/debug_cli.h"
#include "core/eps_data.h"
#include <FreeRTOS.h>
#include <task.h>

#if !PICO_BUILD
    #include "core/usb_console.h"
#endif

void tasks_create_all(void) {

    rs485_task_init();
    telemetry_task_init();
    eps_data_init();
    // hil_sensor_task_init();
    debug_cli_init();
    // sensors_task_init();
    // heartbeat_task_init();
}
