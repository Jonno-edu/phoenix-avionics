// main.c
#include <stdio.h>
#include <stdbool.h>
#include <stdlib.h>
#include <FreeRTOS.h>
#include <task.h>
#include "logging.h"
#include "usb_console.h"
#include "rs485_protocol.h"
#include "hal/rs485_hal.h"
#include "modules/data_store/obc_data.h"
#include "modules/data_store/rocket_data.h"
#include "modules/subsystems/eps_node.h"
#include "bsp/bsp_init.h"
#include "hal/platform_hal.h"
#include "tasks/queue_manager.h"
#include "modules/communication/command_handler_task.h"
#include "modules/communication/rs485_task.h"
#include "modules/communication/telemetry_task.h"
#include "modules/flight_control/pilot_tasks.h"

#if !PICO_BUILD
    #include <unistd.h>
#else
    #include <pico/stdlib.h>
#endif

int main() {
    bsp_hardware_init();
    
    console_init();

    // sleep_ms(2000); // Wait for console to stabilize

    system_data_init();
    rocket_data_init();
    // eps_node_init();
    rocket_data_fill_test_values();
    system_config_init();

#if PICO_BUILD
    // bsp_peripheral_init();
#else
#endif

    // Initialize Queues
    queues_init();

    // Initialize Tasks
    pilot_tasks_init();
    rs485_task_init();
    command_handler_task_init();
    telemetry_task_init();

    ESP_LOGI("MAIN", "All tasks initialized. Starting scheduler...");

    vTaskStartScheduler();

    platform_panic("Scheduler returned unexpectedly!");
}