// main.c
#include <stdio.h>
#include <stdbool.h>
#include <stdlib.h>
#include <FreeRTOS.h>
#include <task.h>
#include "core/usb_console.h"
// #include "core/debug_cli.h"
#include "rs485_protocol.h"
#include "hal/rs485_hal.h"
#include "core/obc_data.h"
#include "core/rocket_data.h"
#include "core/eps_node.h"
#include "bsp/bsp_init.h"
#include "tasks/task_manager.h"
#include "hal/platform_hal.h"

#if !PICO_BUILD
    #include <unistd.h>
#else
    #include <pico/stdlib.h>
#endif

int main() {
    bsp_hardware_init();
    
    console_init();

    // sleep_ms(2000); // Wait for console to stabilize

    // debug_cli_init();
    system_data_init();
    rocket_data_init();
    // eps_node_init();
    rocket_data_fill_test_values();
    system_config_init();

#if PICO_BUILD
    // bsp_peripheral_init();
#else
#endif

    tasks_create_all();

    vTaskStartScheduler();

    platform_panic("Scheduler returned unexpectedly!");
}