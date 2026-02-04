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
#include "core/eps_data.h"
#include "bsp/bsp_init.h"
#include "tasks/task_manager.h"
#include "hal/platform_hal.h"

#if !PICO_BUILD
    #include <unistd.h>
#else
    #include <pico/stdlib.h>
#endif

int main() {
    // stdio_init_all();s
    sleep_ms(2000); // Wait for console to stabilize

    bsp_hardware_init();
    
    console_init();

    // sleep_ms(2000); // Wait for console to stabilize

    // debug_cli_init();
    system_data_init();
    rocket_data_init();
    // eps_data_init();
    //rocket_data_fill_test_values();
    system_config_init();

#if PICO_BUILD
    // Ensure RS485 protocol is initialized with the HAL TX callback and OBC address
    rs485_init(rs485_hal_send, ADDR_OBC);
    // bsp_peripheral_init();
#else
    rs485_init(console_send, ADDR_OBC);
#endif

    tasks_create_all();

    vTaskStartScheduler();

    platform_panic("Scheduler returned unexpectedly!");
}