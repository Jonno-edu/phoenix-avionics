// main.c
#include <stdio.h>
#include <stdbool.h>
#include <stdlib.h>
#include <FreeRTOS.h>
#include <task.h>
#include "usb_console.h"
#include "rs485_protocol.h"
#include "rs485_hal.h"
#include "system_data.h"
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
    system_data_init();

#if PICO_BUILD
    // Ensure RS485 protocol is initialized with the HAL TX callback
    rs485_init(rs485_hal_send_byte);
    bsp_peripheral_init();
#else
    rs485_init(console_send_byte);
#endif

    tasks_create_all();

    vTaskStartScheduler();

    platform_panic("Scheduler returned unexpectedly!");
}
