#include "task_manager.h"
#include "heartbeat_task.h"
#include "eps_polling_task.h"
#include "rs485_task.h"
#include <FreeRTOS.h>
#include <task.h>

#if !PICO_BUILD
    #include "usb_console.h"
#endif

void tasks_create_all(void) {
#if !PICO_BUILD
    // SIL: Create serial RX polling task (high priority to simulate interrupt)
    xTaskCreate(
        vConsoleRxTask,
        "ConsoleRx",
        1024,
        NULL,
        PRIORITY_CONSOLE_RX,
        NULL
    );
#endif

    rs485_task_init();
    eps_polling_task_init();
    heartbeat_task_init();
}
