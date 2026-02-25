#include "datalink.h"
#include "tctlm.h"
#include "hal/rs485_hal.h"
#include "logging.h"

#include "FreeRTOS.h"
#include "task.h"

#if PICO_BUILD
#include "pico/stdio.h"
#endif

static const char *TAG = "DatalinkRX";

void datalink_rs485_rx_task(void *arg) {
    rs485_instance_t *inst = (rs485_instance_t *)arg;
    if (!inst) return;

    LOG_I(TAG, "RS485 RX Task started");

    for (;;) {
        // 1. Block indefinitely until the HAL tells us a byte is ready.
        // Inside the HAL, the UART RX interrupt triggers this semaphore.
        if (rs485_hal_wait_for_bytes(portMAX_DELAY)) {
            while (rs485_hal_bytes_available()) {
                uint8_t b = rs485_hal_read_byte();
                rs485_process_byte(inst, b);
            }
        }

        // 2. Delegate packet assembly check and TCTLM dispatch to the library
        RS485_packet_t pkt;
        rs485_rx_update(inst, &pkt);

        // No vTaskDelay needed! The task naturally sleeps on the wait call.
    }
}

void datalink_usb_rx_task(void *arg) {
    rs485_instance_t *inst = (rs485_instance_t *)arg;
    if (!inst) return;

    LOG_I(TAG, "USB RX Task started");

    for (;;) {
        // 1. Ingest all available bytes from the USB CDC stack
#if PICO_BUILD
        int ch;
        while ((ch = getchar_timeout_us(0)) != PICO_ERROR_TIMEOUT) {
            rs485_process_byte(inst, (uint8_t)ch);
        }
#endif

        // 2. Delegate packet assembly check and TCTLM dispatch to the library
        RS485_packet_t pkt;
        rs485_rx_update(inst, &pkt);

        vTaskDelay(pdMS_TO_TICKS(1));
    }
}
