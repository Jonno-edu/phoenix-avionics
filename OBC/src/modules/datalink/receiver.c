#include "datalink.h"
#include "tctlm.h"
#include "hal/rs485_hal.h"
#include "logging.h"

#include "FreeRTOS.h"
#include "task.h"

static const char *TAG = "DatalinkRX";

void datalink_rs485_rx_task(void *arg) {
    rs485_instance_t *inst = (rs485_instance_t *)arg;
    if (!inst) return;

    LOG_I(TAG, "RS485 RX Task started");

    for (;;) {
        // 1. Ingest all available bytes from the RS485 UART HAL
        while (rs485_hal_bytes_available()) {
            uint8_t b = rs485_hal_read_byte();
            rs485_process_byte(inst, b);
        }

        // 2. Delegate packet assembly check and TCTLM dispatch to the library
        RS485_packet_t pkt;
        rs485_rx_update(inst, &pkt);

        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

void datalink_usb_rx_task(void *arg) {
    rs485_instance_t *inst = (rs485_instance_t *)arg;
    if (!inst) return;

    LOG_I(TAG, "USB RX Task started");

    for (;;) {
        // 1. Ingest all available bytes from the USB CDC stack (TODO)
        // while (usb_cdc_bytes_available()) {
        //     rs485_process_byte(inst, usb_cdc_read_byte());
        // }

        // 2. Delegate packet assembly check and TCTLM dispatch to the library
        RS485_packet_t pkt;
        rs485_rx_update(inst, &pkt);

        vTaskDelay(pdMS_TO_TICKS(1));
    }
}
