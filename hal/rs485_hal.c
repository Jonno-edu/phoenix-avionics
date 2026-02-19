// rs485_hal.c
#include "hal/rs485_hal.h"
#include <stdio.h>
#include "FreeRTOS.h"
#include "task.h"
#include "logging.h"

static const char *TAG = "RS485_HAL";

#if PICO_BUILD
    #include <pico/stdlib.h>
    #include <hardware/uart.h>
    #include <hardware/gpio.h>
    #include <hardware/irq.h>

    #define RS485_UART_ID       uart1
    #define RS485_BAUD_RATE     115200
    #define RS485_TX_PIN        4
    #define RS485_RX_PIN        5
    #define RS485_DE_RE_PIN     3

    static volatile uint8_t rs485_rx_buffer[RS485_BUFFER_SIZE];
    static volatile uint16_t rs485_write_index = 0;
    static volatile uint16_t rs485_read_index = 0;

    void on_rs485_uart_rx(void) {
        while (uart_is_readable(RS485_UART_ID)) {
            uint8_t ch = uart_getc(RS485_UART_ID);
            printf("[RS485 RX RAW] 0x%02X\n", ch);
            rs485_hal_buffer_push(ch);
        }
    }

    void rs485_hal_init(void) {
        uart_init(RS485_UART_ID, RS485_BAUD_RATE);
        gpio_set_function(RS485_TX_PIN, GPIO_FUNC_UART);
        gpio_set_function(RS485_RX_PIN, GPIO_FUNC_UART);
        gpio_init(RS485_DE_RE_PIN);
        gpio_set_dir(RS485_DE_RE_PIN, GPIO_OUT);
        gpio_put(RS485_DE_RE_PIN, 0);
        uart_set_irq_enables(RS485_UART_ID, true, false);
        irq_set_exclusive_handler(UART1_IRQ, on_rs485_uart_rx);
        irq_set_enabled(UART1_IRQ, true);
        ESP_LOGI(TAG, "Bus A initialized on UART1 (GPIO 4/5), DE/RE on GPIO 3");
    }

    void rs485_hal_send(const uint8_t *data, uint16_t len) {
        // --- DEBUG: print exact wire bytes ---
        printf("[RS485 TX] %d bytes: ", len);
        for (uint16_t i = 0; i < len; i++) printf("%02X ", data[i]);
        printf("\n");
        // --- END DEBUG ---

        uart_set_irq_enables(RS485_UART_ID, false, false);
        gpio_put(RS485_DE_RE_PIN, 1);
        __asm volatile("nop\nnop\nnop\nnop\nnop");
        uart_write_blocking(RS485_UART_ID, data, len);
        uart_tx_wait_blocking(RS485_UART_ID);
        gpio_put(RS485_DE_RE_PIN, 0);
        busy_wait_us(200);
        while (uart_is_readable(RS485_UART_ID)) { (void)uart_getc(RS485_UART_ID); }
        uart_set_irq_enables(RS485_UART_ID, true, false);
    }

#else
    void rs485_hal_init(void) {
        printf("[RS485 HAL] SIL mode - hardware init skipped\n");
    }

    void rs485_hal_send(const uint8_t *data, uint16_t len) {
        (void)data;
        (void)len;
    }
#endif

#if PICO_BUILD
void rs485_hal_buffer_push(uint8_t byte) {
    rs485_rx_buffer[rs485_write_index] = byte;
    rs485_write_index = (rs485_write_index + 1) % RS485_BUFFER_SIZE;
    if (rs485_write_index == rs485_read_index) {
        rs485_read_index = (rs485_read_index + 1) % RS485_BUFFER_SIZE;
    }
}

bool rs485_hal_bytes_available(void) {
    taskENTER_CRITICAL();
    bool available = (rs485_read_index != rs485_write_index);
    taskEXIT_CRITICAL();
    return available;
}

uint8_t rs485_hal_read_byte(void) {
    taskENTER_CRITICAL();
    if (rs485_read_index == rs485_write_index) {
        taskEXIT_CRITICAL();
        return 0;
    }
    uint8_t byte = rs485_rx_buffer[rs485_read_index];
    rs485_read_index = (rs485_read_index + 1) % RS485_BUFFER_SIZE;
    taskEXIT_CRITICAL();
    return byte;
}
#else
void rs485_hal_buffer_push(uint8_t byte) { (void)byte; }
bool rs485_hal_bytes_available(void) { return false; }
uint8_t rs485_hal_read_byte(void) { return 0; }
#endif
