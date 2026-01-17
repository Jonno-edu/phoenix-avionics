// rs485_hal.c
#include "hal/rs485_hal.h"
#include <stdio.h>
#include "FreeRTOS.h"
#include "task.h"

#if PICO_BUILD
    #include <pico/stdlib.h>
    #include <hardware/uart.h>
    #include <hardware/gpio.h>
    #include <hardware/irq.h>

    // RS485 Bus A Hardware Configuration
    #define RS485_UART_ID       uart1
    #define RS485_BAUD_RATE     115200
    #define RS485_TX_PIN        4
    #define RS485_RX_PIN        5
    #define RS485_DE_RE_PIN     3       // Direction control (DE and RE tied together)

    // Circular buffer for RS485 RX
    static volatile uint8_t rs485_rx_buffer[RS485_BUFFER_SIZE];
    static volatile uint16_t rs485_write_index = 0;
    static volatile uint16_t rs485_read_index = 0;

    // UART1 RX Interrupt Handler
    void on_rs485_uart_rx(void) {
        while (uart_is_readable(RS485_UART_ID)) {
            uint8_t ch = uart_getc(RS485_UART_ID);
            rs485_hal_buffer_push(ch);
        }
    }

    void rs485_hal_init(void) {
        // Initialize UART1
        uart_init(RS485_UART_ID, RS485_BAUD_RATE);
        
        // Set GPIO functions for UART1
        gpio_set_function(RS485_TX_PIN, GPIO_FUNC_UART);
        gpio_set_function(RS485_RX_PIN, GPIO_FUNC_UART);
        
        // Configure DE/RE control pin as output
        gpio_init(RS485_DE_RE_PIN);
        gpio_set_dir(RS485_DE_RE_PIN, GPIO_OUT);
        gpio_put(RS485_DE_RE_PIN, 0);  // Start in receive mode (LOW)
        
        // Enable UART1 RX interrupt
        uart_set_irq_enables(RS485_UART_ID, true, false);  // RX enable, TX disable
        irq_set_exclusive_handler(UART1_IRQ, on_rs485_uart_rx);
        irq_set_enabled(UART1_IRQ, true);
        
        printf("[RS485 HAL] Bus A initialized on UART1 (GPIO 4/5), DE/RE on GPIO 3\n");
    }

    void rs485_hal_send_byte(uint8_t byte) {
        // Step 1: Enable transmitter (set DE/RE HIGH)
        gpio_put(RS485_DE_RE_PIN, 1);
        
        // Small delay to allow transceiver to switch modes (~1-2 us typical)
        // At 150 MHz, a few NOPs is sufficient
        __asm volatile("nop\nnop\nnop\nnop\nnop");
        
        // Step 2: Send the byte
        uart_putc_raw(RS485_UART_ID, byte);
        
        // Step 3: Wait for UART to finish transmission
        uart_tx_wait_blocking(RS485_UART_ID);
        
        // Step 4: Disable transmitter, enable receiver (set DE/RE LOW)
        gpio_put(RS485_DE_RE_PIN, 0);
    }

#else
    // SIL Build: No real hardware, stubs only
    void rs485_hal_init(void) {
        printf("[RS485 HAL] SIL mode - hardware init skipped\n");
    }

    void rs485_hal_send_byte(uint8_t byte) {
        // In SIL, RS485 HAL doesn't actually send anything.
        // The protocol layer will use the console PTY for testing.
        (void)byte;
    }
#endif

// Circular buffer functions (common to both hardware and SIL)
#if PICO_BUILD
void rs485_hal_buffer_push(uint8_t byte) {
    rs485_rx_buffer[rs485_write_index] = byte;
    rs485_write_index = (rs485_write_index + 1) % RS485_BUFFER_SIZE;
    
    // If buffer full, overwrite oldest data
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
        return 0;  // Buffer empty
    }
    
    uint8_t byte = rs485_rx_buffer[rs485_read_index];
    rs485_read_index = (rs485_read_index + 1) % RS485_BUFFER_SIZE;
    taskEXIT_CRITICAL();
    return byte;
}
#else
// SIL stubs - no actual buffer since we use console PTY
void rs485_hal_buffer_push(uint8_t byte) {
    (void)byte;
}

bool rs485_hal_bytes_available(void) {
    return false;
}

uint8_t rs485_hal_read_byte(void) {
    return 0;
}
#endif
