// rs485_hal.h
#ifndef RS485_HAL_H
#define RS485_HAL_H

#include <stdint.h>
#include <stdbool.h>

#define RS485_BUFFER_SIZE 256

/**
 * @brief Enable or disable raw RX byte logging to USB serial.
 * Default: disabled. Enable during debugging, disable for production.
 * Can be toggled at runtime (e.g. via TC_OBC_LOG_LEVEL command).
 */
void rs485_hal_set_raw_debug(bool enabled);
bool rs485_hal_raw_debug_enabled(void);


/**
 * @brief Initialize RS485 Bus A hardware (UART1 + GPIO control).
 * 
 * Configures:
 * - UART1 on GPIO 4 (TX) and GPIO 5 (RX)
 * - GPIO 3 as direction control (DE/RE)
 * - UART RX interrupt
 * - Default state: Receive mode (DE/RE = LOW)
 */
void rs485_hal_init(void);

/**
 * @brief Print raw RX bytes from the log buffer (TASK SAFE).
 */
void rs485_hal_print_raw_log(void);

/**
 * @brief Set the direction pin.
 */
void rs485_hal_set_direction(bool tx_mode);

/**
 * @brief Send a block of data over RS485 Bus A.
 * 
 * Automatically handles direction control:
 * 1. Sets DE/RE HIGH (enable transmitter)
 * 2. Sends the entire buffer via UART1
 * 3. Waits for transmission to complete
 * 4. Sets DE/RE LOW (enable receiver)
 * 
 * @param data Pointer to data buffer.
 * @param len  Length of data to send.
 */
void rs485_hal_send(const uint8_t *data, uint16_t len);

/**
 * @brief Push a received byte into the RS485 RX circular buffer.
 * 
 * NOTE: This is called from ISR context (UART1 RX interrupt).
 * Do NOT call from task context without proper synchronization.
 * 
 * @param byte The received byte.
 */
void rs485_hal_buffer_push(uint8_t byte);

/**
 * @brief Read a byte from the RS485 RX circular buffer.
 * 
 * Thread-safe. Returns 0 if buffer is empty.
 * 
 * @return The oldest byte in the buffer, or 0 if empty.
 */
uint8_t rs485_hal_read_byte(void);

/**
 * @brief Block the calling task until a byte is received OR the timeout occurs.
 * 
 * Uses a FreeRTOS binary semaphore given by the RX interrupt.
 * 
 * @param timeout_ms Max time to wait. Use portMAX_DELAY for indefinite.
 * @return true if data became available, false on timeout.
 */
bool rs485_hal_wait_for_bytes(uint32_t timeout_ms);

/**
 * @brief Check if there are bytes available in the RS485 RX buffer.
 * 
 * Thread-safe.
 * 
 * @return true if data is available, false otherwise.
 */
bool rs485_hal_bytes_available(void);

#endif // RS485_HAL_H
