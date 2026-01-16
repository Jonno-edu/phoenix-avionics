#ifndef PLATFORM_HAL_H
#define PLATFORM_HAL_H

#include <stdint.h>

/**
 * @brief  System panic handler. Reset or exit.
 * @param  msg Error message to display/log
 */
void platform_panic(const char *msg);

/**
 * @brief  Multiplexed send function.
 *         Sends byte to both RS485 (if available) and debug console.
 *         Used as the callback for the RS485 protocol layer.
 */
void platform_send_byte_mux(uint8_t byte);

#endif // PLATFORM_HAL_H
