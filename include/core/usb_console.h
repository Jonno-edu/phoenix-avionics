// usb_console.h
#ifndef USB_CONSOLE_H
#define USB_CONSOLE_H

#include <stdint.h>
#include <stdbool.h>

#define CONSOLE_BUFFER_SIZE 256

void console_send(const uint8_t *data, uint16_t len);
void console_init(void);

// Circular Buffer Access
void console_buffer_push(uint8_t byte);
uint8_t console_read_byte(void);
bool console_bytes_available(void);

#if !PICO_BUILD
// SIL: FreeRTOS task for polling virtual serial port
void vConsoleRxTask(void *pvParameters);
#endif

#endif // USB_CONSOLE_H
