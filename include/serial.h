// serial.h
#ifndef SERIAL_H
#define SERIAL_H

#include <stdint.h>
#include <stdbool.h>

#define SERIAL_BUFFER_SIZE 256

void serial_send_byte(uint8_t byte);
void serial_init(void);

// Circular Buffer Access
void serial_buffer_push(uint8_t byte);
uint8_t serial_read_byte(void);
bool serial_bytes_available(void);

#endif // SERIAL_H
