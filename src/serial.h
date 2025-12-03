// serial.h
#ifndef SERIAL_H
#define SERIAL_H

#include <stdint.h>
#include <stdbool.h>

#define SERIAL_BUFFER_SIZE 256
#define COMMAND_BYTE 0x80

void serial_send_byte(uint8_t byte);
void serial_init(void);
void serial_process_commands(void);

#endif // SERIAL_H
