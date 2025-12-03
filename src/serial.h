// serial.h
#ifndef SERIAL_H
#define SERIAL_H

#include <stdint.h>

void serial_send_byte(uint8_t byte);
void serial_init(void);

#endif // SERIAL_H
