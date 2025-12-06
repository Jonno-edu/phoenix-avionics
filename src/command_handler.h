// command_handler.h
#ifndef COMMAND_HANDLER_H
#define COMMAND_HANDLER_H

#include <stdint.h>

#define CMD_GET_IDENTIFICATION  0x80
#define CMD_GET_BARO            0x20

/**
 * @brief Process a received command byte
 * @param command The command byte received
 */
void command_handler_process(uint8_t command);

#endif // COMMAND_HANDLER_H
