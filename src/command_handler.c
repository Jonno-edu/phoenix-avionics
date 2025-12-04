// command_handler.c
#include "command_handler.h"
#include "system_data.h"
#include "serial.h"

void command_handler_process(uint8_t command) {
    switch (command) {
        case CMD_GET_IDENTIFICATION: {
            SystemData_t sys_data;
            uint8_t frame_buffer[SYSTEM_DATA_FRAME_LENGTH];
            
            // Get current system data
            system_data_get(&sys_data);
            
            // Pack into byte array
            system_data_pack(&sys_data, frame_buffer);
            
            // Send TLM ID
            serial_send_byte(TLM_ID_IDENTIFICATION);
            
            // Send frame data
            for (int i = 0; i < SYSTEM_DATA_FRAME_LENGTH; i++) {
                serial_send_byte(frame_buffer[i]);
            }
            break;
        }
        
        default:
            // Unknown command - ignore or send error
            break;
    }
}
