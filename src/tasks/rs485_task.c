#include "rs485_task.h"
#include "task_manager.h"
#include "rs485_protocol.h"
#include "rs485_hal.h"
#include "usb_console.h"
#include "system_data.h"
#include <FreeRTOS.h>
#include <task.h>
#include <stdio.h>

static void vRS485Task(void *pvParameters) {
    (void)pvParameters;
    
    RS485Packet_t packet;
    
    while (true) {
        // Feed buffered bytes into the protocol parser
#if PICO_BUILD
        // 1. Process bytes from real RS485 Hardware (Bus A)
        while (rs485_hal_bytes_available()) {
            rs485_process_byte(rs485_hal_read_byte());
        }

        // 2. Process bytes from USB Console (Debug/Test)
        while (console_bytes_available()) {
            rs485_process_byte(console_read_byte());
        }
#else
        // SIL: Read from Virtual Serial Port (Console)
        while (console_bytes_available()) {
            rs485_process_byte(console_read_byte());
        }
#endif

        if (rs485_get_packet(&packet)) {
            // Is it for us? (OBC = 1)
            if (packet.dest_addr == ADDR_OBC) {
                // Determine Message Type & ID
                uint8_t type = GET_MSG_TYPE(packet.msg_desc);
                uint8_t id   = GET_MSG_ID(packet.msg_desc);

                if (type == MSG_TYPE_TELECOMMAND) {
                    if (id == ID_CMD_RESET) {
                        printf("RESET COMMAND RECEIVED!\n");
                    }
                    // Echo back an ACK
                    uint8_t ack_desc = BUILD_MSG_DESC(MSG_TYPE_TC_ACK, id);
                    rs485_send_packet(packet.src_addr, ack_desc, NULL, 0);
                }
                else if (type == MSG_TYPE_TLM_REQ) {
                    if (id == ID_TLM_IDENTIFICATION) {
                        SystemData_t sys_data;
                        uint8_t frame_buffer[SYSTEM_DATA_FRAME_LENGTH];
                        
                        system_data_get(&sys_data);
                        system_data_pack(&sys_data, frame_buffer);
                        
                        uint8_t resp_desc = BUILD_MSG_DESC(MSG_TYPE_TLM_RESP, id);
                        rs485_send_packet(packet.src_addr, resp_desc, frame_buffer, SYSTEM_DATA_FRAME_LENGTH);
                    }
                }
                else if (type == MSG_TYPE_TLM_RESP) {
                    if (id == ID_TLM_IDENTIFICATION) {
                        printf("TLM RESP: Identification from %02X, payload len=%d\n",
                               packet.src_addr, packet.length);

                        // Print payload bytes
                        printf("RX HEX: ");
                        for (int i = 0; i < packet.length; i++) {
                            printf("%02X ", packet.data[i]);
                        }
                        printf("\n");

                        // If payload matches SystemData_t pack format (8 bytes), decode it
                        if (packet.length == SYSTEM_DATA_FRAME_LENGTH) {
                            uint16_t runtime_s = ((uint16_t)packet.data[4] << 8) | packet.data[5];
                            uint16_t runtime_ms = ((uint16_t)packet.data[6] << 8) | packet.data[7];

                            printf("EPS node_type=%u iface=%u fw=%u.%u uptime=%us %ums\n",
                                   packet.data[0], packet.data[1], packet.data[2], packet.data[3],
                                   runtime_s, runtime_ms);
                        }
                    }
                }
            }
        }
        vTaskDelay(pdMS_TO_TICKS(5)); // Yield
    }
}

void rs485_task_init(void) {
    xTaskCreate(
        vRS485Task,
        "RS485",
        1024,
        NULL,
        PRIORITY_RS485_PROCESSING,
        NULL
    );
}
