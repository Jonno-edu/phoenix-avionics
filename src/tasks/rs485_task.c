#include "core/logging.h"
#include "rs485_task.h"
#include "task_manager.h"
#include "rs485_protocol.h"
#include "hal/rs485_hal.h"
#include "core/usb_console.h"
#include "core/system_data.h"
#include <FreeRTOS.h>
#include <task.h>
#include <stdio.h>

#if PICO_BUILD
#include <hardware/watchdog.h>
#endif

static const char *TAG = "RS485";

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
                    // Echo back an ACK (for all telecommands)
                    uint8_t ack_desc = BUILD_MSG_DESC(MSG_TYPE_TC_ACK, id);
                    rs485_send_packet(packet.src_addr, ack_desc, NULL, 0);

                    // Handle telecommand IDs
                    if (id == ID_CMD_RESET) {
                        ESP_LOGE(TAG, "RESET COMMAND RECEIVED! Rebooting...");
                        vTaskDelay(pdMS_TO_TICKS(20));
#if PICO_BUILD
                        watchdog_reboot(0, 0, 0);
#endif
                    }
                    else if (id == ID_CMD_SET_SIM_STATE) {
                        if (packet.length >= 1) {
                            bool enable = (packet.data[0] == 1);
                            system_config_set_sim_mode(enable);
                            ESP_LOGI(TAG, "CMD: Set Sim Mode = %d", enable);
                        }
                    }
                    else if (id == ID_CMD_SET_LOG_LEVEL) {
                        if (packet.length >= 1) {
                            system_config_set_log_level(packet.data[0]);
                            ESP_LOGI(TAG, "CMD: Set Log Level = %d", packet.data[0]);
                        }
                    }
                    else if (id == ID_CMD_SET_TELEM_RATE) {
                        if (packet.length >= 1) {
                            system_config_set_telem_rate(packet.data[0]);
                            ESP_LOGI(TAG, "CMD: Set Telem Rate = %d Hz", packet.data[0]);
                        }
                    }
                }
                else if (type == MSG_TYPE_TLM_REQ) {
                    if (id == ID_TLM_IDENTIFICATION) {
                        SystemData_t sys_data;
                        uint8_t frame_buffer[sizeof(SystemData_t)];
                        
                        system_data_get(&sys_data);
                        system_data_pack(&sys_data, frame_buffer);
                        
                        uint8_t resp_desc = BUILD_MSG_DESC(MSG_TYPE_TLM_RESP, id);
                        rs485_send_packet(packet.src_addr, resp_desc, frame_buffer, sizeof(SystemData_t));
                    }
                }
                else if (type == MSG_TYPE_TLM_RESP) {
                    if (id == ID_TLM_IDENTIFICATION) {
                        ESP_LOGI(TAG, "TLM RESP: Identification from %02X, payload len=%d",
                               packet.src_addr, packet.length);

                        // Print payload bytes
                        ESP_LOG_BUFFER_HEX(TAG, packet.data, packet.length);

                        // Decode identification frame
                        if (packet.length == sizeof(TlmIdentificationPayload_t)) {
                            TlmIdentificationPayload_t *tlm = (TlmIdentificationPayload_t *)packet.data;

                            // For now, assuming raw mapping (Little Endian to Little Endian)
                            ESP_LOGI(TAG, "EPS node_type=%u iface=%u fw=%u.%u uptime=%us %ums flags=0x%02X",
                                   tlm->node_type, tlm->interface_version, 
                                   tlm->firmware_major, tlm->firmware_minor,
                                   tlm->uptime_seconds, tlm->uptime_milliseconds, tlm->status_flags);

                            // Piggyback: if EPS reports a pending command, ask it to send it.
                            if (tlm->status_flags & STATUS_FLAG_CMD_PENDING) {
                                ESP_LOGI(TAG, "[OBC] EPS indicates CMD_PENDING -> requesting pending command...");
                                uint8_t get_desc = BUILD_MSG_DESC(MSG_TYPE_TELECOMMAND, ID_CMD_GET_PENDING_MSG);
                                rs485_send_packet(packet.src_addr, get_desc, NULL, 0);
                            }
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
