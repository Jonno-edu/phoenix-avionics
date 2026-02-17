#include "queue_manager.h"

QueueHandle_t q_command_inbox = NULL;
QueueHandle_t q_usb_out = NULL;
QueueHandle_t q_rs485_out = NULL;

void queues_init(void) {
    q_command_inbox = xQueueCreate(10, sizeof(CommandEvent_t));
    q_usb_out       = xQueueCreate(10, sizeof(RS485_packet_t));
    q_rs485_out     = xQueueCreate(10, sizeof(RS485_packet_t));
}
