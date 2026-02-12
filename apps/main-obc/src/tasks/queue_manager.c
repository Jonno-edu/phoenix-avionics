#include "queue_manager.h"
#include <stdio.h> 

QueueHandle_t xEstimatorQueue = NULL;
QueueHandle_t xPilotCommandQueue = NULL;

QueueHandle_t q_command_inbox = NULL;
QueueHandle_t q_usb_out = NULL;
QueueHandle_t q_rs485_out = NULL;

void queue_manager_init(void) {
    // 1. Command Inbox (Mixed sources)
    // Depth: 10 (2.6KB)
    q_command_inbox = xQueueCreate(10, sizeof(CommandEvent_t));
    if (q_command_inbox == NULL) {
        printf("FAILED to create q_command_inbox\n");
    }

    // 2. USB Outbox (High Speed Stream)
    // Depth: 20 (5.2KB)
    q_usb_out = xQueueCreate(20, sizeof(RS485_packet_t));
    if (q_usb_out == NULL) {
        printf("FAILED to create q_usb_out\n");
    }

    // 3. RS485 Outbox (Low Speed Bus)
    // Depth: 5 (1.3KB)
    q_rs485_out = xQueueCreate(5, sizeof(RS485_packet_t));
    if (q_rs485_out == NULL) {
        printf("FAILED to create q_rs485_out\n");
    }

    // Legacy Queues (Commented out until needed)
    // xEstimatorQueue = xQueueCreate(10, sizeof(SensorData_t));
    // xPilotCommandQueue = xQueueCreate(5, sizeof(Command_t));
}

bool cmd_inject_packet(InterfaceID_t src_interface, uint8_t msg_type, uint8_t msg_id, const uint8_t *payload, uint8_t len) {
    if (q_command_inbox == NULL) return false;

    CommandEvent_t event;
    event.source_interface = src_interface;
    
    // Construct minimal valid packet
    // Note: We use ADDR_OBC (0x01) as Dest because we are injecting TO ourselves.
    // Tests might override Src Addr in the payload or we can add a param for it later.
    event.packet.dest_addr = 0x01; // ADDR_OBC
    event.packet.src_addr  = 0x01; // ADDR_OBC (Self-addressed)
    
    event.packet.msg_desc.type = msg_type;
    event.packet.msg_desc.id   = msg_id;
    event.packet.length        = (len > RS485_MAX_PAYLOAD) ? RS485_MAX_PAYLOAD : len;
    event.packet.crc           = 0; // Not checked for internal injection commands
    
    // Copy Payload
    if (payload != NULL && event.packet.length > 0) {
        for (uint8_t i = 0; i < event.packet.length; i++) {
            event.packet.data[i] = payload[i];
        }
    }

    if (xQueueSend(q_command_inbox, &event, 0) != pdTRUE) {
        printf("FAILED to inject command: Queue Full\n");
        return false;
    }

    return true;
}
