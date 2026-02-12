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
