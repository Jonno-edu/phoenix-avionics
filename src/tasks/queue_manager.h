#ifndef QUEUE_MANAGER_H
#define QUEUE_MANAGER_H

#include <stdint.h>
#include <FreeRTOS.h>
#include <queue.h>
#include "rs485_protocol.h"

// Interface Identifiers
typedef enum {
    IF_NONE = 0,
    IF_RS485 = 1,
    IF_USB = 2
} InterfaceID_t;

// Command Event Structure
typedef struct {
    InterfaceID_t source_interface;
    RS485_packet_t packet;
} CommandEvent_t;

// Global Queue Handles
extern QueueHandle_t q_command_inbox;
extern QueueHandle_t q_usb_out;
extern QueueHandle_t q_rs485_out;

// Task Priorities
#define PRIORITY_PILOT_IMU           configMAX_PRIORITIES - 1
#define PRIORITY_PILOT_ESTIMATOR     configMAX_PRIORITIES - 2
#define PRIORITY_RS485_PROCESSING    configMAX_PRIORITIES - 3
#define PRIORITY_COPILOT_CMD         configMAX_PRIORITIES - 3
#define PRIORITY_PILOT_FSM           configMAX_PRIORITIES - 4
#define PRIORITY_PILOT_TELEM         configMAX_PRIORITIES - 5

// Initialization
void queues_init(void);

#endif // QUEUE_MANAGER_H
