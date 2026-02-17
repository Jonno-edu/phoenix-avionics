#ifndef QUEUE_MANAGER_H
#define QUEUE_MANAGER_H

#include <FreeRTOS.h>
#include <queue.h>
#include "rs485_protocol.h"

// ============================================================================
// Data Types
// ============================================================================

typedef enum {
    IF_USB = 0,
    IF_RS485 = 1,
    IF_LOOPBACK = 2
} InterfaceID_t;

typedef struct {
    RS485_packet_t packet;
    InterfaceID_t source_interface;
} CommandEvent_t;

// ============================================================================
// Queue Handles
// ============================================================================

// 1. Inbox: Commands arriving from any interface (USB or RS485)
//    Consumers: Dispatcher Task
extern QueueHandle_t q_command_inbox;

// 2. Outbox: Packets destined for the Ground Station (USB)
//    Consumers: USB Transport Task
extern QueueHandle_t q_usb_out;

// 3. Outbox: Packets destined for the Avionics Bus (RS485)
//    Consumers: RS485 Transport Task
extern QueueHandle_t q_rs485_out;


// Legacy Queues (To be reviewed/removed)
extern QueueHandle_t xEstimatorQueue;
extern QueueHandle_t xPilotCommandQueue;

// Initialization
void queue_manager_init(void);

/**
 * @brief Inject a synthetic packet into the command inbox.
 *        Useful for internal state changes or simulation.
 * 
 * @param src_interface The source ID (e.g., IF_LOOPBACK)
 * @param msg_type      Message Type (e.g., MSG_TYPE_TELECOMMAND)
 * @param msg_id        Message ID (e.g., TC_COMMON_RESET)
 * @param payload       Data buffer
 * @param len           Length of payload
 * @return true if queued successfully
 */
bool cmd_inject_packet(InterfaceID_t src_interface, uint8_t msg_type, uint8_t msg_id, const uint8_t *payload, uint8_t len);

#endif // QUEUE_MANAGER_H
