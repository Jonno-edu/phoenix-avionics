#ifndef OBC_COMMANDS_H
#define OBC_COMMANDS_H

#include "rs485_protocol.h"
#include "tasks/queue_manager.h" // For InterfaceID_t

/**
 * @brief Handle a Telecommand directed to the OBC
 * 
 * @param src_id Source Interface (RS485 or USB)
 * @param pkt The packet
 */
void obc_handle_telecommand(InterfaceID_t src_id, RS485_packet_t *pkt);

#endif // OBC_COMMANDS_H
