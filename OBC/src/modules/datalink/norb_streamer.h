#pragma once

/**
 * @file norb_streamer.h
 * @brief nORB Topic Streamer
 *
 * Handles streaming of nORB topics to the GSU over RS485.
 *
 * The GSU sends TC_OBC_NORB_SUBSCRIBE (payload: NorbSubscribePayload_t) to
 * subscribe to a topic at a given rate. This module maintains a subscription
 * table and periodically sends TLM_OBC_NORB_STREAM packets to the requester's
 * address whenever the topic has been updated.
 *
 * Packet format of TLM_OBC_NORB_STREAM:
 *   byte[0]      = topic_id  (uint8_t, matches topic_id_t enum)
 *   byte[1..N]   = raw topic struct bytes (little-endian, matches generated header)
 */

#include "rs485_protocol.h"

/**
 * @brief Initialise the nORB streamer and start its FreeRTOS task.
 * Call this once from datalink_init(), after norb_init().
 */
void norb_streamer_init(void);

/**
 * @brief Handle an inbound TC_OBC_NORB_SUBSCRIBE telecommand packet.
 * Called from TCTLM_processTelecommand() when cmd ID == TC_OBC_NORB_SUBSCRIBE.
 */
void norb_streamer_handle_tc(RS485_packet_t *pkt);
