#pragma once

/**
 * @file norb_streamer.h
 * @brief nORB Topic Streamer
 *
 * Streams subscribed nORB topics to the GSU over RS485 using an event-driven
 * architecture (FreeRTOS Task Notifications bitmask).
 *
 * Subscribe/unsubscribe:
 *   The GSU sends TC_OBC_NORB_SUBSCRIBE (payload: NorbSubscribePayload_t).
 *   rate_ms > 0 subscribes at the given throttle cap; rate_ms == 0 unsubscribes.
 *
 * Event delivery:
 *   When a subscribed topic is published via norb_publish(), a callback sets a
 *   bit in the streamer task's notification mask. The task wakes, checks the
 *   per-topic rate throttle, and sends MSG_TYPE_EVENT to ADDR_GSE.
 *
 * Packet format (MSG_TYPE_EVENT / EVENT_OBC_NORB_STREAM = 0x02):
 *   byte[0]      = topic_id        (uint8_t, matches topic_id_t enum)
 *   byte[1..4]   = timestamp_ms    (uint32_t little-endian, from xTaskGetTickCount)
 *   byte[5..N]   = raw topic struct (little-endian, matches generated header layout)
 *
 * Adding a new topic requires zero changes to this module:
 *   1. Create msg/<name>.msg and rebuild.
 *   The generator updates norb_topic_sizes[] and all generated headers automatically.
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
