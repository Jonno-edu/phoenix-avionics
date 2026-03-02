/**
 * @file norb_streamer.c
 * @brief nORB Topic Streamer — streams subscribed topics to the GSU over RS485.
 *
 * Design:
 *   - The GSU sends TC_OBC_NORB_SUBSCRIBE (topic_id, rate_ms) to start/stop a stream.
 *   - This module maintains a per-topic subscription table.
 *   - A single FreeRTOS task wakes every 10 ms, checks which subscriptions are
 *     due, polls the latest value from nORB, and sends a TLM_OBC_NORB_STREAM
 *     packet to the requesting address only when the value has changed.
 *   - rate_ms is a throttle cap: packets are sent at most that often, but only
 *     when the topic's value has actually been updated by its publisher.
 *     Example: EPS polled at 1 Hz with rate_ms=100 → 1 packet/s, not 10.
 */

#include "norb_streamer.h"
#include "datalink.h"
#include "phoenix_icd.h"
#include "norb/norb.h"
#include "norb/topics.h"

/* All generated topic struct definitions */
#include "norb/topic_defs/barometer.h"
#include "norb/topic_defs/eps_ident.h"
#include "norb/topic_defs/eps_measurements.h"
#include "norb/topic_defs/eps_power_status.h"
#include "norb/topic_defs/obc_ident.h"
#include "norb/topic_defs/obc_log_level.h"
#include "norb/topic_defs/sensor_imu.h"
#include "norb/topic_defs/tracking_radio_ident.h"
#include "norb/topic_defs/vehicle_imu.h"
#include "norb/topic_defs/vehicle_state.h"

#include "FreeRTOS.h"
#include "task.h"

#include <string.h>
#include <stdint.h>
#include <stdbool.h>

/* -----------------------------------------------------------------------
 * Topic size lookup table
 * Must stay in sync with the topic_id_t enum order.
 * ----------------------------------------------------------------------- */

/** Maximum payload for any single nORB topic (vehicle_state_t = 76 bytes). */
#define NORB_STREAM_MAX_TOPIC_BYTES  128U

static const uint8_t k_topic_sizes[TOPIC_COUNT] = {
    [TOPIC_BAROMETER]            = (uint8_t)sizeof(barometer_t),              /* 20 B */
    [TOPIC_EPS_IDENT]            = (uint8_t)sizeof(eps_ident_t),              /*  8 B */
    [TOPIC_EPS_MEASUREMENTS]     = (uint8_t)sizeof(eps_measurements_t),       /* 10 B */
    [TOPIC_EPS_POWER_STATUS]     = (uint8_t)sizeof(eps_power_status_t),       /*  2 B */
    [TOPIC_OBC_IDENT]            = (uint8_t)sizeof(obc_ident_t),              /*  8 B */
    [TOPIC_OBC_LOG_LEVEL]        = (uint8_t)sizeof(obc_log_level_t),          /*  1 B */
    [TOPIC_SENSOR_IMU]           = (uint8_t)sizeof(sensor_imu_t),             /* 36 B */
    [TOPIC_TRACKING_RADIO_IDENT] = (uint8_t)sizeof(tracking_radio_ident_t),   /*  8 B */
    [TOPIC_VEHICLE_IMU]          = (uint8_t)sizeof(vehicle_imu_t),            /* 28 B */
    [TOPIC_VEHICLE_STATE]        = (uint8_t)sizeof(vehicle_state_t),          /* 76 B */
};

/* -----------------------------------------------------------------------
 * Subscription table
 * ----------------------------------------------------------------------- */

typedef struct {
    bool     enabled;
    uint16_t rate_ms;
    uint32_t last_sent_ms;
    uint8_t  requester;                             /* RS485 address to send TM to */
    uint8_t  last_buf[NORB_STREAM_MAX_TOPIC_BYTES]; /* last sent value — change detection */
} norb_sub_entry_t;

static norb_sub_entry_t g_subscriptions[TOPIC_COUNT];

/* -----------------------------------------------------------------------
 * FreeRTOS task
 * ----------------------------------------------------------------------- */

static StaticTask_t s_streamer_tcb;
static StackType_t  s_streamer_stack[1024];

static void norb_streamer_task(void *arg)
{
    (void)arg;

    /*
     * Static local buffers — live in BSS, not on the task stack.
     * topic_buf: raw nORB struct bytes (up to 128 B)
     * send_buf:  RS485 payload = [topic_id (1 B)] + [raw bytes]
     */
    static uint8_t topic_buf[NORB_STREAM_MAX_TOPIC_BYTES];
    static uint8_t send_buf[1U + NORB_STREAM_MAX_TOPIC_BYTES];

    for (;;)
    {
        uint32_t now_ms = xTaskGetTickCount() * (1000U / configTICK_RATE_HZ);

        for (int i = 0; i < (int)TOPIC_COUNT; i++)
        {
            norb_sub_entry_t *sub = &g_subscriptions[i];

            if (!sub->enabled || sub->rate_ms == 0U) {
                continue;
            }

            if ((now_ms - sub->last_sent_ms) < (uint32_t)sub->rate_ms) {
                continue;
            }

            /* Try to read the latest value from nORB */
            if (!norb_subscribe_poll((topic_id_t)i, topic_buf)) {
                continue; /* topic never published yet */
            }

            uint8_t sz = k_topic_sizes[i];

            /* rate_ms is a throttle cap — only send if the value changed */
            if (memcmp(topic_buf, sub->last_buf, sz) == 0) {
                sub->last_sent_ms = now_ms;
                continue;
            }

            memcpy(sub->last_buf, topic_buf, sz);
            sub->last_sent_ms = now_ms;

            /* Build TM packet: [topic_id][raw struct] */
            send_buf[0] = (uint8_t)i;
            memcpy(&send_buf[1], topic_buf, sz);

            datalink_send(
                sub->requester,
                MSG_TYPE_TLM_RESP,
                TLM_OBC_NORB_STREAM,
                send_buf,
                (uint8_t)(1U + sz)
            );
        }

        vTaskDelay(pdMS_TO_TICKS(10)); /* 10 ms resolution */
    }
}

/* -----------------------------------------------------------------------
 * Public API
 * ----------------------------------------------------------------------- */

void norb_streamer_init(void)
{
    memset(g_subscriptions, 0, sizeof(g_subscriptions));

    xTaskCreateStatic(
        norb_streamer_task,
        "norb_stream",
        1024U,
        NULL,
        3U,   /* priority — below datalink RX (5) and housekeeping (2 default) */
        s_streamer_stack,
        &s_streamer_tcb
    );
}

void norb_streamer_handle_tc(RS485_packet_t *pkt)
{
    if (pkt->length < (uint8_t)sizeof(NorbSubscribePayload_t)) {
        return;
    }

    /* Safe cast: pkt->data is a flat byte array, struct is packed */
    const NorbSubscribePayload_t *cmd = (const NorbSubscribePayload_t *)pkt->data;

    if (cmd->topic_id >= (uint8_t)TOPIC_COUNT) {
        return; /* unknown topic — ignore */
    }

    norb_sub_entry_t *sub = &g_subscriptions[cmd->topic_id];

    if (cmd->rate_ms == 0U) {
        /* rate_ms == 0  →  unsubscribe */
        sub->enabled = false;
        return;
    }

    sub->rate_ms      = cmd->rate_ms;
    sub->requester    = pkt->src_addr;
    sub->enabled      = true;
    sub->last_sent_ms = 0U; /* send on next tick */

    /* Sentinel so first packet always goes out regardless of value */
    memset(sub->last_buf, 0xFFU, sizeof(sub->last_buf));
}
