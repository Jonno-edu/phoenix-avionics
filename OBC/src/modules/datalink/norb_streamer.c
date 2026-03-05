/**
 * @file norb_streamer.c
 * @brief nORB Topic Streamer — streams subscribed topics to the GSU over RS485.
 *
 * Design (event-driven):
 *   - The GSU sends TC_OBC_NORB_SUBSCRIBE (topic_id, rate_ms) to start/stop a
 *     stream. rate_ms == 0 unsubscribes.
 *   - This module registers a publish callback for every topic via
 *     norb_set_publish_callback(). When a topic is published, the callback
 *     sets a bit in the streamer task's 32-bit notification mask using
 *     xTaskNotify(..., eSetBits).
 *   - The streamer task blocks indefinitely on xTaskNotifyWait (0% CPU when
 *     idle), waking only when at least one topic fires.
 *   - On wake, it iterates only the set bits, applies the per-subscription
 *     rate_ms throttle, reads the latest value via norb_subscribe_poll, and
 *     sends an EVENT_OBC_NORB_STREAM packet to ADDR_GSE.
 *   - Multiple rapid publishes of the same topic coalesce into one bit: the
 *     task cannot be flooded by a bursty 1 kHz IMU.
 *   - rate_ms is a throttle cap: events fire at most that often, never faster.
 *
 * Packet format (MSG_TYPE_EVENT / EVENT_OBC_NORB_STREAM):
 *   byte[0]      = topic_id       (uint8_t)
 *   byte[1..4]   = timestamp_ms   (uint32_t, little-endian, from xTaskGetTickCount)
 *   byte[5..N]   = raw topic struct bytes (matches generated header layout)
 *
 * Topic sizes are provided by the auto-generated norb_topic_sizes[] array
 * declared in norb/topics.h and defined in norb_autogen.c. This file has
 * no per-topic includes and no hardcoded size table — adding a new .msg file
 * and rebuilding is the only action required.
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
#include <assert.h>

/* Task notification bitmask requires TOPIC_COUNT ≤ 32. Currently 10. */
static_assert(TOPIC_COUNT <= 32,
    "TOPIC_COUNT exceeds 32; cannot map all topics into a single 32-bit notification mask. "
    "Split into two streamer tasks or switch to a queue-based trigger.");

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
 *
 * Stripped down vs. the old polling model:
 *   - No last_buf[128] for change detection (saves 1280 bytes static RAM).
 *     Change detection is unnecessary — the callback only fires when
 *     norb_publish() is called, which already implies new data.
 *   - No requester field — all events are dispatched to ADDR_GSE.
 * ----------------------------------------------------------------------- */

typedef struct {
    bool     enabled;
    uint16_t rate_ms;
    uint32_t last_sent_ms;
} norb_sub_entry_t;

static norb_sub_entry_t g_subscriptions[TOPIC_COUNT];

/* -----------------------------------------------------------------------
 * FreeRTOS task
 * ----------------------------------------------------------------------- */

static StaticTask_t  s_streamer_tcb;
static StackType_t   s_streamer_stack[1024];
static TaskHandle_t  s_streamer_task_handle = NULL;

/* -----------------------------------------------------------------------
 * Publish callback — fires in the publisher's task context
 *
 * Called by norb_publish() immediately after xQueueOverwrite. Must not block.
 * Sets bit (topic) in the streamer task's notification value.
 * Multiple calls for the same topic before the task wakes simply set the
 * same bit redundantly — no overflow, no missed events.
 * ----------------------------------------------------------------------- */
static void streamer_notify_cb(topic_id_t topic)
{
    if (s_streamer_task_handle != NULL) {
        xTaskNotify(s_streamer_task_handle, (1UL << (uint32_t)topic), eSetBits);
    }
}

static void norb_streamer_task(void *arg)
{
    (void)arg;

    /*
     * Static local buffers — live in BSS, not on the task stack.
     *   topic_buf : raw nORB struct bytes (up to 128 B)
     *   send_buf  : RS485 event payload = [topic_id (1)] + [timestamp_ms (4)] + [raw bytes]
     */
    static uint8_t topic_buf[NORB_STREAM_MAX_TOPIC_BYTES];
    static uint8_t send_buf[1U + 4U + NORB_STREAM_MAX_TOPIC_BYTES];

    for (;;)
    {
        uint32_t triggered = 0U;

        /*
         * Block indefinitely (0% CPU) until norb_publish fires a callback
         * that sets one or more bits via xTaskNotify.
         *
         * ulBitsToClearOnEntry  = 0         — don't clear bits on entry
         * ulBitsToClearOnExit   = 0xFFFFFFFF — clear all bits on exit (consume)
         */
        xTaskNotifyWait(0U, 0xFFFFFFFFU, &triggered, portMAX_DELAY);

        uint32_t now_ms = xTaskGetTickCount() * (1000U / configTICK_RATE_HZ);

        for (int i = 0; i < (int)TOPIC_COUNT; i++)
        {
            if (!(triggered & (1UL << (uint32_t)i))) {
                continue; /* this topic was not the one published */
            }

            norb_sub_entry_t *sub = &g_subscriptions[i];

            if (!sub->enabled || sub->rate_ms == 0U) {
                continue;
            }

            /* Rate limiter: honour the throttle cap */
            if ((now_ms - sub->last_sent_ms) < (uint32_t)sub->rate_ms) {
                continue;
            }

            /* Read latest value — skip if never published (shouldn't happen
             * here since the callback fires after publish, but be defensive) */
            if (!norb_subscribe_poll((topic_id_t)i, topic_buf)) {
                continue;
            }

            uint8_t sz = norb_topic_sizes[i];
            sub->last_sent_ms = now_ms;

            /* Build event packet:
             *   [0]    topic_id (uint8_t)
             *   [1..4] timestamp_ms (uint32_t LE)
             *   [5..]  raw struct bytes */
            send_buf[0] = (uint8_t)i;
            memcpy(&send_buf[1], &now_ms, sizeof(now_ms));
            memcpy(&send_buf[5], topic_buf, sz);

            datalink_send(
                ADDR_GSE,
                MSG_TYPE_EVENT,
                EVENT_OBC_NORB_STREAM,
                send_buf,
                (uint8_t)(5U + sz)
            );
        }
    }
}

/* -----------------------------------------------------------------------
 * Public API
 * ----------------------------------------------------------------------- */

void norb_streamer_init(void)
{
    memset(g_subscriptions, 0, sizeof(g_subscriptions));

    s_streamer_task_handle = xTaskCreateStatic(
        norb_streamer_task,
        "norb_stream",
        1024U,
        NULL,
        3U,   /* priority — below datalink RX (5), reasonable for telemetry */
        s_streamer_stack,
        &s_streamer_tcb
    );

    /* Register the notify callback for every topic.
     * From this point on, any norb_publish() call will set the corresponding
     * bit and unblock this task. */
    for (int i = 0; i < (int)TOPIC_COUNT; i++) {
        norb_set_publish_callback((topic_id_t)i, streamer_notify_cb);
    }
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
    sub->enabled      = true;
    sub->last_sent_ms = 0U; /* throttle clock starts at 0 → first event fires immediately */
}
