/**
 * @file norb.c
 * @brief NORB implementation — Named Object Request Broker
 *
 * Each topic is backed by a FreeRTOS depth-1 queue.
 *
 * Why depth-1?
 *   A depth-1 queue + xQueueOverwrite() gives "mailbox" semantics:
 *   there is always exactly one value (the latest), and publishing
 *   never blocks regardless of whether anyone has read the old value.
 *   This is correct for sensor/telemetry data where staleness matters
 *   more than history — we never want an old IMU sample queued behind
 *   a new one.
 *
 * Why xQueuePeek instead of xQueueReceive?
 *   Peek reads without removing the item. This means:
 *     - Multiple tasks can subscribe to the same topic simultaneously
 *     - The value persists until a new publish overwrites it
 *     - A slow subscriber never loses data by another subscriber consuming it
 */

#include "norb/norb.h"
#include <FreeRTOS.h>
#include <queue.h>

// Pull in all payload struct definitions
#include "norb/topic_defs/sensor_imu.h"
#include "phoenix_icd.h"

static QueueHandle_t topic_queues[TOPIC_COUNT];

void norb_init(void) {
    // ── Sensor topics ──────────────────────────────────────────────────────
    topic_queues[TOPIC_SENSOR_IMU] =
        xQueueCreate(1, sizeof(sensor_imu_t));

    // ── Node health topics ─────────────────────────────────────────────────
    // TlmIdentificationPayload_t is shared — every node uses the same struct.
    // node_type field distinguishes which node responded.
    topic_queues[TOPIC_EPS_IDENT] =
        xQueueCreate(1, sizeof(TlmIdentificationPayload_t));
    topic_queues[TOPIC_TRACKING_RADIO_IDENT] =
        xQueueCreate(1, sizeof(TlmIdentificationPayload_t));

    // ── EPS power topics ───────────────────────────────────────────────────
    topic_queues[TOPIC_EPS_POWER_STATUS] =
        xQueueCreate(1, sizeof(EpsPowerStatus_t));
    topic_queues[TOPIC_EPS_MEASUREMENTS] =
        xQueueCreate(1, sizeof(EpsMeasurements_t));

    // ── Future topics — uncomment as modules are added ─────────────────────
    // topic_queues[TOPIC_VEHICLE_STATE] = xQueueCreate(1, sizeof(VehicleState_t));
    // topic_queues[TOPIC_BARO]          = xQueueCreate(1, sizeof(sensor_baro_t));
    // topic_queues[TOPIC_GPS]           = xQueueCreate(1, sizeof(sensor_gps_t));
    // topic_queues[TOPIC_ACTUATOR_CMD]  = xQueueCreate(1, sizeof(ActuatorCmd_t));
}

void norb_publish(topic_id_t topic, const void *data) {
    if (topic >= TOPIC_COUNT)       return;
    if (topic_queues[topic] == NULL) return;

    // xQueueOverwrite: always succeeds, never blocks.
    // If the queue already holds a value, it is replaced immediately.
    xQueueOverwrite(topic_queues[topic], data);
}

bool norb_subscribe_poll(topic_id_t topic, void *data_out) {
    if (topic >= TOPIC_COUNT)       return false;
    if (topic_queues[topic] == NULL) return false;

    // Timeout = 0: returns immediately.
    // pdTRUE  → value copied into data_out, queue item NOT removed.
    // pdFALSE → nothing published yet on this topic.
    return (xQueuePeek(topic_queues[topic], data_out, 0) == pdTRUE);
}

bool norb_subscribe(topic_id_t topic, void *data_out, uint32_t timeout_ms) {
    if (topic >= TOPIC_COUNT)       return false;
    if (topic_queues[topic] == NULL) return false;

    // xQueuePeek with non-zero timeout: calling task enters Blocked state,
    // consuming zero CPU until either:
    //   a) norb_publish() writes a value to this topic (task unblocked), OR
    //   b) timeout_ms elapses (returns false).
    // Queue item is NOT removed — other subscribers still see the same value.
    return (xQueuePeek(topic_queues[topic], data_out,
                       pdMS_TO_TICKS(timeout_ms)) == pdTRUE);
}
