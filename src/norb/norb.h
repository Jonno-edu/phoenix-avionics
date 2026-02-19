#ifndef NORB_H
#define NORB_H

/**
 * @file norb.h
 * @brief NORB — Named Object Request Broker
 *
 * NORB is the Phoenix OBC inter-module message bus. It decouples producers
 * (publishers) from consumers (subscribers) so that modules never call each
 * other directly.
 *
 * Design:
 *   - Each topic holds exactly ONE value (depth-1 queue).
 *   - Publishing always overwrites the previous value — no backpressure.
 *   - Subscribers either POLL (non-blocking) or WAIT (blocking with timeout).
 *   - Thread-safe: safe to publish from any task, safe to subscribe from any task.
 *
 * Inspired by PX4's uORB (micro Object Request Broker).
 *
 * ── Adding a new topic ────────────────────────────────────────────────────
 *   1. Add the topic name to topic_id_t in norb/topics.h
 *   2. Define the payload struct in norb/topic_defs/<your_topic>.h
 *   3. Add xQueueCreate() call in norb_init() in norb.c
 *   4. Include your topic_def header in norb.c
 * ─────────────────────────────────────────────────────────────────────────
 */

#include <stdbool.h>
#include <stdint.h>
#include "norb/topics.h"

/**
 * @brief Initialise NORB.
 *
 * Creates all topic queues. MUST be called once before any publish or
 * subscribe call — call it in main() before starting FreeRTOS tasks.
 */
void norb_init(void);

/**
 * @brief Publish a value to a topic.
 *
 * Overwrites any previously published value. Never blocks.
 * Safe to call from any task context.
 *
 * Example:
 *   sensor_imu_t imu = { .accel_x = 100, .accel_y = 0, .accel_z = 980 };
 *   norb_publish(TOPIC_SENSOR_IMU, &imu);
 *
 * @param topic  Topic ID from topic_id_t.
 * @param data   Pointer to value to publish (must match the topic's struct type).
 */
void norb_publish(topic_id_t topic, const void *data);

/**
 * @brief Non-blocking subscribe — get the latest published value immediately.
 *
 * Returns true and copies the value into data_out if ANY value has ever been
 * published to this topic. Returns false immediately if nothing has been
 * published yet.
 *
 * Does NOT consume the value — the same value persists until a new
 * norb_publish() replaces it. Multiple tasks can safely poll the same topic.
 *
 * Use when:
 *   - Your task must NOT stall waiting for data
 *   - You want the latest available snapshot
 *   - Polling at a fixed rate regardless of whether data is fresh
 *
 * Example — housekeeping checking EPS health each second:
 *   EpsMeasurements_t meas;
 *   if (norb_subscribe_poll(TOPIC_EPS_MEASUREMENTS, &meas)) {
 *       printf("Vbat = %u mV\n", meas.battery_voltage_mv);
 *   } else {
 *       // EPS hasn't responded yet this boot — skip silently
 *   }
 *
 * @param topic     Topic ID.
 * @param data_out  Buffer to copy the value into.
 * @return true if a value was available, false if never published to.
 */
bool norb_subscribe_poll(topic_id_t topic, void *data_out);

/**
 * @brief Blocking subscribe — sleep until a value is available or timeout expires.
 *
 * Puts the calling task into the FreeRTOS Blocked state (zero CPU usage) until:
 *   a) A value is published to this topic, OR
 *   b) timeout_ms milliseconds elapse.
 *
 * Does NOT consume the value — multiple tasks can all block on the same topic
 * and all wake when it is published to.
 *
 * Use when:
 *   - Your task is driven entirely by incoming data and should sleep when idle
 *   - You want to synchronise execution to a data arrival event
 *   - Avoiding a busy-wait polling loop
 *
 * Example — EKF sleeping until IMU publishes fresh data:
 *   sensor_imu_t imu;
 *   while (1) {
 *       if (norb_subscribe(TOPIC_SENSOR_IMU, &imu, 10)) {
 *           ekf_update(&imu);
 *       } else {
 *           // 10ms with no IMU data — sensor may have stalled
 *           ESP_LOGW(TAG, "IMU timeout");
 *       }
 *   }
 *
 * Example — datalink waiting before sending a beacon:
 *   vehicle_state_t state;
 *   if (norb_subscribe(TOPIC_VEHICLE_STATE, &state, 1000)) {
 *       tracking_radio_send_beacon(&state);
 *   }
 *
 * @param topic       Topic ID.
 * @param data_out    Buffer to copy the value into.
 * @param timeout_ms  Maximum time to wait in milliseconds.
 * @return true if a value arrived within the timeout, false if timed out.
 */
bool norb_subscribe(topic_id_t topic, void *data_out, uint32_t timeout_ms);

#endif // NORB_H
