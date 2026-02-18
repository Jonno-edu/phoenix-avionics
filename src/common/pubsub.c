#include "pubsub.h"
#include <FreeRTOS.h>
#include <queue.h>
#include "topic_defs/sensor_imu.h"

static QueueHandle_t topic_queues[TOPIC_COUNT];

void pubsub_init(void) {
    // Initialize the depth-1 queue for the IMU
    topic_queues[TOPIC_SENSOR_IMU] = xQueueCreate(1, sizeof(sensor_imu_t));
}

void publish(topic_id_t topic, const void *data) {
    if (topic_queues[topic] != NULL) {
        xQueueOverwrite(topic_queues[topic], data);
    }
}

bool subscribe_poll(topic_id_t topic, void *data_out) {
    if (topic_queues[topic] != NULL) {
        return (xQueuePeek(topic_queues[topic], data_out, 0) == pdTRUE);
    }
    return false;
}
