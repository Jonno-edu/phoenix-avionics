#ifndef NORB_TOPICS_H
#define NORB_TOPICS_H

typedef enum {
    // Sensor raw data
    TOPIC_SENSOR_IMU,

    // Node health (populated by housekeeping)
    TOPIC_EPS_IDENT,
    TOPIC_TRACKING_RADIO_IDENT,

    // EPS power (populated by housekeeping)
    TOPIC_EPS_POWER_STATUS,
    TOPIC_EPS_MEASUREMENTS,

    // OBC Configuration
    TOPIC_OBC_LOG_LEVEL,
    TOPIC_OBC_IDENT,

    TOPIC_COUNT   // ← always last, used to size the queue array
} topic_id_t;

#endif // NORB_TOPICS_H
