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

    // Future
    // TOPIC_VEHICLE_STATE,
    // TOPIC_BARO,
    // TOPIC_GPS,
    // TOPIC_ACTUATOR_CMD,

    TOPIC_COUNT   // ← always last, used to size the queue array
} topic_id_t;

#endif // NORB_TOPICS_H
