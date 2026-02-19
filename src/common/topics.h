#pragma once

typedef enum {
    TOPIC_SENSOR_IMU,
    TOPIC_TRACKING_RADIO_IDENT,
    TOPIC_EPS_IDENT,
    TOPIC_EPS_POWER_STATUS,       // which lines are on/off
    TOPIC_EPS_MEASUREMENTS,       // voltage, current
    TOPIC_COUNT
} topic_id_t;
