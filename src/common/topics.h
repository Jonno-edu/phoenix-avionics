#pragma once

typedef enum {
    TOPIC_SENSOR_IMU,
    TOPIC_TRACKING_RADIO_IDENT,   // ← add this line only
    // Add TOPIC_VEHICLE_STATE, TOPIC_BARO, etc., here later
    TOPIC_COUNT
} topic_id_t;
