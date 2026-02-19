#pragma once
#include <stdint.h>

typedef struct {
    uint64_t timestamp_us;
    float accel_ms2[3];
    float gyro_rads[3];
    float temperature_c;
} sensor_imu_t;
