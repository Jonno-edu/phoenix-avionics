# sensor_imu.msg
# Raw IMU sensor data published by the IMU driver task.
# Consumers: estimator/ekf, housekeeping (temperature monitoring)

uint64_t  timestamp_us      # microseconds since boot
float[3]  accel_ms2         # accelerometer X/Y/Z (m/s^2)
float[3]  gyro_rads         # gyroscope X/Y/Z (rad/s)
float     temperature_c     # on-die temperature (Celsius)
