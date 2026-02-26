# vehicle_state.msg
# EKF-estimated vehicle state. Published by the estimator module.
# Consumers: commander, datalink (telemetry downlink)

uint64_t  timestamp_us      # microseconds since boot
float[4]  quaternion        # orientation quaternion [w, x, y, z]
float[3]  velocity_ms       # velocity in NED frame (m/s)
float[3]  position_m        # position in NED frame (m)
float[3]  accel_bias_ms2    # accelerometer bias estimate (m/s^2)
float[3]  gyro_bias_rads    # gyroscope bias estimate (rad/s)
float     altitude_m        # altitude above launch site (m)
