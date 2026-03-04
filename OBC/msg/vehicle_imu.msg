# vehicle_imu.msg
# Integrated IMU output published by the integrator.
# Consumers: estimator/ekf

uint64_t  timestamp_us      # microseconds since boot
float[3]  delta_angle       # integrated angular rate (rad), one IMU cycle
float[3]  delta_velocity    # integrated acceleration (m/s), one IMU cycle
float     dt_s              # integration period (seconds)
