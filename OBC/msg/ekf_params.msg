# ekf_params.msg
# Live-tunable EKF parameter block.
# Published by the ground-tuning or config module; consumed by the estimator.
# On startup the EKF uses hard-coded defaults (_load_default_params) until
# a publisher populates this topic.

# Physical Configuration
float[3] imu_lever_arm      # Offset of IMU from CG [x, y, z] in metres
float gravity_ms2           # Local gravity magnitude (e.g. 9.80665 m/s^2)

# Process Noise Variances
float[3] accel_noise_var    # Per-axis accelerometer noise variance
float gyro_noise_var        # Gyroscope noise variance

# Sensor Noise Variances
float[3] gps_pos_var        # GPS position noise variance (m^2) per axis
float[3] gps_vel_var        # GPS velocity noise variance (m/s)^2 per axis
float baro_noise_var        # Barometer altitude noise variance (m^2)
float mag_noise_var_pad     # Magnetometer noise variance on the pad (Gauss^2)
float mag_noise_var_flight  # Magnetometer noise variance during flight (Gauss^2)

# Dynamic Chi-Squared Gates (sigma multiples)
float gps_pos_gate          # GPS position gate (e.g. 3.0 = 3-sigma)
float gps_vel_gate          # GPS velocity gate
float baro_gate             # Barometer gate
float mag_magnitude_gate    # Magnetometer magnitude gate

# Recovery Timeouts
uint32_t gps_timeout_samples  # Consecutive rejected samples before GPS hard reset
