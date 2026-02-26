# barometer.msg
# Raw barometric pressure sensor data.
# Consumers: estimator/ekf (altitude fusion), housekeeping

uint64_t  timestamp_us      # microseconds since boot
float     pressure_pa       # barometric pressure (Pa)
float     temperature_c     # sensor temperature (Celsius)
float     altitude_m        # pressure-derived altitude above sea level (m)
