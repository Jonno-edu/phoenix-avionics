/**
 * @file imu.c
 * @brief Hardware IMU driver — placeholder for physical sensor integration.
 *
 * This file will contain the real SPI/I2C driver for the flight IMU once
 * hardware bring-up begins.  For software-in-the-loop testing use the
 * simulated IMU driver at src/modules/sensors/sim/sim_imu.c instead.
 *
 * When implementing:
 *   1. Initialise the IMU peripheral in imu_init() (SPI/I2C config, FIFO
 *      setup, interrupt registration).
 *   2. Replace the stub below with a real 1 kHz read loop that populates
 *      sensor_imu_t from hardware registers.
 *   3. Call imu_init() from sensors_init() after bsp_peripheral_init().
 */

/* TODO: implement hardware driver */

