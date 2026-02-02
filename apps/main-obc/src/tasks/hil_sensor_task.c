// hil_sensor_task.c

#include "hil_sensor_task.h"
#include "task_manager.h"
#include "core/logging.h"
#include "core/rocket_data.h"
#include "hil_protocol.h" 
#include "sensor_config.h" 

#include <FreeRTOS.h>
#include <task.h>
#include <stdio.h>

#if PICO_BUILD
#include "pico/stdlib.h"
#include "hardware/i2c.h"

// --- HARDWARE CONFIGURATION ---
// Ensure these match your OBC schematic
#define HIL_I2C_PORT       i2c1
#define HIL_PIN_SDA        6
#define HIL_PIN_SCL        7
#define HIL_I2C_OE_PIN     46 

#define ENABLE_HIL_SENSORS      1

// FORCE CORRECT ADDRESS HERE IF HEADER IS WRONG
#undef HIL_SLAVE_ADDR
#define HIL_SLAVE_ADDR     0x55 

#endif

static const char *TAG = "HILSensorTask";

#if PICO_BUILD

// --- I2C SCANNER ---
static void scan_i2c_bus(void) {
    ESP_LOGI(TAG, "--- Starting I2C Bus Scan ---");
    int found_count = 0;
    
    // Scan standard 7-bit addresses
    for (int addr = 0x08; addr < 0x78; addr++) {
        uint8_t rx_data;
        int ret = i2c_read_blocking(HIL_I2C_PORT, addr, &rx_data, 1, false);
        
        if (ret >= 0) {
            ESP_LOGI(TAG, "DEVICE FOUND AT ADDRESS: 0x%02X", addr);
            found_count++;
        }
    }
    
    if (found_count == 0) {
        ESP_LOGE(TAG, "Scan complete: NO DEVICES FOUND!");
    } else {
        ESP_LOGI(TAG, "Scan complete: Found %d device(s).", found_count);
    }
}

static bool hil_i2c_init(void) {
    // 1. Enable Level Shifter (Critical)
    gpio_init(HIL_I2C_OE_PIN);
    gpio_set_dir(HIL_I2C_OE_PIN, GPIO_OUT);
    gpio_put(HIL_I2C_OE_PIN, 1);
    
    // 2. Init I2C Hardware (400kHz)
    uint32_t real_baud = i2c_init(HIL_I2C_PORT, 400000); 

    gpio_set_function(HIL_PIN_SDA, GPIO_FUNC_I2C);
    gpio_set_function(HIL_PIN_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(HIL_PIN_SDA);
    gpio_pull_up(HIL_PIN_SCL);
    
    sleep_ms(100); 
    
    ESP_LOGI(TAG, "I2C Init: %lu Hz (Target: 400000)", real_baud);

    // 3. Scan Bus (Should see 0x55)
    scan_i2c_bus();
    
    // 4. Targeted Ping to 0x55
    uint8_t reg_ptr = HIL_REG_WHOAMI; 
    int ret = i2c_write_blocking(HIL_I2C_PORT, HIL_SLAVE_ADDR, &reg_ptr, 1, false);
    
    if (ret < 0) {
        ESP_LOGE(TAG, "Ping to HIL Node (0x%02X) Failed: %d", HIL_SLAVE_ADDR, ret);
        return false;
    }
    return true;
}

static bool read_sensor(uint8_t reg, void* dest, size_t size) {
    // 1. Write Register Address (No Stop)
    int ret = i2c_write_blocking(HIL_I2C_PORT, HIL_SLAVE_ADDR, &reg, 1, true);
    if (ret < 0) return false;
    
    // 2. Read Payload
    ret = i2c_read_blocking(HIL_I2C_PORT, HIL_SLAVE_ADDR, (uint8_t*)dest, size, false);
    return (ret > 0);
}

static void vHILSensorTask(void *pvParameters) {
    (void)pvParameters;
    vTaskDelay(pdMS_TO_TICKS(2000)); // Wait for boot
    
    if (!hil_i2c_init()) {
        ESP_LOGE(TAG, "HIL Hardware Init Failed. Suspending Task.");
        vTaskSuspend(NULL);
        return;
    }
    
    ESP_LOGI(TAG, "HIL Link Established (Addr: 0x%02X). Starting Scheduler.", HIL_SLAVE_ADDR);

    // Timing Logic (Microseconds)
    uint64_t next_imu_us  = time_us_64();
    uint64_t next_baro_us = time_us_64() + 2000;
    uint64_t next_gps_us  = time_us_64() + 4000;
    uint64_t next_mag_us  = time_us_64() + 6000;
    uint64_t next_temp_us = time_us_64() + 8000;

    // Data Structures
    packet_imu_t  imu = {0};
    packet_baro_t baro = {0};
    packet_gps_t  gps = {0};
    packet_mag_t  mag = {0};
    packet_temp_t temp = {0};

    // Stats
    uint32_t valid_imu_count = 0;
    uint32_t last_log_ms = 0;

    while (true) {
        uint64_t now = time_us_64();

        // ------------------------------------------
        // TASK 1: IMU (1000 Hz)
        // ------------------------------------------
        if (now >= next_imu_us) {
            next_imu_us += 1000;
            if (read_sensor(HIL_REG_IMU, &imu, sizeof(imu))) {
                // Pass raw LSBs to rocket_data. Getters handle SI units.
                rocket_data_update_accel(
                    imu.accel[0] * SENSOR_IMU_ACCEL_SENSITIVITY, 
                    imu.accel[1] * SENSOR_IMU_ACCEL_SENSITIVITY, 
                    imu.accel[2] * SENSOR_IMU_ACCEL_SENSITIVITY
                );
                rocket_data_update_gyro(
                    imu.gyro[0] * SENSOR_IMU_GYRO_SENSITIVITY, 
                    imu.gyro[1] * SENSOR_IMU_GYRO_SENSITIVITY, 
                    imu.gyro[2] * SENSOR_IMU_GYRO_SENSITIVITY
                );
                valid_imu_count++;
            }
        }

        // ------------------------------------------
        // TASK 2: Baro (50 Hz) - HIL_REG_BARO1
        // ------------------------------------------
        if (now >= next_baro_us) {
            next_baro_us += 20000; // 20ms
            if (read_sensor(HIL_REG_BARO1, &baro, sizeof(baro))) {
                // Baro pressure is sent as raw units from HIL.
                // Convert to SI (Pa) using the sensitivity factor.
                rocket_data_update_baro(
                    baro.pressure_pa * SENSOR_BMP581_SENSITIVITY, 
                    baro.temp_c * SENSOR_BMP581_TEMP_SENSITIVITY
                );
            }
        }

        // ------------------------------------------
        // TASK 3: GPS (10 Hz)
        // ------------------------------------------
        if (now >= next_gps_us) {
            next_gps_us += 100000; // 100ms
            if (read_sensor(HIL_REG_GPS, &gps, sizeof(gps))) {
                // Struct uses 'int32_t' for vel_n/e/d
                rocket_data_update_gps(
                    gps.lat * (double)SENSOR_GPS_LAT_LON_SENSITIVITY,
                    gps.lon * (double)SENSOR_GPS_LAT_LON_SENSITIVITY,
                    gps.alt * SENSOR_GPS_ALTITUDE_SENSITIVITY,
                    gps.vel_n * SENSOR_GPS_VELOCITY_SENSITIVITY,
                    gps.vel_e * SENSOR_GPS_VELOCITY_SENSITIVITY,
                    gps.vel_d * SENSOR_GPS_VELOCITY_SENSITIVITY,
                    0, 0 // iTOW/Week not in packet
                );
            }
        }

        // ------------------------------------------
        // TASK 4: Magnetometer (100 Hz)
        // ------------------------------------------
        if (now >= next_mag_us) {
            next_mag_us += 10000; // 10ms
            if (read_sensor(HIL_REG_MAG, &mag, sizeof(mag))) {
                // Struct uses array mag[3]
                rocket_data_update_mag(
                    mag.mag[0] * SENSOR_MAG_SENSITIVITY, 
                    mag.mag[1] * SENSOR_MAG_SENSITIVITY, 
                    mag.mag[2] * SENSOR_MAG_SENSITIVITY
                );
            }
        }

        // ------------------------------------------
        // TASK 5: Temperature (1 Hz)
        // ------------------------------------------
        if (now >= next_temp_us) {
            next_temp_us += 1000000; // 1s
            if (read_sensor(HIL_REG_TEMP, &temp, sizeof(temp))) {
                // HIL sends raw LSB (C * 128), which is what rocket_data expects
                rocket_data_update_temp_stack(temp.temp_c * SENSOR_TEMP_SENSITIVITY); 
            }
        }

        // ------------------------------------------
        // LOGGING (1 Hz) - Scaled via RocketData Getters
        // ------------------------------------------
        if (to_ms_since_boot(get_absolute_time()) - last_log_ms > 1000) {
            ESP_LOGI(TAG, "--- HIL DEBUG (RocketData Getters) ---");
            ESP_LOGI(TAG, "IMU  | Accel: [%.2f, %.2f, %.2f] m/s^2 | Gyro: [%.3f, %.3f, %.3f] rad/s",
                     rocket_data_get_accel_x_si(), rocket_data_get_accel_y_si(), rocket_data_get_accel_z_si(),
                     rocket_data_get_gyro_x_si(), rocket_data_get_gyro_y_si(), rocket_data_get_gyro_z_si());
            
            ESP_LOGI(TAG, "BARO | Pres: %.0f Pa | Temp: %.2f C", 
                     rocket_data_get_baro_press_si(), rocket_data_get_baro_temp_si());

            ESP_LOGI(TAG, "GPS  | Pos: [%.6f, %.6f] | Alt: %.2f m | Vel: [%.1f, %.1f, %.1f] m/s",
                     rocket_data_get_gps_lat_si(), rocket_data_get_gps_lon_si(), rocket_data_get_gps_alt_si(),
                     rocket_data_get_gps_vel_n_si(), rocket_data_get_gps_vel_e_si(), rocket_data_get_gps_vel_d_si());

            ESP_LOGI(TAG, "MAG  | [%.2f, %.2f, %.2f] uT | STACK: %.2f C",
                     rocket_data_get_mag_x_si(), rocket_data_get_mag_y_si(), rocket_data_get_mag_z_si(),
                     rocket_data_get_temp_stack_si());

            valid_imu_count = 0;
            last_log_ms = to_ms_since_boot(get_absolute_time());
        }
    }
}
#else
// Mock for non-Pico
static void vHILSensorTask(void *pvParameters) {
    (void)pvParameters;
    while(1) vTaskDelay(1000);
}
#endif

#if ENABLE_HIL_SENSORS

void hil_sensor_task_init(void) {
    xTaskCreate(vHILSensorTask, "HIL_Sensor", 4096, NULL, PRIORITY_HIL_SENSORS, NULL);
}

#else

void hil_sensor_task_init(void) {
    ESP_LOGW(TAG, "HIL Sensor Task Disabled at Compile Time.");
}

#endif