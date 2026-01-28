#include "hil_sensor_task.h"
#include "task_manager.h"
#include "core/logging.h"
#include "core/rocket_data.h"
#include "hil_protocol.h"
#include <FreeRTOS.h>
#include <task.h>

#if PICO_BUILD
#include "pico/stdlib.h"
#include "hardware/i2c.h"

// I2C Hardware Configuration
#define HIL_I2C_PORT       i2c1
#define HIL_PIN_SDA        6
#define HIL_PIN_SCL        7
#define HIL_I2C_OE_PIN     46  // OBC v2.0 - adjust if using v1.0 (GPIO 15)

#endif

static const char *TAG = "HILSensorTask";

#if PICO_BUILD
/**
 * @brief Initialize I2C hardware for HIL communication
 * @return true if successful, false otherwise
 */
static bool hil_i2c_init(void) {
    // Enable I2C output buffer
    gpio_init(HIL_I2C_OE_PIN);
    gpio_set_dir(HIL_I2C_OE_PIN, GPIO_OUT);
    gpio_put(HIL_I2C_OE_PIN, 1);
    
    // Initialize I2C peripheral
    i2c_init(HIL_I2C_PORT, HIL_I2C_BAUDRATE);
    gpio_set_function(HIL_PIN_SDA, GPIO_FUNC_I2C);
    gpio_set_function(HIL_PIN_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(HIL_PIN_SDA);
    gpio_pull_up(HIL_PIN_SCL);
    
    ESP_LOGI(TAG, "I2C initialized: Port=i2c1, Baud=%u, SDA=%u, SCL=%u, OE=%u",
             HIL_I2C_BAUDRATE, HIL_PIN_SDA, HIL_PIN_SCL, HIL_I2C_OE_PIN);
    
    // Test I2C by setting pointer to register 0x00
    uint8_t reg_ptr = HIL_REG_FAST;
    int ret = i2c_write_blocking(HIL_I2C_PORT, HIL_I2C_SLAVE_ADDR, &reg_ptr, 1, false);
    if (ret < 0) {
        ESP_LOGE(TAG, "I2C test write failed: %d", ret);
        return false;
    }
    
    return true;
}
#endif

/**
 * @brief Main HIL sensor polling task
 * 
 * Runs at 1kHz base rate with multi-rate reads:
 * - Every loop (1ms): Read fast packet (IMU)
 * - Every 10 loops (10ms): Read medium packet (baro/temp)
 * - Every 100 loops (100ms): Read slow packet (GPS)
 */
static void vHILSensorTask(void *pvParameters) {
    (void)pvParameters;
    
    // Delay to allow system initialization
    vTaskDelay(pdMS_TO_TICKS(1000));
    
#if PICO_BUILD
    ESP_LOGI(TAG, "Initializing HIL I2C interface...");
    
    if (!hil_i2c_init()) {
        ESP_LOGE(TAG, "HIL I2C initialization failed! Task suspended.");
        vTaskSuspend(NULL);  // Suspend this task permanently
        return;
    }
    
    ESP_LOGI(TAG, "HIL sensor task started - polling at 1kHz base rate");
    
    // Data buffers
    fast_packet_t fast;
    medium_packet_t medium;
    slow_packet_t slow;
    
    uint8_t reg_ptr;
    uint32_t loop_counter = 0;
    
    // Reset slave pointer to fast register
    reg_ptr = HIL_REG_FAST;
    i2c_write_blocking(HIL_I2C_PORT, HIL_I2C_SLAVE_ADDR, &reg_ptr, 1, false);
    
    while (true) {
        uint64_t start_time = time_us_64();
        
        // --- FAST READ (Every 1ms - 1kHz) ---
        // Pointer assumed to be at 0x00. Direct read.
        int ret = i2c_read_blocking(HIL_I2C_PORT, HIL_I2C_SLAVE_ADDR, 
                                    (uint8_t*)&fast, sizeof(fast_packet_t), false);
        if (ret < 0) {
            ESP_LOGW(TAG, "Fast read error: %d", ret);
        } else {
            // Update rocket data with IMU values
            rocket_data_update_accel(fast.accel[0], fast.accel[1], fast.accel[2]);
            rocket_data_update_gyro(fast.gyro[0], fast.gyro[1], fast.gyro[2]);
        }
        
        // --- MEDIUM READ (Every 10ms - 100Hz) ---
        if (loop_counter % 10 == 0) {
            reg_ptr = HIL_REG_MEDIUM;
            i2c_write_blocking(HIL_I2C_PORT, HIL_I2C_SLAVE_ADDR, &reg_ptr, 1, true);  // Restart
            ret = i2c_read_blocking(HIL_I2C_PORT, HIL_I2C_SLAVE_ADDR, 
                                   (uint8_t*)&medium, sizeof(medium_packet_t), false);
            if (ret < 0) {
                ESP_LOGW(TAG, "Medium read error: %d", ret);
            } else {
                // Update rocket data with barometer values
                // Note: baro_temp in medium packet is int8_t (°C), but rocket_data expects int32_t (°C * 100)
                rocket_data_update_baro(medium.baro_pres, (int32_t)medium.baro_temp * 100);
                rocket_data_update_temp_stack(medium.temp_stack);
            }
            
            // Reset pointer to fast register
            reg_ptr = HIL_REG_FAST;
            i2c_write_blocking(HIL_I2C_PORT, HIL_I2C_SLAVE_ADDR, &reg_ptr, 1, false);
        }
        
        // --- SLOW READ (Every 100ms - 10Hz, PHASE-SHIFTED by 5ms) ---
        // Runs at ticks 5, 105, 205... to avoid collision with medium reads (0, 10, 20...)
        else if (loop_counter % 100 == 5) {
            reg_ptr = HIL_REG_SLOW;
            i2c_write_blocking(HIL_I2C_PORT, HIL_I2C_SLAVE_ADDR, &reg_ptr, 1, true);  // Restart
            ret = i2c_read_blocking(HIL_I2C_PORT, HIL_I2C_SLAVE_ADDR, 
                                   (uint8_t*)&slow, sizeof(slow_packet_t), false);
            if (ret < 0) {
                ESP_LOGW(TAG, "Slow read error: %d", ret);
            } else {
                // Update rocket data with GPS values
                // Note: HIL protocol uses mm/s for velocity, but rocket_data expects m/s * 10
                // Convert: mm/s -> m/s * 10 by dividing by 100
                rocket_data_update_gps(slow.lat, slow.lon, slow.alt, 
                                      slow.vel_n / 100, slow.vel_e / 100, slow.vel_d / 100,
                                      slow.i_tow, slow.gps_week);
            }
            
            // Reset pointer to fast register
            reg_ptr = HIL_REG_FAST;
            i2c_write_blocking(HIL_I2C_PORT, HIL_I2C_SLAVE_ADDR, &reg_ptr, 1, false);
        }
        
        // --- PERIODIC REPORTING (Every 1000ms - 1Hz) ---
        // Read from rocket_data to verify the full data pipeline
        if (loop_counter % 1000 == 0) {
            TlmTrackingBeaconPayload_t beacon;
            getRocketTrackingInfo(&beacon);
            ESP_LOGI(TAG, "RX: HIL Data | Accel:[%d,%d,%d] | Gyro:[%d,%d,%d] | Baro:%u Pa | GPS:[%d,%d,%d]",
                     beacon.accel_x, beacon.accel_y, beacon.accel_z,
                     beacon.gyro_x, beacon.gyro_y, beacon.gyro_z,
                     beacon.baro_pressure,
                     beacon.gps_lat, beacon.gps_lon, beacon.gps_alt);
        }
        
        // --- TIMING ENFORCEMENT ---
        uint64_t elapsed_us = time_us_64() - start_time;
        if (elapsed_us > 1000) {
            static uint32_t violation_count = 0;
            if (violation_count % 100 == 0) {
                ESP_LOGW(TAG, "TIMING VIOLATION: Loop took %llu us (target: 1000 us) [x%u]", 
                         elapsed_us, violation_count + 1);
            }
            violation_count++;
        } else {
            // Sleep to maintain 1kHz rate
            vTaskDelay(pdMS_TO_TICKS(1));  // 1ms delay
        }
        
        loop_counter++;
    }
#else
    // SIL mode - no HIL hardware available
    ESP_LOGI(TAG, "HIL sensor task running in SIL mode (no-op)");
    while (true) {
        vTaskDelay(pdMS_TO_TICKS(1000));  // Just sleep
    }
#endif
}

void hil_sensor_task_init(void) {
    BaseType_t result = xTaskCreate(
        vHILSensorTask,
        "HIL_Sensor",
        2048,  // Stack size - increased for I2C operations
        NULL,
        PRIORITY_HIL_SENSORS,
        NULL
    );
    
    if (result != pdPASS) {
        ESP_LOGE(TAG, "Failed to create HIL sensor task!");
    }
}
