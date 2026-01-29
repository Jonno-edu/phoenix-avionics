#include "hil_sensor_task.h"
#include "task_manager.h"
#include "core/logging.h"
#include "core/rocket_data.h"
// This header contains the correct Baud/Regs/Structs
#include "hil_protocol.h" 

#include <FreeRTOS.h>
#include <task.h>
#include <stdio.h>

#if PICO_BUILD
#include "pico/stdlib.h"
#include "hardware/i2c.h"

// Hardware Pin Config (Specific to this Board/Task)
#define HIL_I2C_PORT       i2c1
#define HIL_PIN_SDA        6
#define HIL_PIN_SCL        7
#define HIL_I2C_OE_PIN     46

#endif

static const char *TAG = "HILSensorTask";

#if PICO_BUILD

// --- I2C SCANNER ---
static void scan_i2c_bus(void) {
    ESP_LOGI(TAG, "--- Starting I2C Bus Scan ---");
    int found_count = 0;
    
    // Standard I2C 7-bit address range (0x08 to 0x77)
    // We skip reserved addresses (0x00-0x07 and 0x78-0x7F)
    for (int addr = 0x08; addr < 0x78; addr++) {
        uint8_t rx_data;
        // Try to read 1 byte. If it returns > 0, a device ACKed.
        int ret = i2c_read_blocking(HIL_I2C_PORT, addr, &rx_data, 1, false);
        
        if (ret >= 0) {
            ESP_LOGI(TAG, "DEVICE FOUND AT ADDRESS: 0x%02X", addr);
            found_count++;
        }
    }
    
    if (found_count == 0) {
        ESP_LOGE(TAG, "Scan complete: NO DEVICES FOUND!");
        ESP_LOGE(TAG, "Troubleshooting: 1. Check Pull-up Resistors (SDA/SCL to 3V3)");
        ESP_LOGE(TAG, "Troubleshooting: 2. Check wiring (SDA->SDA, SCL->SCL)");
        ESP_LOGE(TAG, "Troubleshooting: 3. Verify HIL Node has power and is running");
    } else {
        ESP_LOGI(TAG, "Scan complete: Found %d device(s).", found_count);
    }
}

static bool hil_i2c_init(void) {
    // 1. Enable Output Buffer (Level Shifter)
    gpio_init(HIL_I2C_OE_PIN);
    gpio_set_dir(HIL_I2C_OE_PIN, GPIO_OUT);
    gpio_put(HIL_I2C_OE_PIN, 1);
    
    // 2. Init I2C Hardware using settings from hil_protocol.h
    uint32_t real_baud = i2c_init(HIL_I2C_PORT, HIL_I2C_BAUDRATE); 

    gpio_set_function(HIL_PIN_SDA, GPIO_FUNC_I2C);
    gpio_set_function(HIL_PIN_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(HIL_PIN_SDA);
    gpio_pull_up(HIL_PIN_SCL);
    
    // Allow electrical settling time
    sleep_ms(100); 
    
    ESP_LOGI(TAG, "I2C Init: %u Hz (Target: %d)", real_baud, HIL_I2C_BAUDRATE);

    // 3. Run Bus Scan immediately to verify connectivity
    scan_i2c_bus();
    
    // 4. Specific Ping Test for HIL Node
    uint8_t reg_ptr = HIL_REG_FAST; 
    int ret = -1;
    for (int i = 0; i < 5; i++) {
        // Just write the register address to see if it ACKs
        ret = i2c_write_blocking(HIL_I2C_PORT, HIL_SLAVE_ADDR, &reg_ptr, 1, false);
        if (ret >= 0) return true; // Success!
        vTaskDelay(pdMS_TO_TICKS(50));
    }
    
    ESP_LOGE(TAG, "Specific Ping to HIL Node (0x%02X) Failed: %d", HIL_SLAVE_ADDR, ret);
    return false;
}

static bool read_sensor(uint8_t reg, void* dest, size_t size) {
    // Write register address, NO STOP (true)
    int ret = i2c_write_blocking(HIL_I2C_PORT, HIL_SLAVE_ADDR, &reg, 1, true);
    if (ret < 0) return false;
    
    // Read data
    ret = i2c_read_blocking(HIL_I2C_PORT, HIL_SLAVE_ADDR, (uint8_t*)dest, size, false);
    return (ret > 0);
}

static void vHILSensorTask(void *pvParameters) {
    (void)pvParameters;
    // Wait for system to stabilize
    vTaskDelay(pdMS_TO_TICKS(2000));
    
    if (!hil_i2c_init()) {
        ESP_LOGE(TAG, "HIL Hardware Init Failed. Suspending Task.");
        vTaskSuspend(NULL);
        return;
    }
    
    ESP_LOGI(TAG, "HIL Link Established. Starting Scheduler.");

    uint64_t next_imu_us  = time_us_64();
    uint64_t next_baro_us = time_us_64();
    uint64_t next_gps_us  = time_us_64();

    // Structs are defined in hil_protocol.h
    packet_imu_t  imu = {0};
    packet_baro_t baro = {0};
    packet_gps_t  gps = {0};

    while (true) {
        uint64_t now = time_us_64();

        // ------------------------------------------
        // TASK 1: IMU (1000 Hz)
        // ------------------------------------------
        if (now >= next_imu_us) {
            next_imu_us += 1000;
            if (read_sensor(HIL_REG_FAST, &imu, sizeof(imu))) {
                // Update Global Rocket State
                // Currently only X-axis is populated in HIL
                rocket_data_update_accel(imu.accel_x, 0, 0); 
            }
        }

        // ------------------------------------------
        // TASK 2: Baro (100 Hz)
        // ------------------------------------------
        if (now >= next_baro_us) {
            next_baro_us += 10000;
            if (read_sensor(HIL_REG_MEDIUM, &baro, sizeof(baro))) {
                rocket_data_update_baro(baro.pressure, 0);
            }
        }

        // ------------------------------------------
        // TASK 3: GPS (10 Hz)
        // ------------------------------------------
        if (now >= next_gps_us) {
            next_gps_us += 100000;
            if (read_sensor(HIL_REG_SLOW, &gps, sizeof(gps))) {
                // Logging for verification
                ESP_LOGI(TAG, "[OBC] T:%u ms | ACC_X:%d | PRES:%d | LAT:%d",
                       imu.time_ms, imu.accel_x, baro.pressure, gps.lat);
            }
        }
    }
}
#else
// Mock task for non-Pico builds
static void vHILSensorTask(void *pvParameters) {
    (void)pvParameters;
    while(1) vTaskDelay(1000);
}
#endif

void hil_sensor_task_init(void) {
    // Increased stack size to 4096 to be safe with logging/I2C
    xTaskCreate(vHILSensorTask, "HIL_Sensor", 4096, NULL, PRIORITY_HIL_SENSORS, NULL);
}