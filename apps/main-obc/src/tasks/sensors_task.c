// sensors_task.c

#include "sensors_task.h"
#include "task_manager.h"
#include "core/logging.h"
#include "core/rocket_data.h"
#include <FreeRTOS.h>
#include <task.h>

#if PICO_BUILD
#include "tempTMP117.h"
#include "i2c.h"
#include "pico/stdlib.h"

#define LED_2_PIN 2
#endif

static const char *TAG = "SensorsTask";

static void vSensorsTask(void *pvParameters) {
    (void)pvParameters;

    // Delay 2 seconds to allow user to connect serial monitor before init logs/scan
    vTaskDelay(pdMS_TO_TICKS(2000));

#if PICO_BUILD
    ESP_LOGI(TAG, "Initializing I2C...");
    if (!I2C_init()) {
        ESP_LOGE(TAG, "I2C initialization failed!");
    } else {
        ESP_LOGI(TAG, "I2C initialized.");
    }

    // Initialize LED pin on OBC
    gpio_init(LED_2_PIN);
    gpio_set_dir(LED_2_PIN, GPIO_OUT);
    gpio_put(LED_2_PIN, 1); // Turn on LED to indicate startup

    ESP_LOGI(TAG, "Initializing TMP117 sensor...");
    // Simple retry mechanism for initialization
    while (!TEMP_TMP117_init()) {
        ESP_LOGE(TAG, "Failed to initialize TMP117 sensor! Retrying in 5s...");
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
    ESP_LOGI(TAG, "TMP117 initialized successfully.");

#else
    ESP_LOGI(TAG, "Sensors task running in simulation mode.");
#endif

    while (true) {
#if PICO_BUILD
        double temperature = 0.0;
        if (TEMP_TMP117_readTempC(&temperature)) {
             ESP_LOGD(TAG, "Temperature: %.2f deg C", temperature);
            // Update rocket data (SI Units: C)
            rocket_data_update_temp_stack((float)temperature);
        } else {
            ESP_LOGW(TAG, "Failed to read temperature from TMP117.");
        }
        
        // Blink LED to indicate activity
        gpio_put(LED_2_PIN, 0); // Turn off LED
        vTaskDelay(pdMS_TO_TICKS(10));
        gpio_put(LED_2_PIN, 1); // Turn on LED
        vTaskDelay(pdMS_TO_TICKS(10));
#else
        // Simulate sensor data
        rocket_data_update_temp_stack(25.5f);
#endif
        // Poll sensors at 10Hz? Or 1Hz? The temp task was 1Hz. 
        // 10Hz is probably better for a general "sensors" task if we add IMU later, but for now temp is slow.
        // Let's stick to 1Hz for now to match previous behavior purely for temperature. 
        // If we merge IMU later we will increase this rate.
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void sensors_task_init(void) {
    // Assuming PRIORITY_TEMP_MONITOR or similar appropriate priority. 
    // We can reuse the priority macro or define a new one. 
    // I'll check task_manager.h for priorities.
    // For now using PRIORITY_TEMP_MONITOR as it is conceptually similar.
    xTaskCreate(
        vSensorsTask,
        "Sensors",
        2048, // Increased stack size slightly for potential future sensors
        NULL,
        PRIORITY_SENSORS, 
        NULL
    );
}



/*

HIGH PERFORMANCE SENSOR TASK

// sensor_task.c

void SensorTask(void *pvParameters) {
    // ... setup code ...
    uint32_t loop_counter = 0;

    for (;;) {
        // 1. Wait for 1kHz Timer
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        // 2. Read IMU (SPI - Blocking - Fast!)
        // Takes ~12us. Safe to do in high priority task.
        bsp_spi_read_imu(&ax, &ay, &az, &gx, &gy, &gz);
        rocket_data_update_accel(ax, ay, az);
        rocket_data_update_gyro(gx, gy, gz);

        // 3. Read Barometer (I2C - Blocking)
        // Run this only every 20th loop (50Hz)
        if (loop_counter % 20 == 0) {
            // Takes ~300us. Still safe, just slightly longer.
            bsp_i2c_read_baro(&press, &temp);
            rocket_data_update_baro(press, temp);
        }

        loop_counter++;
    }
}



*/