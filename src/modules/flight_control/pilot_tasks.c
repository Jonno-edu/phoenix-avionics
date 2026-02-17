#include "pilot_tasks.h"
#include "tasks/queue_manager.h"
#include "logging.h"
#include <FreeRTOS.h>
#include <task.h>

static const char *TAG = "PILOT";

// ============================================================================
// CORE 0: THE PILOT
// ============================================================================

static void vIMUTask(void *pvParameters) {
    (void)pvParameters;
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(1); // 1 kHz

    while (true) {
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
        // TODO: Drain SPI DMA buffer
        // TODO: Apply Calibration
        // TODO: Push to xEstimatorQueue
    }
}

static void vEstimatorTask(void *pvParameters) {
    (void)pvParameters;
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(5); // 200 Hz

    while (true) {
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
        // TODO: Pop from xEstimatorQueue
        // TODO: Run EKF
        // TODO: Update Internal State
    }
}

static void vFlightStateTask(void *pvParameters) {
    (void)pvParameters;
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(10); // 100 Hz

    while (true) {
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
        // TODO: Check xPilotCommandQueue
        // TODO: Check Apogee Conditions
        // TODO: Manage Pyros
    }
}

static void vTelemSnapshotTask(void *pvParameters) {
    (void)pvParameters;
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(100); // 10 Hz

    while (true) {
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
        // TODO: Read Internal State
        // TODO: rocket_data_update_* (Thread Safe)
    }
}

void pilot_tasks_init(void) {
    TaskHandle_t hIMU = NULL;
    TaskHandle_t hEst = NULL;
    TaskHandle_t hFSM = NULL;
    TaskHandle_t hTlm = NULL;

    // 1. IMU Acquisition (1 kHz)
    xTaskCreate(vIMUTask, "IMU_Acq", 1024, NULL, PRIORITY_PILOT_IMU, &hIMU);
    vTaskCoreAffinitySet(hIMU, (1 << 0));

    // 2. Estimator (200 Hz)
    xTaskCreate(vEstimatorTask, "Estimator", 4096, NULL, PRIORITY_PILOT_ESTIMATOR, &hEst);
    vTaskCoreAffinitySet(hEst, (1 << 0));

    // 3. Flight State (100 Hz)
    xTaskCreate(vFlightStateTask, "FlightState", 2048, NULL, PRIORITY_PILOT_FSM, &hFSM);
    vTaskCoreAffinitySet(hFSM, (1 << 0));

    // 4. Telemetry Snapshot (10 Hz)
    xTaskCreate(vTelemSnapshotTask, "TelemSnap", 1024, NULL, PRIORITY_PILOT_TELEM, &hTlm);
    vTaskCoreAffinitySet(hTlm, (1 << 0));
    
    ESP_LOGI(TAG, "Pilot Tasks Initialized on Core 0");
}
