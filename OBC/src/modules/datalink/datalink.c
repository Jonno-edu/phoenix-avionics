#include "datalink.h"
#include "rs485_protocol.h"
#include "tctlm.h"
#include "hal/platform_hal.h"
#include "hal/rs485_hal.h"
#include "phoenix_icd.h"
#include "logging.h"

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

static const char *TAG = "Datalink";

// --- Private: one RS485 context, invisible to all other modules ---
static rs485_instance_t g_rs485;
static SemaphoreHandle_t g_bus_mutex = NULL;
static bool g_initialised = false;

static void rs485_tx_callback(const uint8_t *data, uint16_t len) {
    platform_send_mux(data, len);
}

// --- Library Glue: OS-specific callbacks ---
static uint32_t get_time_ms_cb(void) {
    return xTaskGetTickCount() * (1000 / configTICK_RATE_HZ);
}

static void poll_cb(void) {
    while (rs485_hal_bytes_available()) {
        uint8_t b = rs485_hal_read_byte();
        rs485_process_byte(&g_rs485, b);
    }
    vTaskDelay(pdMS_TO_TICKS(1)); // yield to other tasks
}

void datalink_init(void) {
    rs485_hal_init();
    rs485_init(&g_rs485, rs485_tx_callback, ADDR_OBC);
    g_bus_mutex = xSemaphoreCreateMutex();
    g_initialised = true;
}

datalink_status_t datalink_request_response(
    uint8_t          target_addr,
    RS485_msgType_t  req_type,
    uint8_t          req_id,
    const uint8_t   *req_payload,
    uint8_t          req_len,
    RS485_msgType_t  expected_resp_type,
    uint8_t          expected_resp_id,
    RS485_packet_t  *out_resp,
    uint32_t         timeout_ms)
{
    // Acquire bus — blocks if another task is mid-transaction
    if (xSemaphoreTake(g_bus_mutex, pdMS_TO_TICKS(timeout_ms)) != pdTRUE) {
        return DATALINK_TIMEOUT;
    }

    rs485_status_t status;
    if (req_type == MSG_TYPE_TLM_REQ) {
        status = tctlm_send_telemetry_request(&g_rs485, target_addr, req_id, out_resp, timeout_ms, get_time_ms_cb, poll_cb);
    } else if (req_type == MSG_TYPE_TELECOMMAND) {
        status = tctlm_send_telecommand(&g_rs485, target_addr, req_id, (uint8_t*)req_payload, req_len, out_resp, timeout_ms, get_time_ms_cb, poll_cb);
    } else {
        // Generic fall-back if needed, but the library currently focuses on TLM and TC
        status = RS485_ERR_BAD_RESPONSE; 
    }

    xSemaphoreGive(g_bus_mutex); // release bus

    if (status == RS485_OK) {
        ESP_LOGI(TAG, "Response from 0x%02X: Type:%d ID:%d Len:%d", 
                 out_resp->src_addr, out_resp->msg_desc.type, out_resp->msg_desc.id, out_resp->length);
        return DATALINK_OK;
    } else if (status == RS485_ERR_TIMEOUT) {
        ESP_LOGW(TAG, "TIMEOUT waiting for resp from 0x%02X (type:%d id:%d)",
                 target_addr, expected_resp_type, expected_resp_id);
        return DATALINK_TIMEOUT;
    } else {
        return DATALINK_BAD_RESPONSE;
    }
}

void datalink_send(
    uint8_t          target_addr,
    RS485_msgType_t  type,
    uint8_t          id,
    const uint8_t   *payload,
    uint8_t          len)
{
    if (xSemaphoreTake(g_bus_mutex, pdMS_TO_TICKS(10)) != pdTRUE) return;
    rs485_send_packet(&g_rs485,
                      target_addr,
                      type,
                      id,
                      (uint8_t *)payload,
                      len);
    xSemaphoreGive(g_bus_mutex);
}
