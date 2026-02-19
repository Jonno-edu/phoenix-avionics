#include "datalink.h"
#include "rs485_protocol.h"
#include "hal/rs485_hal.h"
#include "common/include/protocol/phoenix_node_addrs.h"
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
    rs485_hal_send(data, len);
}

void datalink_init(void) {
    rs485_hal_init();
    rs485_init(&g_rs485, rs485_tx_callback, ADDR_OBC);
    g_bus_mutex = xSemaphoreCreateMutex();
    g_initialised = true;
}

// --- Drain the ISR ring buffer into the protocol parser ---
static void datalink_drain_rx(void) {
    while (rs485_hal_bytes_available()) {
        uint8_t b = rs485_hal_read_byte();
        rs485_process_byte(&g_rs485, b);
    }
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

    // 1. Send the request
    rs485_send_packet(&g_rs485,
                      target_addr,
                      req_type,
                      req_id,
                      (uint8_t *)req_payload,
                      req_len);

    // 2. Block until matching response or timeout
    TickType_t deadline = xTaskGetTickCount() + pdMS_TO_TICKS(timeout_ms);
    datalink_status_t result = DATALINK_TIMEOUT;

    while (xTaskGetTickCount() < deadline) {
        datalink_drain_rx();

        RS485_packet_t pkt;
        if (rs485_get_packet(&g_rs485, &pkt)) {
            // Must come from who we sent to
            if (pkt.src_addr != target_addr) {
                // Unexpected source — discard and keep waiting
                continue;
            }
            // Must be the right type and ID
            if (pkt.msg_desc.type == expected_resp_type &&
                pkt.msg_desc.id   == expected_resp_id) {
                if (out_resp) *out_resp = pkt;
                result = DATALINK_OK;
                ESP_LOGI(TAG, "Response from 0x%02X: Type:%d ID:%d Len:%d", 
                         pkt.src_addr, pkt.msg_desc.type, pkt.msg_desc.id, pkt.length);
            } else {
                result = DATALINK_BAD_RESPONSE;
                ESP_LOGW(TAG, "Unexpected response from 0x%02X: Type:%d ID:%d", 
                         pkt.src_addr, pkt.msg_desc.type, pkt.msg_desc.id);
            }
            break;
        }

        vTaskDelay(pdMS_TO_TICKS(1)); // yield — 1 ms resolution
    }

    xSemaphoreGive(g_bus_mutex); // release bus
    return result;
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
