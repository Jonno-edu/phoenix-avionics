#include "core/eps_node.h"
#include "core/logging.h"
#include "rs485_protocol.h"
#include "../tasks/rs485_task.h"
#include <string.h>
#include <FreeRTOS.h>
#include <semphr.h>
#include <pico/stdlib.h>

#define GET_TIME_MS() to_ms_since_boot(get_absolute_time())

static const char *TAG = "EPS_NODE";

// ============================================================================
// STORAGE STRUCTURE
// ============================================================================

typedef struct {
    EpsPowerStatus_t power_status;
    EpsMeasurements_t measurements;

    uint32_t power_status_age_ms;
    uint32_t measurements_age_ms;

    bool power_status_valid;
    bool measurements_valid;
} EpsDataStore_t;

static EpsDataStore_t eps_store;

static SemaphoreHandle_t eps_mutex;

#define EPS_NODE_TIMEOUT_MS     5000

// ============================================================================
// INITIALIZATION
// ============================================================================

void eps_node_init(void) {
    eps_mutex = xSemaphoreCreateMutex();
    printf("eps_mutex\n");
    //Clear all stored data
    memset(&eps_store, 0, sizeof(EpsDataStore_t));
    printf("eps_store\n");
    eps_store.power_status_valid = false;
    eps_store.measurements_valid = false;

    ESP_LOGI(TAG, "EPS data storage initialized");
}   



// ============================================================================
// STORE FUNCTIONS
// ============================================================================

void eps_node_store_power_status(const EpsPowerStatus_t *status) {
    if (status == NULL) {
        ESP_LOGW(TAG, "NULL pointer passed to store_power_status");
        return;
    }

    if (xSemaphoreTake(eps_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        memcpy(&eps_store.power_status, status, sizeof(EpsPowerStatus_t));
        eps_store.power_status_age_ms = GET_TIME_MS();
        eps_store.power_status_valid = true;
        xSemaphoreGive(eps_mutex);

        ESP_LOGI(TAG, "Power status stored: 3V3[%d, %d, %d], 5V[%d, %d, %d], 12V[%d]",
                 status->line_3v3_1, status->line_3v3_2, status->line_3v3_3, 
                 status->line_5v_1, status->line_5v_2, status->line_5v_3, 
                 status->line_12v);
    } else {
        ESP_LOGW(TAG, "Failed to acquire mutex to store power status");
    }
}

void eps_node_store_measurements(const EpsMeasurements_t *measurements) {
    if (measurements == NULL) {
        ESP_LOGW(TAG, "NULL pointer passed to store_measurements");
        return;
    }

    if (xSemaphoreTake(eps_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        memcpy(&eps_store.measurements, measurements, sizeof(EpsMeasurements_t));
        eps_store.measurements_age_ms = GET_TIME_MS();
        eps_store.measurements_valid = true;
        xSemaphoreGive(eps_mutex);

        ESP_LOGI(TAG, "Measurements stored: Vbat=%dmV, Ibat=%dmA, I3V3=%dmA, I5V=%dmA, I12V=%dmA",
                 measurements->battery_voltage_mv, measurements->battery_current_ma, 
                 measurements->line_3v3_1_current_ma, measurements->line_5v_1_current_ma, 
                 measurements->line_12v_current_ma);
    } else {
        ESP_LOGW(TAG, "Failed to acquire mutex to store measurements");
    }
}

// ============================================================================
// RETRIEVE FUNCTIONS
// ============================================================================
bool eps_node_get_power_status(EpsPowerStatus_t *out_status) {
    if (out_status == NULL) return false;

    bool success = false;
    if (xSemaphoreTake(eps_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        if (eps_store.power_status_valid) {
            memcpy(out_status, &eps_store.power_status, sizeof(EpsPowerStatus_t));
            success = true;
        }
        xSemaphoreGive(eps_mutex);
    }

    return success;
}

bool eps_node_get_measurements(EpsMeasurements_t *out_measurements) {
    if (out_measurements == NULL) return false;

    bool success = false;
    if (xSemaphoreTake(eps_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        if (eps_store.measurements_valid) {
            memcpy(out_measurements, &eps_store.measurements, sizeof(EpsMeasurements_t));
            success = true;
        }
        xSemaphoreGive(eps_mutex);
    }

    return success;
}

bool eps_node_is_power_status_valid(void) {
    bool valid = false;
    if (xSemaphoreTake(eps_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        if (eps_store.power_status_valid) {
            uint32_t age = GET_TIME_MS() - eps_store.power_status_age_ms;
            if (age < EPS_NODE_TIMEOUT_MS) {
                valid = true;
            }
        }
        xSemaphoreGive(eps_mutex);
    }

    return valid;
}

bool eps_node_is_measurements_valid(void) {
    bool valid = false;
    if (xSemaphoreTake(eps_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        if (eps_store.measurements_valid) {
            uint32_t age = GET_TIME_MS() - eps_store.measurements_age_ms;
            if (age < EPS_NODE_TIMEOUT_MS) {
                valid = true;
            }
        }
        xSemaphoreGive(eps_mutex);
    }

    return valid;
}

uint32_t eps_node_get_power_status_age_ms(void) {
    uint32_t age = UINT32_MAX;
    if (xSemaphoreTake(eps_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        if (eps_store.power_status_valid) {
            age = GET_TIME_MS() - eps_store.power_status_age_ms;
        }
        xSemaphoreGive(eps_mutex);
    }

    return age;
}

uint32_t eps_node_get_measurements_age_ms(void) {
    uint32_t age = UINT32_MAX;
    if (xSemaphoreTake(eps_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        if (eps_store.measurements_valid) {
            age = GET_TIME_MS() - eps_store.measurements_age_ms;
        }
        xSemaphoreGive(eps_mutex);
    }

    return age;
}

// ============================================================================
// SEND FUNCTIONS (send requests/commands to EPS)
// ============================================================================

void eps_request_power_status(void) {
    ESP_LOGD(TAG, "Requesting EPS power status");
    rs485_send_packet(rs485_get_default_instance(), ADDR_EPS, MSG_TYPE_TLM_REQ, TLM_ID_EPS_POWER, NULL, 0);
}

void eps_request_measurements(void) {
    ESP_LOGD(TAG, "Requesting EPS measurements");
    rs485_send_packet(rs485_get_default_instance(), ADDR_EPS, MSG_TYPE_TLM_REQ, TLM_ID_EPS_MEASURE, NULL, 0);
}

void eps_send_power_command(const EpsPowerSetCmd_t *cmd) {
    if (cmd == NULL) {
        ESP_LOGW(TAG, "NULL pointer passed to send_power_command");
        return;
    }

    ESP_LOGI(TAG, "Sending EPS power command: 3V3[%d, %d, %d], 5V[%d, %d, %d], 12V[%d]",
             cmd->line_3v3_1, cmd->line_3v3_2, cmd->line_3v3_3, 
             cmd->line_5v_1, cmd->line_5v_2, cmd->line_5v_3, 
             cmd->line_12v);

    // DEBUG: Hex dump the command payload
    uint8_t *raw = (uint8_t*)cmd;
    ESP_LOGI(TAG, "CMD HEX: %02X %02X", raw[0], raw[1]);

    rs485_send_packet(rs485_get_default_instance(), ADDR_EPS, MSG_TYPE_TELECOMMAND, TC_ID_EPS_POWER, (uint8_t*)cmd, sizeof(EpsPowerSetCmd_t));
}

// ============================================================================
// CONVENIENCE HELPER FUNCTIONS
// ============================================================================

void eps_set_all_lines(PowerSelect_t state) {
    EpsPowerSetCmd_t cmd = {
        .line_3v3_1  = 1,
        .line_3v3_2  = state,
        .line_3v3_3  = state,
        .line_5v_1   = state,
        .line_5v_2   = state,
        .line_5v_3   = state,
        .line_12v    = state,
        .reserved    = 0
    };
    eps_send_power_command(&cmd);
}

void eps_set_line(EpsLineIndex_t line, PowerSelect_t state) {
    EpsPowerSetCmd_t cmd = {0};

    if (eps_node_get_power_status(&cmd)) {

    } else {
        ESP_LOGW(TAG, "No current power status, setting single line with others OFF");
    }

    switch (line) {
        case EPS_LINE_3V3_1:
            cmd.line_3v3_1 = state;
            break;
        case EPS_LINE_3V3_2:
            cmd.line_3v3_2 = state;
            break;
        case EPS_LINE_3V3_3:
            cmd.line_3v3_3 = state;
            break;
        case EPS_LINE_5V_1:
            cmd.line_5v_1 = state;
            break;
        case EPS_LINE_5V_2:
            cmd.line_5v_2 = state;
            break;
        case EPS_LINE_5V_3:
            cmd.line_5v_3 = state;
            break;
        case EPS_LINE_12V:
            cmd.line_12v = state;
            break;
        default:
            ESP_LOGW(TAG, "Invalid line index: %d", line);
            return;
    }

    cmd.reserved = 0;
    eps_send_power_command(&cmd);
}

void eps_turn_on_line(EpsLineIndex_t line) {
    eps_set_line(line, POWER_ON);
}

void eps_turn_off_line(EpsLineIndex_t line) {
    eps_set_line(line, POWER_OFF);
}

PowerSelect_t eps_get_line_state(EpsLineIndex_t line) {
    EpsPowerStatus_t status;
    if (!eps_node_get_power_status(&status)) {
        return POWER_OFF;
    }

    switch (line) {
        case EPS_LINE_3V3_1:
            return (PowerSelect_t)status.line_3v3_1;
        case EPS_LINE_3V3_2:
            return (PowerSelect_t)status.line_3v3_2;
        case EPS_LINE_3V3_3:
            return (PowerSelect_t)status.line_3v3_3;
        case EPS_LINE_5V_1:
            return (PowerSelect_t)status.line_5v_1;
        case EPS_LINE_5V_2:
            return (PowerSelect_t)status.line_5v_2;
        case EPS_LINE_5V_3:
            return (PowerSelect_t)status.line_5v_3;
        case EPS_LINE_12V:
            return (PowerSelect_t)status.line_12v;
        default:
            ESP_LOGW(TAG, "Invalid line index: %d", line);
            return POWER_OFF;
    }
}

uint16_t eps_get_battery_voltage_mv(void) {
    EpsMeasurements_t measurements;
    if (eps_node_get_measurements(&measurements)) {
        return measurements.battery_voltage_mv;
    }

    return 0;
}

uint16_t eps_get_battery_current_ma(void) {
    EpsMeasurements_t measurements;
    if (eps_node_get_measurements(&measurements)) {
        return measurements.battery_current_ma;
    }

    return 0;
}




    

// ============================================================================
// RESPONSE HANDLER
// ============================================================================

void eps_handle_telecommand_ack(RS485_packet_t *pkt) {
    if (pkt == NULL) return;
    
    uint8_t id = pkt->msg_desc.id;
    
    switch (id) {
        case TC_ID_EPS_POWER:
            ESP_LOGI(TAG, "EPS Power command acknowledged by 0x%02X", pkt->src_addr);
            break;

        case TC_ID_RESET:
            ESP_LOGI(TAG, "Reset command acknowledged by 0x%02X", pkt->src_addr);
            break;

        default:
            ESP_LOGD(TAG, "EPS TC ACK received: ID=%d from 0x%02X", id, pkt->src_addr);
            break;
    }
}

void eps_handle_telemetry_response(RS485_packet_t *pkt) {
    if (pkt == NULL) return;

    uint8_t id = pkt->msg_desc.id;

    switch (id) {
        case TLM_ID_IDENTIFICATION:
            /* Note: Identification is common, so we might want to store specific EPS info if needed 
               For now just logging or standard struct. EPS uses legacy or common struct. */
               // The original code didn't do much specific for EPS IDENT except logging.
            ESP_LOGI(TAG, "EPS Identification received from 0x%02X", pkt->src_addr);
            break;

        case TLM_ID_EPS_POWER:
            if (pkt->length >= sizeof(EpsPowerStatus_t)) {
                EpsPowerStatus_t power_status;
                memcpy(&power_status, pkt->data, sizeof(EpsPowerStatus_t));
                eps_node_store_power_status(&power_status);
            } else {
                ESP_LOGW(TAG, "EPS Power response too short: %d bytes (expected %d)", pkt->length, sizeof(EpsPowerStatus_t));
            }
            break;

        case TLM_ID_EPS_MEASURE: 
            if (pkt->length >= sizeof(EpsMeasurements_t)) {
                EpsMeasurements_t measurements;
                memcpy(&measurements, pkt->data, sizeof(EpsMeasurements_t));
                eps_node_store_measurements(&measurements);
            } else {
                ESP_LOGW(TAG, "EPS Measurements response too short: %d bytes (expected %d)", pkt->length, sizeof(EpsMeasurements_t));
            }
            break;

        default:
            ESP_LOGD(TAG, "Unknown EPS TLM Response ID: %d", id);
            break;
    }
}
