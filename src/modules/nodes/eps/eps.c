#include "eps.h"
#include "modules/datalink/datalink.h"
#include "protocol/telemetry_defs.h"
#include "common/logging.h"
#include <string.h>

static const char *TAG = "EPS";

// ── Helpers ───────────────────────────────────────────────────────────────────

// Generic telemetry request: sends TLM_REQ, waits for TLM_RESP with same ID.
static datalink_status_t eps_request_tlm(uint8_t id,
                                          RS485_packet_t *resp,
                                          uint32_t timeout_ms)
{
    return datalink_request_response(
        ADDR_EPS,
        MSG_TYPE_TLM_REQ,  id,
        NULL, 0,
        MSG_TYPE_TLM_RESP, id,
        resp, timeout_ms
    );
}

// ── Telemetry requests ────────────────────────────────────────────────────────

bool eps_request_ident(TlmIdentificationPayload_t *out, uint32_t timeout_ms)
{
    RS485_packet_t resp;
    if (eps_request_tlm(TLM_COMMON_IDENT, &resp, timeout_ms) != DATALINK_OK)
        return false;

    // Accept partial — EPS firmware may send fewer bytes than the full struct
    memset(out, 0, sizeof(*out));
    uint8_t n = resp.length < sizeof(*out) ? resp.length : sizeof(*out);
    memcpy(out, resp.data, n);

    ESP_LOGI(TAG, "Ident: type=0x%02X fw=%u.%u uptime=%us",
             out->node_type, out->firmware_major,
             out->firmware_minor, out->uptime_seconds);
    return true;
}

bool eps_request_power_status(EpsPowerStatus_t *out, uint32_t timeout_ms)
{
    RS485_packet_t resp;
    if (eps_request_tlm(TLM_EPS_POWER, &resp, timeout_ms) != DATALINK_OK)
        return false;

    if (resp.length < sizeof(*out)) {
        ESP_LOGW(TAG, "Power status too short: %u bytes", resp.length);
        return false;
    }

    memcpy(out, resp.data, sizeof(*out));
    ESP_LOGI(TAG, "Power: 3V3[%d,%d,%d] 5V[%d,%d,%d] 12V[%d]",
             out->rail_3v3_1, out->rail_3v3_2, out->rail_3v3_3,
             out->rail_5v_1,  out->rail_5v_2,  out->rail_5v_3,
             out->rail_12v);
    return true;
}

bool eps_request_measurements(EpsMeasurements_t *out, uint32_t timeout_ms)
{
    RS485_packet_t resp;
    if (eps_request_tlm(TLM_EPS_MEASURE, &resp, timeout_ms) != DATALINK_OK)
        return false;

    if (resp.length < sizeof(*out)) {
        ESP_LOGW(TAG, "Measurements too short: %u bytes", resp.length);
        return false;
    }

    memcpy(out, resp.data, sizeof(*out));
    ESP_LOGI(TAG, "Vbat=%umV Ibat=%umA current_5v_1=%umA",
             out->batt_voltage_mv, out->batt_current_ma, out->current_5v_1_ma);
    return true;
}

// ── Telecommands ──────────────────────────────────────────────────────────────

bool eps_send_power_command(const EpsPowerSetCmd_t *cmd, uint32_t timeout_ms)
{
    if (!cmd) return false;
    datalink_status_t st = datalink_request_response(
        ADDR_EPS,
        MSG_TYPE_TELECOMMAND,    TC_EPS_POWER,
        (uint8_t *)cmd, sizeof(*cmd),
        MSG_TYPE_TELECOMMAND_ACK, TC_EPS_POWER,
        NULL, timeout_ms
    );
    if (st == DATALINK_OK)
        ESP_LOGI(TAG, "Power command ACK'd");
    else
        ESP_LOGW(TAG, "Power command %s", st == DATALINK_TIMEOUT ? "TIMEOUT" : "BAD RESP");
    return (st == DATALINK_OK);
}

bool eps_set_line(EpsLineIndex_t line, PowerSelect_t state, uint32_t timeout_ms)
{
    // Read-modify-write: get current state first so we don't clobber other lines
    EpsPowerStatus_t current;
    EpsPowerSetCmd_t cmd = {0};

    if (eps_request_power_status(&current, timeout_ms)) {
        // Copy current line states into the command
        cmd.rail_3v3_1 = current.rail_3v3_1;
        cmd.rail_3v3_2 = current.rail_3v3_2;
        cmd.rail_3v3_3 = current.rail_3v3_3;
        cmd.rail_5v_1  = current.rail_5v_1;
        cmd.rail_5v_2  = current.rail_5v_2;
        cmd.rail_5v_3  = current.rail_5v_3;
        cmd.rail_12v   = current.rail_12v;
    } else {
        ESP_LOGW(TAG, "set_line: no current status, all other lines default OFF");
    }

    // Apply the desired change
    switch (line) {
        case EPS_LINE_3V3_1: cmd.rail_3v3_1 = state; break;
        case EPS_LINE_3V3_2: cmd.rail_3v3_2 = state; break;
        case EPS_LINE_3V3_3: cmd.rail_3v3_3 = state; break;
        case EPS_LINE_5V_1:  cmd.rail_5v_1  = state; break;
        case EPS_LINE_5V_2:  cmd.rail_5v_2  = state; break;
        case EPS_LINE_5V_3:  cmd.rail_5v_3  = state; break;
        case EPS_LINE_12V:   cmd.rail_12v   = state; break;
        default:
            ESP_LOGW(TAG, "Invalid line index: %d", line);
            return false;
    }

    return eps_send_power_command(&cmd, timeout_ms);
}

bool eps_turn_on_line(EpsLineIndex_t line) {
    return eps_set_line(line, POWER_ON, 200);
}

bool eps_turn_off_line(EpsLineIndex_t line) {
    return eps_set_line(line, POWER_OFF, 200);
}
