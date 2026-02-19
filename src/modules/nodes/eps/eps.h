#pragma once
#include <stdbool.h>
#include <stdint.h>
#include "protocol/eps_payloads.h"

// ── Telemetry requests (called by housekeeping) ──────────────────────────────

// Request identification from EPS. Returns true + populates *out on success.
bool eps_request_ident(TlmIdentificationPayload_t *out, uint32_t timeout_ms);

// Request current power line on/off status.
bool eps_request_power_status(EpsPowerStatus_t *out, uint32_t timeout_ms);

// Request voltage + current measurements.
bool eps_request_measurements(EpsMeasurements_t *out, uint32_t timeout_ms);

// ── Telecommands (called by flight_control or power manager) ─────────────────

// Set all power lines at once.
bool eps_send_power_command(const EpsPowerSetCmd_t *cmd, uint32_t timeout_ms);

// Convenience wrappers.
bool eps_set_line(EpsLineIndex_t line, PowerSelect_t state, uint32_t timeout_ms);
bool eps_turn_on_line(EpsLineIndex_t line);
bool eps_turn_off_line(EpsLineIndex_t line);
