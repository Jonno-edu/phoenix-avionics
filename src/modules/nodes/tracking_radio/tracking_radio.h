#pragma once
#include <stdbool.h>
#include <stdint.h>
#include "phoenix_icd.h"

// Request the tracking radio's identification/health status.
// Blocks until response received or timeout_ms elapses.
// Returns true and populates *out on success.
bool tracking_radio_request_ident(TlmIdentificationPayload_t *out,
                                  uint32_t timeout_ms);

// Send tracking beacon to the radio (placeholder - not implemented yet).
bool tracking_radio_send_beacon(const TlmTrackingBeaconPayload_t *beacon,
                                uint32_t timeout_ms);
