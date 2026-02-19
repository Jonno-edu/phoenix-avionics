#pragma once
#include <stdbool.h>
#include <stdint.h>
#include "protocol/phoenix_tctlm_payloads.h"

// Request EPS identification. Blocks up to timeout_ms.
bool eps_request_ident(TlmIdentificationPayload_t *out, uint32_t timeout_ms);
