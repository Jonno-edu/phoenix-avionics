#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "rs485_protocol.h"

// Function pointer type for sending a stream's data
typedef void (*stream_send_fn_t)(rs485_instance_t *inst, uint8_t target_addr);

typedef struct {
    uint32_t interval_ms;
    uint32_t last_sent_ms;
    stream_send_fn_t send_fn;
    bool enabled;
} datalink_stream_t;

// Helper to check if a stream is due to be sent
static inline bool stream_is_due(datalink_stream_t *stream, uint32_t current_time_ms) {
    if (!stream->enabled || stream->interval_ms == 0) {
        return false;
    }
    return (current_time_ms - stream->last_sent_ms) >= stream->interval_ms;
}
