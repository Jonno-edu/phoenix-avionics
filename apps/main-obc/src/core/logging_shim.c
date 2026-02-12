#include "core/logging_shim.h"
#include "../tasks/queue_manager.h"
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include "FreeRTOS.h"
#include "queue.h"
#include "rs485_protocol.h" 

void GSU_LOG(uint8_t level, const char *format, ...) {
    char buffer[200]; // Payload limit
    
    // 1. First Byte = Level
    buffer[0] = level;
    
    // 2. Format the rest
    va_list args;
    va_start(args, format);
    // Vsnprintf writes at buffer[1]
    int len = vsnprintf(&buffer[1], sizeof(buffer)-1, format, args);
    va_end(args);

    if (len < 0) return; // Encoding error
    
    // Cap length to buffer capacity
    if (len > (int)(sizeof(buffer) - 2)) { 
         len = sizeof(buffer) - 2;
    }
    
    uint8_t payload_len = 1 + len;

    // 2. Build the Packet
    RS485_packet_t pkt;
    memset(&pkt, 0, sizeof(pkt)); // Zero out
    pkt.length = payload_len;
    pkt.dest_addr = ADDR_GSE; 
    pkt.src_addr  = ADDR_OBC; 
    
    pkt.msg_desc.type = MSG_TYPE_TLM_RESP; 
    pkt.msg_desc.id   = TLM_COMMON_LOG;

    memcpy(pkt.data, buffer, payload_len);
    
    // 3. Queue it (Non-blocking!)
    if (q_usb_out) {
        // Send to back of queue, do not wait if full
        xQueueSend(q_usb_out, &pkt, 0);
    }
}
