#include "core/rs485_monitor.h"
#include "telemetry_defs.h"
#include <stdio.h>
#include <string.h>

// ============================================================================
// CONSTANTS
// ============================================================================
#define ESC_BYTE    0x1F
#define SOM_BYTE    0x7F
#define EOM_BYTE    0xFF

// ============================================================================
// STATE
// ============================================================================
static Rs485MonitorMode_t monitor_mode = RS485_MON_OFF;

// ============================================================================
// HELPER: Get device name from address
// ============================================================================
static const char* get_device_name(uint8_t addr) {
    switch (addr) {
        case 0x01: return "OBC";
        case 0x02: return "EPS";
        case 0x03: return "RADIO";
        case 0xF0: return "GSE";
        case 0xFF: return "BCAST";
        default:   return "???";
    }
}

// ============================================================================
// HELPER: Get message type name and color
// ============================================================================
static const char* get_msg_type_name(uint8_t type) {
    switch (type) {
        case 1: return "EVT";
        case 2: return "TC";
        case 3: return "TC_ACK";
        case 4: return "TLM_REQ";
        case 5: return "TLM_RESP";
        case 7: return "BULK";
        default: return "???";
    }
}

static const char* get_msg_type_color(uint8_t type) {
    switch (type) {
        case 1: return MON_COLOR_EVT;
        case 2: return MON_COLOR_TC;
        case 3: return MON_COLOR_TC_ACK;
        case 4: return MON_COLOR_TLM_REQ;
        case 5: return MON_COLOR_TLM_RESP;
        case 7: return MON_COLOR_BULK;
        default: return MON_COLOR_ERROR;
    }
}

// ============================================================================
// HELPER: Get telemetry/command ID name
// ============================================================================
static const char* get_tlm_id_name(uint8_t id) {
    switch (id) {
        case 0: return "IDENT";
        case 1: return "EPS_PWR";
        case 2: return "EPS_MEAS";
        case 3: return "TRK_BEACON";
        case 4: return "STATUS";
        default: return "???";
    }
}

static const char* get_tc_id_name(uint8_t id) {
    switch (id) {
        case 0: return "RESET";
        case 1: return "EPS_PWR";
        case 5: return "TELEM_RATE";
        case 6: return "LOG_LVL";
        case 8: return "SIM_STATE";
        default: return "???";
    }
}

// ============================================================================
// INITIALIZATION
// ============================================================================
void rs485_monitor_init(void) {
    monitor_mode = RS485_MON_OFF;
}

// ============================================================================
// MODE CONTROL
// ============================================================================
void rs485_monitor_set_mode(Rs485MonitorMode_t mode) {
    monitor_mode = mode;
    
    const char *mode_names[] = {"OFF", "RAW", "DECODED", "VERBOSE"};
    printf("[MON] RS485 Monitor mode: %s\n", mode_names[mode]);
}

Rs485MonitorMode_t rs485_monitor_get_mode(void) {
    return monitor_mode;
}

bool rs485_monitor_is_enabled(void) {
    return (monitor_mode != RS485_MON_OFF);
}

// ============================================================================
// RAW HEX OUTPUT
// ============================================================================
static void print_raw_hex(const uint8_t *data, uint16_t len, const char *color) {
    printf("%s", color);
    for (uint16_t i = 0; i < len; i++) {
        printf("%02X ", data[i]);
    }
    printf("%s", MON_COLOR_RESET);
}

// ============================================================================
// DECODED PACKET OUTPUT
// ============================================================================
static void print_decoded_packet(const uint8_t *data, uint16_t len, bool is_tx) {
    // Minimum valid packet: ESC SOM SIZE DEST SRC DESC CRC16 ESC EOM = 9 bytes
    if (len < 9) {
        printf("%s[INVALID - too short: %d bytes]%s\n", MON_COLOR_ERROR, len, MON_COLOR_RESET);
        return;
    }
    
    // Check framing
    if (data[0] != ESC_BYTE || data[1] != SOM_BYTE) {
        printf("%s[INVALID - bad SOM]%s\n", MON_COLOR_ERROR, MON_COLOR_RESET);
        return;
    }
    
    if (data[len-2] != ESC_BYTE || data[len-1] != EOM_BYTE) {
        printf("%s[INVALID - bad EOM]%s\n", MON_COLOR_ERROR, MON_COLOR_RESET);
        return;
    }
    
    // Extract fields
    uint8_t payload_size = data[2];
    uint8_t dest_addr = data[3];
    uint8_t src_addr = data[4];
    uint8_t descriptor = data[5];
    uint8_t msg_type = (descriptor >> 5) & 0x07;
    uint8_t msg_id = descriptor & 0x1F;
    
    // Print direction
    if (is_tx) {
        printf("%s>>> TX ", MON_COLOR_TX);
    } else {
        printf("%s<<< RX ", MON_COLOR_RX);
    }
    
    // Print addresses
    printf("%s%s(0x%02X)->%s(0x%02X) ", 
           MON_COLOR_ADDR,
           get_device_name(src_addr), src_addr,
           get_device_name(dest_addr), dest_addr);
    
    // Print message type with color
    printf("%s[%s] ", get_msg_type_color(msg_type), get_msg_type_name(msg_type));
    
    // Print ID name based on message type
    const char *id_name;
    if (msg_type == 2 || msg_type == 3) {  // TC or TC_ACK
        id_name = get_tc_id_name(msg_id);
    } else {
        id_name = get_tlm_id_name(msg_id);
    }
    printf("%sID=%d(%s) ", MON_COLOR_DESC, msg_id, id_name);
    
    // Print payload size
    printf("%sLen=%d ", MON_COLOR_DATA, payload_size);
    
    // Print payload if present
    if (payload_size > 0 && len > 9) {
        printf("Data=[");
        for (uint8_t i = 0; i < payload_size && (6 + i) < (len - 3); i++) {
            printf("%02X", data[6 + i]);
            if (i < payload_size - 1) printf(" ");
        }
        printf("]");
    }
    
    printf("%s\n", MON_COLOR_RESET);
}

// ============================================================================
// VERBOSE PACKET OUTPUT (with payload interpretation)
// ============================================================================
static void print_verbose_packet(const uint8_t *data, uint16_t len, bool is_tx) {
    // First print the decoded version
    print_decoded_packet(data, len, is_tx);
    
    // Then print additional interpretation if applicable
    if (len < 9) return;
    
    uint8_t payload_size = data[2];
    uint8_t descriptor = data[5];
    uint8_t msg_type = (descriptor >> 5) & 0x07;
    uint8_t msg_id = descriptor & 0x1F;
    
    // Interpret payload based on message type and ID
    if (payload_size > 0 && len > 9) {
        const uint8_t *payload = &data[6];
        
        // EPS Power Status (TLM_RESP, ID=1)
        if (msg_type == 5 && msg_id == 1 && payload_size >= 2) {
            printf("         Power Status: 3V3[%d,%d,%d] 5V[%d,%d,%d] 12V[%d]\n",
                   (payload[0] >> 0) & 0x03,
                   (payload[0] >> 2) & 0x03,
                   (payload[0] >> 4) & 0x03,
                   (payload[0] >> 6) & 0x03,
                   (payload[1] >> 0) & 0x03,
                   (payload[1] >> 2) & 0x03,
                   (payload[1] >> 4) & 0x03);
        }
        
        // EPS Measurements (TLM_RESP, ID=2)
        else if (msg_type == 5 && msg_id == 2 && payload_size >= 10) {
            uint16_t vbat = payload[0] | (payload[1] << 8);
            uint16_t ibat = payload[2] | (payload[3] << 8);
            uint16_t i3v3 = payload[4] | (payload[5] << 8);
            uint16_t i5v = payload[6] | (payload[7] << 8);
            uint16_t i12v = payload[8] | (payload[9] << 8);
            printf("         Measurements: Vbat=%dmV Ibat=%dmA I_3V3=%dmA I_5V=%dmA I_12V=%dmA\n",
                   vbat, ibat, i3v3, i5v, i12v);
        }
        
        // EPS Power Command (TC, ID=1)
        else if (msg_type == 2 && msg_id == 1 && payload_size >= 2) {
            printf("         Power Cmd: 3V3[%d,%d,%d] 5V[%d,%d,%d] 12V[%d]\n",
                   (payload[0] >> 0) & 0x03,
                   (payload[0] >> 2) & 0x03,
                   (payload[0] >> 4) & 0x03,
                   (payload[0] >> 6) & 0x03,
                   (payload[1] >> 0) & 0x03,
                   (payload[1] >> 2) & 0x03,
                   (payload[1] >> 4) & 0x03);
        }
        
        // Identification (TLM_RESP, ID=0)
        else if (msg_type == 5 && msg_id == 0 && payload_size >= 8) {
            printf("         Ident: Type=%d Ver=%d FW=%d.%d Uptime=%ds\n",
                   payload[0], payload[1], payload[2], payload[3],
                   payload[4] | (payload[5] << 8));
        }
    }
}

// ============================================================================
// MAIN LOGGING FUNCTIONS
// ============================================================================
void rs485_monitor_log_tx(const uint8_t *data, uint16_t len) {
    if (monitor_mode == RS485_MON_OFF) return;
    if (data == NULL || len == 0) return;
    
    switch (monitor_mode) {
        case RS485_MON_RAW:
            printf("%s>>> TX: ", MON_COLOR_TX);
            print_raw_hex(data, len, MON_COLOR_TX);
            printf("%s\n", MON_COLOR_RESET);
            break;
            
        case RS485_MON_DECODED:
            print_decoded_packet(data, len, true);
            break;
            
        case RS485_MON_VERBOSE:
            print_verbose_packet(data, len, true);
            break;
            
        default:
            break;
    }
}

void rs485_monitor_log_rx(const uint8_t *data, uint16_t len) {
    if (monitor_mode == RS485_MON_OFF) return;
    if (data == NULL || len == 0) return;
    
    switch (monitor_mode) {
        case RS485_MON_RAW:
            printf("%s<<< RX: ", MON_COLOR_RX);
            print_raw_hex(data, len, MON_COLOR_RX);
            printf("%s\n", MON_COLOR_RESET);
            break;
            
        case RS485_MON_DECODED:
            print_decoded_packet(data, len, false);
            break;
            
        case RS485_MON_VERBOSE:
            print_verbose_packet(data, len, false);
            break;
            
        default:
            break;
    }
}

void rs485_monitor_log_byte(uint8_t byte, bool is_tx) {
    if (monitor_mode == RS485_MON_OFF) return;
    
    if (is_tx) {
        printf("%s%02X %s", MON_COLOR_TX, byte, MON_COLOR_RESET);
    } else {
        printf("%s%02X %s", MON_COLOR_RX, byte, MON_COLOR_RESET);
    }
}

// ============================================================================
// HELP / LEGEND
// ============================================================================
void rs485_monitor_print_help(void) {
    printf("\n");
    printf("============== RS485 MONITOR LEGEND ==============\n");
    printf("\n");
    printf("Direction:\n");
    printf("  %s>>> TX%s  = Transmitted (OBC sending)\n", MON_COLOR_TX, MON_COLOR_RESET);
    printf("  %s<<< RX%s  = Received (OBC receiving)\n", MON_COLOR_RX, MON_COLOR_RESET);
    printf("\n");
    printf("Message Types:\n");
    printf("  %s[EVT]%s      = Event\n", MON_COLOR_EVT, MON_COLOR_RESET);
    printf("  %s[TC]%s       = Telecommand\n", MON_COLOR_TC, MON_COLOR_RESET);
    printf("  %s[TC_ACK]%s   = Telecommand Acknowledgment\n", MON_COLOR_TC_ACK, MON_COLOR_RESET);
    printf("  %s[TLM_REQ]%s  = Telemetry Request\n", MON_COLOR_TLM_REQ, MON_COLOR_RESET);
    printf("  %s[TLM_RESP]%s = Telemetry Response\n", MON_COLOR_TLM_RESP, MON_COLOR_RESET);
    printf("  %s[BULK]%s     = Bulk Transfer\n", MON_COLOR_BULK, MON_COLOR_RESET);
    printf("\n");
    printf("Devices:\n");
    printf("  0x01 = OBC    0x02 = EPS    0x03 = RADIO\n");
    printf("  0xF0 = GSE    0xFF = BROADCAST\n");
    printf("\n");
    printf("Packet Format:\n");
    printf("  [1F 7F] SIZE DEST SRC DESC [PAYLOAD...] CRC16 [1F FF]\n");
    printf("\n");
    printf("Monitor Modes:\n");
    printf("  0 = OFF      - No monitoring\n");
    printf("  1 = RAW      - Raw hex bytes only\n");
    printf("  2 = DECODED  - Decoded with labels\n");
    printf("  3 = VERBOSE  - Full decode with payload interpretation\n");
    printf("\n");
    printf("==================================================\n");
    printf("\n");
}