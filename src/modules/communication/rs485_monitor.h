#ifndef RS485_MONITOR_H
#define RS485_MONITOR_H

#include <stdint.h>
#include <stdbool.h>
#include "rs485_protocol.h"

// ============================================================================
// MONITOR CONFIGURATION
// ============================================================================

// Enable/disable colors in monitor output
// Set to 0 if your terminal doesn't support ANSI colors
#define RS485_MONITOR_USE_COLORS  1

// ============================================================================
// COLOR DEFINITIONS
// ============================================================================
#if RS485_MONITOR_USE_COLORS
    #define MON_COLOR_RESET     "\033[0m"
    #define MON_COLOR_TX        "\033[33m"      // Yellow - Transmit
    #define MON_COLOR_RX        "\033[36m"      // Cyan - Receive
    #define MON_COLOR_HEADER    "\033[90m"      // Gray - Frame header/footer
    #define MON_COLOR_ADDR      "\033[35m"      // Magenta - Addresses
    #define MON_COLOR_DESC      "\033[32m"      // Green - Descriptor
    #define MON_COLOR_DATA      "\033[37m"      // White - Payload data
    #define MON_COLOR_CRC       "\033[90m"      // Gray - CRC
    #define MON_COLOR_ERROR     "\033[31m"      // Red - Errors
    
    // Message type colors
    #define MON_COLOR_EVT       "\033[34m"      // Blue - Event
    #define MON_COLOR_TC        "\033[33m"      // Yellow - Telecommand
    #define MON_COLOR_TC_ACK    "\033[32m"      // Green - TC Acknowledgment
    #define MON_COLOR_TLM_REQ   "\033[36m"      // Cyan - Telemetry Request
    #define MON_COLOR_TLM_RESP  "\033[35m"      // Magenta - Telemetry Response
    #define MON_COLOR_BULK      "\033[37m"      // White - Bulk Transfer
#else
    #define MON_COLOR_RESET     ""
    #define MON_COLOR_TX        ""
    #define MON_COLOR_RX        ""
    #define MON_COLOR_HEADER    ""
    #define MON_COLOR_ADDR      ""
    #define MON_COLOR_DESC      ""
    #define MON_COLOR_DATA      ""
    #define MON_COLOR_CRC       ""
    #define MON_COLOR_ERROR     ""
    #define MON_COLOR_EVT       ""
    #define MON_COLOR_TC        ""
    #define MON_COLOR_TC_ACK    ""
    #define MON_COLOR_TLM_REQ   ""
    #define MON_COLOR_TLM_RESP  ""
    #define MON_COLOR_BULK      ""
#endif

// ============================================================================
// MONITOR MODES
// ============================================================================
typedef enum {
    RS485_MON_OFF = 0,      // Monitor disabled
    RS485_MON_RAW,          // Raw hex bytes only
    RS485_MON_DECODED,      // Decoded with labels
    RS485_MON_VERBOSE,      // Full decode with payload interpretation
    RS485_MON_STREAM        // Direct binary stream (no formatting)
} Rs485MonitorMode_t;

// ============================================================================
// FUNCTION PROTOTYPES
// ============================================================================

/**
 * @brief Initialize the RS485 monitor.
 */
void rs485_monitor_init(void);

/**
 * @brief Set the monitor mode.
 * @param mode Monitor mode (OFF, RAW, DECODED, VERBOSE)
 */
void rs485_monitor_set_mode(Rs485MonitorMode_t mode);

/**
 * @brief Get the current monitor mode.
 * @return Current monitor mode
 */
Rs485MonitorMode_t rs485_monitor_get_mode(void);

/**
 * @brief Check if monitor is enabled.
 * @return true if monitor is active (mode != OFF)
 */
bool rs485_monitor_is_enabled(void);

/**
 * @brief Log a transmitted packet.
 * Call this when sending a packet.
 * @param data Raw packet bytes (including framing)
 * @param len Length of packet
 */
void rs485_monitor_log_tx(const uint8_t *data, uint16_t len);

/**
 * @brief Log a received packet.
 * Call this when a complete packet is received.
 * @param data Raw packet bytes (including framing)
 * @param len Length of packet
 */
void rs485_monitor_log_rx(const uint8_t *data, uint16_t len);

/**
 * @brief Log a raw byte (for byte-by-byte monitoring).
 * @param byte The byte
 * @param is_tx true if transmit, false if receive
 */
void rs485_monitor_log_byte(uint8_t byte, bool is_tx);

/**
 * @brief Print monitor help/legend.
 */
void rs485_monitor_print_help(void);

#endif // RS485_MONITOR_H