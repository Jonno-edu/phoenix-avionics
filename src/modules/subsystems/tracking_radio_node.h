#ifndef TRACKING_RADIO_NODE_H
#define TRACKING_RADIO_NODE_H

#include <stdint.h>
#include <stdbool.h>
#include "telemetry_defs.h"
#include "modules/communication/telemetry.h"
#include "rs485_protocol.h"

// ============================================================================
// TRACKING RADIO STATUS STRUCT
// ============================================================================

typedef struct {
    uint8_t  node_type;
    uint8_t  interface_version;
    uint8_t  firmware_major;
    uint8_t  firmware_minor;
    uint16_t uptime_seconds;
    uint16_t uptime_milliseconds;
} PACKED_STRUCT TrackingRadioStatus_t;

// ============================================================================
// FUNCTION PROTOTYPES - INITIALIZATION
// ============================================================================

/**
 * @brief Initialize Tracking Radio node storage. Call once in main()
 */
void tracking_radio_node_init(void);

// ============================================================================
// FUNCTION PROTOTYPES - REQUESTS
// ============================================================================

/**
 * @brief Send a request for health/status to the tracking radio
 */
void tracking_radio_request_health(void);

/**
 * @brief Send tracking beacon packet with retries
 * @param beacon_data Pointer to the beacon structure
 * @return true if ACK received, false if timeout/failed
 */
bool tracking_radio_send_beacon(const TrackingBeacon_t *beacon_data);

/**
 * @brief Callback for when a beacon ACK is received
 *        Should be called from the TCTLM task
 */
void tracking_radio_confirm_beacon_ack(void);

/**
 * @brief Handle a TC ACK from the Tracking Radio
 * @param pkt The packet received
 */
void tracking_radio_handle_telecommand_ack(RS485_packet_t *pkt);

/**
 * @brief Handle a Telemetry Response from the Tracking Radio
 * @param pkt The packet received
 */
void tracking_radio_handle_telemetry_response(RS485_packet_t *pkt);

// ============================================================================
// FUNCTION PROTOTYPES - STORE DATA
// ============================================================================

/**
 * @brief Store received Tracking Radio status
 * @param status pointer to received status data
 */
void tracking_radio_node_store_status(const TrackingRadioStatus_t *status);

// ============================================================================
// FUNCTION PROTOTYPES - RETRIEVE DATA
// ============================================================================

/**
 * @brief Get stored Tracking Radio status
 * @param out_status pointer to destination struct
 * @return true if valid data available, false otherwise
 */
bool tracking_radio_node_get_status(TrackingRadioStatus_t *out_status);

// ============================================================================
// FUNCTION PROTOTYPES - STATUS
// ============================================================================

/**
 * @brief Check if stored status is valid/fresh
 * @return true if valid
 */
bool tracking_radio_node_is_status_valid(void);

/**
 * @brief Get age of stored status
 * @return age in milliseconds
 */
uint32_t tracking_radio_node_get_status_age_ms(void);

#endif // TRACKING_RADIO_NODE_H
