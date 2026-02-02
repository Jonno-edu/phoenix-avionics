#ifndef EPS_DATA_H
#define EPS_DATA_H

#include <stdint.h>
#include <stdbool.h>
#include "telemetry_defs.h"

// ============================================================================
// POWER LINE STATE ENUMERATION (2 bits each)
// ============================================================================

typedef enum {
    POWER_OFF   = 0, 
    POWER_ON    = 1, 
    POWER_AUTO  = 2, 
    POWER_SIM   = 3
} PowerSelect_t;

// ============================================================================
// EPS POWER STATUS TELEMETRY PAYLOAD
// ============================================================================

typedef struct {
    //Byte 0 
    uint8_t line_3v3_1  : 2;
    uint8_t line_3v3_2  : 2;
    uint8_t line_3v3_3  : 2;
    uint8_t line_5v_1   : 2;

    //Byte 1
    uint8_t line_5v_2   : 2;
    uint8_t line_5v_3   : 2;
    uint8_t line_12v    : 2;
    uint8_t reserved    : 2;
} PACKED_STRUCT EpsPowerStatus_t;


// ============================================================================
// EPS MEASUREMENT TELEMETRY PAYLOAD
// ============================================================================

typedef struct {
    uint16_t battery_voltage_mv;
    uint16_t battery_current_ma;
    uint16_t line_3v3_1_current_ma;
    uint16_t line_5v_1_current_ma;
    uint16_t line_12v_current_ma;
} PACKED_STRUCT EpsMeasurements_t;


// ============================================================================
// EPS POWER SET COMMAND PAYLOAD
// ============================================================================

typedef EpsPowerStatus_t EpsPowerSetCmd_t;


// ============================================================================
// FUNCTION PROTOTYPES - INITIALIZATION
// ============================================================================

/**
 * @brief Initialize EPS data storage. Call once in main()
 */
void eps_data_init(void);


// ============================================================================
// FUNCTION PROTOTYPES - STORE DATA
// ============================================================================

/**
 * @brief Store received EPS power status
 * @param status powinter to received power status data
 */
void eps_data_store_power_status(const EpsPowerStatus_t *status);

/**
 * @brief Store received EPS measurements
 * @param measurements pointer to received measurements data
 */
void eps_data_store_measurements(const EpsMeasurements_t *measurements);


// ============================================================================
// FUNCTION PROTOTYPES - RETRIEVE DATA
// ============================================================================

/**
 * @brief Get stored EPS power status
 * @param out_status pointer to destination struct
 * @return true if valid data available, false otherwise
 */
bool eps_data_get_power_status(EpsPowerStatus_t *out_status);

/**
 * @brief Get stored EPS measurements
 * @param out_measurements pointer to destination struct
 * @return true if valid data available, false otherwise
 */
bool eps_data_get_measurements(EpsMeasurements_t *out_measurements);

/**
 * @brief Check if power status data is valid and recent
 * @return true if data is valid and not stale
 */
bool eps_data_is_power_status_valid(void);

/**
 * @brief Check if measurements data is valid and recent
 * @return true if data is valid and not stale
 */
bool eps_data_is_measurements_valid(void);

/**
 * @brief Get age of power status data in millisceconds. 
 * @return Age in ms, or uint32_MAX if never received
 */
uint32_t eps_data_get_power_status_age_ms(void);

/**
 * @brief Get age of mesurements data in millisceconds. 
 * @return Age in ms, or uint32_MAX if never received
 */
uint32_t eps_data_get_measurements_age_ms(void);

// ============================================================================
// FUNCTION PROTOTYPES - SEND REQUESTS/COMMANDS TO EPS
// ============================================================================

/**
 * @brief Request EPS power status
 */
void eps_request_power_status(void);

/**
 * @brief Request EPS measurements
 */
void eps_request_measurements(void);

/**
 * @brief Send power line configuration command to EPS
 * @param cmd Pointer to command payload with desired line states
 */
void eps_send_power_command(const EpsPowerSetCmd_t *cmd);

// ============================================================================
// FUNCTION PROTOTYPES - CONVENIENCE HELPERS
// ============================================================================

/**
 * @brief Power line index enumeration for convenience functions. 
 */
typedef enum {
    EPS_LINE_3V3_1  = 0,
    EPS_LINE_3V3_2  = 1,
    EPS_LINE_3V3_3  = 2,
    EPS_LINE_5V_1   = 3,
    EPS_LINE_5V_2   = 4,
    EPS_LINE_5V_3   = 5,
    EPS_LINE_12V    = 6,
    EPS_LINE_COUNT  = 7
} EpsLineIndex_t;

/**
 * @brief Set all power lines to a specific state.
 * @param state The state to set all line to
 */
void eps_set_all_lines(PowerSelect_t state);

/**
 * @brief Set a specific power line to a state.
 * @param line line inddex (use EpsLineIndex_t enum)
 * @param state The state to set
 */
void eps_set_line(EpsLineIndex_t line, PowerSelect_t state);

/**
 * @brief Turn on a specific power line (sets to POWER_ON)
 * @param line Line index (use EpsLineIndex_t line)
 */
void eps_turn_on_line(EpsLineIndex_t line);

/**
 * @brief Turn off a specific power line (sets to POWER_OFF)
 * @param line Line index (use EpsLineIndex_t line)
 */
void eps_turn_off_line(EpsLineIndex_t line);

/**
 * @brief Get the state of a specific power line from stored status. 
 * @param line Line index (use EpsLineIndex_t line)
 * @return The power state, or POWER_OFF if invalid/unavailable
 */
PowerSelect_t eps_get_line_state(EpsLineIndex_t line);

/**
 * @brief Get battery voltage in millivolts.
 * @return Voltage in mV, or 0 if data not available
 */
uint16_t eps_get_battery_voltage_mv(void);

/**
 * @brief Get battery current in milliamps.
 * @return Current in mA, or 0 if data not available
 */
uint16_t eps_get_battery_current_ma(void);

#endif // EPS_DATA_H