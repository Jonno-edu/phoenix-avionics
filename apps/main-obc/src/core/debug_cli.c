#include "core/debug_cli.h"
#include "core/logging.h"
#include "core/eps_node.h"
#include "core/obc_data.h"
#include "rs485_protocol.h"
#include "telemetry_defs.h"
#include "core/rs485_monitor.h"
#include <stdio.h>
#include <stdbool.h>

static const char *TAG = "DEBUG_CLI";

static bool cli_enabled = true;

// ============================================================================
// INITIALIZATION
// ============================================================================

void debug_cli_init(void) {
    cli_enabled = true;
    printf("\n");
    printf("===================================================\n");
    printf("    Phoenix Avionics Debug CLI\n");
    printf("===================================================\n");
    printf("Press 'h' for help, '?' for command list.\n");
    printf("===================================================\n");

    fflush(stdout);
}

// ============================================================================
// HELP MENU
// ============================================================================
void debug_cli_help_menu(void) {
    printf("\n");
    printf("=============== DEBUG CLI COMMANDS ===============\n");
    printf("\n");
    printf("--- EPS Telemetry Requests --- \n");
    printf(" S Request EPS Power Status\n");
    printf(" M Request EPS Measurements\n");
    printf("\n");
    printf("--- EPS Power Commands --- \n");
    printf(" a Turn ON all power lines\n");
    printf(" o Turn OFF all power lines\n");
    printf("\n");
    fflush(stdout);
    printf("--- Identification ---\n");
    printf(" i Request EPS Identification\n");
    printf(" I Request Radio Identification\n");
    printf("\n");
    printf("--- System Commands ---\n");
    printf(" s Print current EPS status\n");
    printf(" m Print current EPS measurements\n");
    printf("\n");
    fflush(stdout);
    printf("--- Log Level ---\n");
    printf(" 0 Set log level: None\n");
    printf(" 1 Set log level: Error\n");
    printf(" 2 Set log level: Info\n");
    printf(" 3 Set log level: Debug\n");
    printf("\n");
    fflush(stdout);
    printf("--- CLI Control ---\n");
    printf(" h Print this help\n");
    printf(" ? Print this help\n");
    printf(" x Exit CLI mode (pass-through to RS485)\n");
    printf(" X Enter CLI mode \n");
    printf("\n");
    printf("===================================================\n");
    printf("\n");
    fflush(stdout);
}

// ============================================================================
// STATUS PRINTING
// ============================================================================

static void print_eps_power_status(void) {
    EpsPowerStatus_t status;
    
    if (!eps_node_get_power_status(&status)) {
        printf("[CLI] No EPS power status available (request with '1' first)\n");
        return;
    }
    
    uint32_t age = eps_node_get_power_status_age_ms();
    bool valid = eps_node_is_power_status_valid();
    
    printf("\n--- EPS Power Status (age: %lu ms, %s) ---\n", 
           (unsigned long)age, valid ? "VALID" : "STALE");
    printf("  3.3V Line 1: %d (%s)\n", status.line_3v3_1, 
           status.line_3v3_1 == POWER_ON ? "ON" : 
           status.line_3v3_1 == POWER_OFF ? "OFF" : 
           status.line_3v3_1 == POWER_AUTO ? "AUTO" : "SIM");
    printf("  3.3V Line 2: %d (%s)\n", status.line_3v3_2,
           status.line_3v3_2 == POWER_ON ? "ON" : 
           status.line_3v3_2 == POWER_OFF ? "OFF" : 
           status.line_3v3_2 == POWER_AUTO ? "AUTO" : "SIM");
    printf("  3.3V Line 3: %d (%s)\n", status.line_3v3_3,
           status.line_3v3_3 == POWER_ON ? "ON" : 
           status.line_3v3_3 == POWER_OFF ? "OFF" : 
           status.line_3v3_3 == POWER_AUTO ? "AUTO" : "SIM");
    printf("  5V Line 1:   %d (%s)\n", status.line_5v_1,
           status.line_5v_1 == POWER_ON ? "ON" : 
           status.line_5v_1 == POWER_OFF ? "OFF" : 
           status.line_5v_1 == POWER_AUTO ? "AUTO" : "SIM");
    printf("  5V Line 2:   %d (%s)\n", status.line_5v_2,
           status.line_5v_2 == POWER_ON ? "ON" : 
           status.line_5v_2 == POWER_OFF ? "OFF" : 
           status.line_5v_2 == POWER_AUTO ? "AUTO" : "SIM");
    printf("  5V Line 3:   %d (%s)\n", status.line_5v_3,
           status.line_5v_3 == POWER_ON ? "ON" : 
           status.line_5v_3 == POWER_OFF ? "OFF" : 
           status.line_5v_3 == POWER_AUTO ? "AUTO" : "SIM");
    printf("  12V Line:    %d (%s)\n", status.line_12v,
           status.line_12v == POWER_ON ? "ON" : 
           status.line_12v == POWER_OFF ? "OFF" : 
           status.line_12v == POWER_AUTO ? "AUTO" : "SIM");
    printf("\n");
    fflush(stdout);
}

static void print_eps_measurements(void) {
    EpsMeasurements_t meas;
    
    if (!eps_node_get_measurements(&meas)) {
        printf("[CLI] No EPS measurements available (request with '2' first)\n");
        return;
    }
    
    uint32_t age = eps_node_get_measurements_age_ms();
    bool valid = eps_node_is_measurements_valid();
    
    printf("\n--- EPS Measurements (age: %lu ms, %s) ---\n",
           (unsigned long)age, valid ? "VALID" : "STALE");
    printf("  Battery Voltage:    %d mV\n", meas.battery_voltage_mv);
    printf("  Battery Current:    %d mA\n", meas.battery_current_ma);
    printf("  3.3V Line Current:  %d mA\n", meas.line_3v3_1_current_ma);
    printf("  5V Line Current:    %d mA\n", meas.line_5v_1_current_ma);
    printf("  12V Line Current:   %d mA\n", meas.line_12v_current_ma);
    printf("\n");
    fflush(stdout);
}

// ============================================================================
// TOGGLE HELPERS
// ============================================================================

static void toggle_line(EpsLineIndex_t line, const char *name) {
    PowerSelect_t current = eps_get_line_state(line);
    PowerSelect_t new_state;
    
    if (current == POWER_ON) {
        new_state = POWER_OFF;
        printf("[CLI] Turning %s OFF\n", name);
    } else {
        new_state = POWER_ON;
        printf("[CLI] Turning %s ON\n", name);
    }
    
    eps_set_line(line, new_state);
}

// ============================================================================
// MAIN COMMAND PROCESSOR
// ============================================================================

bool debug_cli_process_char(uint8_t ch) {
    // If CLI is disable, don't process (let RS485 handle it)
    if(!cli_enabled) {
        // But still check for CLI enable character
        if (ch == 'X') {
            cli_enabled = true;
            printf("[CLI] Debug CLI enabled. Press 'h' for help.\n");
            return true;
        }
        return false;
    }
    
    switch(ch) {
        // EPS TELEMETRY REQUESTS
        case 'S':
            printf("[CLI] Requesting EPS Power Status\n");
            eps_request_power_status();
            break;
        case 'M':
            printf("[CLI] Requesting EPS Measurements\n");
            eps_request_measurements();
            break;
        
        // EPS POWER COMMANDS
        case 'a':
            printf("[CLI] Turning all power lines ON\n");
            eps_set_all_lines(POWER_ON);
            break;
        case 'o':
            printf("[CLI] Turning all power lines OFF\n");
            eps_set_all_lines(POWER_OFF);
            break;
        

        // Identification Requests
        case 'i':
            printf("[CLI] Requesting EPS Identification\n");
            rs485_send_packet(ADDR_EPS, MSG_TYPE_TLM_REQ, TLM_ID_IDENTIFICATION, NULL, 0);
            break;
        case 'I':
            printf("[CLI] Requesting Tracking Radio Identification\n");
            rs485_send_packet(ADDR_TRACKING_RADIO, MSG_TYPE_TLM_REQ, TLM_ID_IDENTIFICATION, NULL, 0);
            break;

        // System Commands
        case 's':
            print_eps_power_status();
            break;
        case 'm':
            print_eps_measurements();
            break;  

        // Log Level
        // case '0':
        //     printf("[CLI] Setting log level: None\n");
        //     system_config_set_log_level(0);
        //     break;
        // case '1':
        //     printf("[CLI] Setting log level: Error\n");
        //     system_config_set_log_level(1);
        //     break;
        // case '2':
        //     printf("[CLI] Setting log level: Info\n");
        //     system_config_set_log_level(2);
        //     break;
        // case '3':
        //     printf("[CLI] Setting log level: Debug\n");
        //     system_config_set_log_level(3);
        //     break;

        //RS485 Monitor Control
        case '0':
            printf("[CLI] Setting RS485 Monitor Mode: OFF\n");
            rs485_monitor_set_mode(RS485_MON_OFF);
            break;
        case '1':
            printf("[CLI] Setting RS485 Monitor Mode: RAW\n");
            rs485_monitor_set_mode(RS485_MON_RAW);
            break;
        case '2':
            printf("[CLI] Setting RS485 Monitor Mode: DECODED\n");
            rs485_monitor_set_mode(RS485_MON_DECODED);
            break;
        case '3':
            printf("[CLI] Setting RS485 Monitor Mode: VERBOSE\n");
            rs485_monitor_set_mode(RS485_MON_VERBOSE);
            break;


        // CLI Control
        case 'h':
            debug_cli_help_menu();
            break;
        case '?':
            debug_cli_help_menu();
            break;
        case 'x':
            cli_enabled = false;
            printf("[CLI] Debug CLI disabled. Press 'X' to enable.\n");
            printf("[CLI] Press 'X' to re-enable.\n");
            break;

        case 'X':
            cli_enabled = true;
            printf("[CLI] Debug CLI enabled. Press 'h' for help.\n");
            break;

        // Ignore Common Noise
        case '\r':
        case '\n':
        case ' ':
            break;
        
        // Unknown Command
        default:
            printf("[CLI] Unknown command: '%c' (0x%02X). Press 'h' for help.\n", (ch >= 32 && ch <= 126) ? ch : '?', ch);
            break;
    }
    
    return true;
}
    
// ============================================================================
// CLI STATE ACCESSORS
// ============================================================================
bool debug_cli_is_enabled(void) {
    return cli_enabled;
}

void debug_cli_set_enabled(bool enable) {
    cli_enabled = enable;
    if (enable) {
        printf("[CLI] Debug CLI enabled.\n");
    } else {
        printf("[CLI] Debug CLI disabled.\n");
    }
}