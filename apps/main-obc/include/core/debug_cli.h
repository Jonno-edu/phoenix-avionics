#ifndef DEBUG_CLI_H
#define DEBUG_CLI_H

#include <stdint.h>
#include <stdbool.h>

/**
 * @brief Initialize the debuge CLI
 * Call once from main() after coonsole init()
 */
void debug_cli_init(void);

/**
 * @brief Process a received character for debug commands
 * Call this from the USB RX callback or polling loop. 
 * 
 * @param ch The received character
 * @return true if caracter was handled as debug command
 * @return false if character should be passed to RS485 protocol
 */
bool debug_cli_process_char(uint8_t ch);

/**
 * @brief Check if debug CLI mode is enable. 
 * When enabled, characters go to CLI instead of RS485.
 * 
 * @return true if CLI mode is active
 */
bool debug_cli_is_enabled(void);

/**
 * @brief Enable or disable debug CLI mode. 
 * @param enable ture to enable, false to deisable
 */
void debug_cli_set_enabled(bool enable);

/**
 * @brief Print the help menu showing all available commands.
 */
void debug_cli_print_help(void);


#endif // DEBUG_CLI_H