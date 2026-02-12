#ifndef RS485_TASK_H
#define RS485_TASK_H

#include "rs485_protocol.h"

void rs485_task_init(void);

/**
 * @brief Get the default RS485 instance (Main Bus).
 *        Used for legacy compatibility during refactor.
 */
rs485_instance_t *rs485_get_default_instance(void);

#endif
