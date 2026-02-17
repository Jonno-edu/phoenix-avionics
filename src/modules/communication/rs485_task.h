#ifndef RS485_TASK_H
#define RS485_TASK_H

#include "rs485_protocol.h"

void rs485_task_init(void);

rs485_instance_t *rs485_get_default_instance(void);

#endif
