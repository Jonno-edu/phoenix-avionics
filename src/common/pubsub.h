#pragma once
#include <stdbool.h>
#include "topics.h"

void pubsub_init(void);
void publish(topic_id_t topic, const void *data);
bool subscribe_poll(topic_id_t topic, void *data_out);
