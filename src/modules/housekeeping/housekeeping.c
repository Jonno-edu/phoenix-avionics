#include "housekeeping.h"
#include "modules/datalink/datalink.h"
#include "modules/tracking_radio/tracking_radio.h"
#include "common/pubsub.h"
#include "common/topics.h" // Added this include which was missing in the prompt but implied by usage

#include "FreeRTOS.h"
#include "task.h"

#define TRACKING_RADIO_POLL_MS  1000
#define TRACKING_RADIO_TIMEOUT_MS  100

void housekeeping_task(void *pvParameters)
{
    (void)pvParameters;

    // datalink_init() owns all hardware setup — call it here once
    datalink_init();

    TickType_t last_wake = xTaskGetTickCount();

    while (1)
    {
        TlmIdentificationPayload_t ident;

        bool ok = tracking_radio_request_ident(&ident, TRACKING_RADIO_TIMEOUT_MS);

        if (ok) {
            // Publish to the bus — any module can subscribe_poll this
            publish(TOPIC_TRACKING_RADIO_IDENT, &ident);
        }
        // If not ok: topic simply won't update this cycle.
        // Subscribers can detect staleness if they need to (future work).

        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(TRACKING_RADIO_POLL_MS));
    }
}
