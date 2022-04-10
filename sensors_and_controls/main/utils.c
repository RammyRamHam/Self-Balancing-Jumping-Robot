#include "utils.h"

void delayMs(TickType_t ms) {
    vTaskDelay(ms/portTICK_RATE_MS);
}