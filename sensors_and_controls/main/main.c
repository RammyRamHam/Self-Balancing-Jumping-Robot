#include <stdio.h>
#include <stdbool.h>
#include "esp_log.h"
//#include "i2c.h"
#include "BNO055.h"
#include "utils.h"

#define SAMPLE_DELAY 1

static const char *TAG = "sensors_and_controls";

void app_main(void) {
    bno055_t bno055Dev = {
        .devAdd = BNO055_ADDRESS_A,
        .mode = OPERATION_MODE_CONFIG,
    };
    
    if (!bno055Begin(OPERATION_MODE_NDOF, &bno055Dev)) {
        ESP_LOGE (TAG, "BNO055 COULD NOT BE INITIALIZED!");
    } else {
        ESP_LOGI(TAG, "BNO055 initialized successfully");
    }

    delayMs(1000);

    //bno055SetExtCrystalUse(true, &bno055Dev);
    int i = 0;
    double posData[3];
    while (1) {
        bno055GetVector(posData, VECTOR_EULER, &bno055Dev);
        ESP_LOGI(TAG, "X = %f, Y = %f, Z = %f", posData[0], posData[1], posData[2]);
        delayMs(SAMPLE_DELAY);
    }
}
