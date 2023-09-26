#include <stdio.h>
#include <stdbool.h>
#include "esp_log.h"

#include "BNO055.h"
#include "utils.h"
#include "spi_master.h"
#include "control.h"
#include "config.h"
#include "speaker.h"

#define SAMPLE_DELAY 1

#define NUM_MOTORS 4

static const char *TAG = "sensors_and_controls";


void sendReceiveTest(void) {
    float sendAndReceive[NUM_MOTORS];

    for (uint8_t i = 0; i < NUM_MOTORS; i++) {
        sendAndReceive[i] = 0.0;
    }

    spi_device_handle_t spiSlave;
    configSpiMaster(&spiSlave);

    while (1) {
        sendReceiveSpi(&spiSlave, sendAndReceive, sendAndReceive, sizeof(sendAndReceive));
    }
}

void currTest(void) {
    speakerInit();
    xTaskCreate(speakerPlay, "speakerPlay", 1024 * 2, NULL, 5, NULL);

    bno055_t bno055Dev = {
        .devAdd = BNO055_ADDRESS_A,
        .mode = OPERATION_MODE_CONFIG,
    };
    
    if (!bno055Begin(OPERATION_MODE_NDOF, &bno055Dev)) {
        ESP_LOGE (TAG, "BNO055 COULD NOT BE INITIALIZED!");
    } else {
        ESP_LOGI(TAG, "BNO055 initialized successfully!");
    }

    delayMs(1000);

    double posData[3];

    float toSend [NUM_MOTORS];
    float toReceive [NUM_MOTORS];

    for (uint8_t i = 0; i < NUM_MOTORS; i++) {
        toSend[i] = 0.0;
        toReceive[i] = 0.0;
    }

    spi_device_handle_t spiSlave;
    configSpiMaster(&spiSlave);

    //bno055GetVector(posData, VECTOR_EULER, &bno055Dev);
    //float initAngle = posData[1];

    while (1) {
        bno055GetVector(posData, VECTOR_EULER, &bno055Dev);

        //ESP_LOGI(TAG, "X = %f, Y = %f, Z = %f", posData[0], posData[1], posData[2]);

        //float currSpeed = initAngle - posData[1];
        float currSpeed = pid(ANGLE_SETPOINT, posData[1]);
        toSend[0] = currSpeed;
        toSend[1] = currSpeed;

        ESP_LOGI(TAG, "Speed = %f", currSpeed);

        //sendReceiveSpi(&spiSlave, toSend, toReceive, sizeof(toSend));
    }
}

void currTest2(void) {
    speakerInit();
    xTaskCreate(speakerPlay, "speakerPlay", 1024 * 2, NULL, 5, NULL);

    
    bno055_t bno055Dev = {
        .devAdd = BNO055_ADDRESS_A,
        .mode = OPERATION_MODE_CONFIG,
    };
    
    if (!bno055Begin(OPERATION_MODE_NDOF, &bno055Dev)) {
        ESP_LOGE (TAG, "BNO055 COULD NOT BE INITIALIZED!");
    } else {
        ESP_LOGI(TAG, "BNO055 initialized successfully!");
    }

    delayMs(1000);

    double posData[3];

    while (1) {
        bno055GetVector(posData, VECTOR_EULER, &bno055Dev);

        ESP_LOGI(TAG, "X = %f, Y = %f, Z = %f", posData[0], posData[1], posData[2]);

    }
}

void audioTest(void) {
    speakerInit();
    xTaskCreate(speakerPlay, "speakerPlay", 1024 * 2, NULL, 5, NULL);
}

void app_main(void) {
    //audioTest();
    //gyroTest();
    //currTest();
    //sendReceiveTest();
    audioTest();
}
