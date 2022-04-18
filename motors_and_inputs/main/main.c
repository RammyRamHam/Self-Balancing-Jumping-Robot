#include <stdio.h>
#include <stdint.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "sdkconfig.h"
#include "freertos/queue.h"
#include "driver/mcpwm.h"
#include "esp_console.h"
#include "argtable3/argtable3.h"

#include "motors.h"
#include "spi_slave.h"

#define NUM_MOTORS 4

static const char *TAG = "motors_and_inputs";

// void configureGPIO(void) {
//     gpio_reset_pin(MOTOR_LEFT_GPIO);
//     gpio_reset_pin(MOTOR_RIGHT_GPIO);

//     gpio_set_direction(MOTOR_LEFT_GPIO, GPIO_MODE_OUTPUT);
//     gpio_set_direction(MOTOR_RIGHT_GPIO, GPIO_MODE_OUTPUT);
// }

void sendReceiveTest(void) {
    motor_t leftDriveMotor = getLeftDriveMotor();
    motor_t rightDriveMotor = getRightDriveMotor();
    motor_t leftJumpMotor = getLeftJumpMotor();
    motor_t rightJumpMotor = getRightJumpMotor();

    configureMotor(&leftDriveMotor);
    configureMotor(&rightDriveMotor);
    configureMotor(&leftJumpMotor);
    configureMotor(&rightJumpMotor);

    configSpiSlave();
    //startMotor();

    float toSend [NUM_MOTORS];
    float toReceive [NUM_MOTORS];

    for (uint8_t i = 0; i < NUM_MOTORS; i++) {
        toSend[i] = 0.0;
        toReceive[i] = 0.0;
    }

    while (1) {
        for (uint8_t i = 0; i < NUM_MOTORS; i++) {
            toSend[i] += 1.0;
            if (toSend[i] > 100.0) {
                toSend[i] = 0.0;
            }
        }

        sendReceiveSpi(toSend, toReceive, sizeof(toSend));

        setMotorSpeed(&leftDriveMotor, toReceive[0]);
        setMotorSpeed(&rightDriveMotor, toReceive[1]);
        //setMotorSpeed(&leftJumpMotor, toReceive[2]);
        //setMotorSpeed(&rightJumpMotor, toReceive[3]);
        vTaskDelay(75 / portTICK_PERIOD_MS);
    }
}

void motorTest(void) {
    motor_t leftDriveMotor = getLeftDriveMotor();
    motor_t rightDriveMotor = getRightDriveMotor();
    motor_t leftJumpMotor = getLeftJumpMotor();
    motor_t rightJumpMotor = getRightJumpMotor();

    configureMotor(&leftDriveMotor);
    configureMotor(&rightDriveMotor);
    configureMotor(&leftJumpMotor);
    configureMotor(&rightJumpMotor);

    float power = 0.0;
    while (1) {
        power += 1.0;
        if (power > 100.0) {
            power = 0.0;
        }

        setMotorSpeed(&leftDriveMotor, power);
        setMotorSpeed(&rightDriveMotor, power);
        //setMotorSpeed(&leftJumpMotor, toReceive[2]);
        //setMotorSpeed(&rightJumpMotor, toReceive[3]);
        vTaskDelay(75 / portTICK_PERIOD_MS);
    }
}

void currTest(void) {
    motor_t leftDriveMotor = getLeftDriveMotor();
    motor_t rightDriveMotor = getRightDriveMotor();
    motor_t leftJumpMotor = getLeftJumpMotor();
    motor_t rightJumpMotor = getRightJumpMotor();

    configureMotor(&leftDriveMotor);
    configureMotor(&rightDriveMotor);
    configureMotor(&leftJumpMotor);
    configureMotor(&rightJumpMotor);

    configSpiSlave();
    //startMotor();

    float toSend [NUM_MOTORS];
    float toReceive [NUM_MOTORS];

    for (uint8_t i = 0; i < NUM_MOTORS; i++) {
        toSend[i] = 0.0;
        toReceive[i] = 0.0;
    }

    while (1) {
        sendReceiveSpi(toSend, toReceive, sizeof(toSend));

        ESP_LOGI(TAG, "Left Speed = %f, Right Speed = %f", toReceive[0], toReceive[1]);

        setMotorSpeed(&leftDriveMotor, toReceive[0]);
        setMotorSpeed(&rightDriveMotor, toReceive[1]);
        //setMotorSpeed(&leftJumpMotor, toReceive[2]);
        //setMotorSpeed(&rightJumpMotor, toReceive[3]);
    }
}

void app_main(void) {
    //currTest();
    //sendReceiveTest();
    //motorTest();
}
