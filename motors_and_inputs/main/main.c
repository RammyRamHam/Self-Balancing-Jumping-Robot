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

static const char *TAG = "motors_and_inputs";


// void configureGPIO(void) {
//     gpio_reset_pin(MOTOR_LEFT_GPIO);
//     gpio_reset_pin(MOTOR_RIGHT_GPIO);

//     gpio_set_direction(MOTOR_LEFT_GPIO, GPIO_MODE_OUTPUT);
//     gpio_set_direction(MOTOR_RIGHT_GPIO, GPIO_MODE_OUTPUT);
// }

void app_main(void) {
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

    double toSend;
    double toReceive = 75.0;
    while (1) {
        // leftDriveSpeed -= 1.0;
        // rightDriveSpeed -= 1.0;
        // leftJumpSpeed -= 1.0;
        // rightJumpSpeed -= 1.0;

        // if (leftDriveSpeed < -100.0) {
        //     leftDriveSpeed = 0.0;
        // }
        // if (rightDriveSpeed < -100.0) {
        //     rightDriveSpeed = 0.0;
        // }
        // if (leftJumpSpeed < -100.0) {
        //     leftJumpSpeed = 0.0;
        // }
        // if (rightJumpSpeed < -100.0) {
        //     rightJumpSpeed = 0.0;
        // }

        // toSend[0] = leftDriveSpeed;
        // toSend[1] = rightDriveSpeed;
        // toSend[2] = leftJumpSpeed;
        // toSend[3] = rightJumpSpeed;
        // sendReceiveSpi(toSend, toReceive, sizeof(toSend));

        // setMotorSpeed(&leftDriveMotor, toReceive[0]);
        // setMotorSpeed(&rightDriveMotor, toReceive[1]);
        // setMotorSpeed(&leftJumpMotor, toReceive[2]);
        // setMotorSpeed(&rightJumpMotor, toReceive[3]);
        // vTaskDelay(20 / portTICK_PERIOD_MS);

        //sendReceiveSpi(&toSend, &toReceive, sizeof(toReceive));

        setMotorSpeed(&leftDriveMotor, toReceive);
        setMotorSpeed(&rightDriveMotor, toReceive);
        setMotorSpeed(&leftJumpMotor, toReceive);
        setMotorSpeed(&rightJumpMotor, toReceive);

        ESP_LOGI(TAG, "Speed = %f", toReceive);

        vTaskDelay(20 / portTICK_PERIOD_MS);
    }
}