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

    setMotorSpeed(&leftDriveMotor, -10.0);
    setMotorSpeed(&rightDriveMotor, -30.0);
    setMotorSpeed(&leftJumpMotor, -50.0);
    setMotorSpeed(&rightJumpMotor, -70.0);

    //startMotor();

    while (1) {
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}