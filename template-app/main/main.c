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

#define MOTOR_LEFT_GPIO GPIO_NUM_2
#define MOTOR_RIGHT_GPIO GPIO_NUM_15


// void configureGPIO(void) {
//     gpio_reset_pin(MOTOR_LEFT_GPIO);
//     gpio_reset_pin(MOTOR_RIGHT_GPIO);

//     gpio_set_direction(MOTOR_LEFT_GPIO, GPIO_MODE_OUTPUT);
//     gpio_set_direction(MOTOR_RIGHT_GPIO, GPIO_MODE_OUTPUT);
// }

mcpwm_config_t getMotorConfig(void) {
    mcpwm_config_t motorConfig;
    motorConfig.frequency = 1500;
    motorConfig.cmpr_a = 0;
    motorConfig.cmpr_b = 0;
    motorConfig.duty_mode = MCPWM_DUTY_MODE_0;
    motorConfig.counter_mode = MCPWM_UP_COUNTER;

    return motorConfig;
}

void configureMotor(mcpwm_config_t* motorConfig) {
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A , MOTOR_LEFT_GPIO);
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0B, MOTOR_RIGHT_GPIO);

    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, motorConfig);
}

void setMotorSpeed(float speed) {
    if (speed > 0) {
        mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_GEN_A, speed);
        mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_GEN_B, 0.0);
    } else {
        mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_GEN_A, 0.0);
        mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_GEN_B, -speed);
    }
}

void startMotor(void) {
    mcpwm_start(MCPWM_UNIT_0, MCPWM_TIMER_0);
}

void stopMotor(void) {
    mcpwm_stop(MCPWM_UNIT_0, MCPWM_TIMER_0);
}

void app_main(void) {
    // mcpwm_config_t motorConfig;
    // motorConfig.frequency = 10000000;
    // motorConfig.cmpr_a = 0;
    // motorConfig.cmpr_b = 0;
    // motorConfig.duty_mode = MCPWM_DUTY_MODE_0;
    // motorConfig.counter_mode = MCPWM_UP_COUNTER;

    // configureGPIO();
    mcpwm_config_t motorConfig = getMotorConfig();
    configureMotor(&motorConfig);

    setMotorSpeed(10.0);

    //startMotor();

    while (1) {
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}