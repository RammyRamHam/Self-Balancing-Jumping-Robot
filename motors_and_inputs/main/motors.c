#include "motors.h"
#include "config.h"

mcpwm_config_t getDefaultMotorConfig(void) {
    mcpwm_config_t motorConfig = {
        .frequency = 15000,
        .cmpr_a = 0.0,
        .cmpr_b = 0.0,
        .duty_mode = MCPWM_DUTY_MODE_0,
        .counter_mode = MCPWM_UP_COUNTER,
    };

    return motorConfig;
}

void configureMotor(motor_t* motor) {
    mcpwm_gpio_init(motor->mcpwmUnit, motor->leftPwmOut, motor->leftGpio);
    mcpwm_gpio_init(motor->mcpwmUnit, motor->rightPwmOut, motor->rightGpio);

    mcpwm_init(motor->mcpwmUnit, motor->timer, &(motor->mcpwmConfig));    
}

motor_t getLeftDriveMotor(void) {
    motor_t leftDrive = {
        .mcpwmConfig = getDefaultMotorConfig(),
        .mcpwmUnit = MCPWM_UNIT_0,
        .timer = MCPWM_TIMER_0,
        .leftPwmOut = MCPWM0A,
        .rightPwmOut = MCPWM0B,
        .flipped = false,
        .maxSpeed = 35.0,
        .leftGpio = LEFT_DRIVE_MOTOR_LEFT_GPIO,
        .rightGpio = LEFT_DRIVE_MOTOR_RIGHT_GPIO,
    };

    return leftDrive;
}

motor_t getRightDriveMotor(void) {
    motor_t rightDrive = {
        .mcpwmConfig = getDefaultMotorConfig(),
        .mcpwmUnit = MCPWM_UNIT_0,
        .timer = MCPWM_TIMER_1,
        .leftPwmOut = MCPWM1A,
        .rightPwmOut = MCPWM1B,
        .flipped = true,
        .maxSpeed = 35.0,
        .leftGpio = RIGHT_DRIVE_MOTOR_LEFT_GPIO,
        .rightGpio = RIGHT_DRIVE_MOTOR_RIGHT_GPIO,
    };

    return rightDrive;
}

motor_t getLeftJumpMotor(void) {
    motor_t leftJump = {
        .mcpwmConfig = getDefaultMotorConfig(),
        .mcpwmUnit = MCPWM_UNIT_1,
        .timer = MCPWM_TIMER_0,
        .leftPwmOut = MCPWM0A,
        .rightPwmOut = MCPWM0B,
        .flipped = false,
        .maxSpeed = 35.0,
        .leftGpio = LEFT_JUMP_MOTOR_LEFT_GPIO,
        .rightGpio = LEFT_JUMP_MOTOR_RIGHT_GPIO,
    };

    return leftJump;
}

motor_t getRightJumpMotor(void) {
    motor_t rightJump = {
        .mcpwmConfig = getDefaultMotorConfig(),
        .mcpwmUnit = MCPWM_UNIT_1,
        .timer = MCPWM_TIMER_1,
        .leftPwmOut = MCPWM1A,
        .rightPwmOut = MCPWM1B,
        .flipped = false,
        .maxSpeed = 35.0,
        .leftGpio = RIGHT_JUMP_MOTOR_LEFT_GPIO,
        .rightGpio = RIGHT_JUMP_MOTOR_RIGHT_GPIO,
    };

    return rightJump;
}

void setMotorSpeed(motor_t* motor, float speed) {
    speed = motor->flipped ? -speed : speed;
    if (speed > 0) {
        speed = speed > motor->maxSpeed ? motor->maxSpeed : speed;
        mcpwm_set_duty(motor->mcpwmUnit, motor->timer, MCPWM_GEN_A, speed);
        mcpwm_set_duty(motor->mcpwmUnit, motor->timer, MCPWM_GEN_B, 0.0);
    } else {
        speed = speed < -motor->maxSpeed ? -motor->maxSpeed : speed;
        mcpwm_set_duty(motor->mcpwmUnit, motor->timer, MCPWM_GEN_A, 0.0);
        mcpwm_set_duty(motor->mcpwmUnit, motor->timer, MCPWM_GEN_B, -speed);
    }
}

void startMotor(motor_t* motor) {
    mcpwm_start(motor->mcpwmUnit, motor->timer);
}

void stopMotor(motor_t* motor) {
    mcpwm_stop(motor->mcpwmUnit, motor->timer);
}