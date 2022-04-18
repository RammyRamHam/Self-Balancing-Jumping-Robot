#ifndef _MOTORS_H_
#define _MOTORS_H_

#include <stdbool.h>
#include "driver/mcpwm.h"

typedef struct {
    mcpwm_config_t mcpwmConfig;
    mcpwm_unit_t mcpwmUnit;
    mcpwm_timer_t timer;
    mcpwm_io_signals_t leftPwmOut;
    mcpwm_io_signals_t rightPwmOut;
    bool flipped;
    float maxSpeed;
    int leftGpio;
    int rightGpio;
} motor_t;

mcpwm_config_t getDefaultMotorConfig(void);
void configureMotor(motor_t* motor);

motor_t getLeftDriveMotor(void);
motor_t getRightDriveMotor(void);
motor_t getLeftJumpMotor(void);
motor_t getRightJumpMotor(void);


void setMotorSpeed(motor_t* motor, float speed);
void startMotor(motor_t* motor);
void stopMotor(motor_t* motor);

#endif