#include "esp_timer.h"
#include <math.h>

#include "control.h"
#include "config.h"

static float pidI = 0.0;
static float pidPrevError = 0.0;
static int64_t prevTime = 0.0;

float pid(float target, float currentLoc) {
    float error = target - currentLoc;
    int64_t currTime = esp_timer_get_time();
    float deltaTime = (currTime - prevTime);
    //pidI += abs(currentLoc) > abs(target) * 0.8 ? error*deltaTime : 0;
    pidI += error*deltaTime;
    //i = Math.min(maxI, Math.max(-maxI, i));
    float d = (error - pidPrevError)/deltaTime;
    float power = (PID_KP*error) + (PID_KI*pidI) + (PID_KD*d);
    prevTime = currTime;
    pidPrevError = error;

    return power;
}

void resetPid() {
    pidI = 0.0;
    pidPrevError = 0.0;
}
