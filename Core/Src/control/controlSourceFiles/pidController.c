#include "control/controlHeaderFiles/pidController.h"
#include <math.h>

void pidController_setup(PID* pid, float Kp, float Ki, float Kd, float min, float max) {
	pid->Kp = Kp;
	pid->Ki = Ki;
	pid->Kd = Kd;

	pid->integral = 0.0f;
	pid->last_error = 0.0f;

	pid->min = min;
	pid->max = max;

	pid->last_time = timers_stopwatch();
}

float pidController_compute(PID* pid, float setPoint, float input) {
    uint32_t now = timers_stopwatch();
    float dt = (now - pid->last_time) / 1000000.0f; // micros → seconds
    if (dt <= 0.0f) return pid->min;

    float error = setPoint - input;

    // --- Static integral limits ---
    static float integral_max = 0.0f;
    static float integral_min = 0.0f;
    if (integral_max == 0.0f && pid->Ki != 0.0f) {
        integral_max = pid->max / pid->Ki;
        integral_min = pid->min / pid->Ki;
    }

    // --- Conditional integral (deadband) ---
    const float integral_deadband = 0.01f * (pid->max - pid->min); // 1% of output range
    if (fabs(error) > integral_deadband) {
        pid->integral += error * dt;
        if (pid->integral > integral_max) pid->integral = integral_max;
        if (pid->integral < integral_min) pid->integral = integral_min;
    }

    float derivative = (error - pid->last_error) / dt;

    float effort = pid->Kp * error + pid->Ki * pid->integral + pid->Kd * derivative;
	//uart_send("error:%f,",error); uart_send("effort:%f\n",effort);
    // Clamp output
    if (effort > pid->max) effort = pid->max;
    if (effort < pid->min) effort = pid->min;

    effort = fabs(effort);
    //uart_send("error:%f,",pid->Kp * error); uart_send("effort:%f\n",effort);

    pid->last_error = error;
    pid->last_time = now;

    return effort;
}


