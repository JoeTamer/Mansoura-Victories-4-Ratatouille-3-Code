#include "hardware/hardwareHeaderFiles/motors.h"

// ---------- Setup ----------
void motors_setup(Motor* myMotor, uint16_t FREQ) {
    gpio_setupPin(myMotor->IN1port, myMotor->IN1pin, OUT_50, GP_PP);
    gpio_setupPin(myMotor->IN2port, myMotor->IN2pin, OUT_50, GP_PP);

    timers_pwmSetup(myMotor->ENport, myMotor->ENpin, FREQ);
}

// ---------- Control ----------
void motors_control(Motor* myMotor, MOTORDIR dir, uint16_t speed) {
	if (dir == STOP) { motors_stop(myMotor); return; }

	gpio_writePin(myMotor->IN1port, myMotor->IN1pin, dir);
	gpio_writePin(myMotor->IN2port, myMotor->IN2pin, !dir);

	uint16_t compare = (((timers_getTimer(myMotor->ENport, myMotor->ENpin))->ARR) * speed) / 10000.0;
	timers_pwm(myMotor->ENport, myMotor->ENpin, compare);
}
void motors_stop(Motor* myMotor) {
	timers_pwm(myMotor->ENport, myMotor->ENpin, 0);
	gpio_writePin(myMotor->IN1port, myMotor->IN1pin, LOW);
	gpio_writePin(myMotor->IN2port, myMotor->IN2pin, LOW);
}
