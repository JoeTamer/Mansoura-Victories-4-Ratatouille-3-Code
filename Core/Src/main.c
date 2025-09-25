#include "core/coreHeaders.h"
#include "hardware/hardwareHeaders.h"
#include "peripherals/peripheralsHeaders.h"
#include <stm32f103xb.h>

void calibrateFront();
void FloodFill();

int main(void) {
	// == System CLK and Global Counter ==
	clk_setup();
    timers_stopwatchSetup(TIM2); // Global Counter using TIM4
    // == Peripherals Setup ==
    systick_setup();
    uart_setup();
    // == Core Setup ==
    core_setup();
    gpio_setupPin(GPIOC,13,OUT_50,GP_PP);

    // TODO : Adjust PID more
    pidController_setup(&RIGHT_SPEED_PID, 150.0f, 50.0f, 3.5f, 1000.0f, 10000.0f);
    pidController_setup(&LEFT_SPEED_PID , 150.0f, 50.0f, 13.5f, 800.0f, 10000.0f);

    pidController_setup(&STABLE_GYRO_PID, 15.5f, 0.5f, 0.01f, 0.0f, 15.0f);
    pidController_setup(&STABLE_IR_PID, 15.5f, 0.0f, 0.01f, 0.0f, 50.0f);

    pidController_setup(&SPIN_PID, 0.5f, 0.5f, 0.02f, 80.0f, 150.0f);
    pidController_setup(&IR_PID, 0.3f, 0.5f, 0.0f, 70.0f, 100.0f);

    // Calibrate Front
    systick_delayMillis(2000);
    calibrateFront();
    remi(DEADEND);
    // Mode Select
    core_modeSelect();
    core_speak(BEEP);
    systick_delayMillis(1000);
    // Wait For Touch
    while (!(walls & 0b0010)) {
		readWalls();
	}
    walls &=~ 0b0110;
	systick_delayMillis(500);
	core_speak(BEEP);
	systick_delayMillis(2000);

    while (1) {

    	if (CORE_MODE == 0) {
    		FloodFill();
    	}
    	else if (CORE_MODE == 1) { // TEST SENSORS
    		readWalls();
    	}
    	else if (CORE_MODE == 2) { // TEST MOTORS
    		core_MotorPidControl(&MR, FORWARD, 200); uart_send("R:%f\n",MR.CURR_RPM);
    		core_MotorPidControl(&ML, FORWARD, 200); uart_send("L:%f\n",ML.CURR_RPM);
    	}
    	else if (CORE_MODE == 3) {
    		aStar();
    	}

    }
    return 0;
}

void calibrateFront() {
	RIGHT_GOAL = IR_READINGS[1];
	LEFT_GOAL  = IR_READINGS[2];
}

void FloodFill() {
	// SEARCH
	SPEED = 200.0f;
	gpio_writePin(runLedPort   , runLedPin   , LOW);
	gpio_writePin(searchLedPort, searchLedPin, HIGH);
	do {
		floodfill(0);
	} while (FLOOD_MAP[currX][currY] != 0);
	remi(HALT);
	core_speak(CELEBRATE1);
	// RETURN
	SPEED = 200.0f;
	do {
		floodfill(1);
	} while (FLOOD_MAP[currX][currY] != 0);
	remi(HALT);
	core_speak(BEEP);

	// RUN 1
	SPEED = 200.0f;
	gpio_writePin(runLedPort   , runLedPin   , HIGH);
	gpio_writePin(searchLedPort, searchLedPin, LOW);
	do {
		floodfill(0);
	} while (FLOOD_MAP[currX][currY] != 0);
	remi(HALT);
	core_speak(CELEBRATE1);
	// RETURN
	SPEED = 200.0f;
	do {
		floodfill(1);
	} while (FLOOD_MAP[currX][currY] != 0);
	remi(HALT);
	core_speak(BEEP);

	// RUN 2
	SPEED = 225.0f;
	gpio_writePin(runLedPort   , runLedPin   , HIGH);
	gpio_writePin(searchLedPort, searchLedPin, LOW);
	do {
		floodfill(0);
	} while (FLOOD_MAP[currX][currY] != 0);
	remi(HALT);
	core_speak(CELEBRATE2);
	// RETURN
	SPEED = 200.0f;
	do {
		floodfill(1);
	} while (FLOOD_MAP[currX][currY] != 0);
	remi(HALT);
	core_speak(BEEP);
}
