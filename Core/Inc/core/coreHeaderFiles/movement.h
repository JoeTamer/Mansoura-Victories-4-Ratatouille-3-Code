#ifndef INC_CORE_COREHEADERFILES_MOVEMENT_H_
#define INC_CORE_COREHEADERFILES_MOVEMENT_H_

/**
 * @file movement.h
 * @brief Header for Remi Micromouse movement control.
 *
 * Contains function declarations for hardware setup,
 * encoder handling, gyroscope-based spinning, and control logic.
 */

#include <math.h>
#include <stdio.h>
#include "control/controlHeaders.h"
#include "core/remiConfig.h"
#include "hardware/hardwareHeaders.h"


extern uint8_t walls;
#define NUM_OF_SENSORS 4
extern uint16_t IR_READINGS[NUM_OF_SENSORS];

extern float SPEED;

/**
 * @enum ACTIONS
 * @brief Enum representing available movement actions.
 */
typedef enum {
	ONCE, WALLSTEP, HALFSTEP, FULLSTEP, SPINRIGHT, SPINLEFT,
	DEADEND, READWALLS, HALT
} ACTIONS;

typedef enum {
	BEEP, CELEBRATE1, CELEBRATE2, FAULT
} TONE;

// Extern declarations for hardware structs
extern Motor MR, ML;
extern Encoder ENR, ENL;
extern MPU6050 MPU;
extern INFRARED IRX[4];

extern PID LEFT_SPEED_PID;
extern PID RIGHT_SPEED_PID;

extern PID STABLE_GYRO_PID;
extern PID STABLE_IR_PID;

extern PID SPIN_PID;
extern PID IR_PID;

// Extras
extern uint8_t CORE_MODE;

// Infrared Front
extern uint16_t LEFT_GOAL;
extern uint16_t RIGHT_GOAL;

/* ========================================================================================================================
 * === SETUP FUNCTIONS ===
 * ======================================================================================================================== */
void core_setup();
void core_setupMotors();
void core_setupEncoders(EDGESTATE ES);
void core_setupMPU6050();
void core_setupInfraredSensors();
void core_setupExtras(uint32_t buzzFreq);

/* ========================================================================================================================
 * === MOTOR FUNCTIONS ===
 * ======================================================================================================================== */
void core_MotorPidControl(Motor* myMotor, MOTORDIR dir, float targetRPM);

/* ========================================================================================================================
 * === ENCODER FUNCTIONS ===
 * ======================================================================================================================== */
void rightCountRisesCH1();
void rightCountRisesCH2();
void leftCountRisesCH1();
void leftCountRisesCH2();
void resetEncoders();
/* ======================================================================================================================== */
void core_calcDistance();
void core_calcRPM();

/* ========================================================================================================================
 * === GYROSCOPE FUNCTIONS ===
 * ======================================================================================================================== */
float core_calcAngles();

/* ========================================================================================================================
 * === INFRARED SENSOR FUNCTIONS ===
 * ======================================================================================================================== */

/* ========================================================================================================================
 * === BUZZER CONTROL FUNCTIONS ===
 * ======================================================================================================================== */
void core_Buzz(uint16_t freq, uint16_t duration);
void core_speak(TONE melody);

/* ========================================================================================================================
 * === DIP SWITCH FUNCTIONS ===
 * ======================================================================================================================== */
void core_modeSelect();

/* ========================================================================================================================
 * === TEST FUNCTIONS ===
 * ======================================================================================================================== */
void core_spinOnce();
void core_rpmLimits(Motor* myMotor);
void core_printCount(Encoder* myEncoder);

/* ========================================================================================================================
 * === MAIN CONTROL FUNCTION ===
 * ======================================================================================================================== */
uint8_t remi(ACTIONS remiActions);

/* ========================================================================================================================
 * === MOVEMENT / ACTION FUNCTIONS ===
 * ======================================================================================================================== */
void walk(float targetDistance, float baseRPM, MOTORDIR State);
void spin(float targetAngle);
void stop();
uint8_t readWalls();

/* ========================================================================================================================
 * === HELPER FUNCTIONS ===
 * ======================================================================================================================== */
void move(float targetDistance, float rpm);
void still();
void frontCalibration();
void backCalibration(ACTIONS x);
void resetPID();
void resetCore();

#endif /* INC_CORE_COREHEADERFILES_MOVEMENT_H_ */
