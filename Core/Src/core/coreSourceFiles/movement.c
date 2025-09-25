/******************************************************
 * File: movement.c
 * Description: Controls movement for Remi Micromouse.
 *              Handles hardware setup, encoder tracking,
 *              MPU6050 gyro-based spinning, and main movement logic.
 *
 * Author: Jo
 * Date: ~
 ******************************************************/

#include <core/coreHeaderFiles/movement.h>
#define DEBUG_MODE

/* ========================================================================================================================
 * === GLOBAL CONSTANTS & VARIABLES ===
 * - Defines hardware structs
 * - Tracks encoder values, movement distance, and gyro spin
 * ======================================================================================================================== */

// Hardware Structs
Motor MR, ML;
Encoder ENR, ENL;
MPU6050 MPU;
INFRARED IRX[4];

// Control Structs
PID LEFT_SPEED_PID;
PID RIGHT_SPEED_PID;

PID STABLE_GYRO_PID;
PID STABLE_IR_PID;

PID SPIN_PID;
PID IR_PID;

/* === MOTORS === */
uint16_t motorFreq = 20000;
float SPEED = 150.0f;

/* === ENCODER === */
// Gear Ratios
#define METAL_GEARS   200.0f
#define PLASTIC_GEARS 0.5f
#define ALL_GEARS     (METAL_GEARS * PLASTIC_GEARS)
// Wheel Geometry
#define PI            3.14159f
#define DIAMETER      25.0f
#define CIRCUMFERENCE PI * DIAMETER
// Tracking
float PRRwheel    = 0;
float DISPERCOUNT = 0;
// Counts
volatile uint32_t rightEncoderCount = 0;
volatile uint32_t leftEncoderCount  = 0;
volatile float walkedDistanceRight  = 0;
volatile float walkedDistanceLeft   = 0;
volatile float walkedDistanceAvg    = 0;
volatile double spins               = 0;
volatile uint32_t lastRpmTime       = 0;
volatile double lastSpins           = 0;
#define RPM_SAMPLE_INTERVAL 10000 // 10 ms in µs
#define RPM_SMOOTH_ALPHA 0.15f  // smaller = smoother, larger = more responsive
uint32_t lastSampleTime[2] = {0,0};
uint32_t lastCount[2]      = {0,0};
uint32_t currTime = 0;

float distance = 200.0f;

// Gyroscope Tracking
uint32_t lastGyroTime = 0;
volatile float spunAngles = 0;

// InfraRed Data
#define NUM_OF_SENSORS 4
uint16_t IR_READINGS[NUM_OF_SENSORS];
uint16_t wallThreshold = 4010;
uint8_t walls = 0;

uint16_t LEFT_GOAL  = 3780;
uint16_t RIGHT_GOAL = 3830;

// Extras
uint8_t CORE_MODE;

/* ========================================================================================================================
 * === SETUP HARDWARE FUNCTIONS ===
 * - Fills Motor, Encoder, and MPU6050 structs
 * - Calls respective hardware setup functions
 * ======================================================================================================================== */
void core_setup() {
	core_setupMotors();
	core_setupEncoders(CHANGE);
	core_setupMPU6050();
	core_setupInfraredSensors();
	core_setupExtras(2000);

	resetCore();
}
void core_setupMotors() {
	// Right Motor Pins
	MR.IN1port = rightMotorIN1Port; MR.IN1pin   = rightMotorIN1Pin;
	MR.IN2port = rightMotorIN2Port; MR.IN2pin   = rightMotorIN2Pin;
	MR.ENport  = rightMotorENPort ; MR.ENpin    = rightMotorENPin ;
    MR.CURR_RPM = 0;

	// Left Motor Pins
	ML.IN1port = leftMotorIN1Port;  ML.IN1pin   = leftMotorIN1Pin;
	ML.IN2port = leftMotorIN2Port;  ML.IN2pin   = leftMotorIN2Pin;
	ML.ENport  = leftMotorENPort ;  ML.ENpin    = leftMotorENPin ;
	ML.CURR_RPM = 0;

	// Initialize Motors
	motors_setup(&MR, motorFreq);
	motors_setup(&ML, motorFreq);

	#ifdef DEBUG_MODE
	uart_send("SETUP MOTORS COMPLETE !!! \n");
	#endif
}
void core_setupEncoders(EDGESTATE ES) {
	// Right Encoder Pins
	ENR.channels[0].port = rightEncoderCH1Port; ENR.channels[0].pin = rightEncoderCH1Pin;
	ENR.channels[1].port = rightEncoderCH2Port; ENR.channels[1].pin = rightEncoderCH2Pin;
	ENR.numChannels = 2;
	ENR.PPR = rightEncoderPPR;

	// Left Encoder Pins
	ENL.channels[0].port = leftEncoderCH1Port; ENL.channels[0].pin = leftEncoderCH1Pin;
	ENL.channels[1].port = leftEncoderCH2Port; ENL.channels[1].pin = leftEncoderCH2Pin;
	ENL.numChannels = 2;
	ENL.PPR = leftEncoderPPR;

	// Count Pulses Per 1 Wheel Rotation
	PRRwheel = ALL_GEARS * ENR.PPR * ENR.numChannels; if (ES == CHANGE) PRRwheel *= 2;
	DISPERCOUNT = (CIRCUMFERENCE / PRRwheel);
	// Setup Encoder Interrupts
	encoder_setupPin(&ENR, 1, ES, rightCountRisesCH1);
	encoder_setupPin(&ENR, 2, ES, rightCountRisesCH2);
	encoder_setupPin(&ENL, 1, ES, leftCountRisesCH1 );
	encoder_setupPin(&ENL, 2, ES, leftCountRisesCH2 );

	#ifdef DEBUG_MODE
	uart_send("SETUP ENCODERS COMPLETE !!! \n");
	uart_send("Wheel PPR = %f\n",PRRwheel);
	#endif
}
void core_setupMPU6050() {
	MPU.I2Cx    = mpuI2CPeripheral;
	MPU.address = mpuAddress;

	mpu6050_setup(&MPU, mpuSpeedMode, mpuDlpfConfig, mpuGyroConfig, mpuI2CAlternatePins);

	#ifdef DEBUG_MODE
	uart_send("SETUP MPU6050 COMPLETE !!! \n");
	#endif
}
void core_setupInfraredSensors() {

	for (uint8_t i = 0; i<NUM_OF_SENSORS; i++) {
		IRX[i].ADCx = irADCPeripheral;
		IRX[i].port = irADCPort;
		IRX[i].pin  = irADCPin + i;
	}

	infraredSensors_setup(ADC1, IRX, NUM_OF_SENSORS, IR_READINGS);

	#ifdef DEBUG_MODE
	uart_send("SETUP INFRARED COMPLETE !!! \n");
	#endif
}
void core_setupExtras(uint32_t buzzFreq) {
	// DIP_SWITCHES
	gpio_setupPin(dip0Port,dip0Pin,IN,PULLUP);
	gpio_setupPin(dip1Port,dip1Pin,IN,PULLUP);
	// LEDs
	gpio_setupPin(rightLedPort,rightLedPin,OUT_50,GP_PP);
	gpio_setupPin(frontLedPort,frontLedPin,OUT_50,GP_PP);
	gpio_setupPin(leftLedPort ,leftLedPin ,OUT_50,GP_PP);

	gpio_setupPin(searchLedPort,searchLedPin,OUT_50,GP_PP);
	gpio_setupPin(runLedPort   ,runLedPin   ,OUT_50,GP_PP);
	// BUZZER
	timers_pwmSetup(buzzerPort, buzzerPin, buzzFreq);

	#ifdef DEBUG_MODE
	uart_send("SETUP EXTRAS COMPLETE !!! \n");
	#endif
}

/* ========================================================================================================================
 * === MOTORS LOGIC FUNCTIONS ===
 * - PID controlled Movement Functions
 * ======================================================================================================================== */
void core_MotorPidControl(Motor* myMotor, MOTORDIR dir, float targetRPM) {
    core_calcRPM();
    float pidout = 0;

    if (myMotor == &MR) {
    	pidout = pidController_compute(&RIGHT_SPEED_PID, targetRPM-20, myMotor->CURR_RPM);
    }
    else if (myMotor == &ML) {
    	pidout = pidController_compute(&LEFT_SPEED_PID, targetRPM-21, myMotor->CURR_RPM);
    }
    motors_control(myMotor, dir, (uint16_t)(pidout));
}

/* ========================================================================================================================
 * === ENCODER LOGIC FUNCTIONS ===
 * - Interrupt callbacks increment counts and calculate walked distance
 * ======================================================================================================================== */
void rightCountRisesCH1() {
	rightEncoderCount++;
	core_calcDistance();
}
void rightCountRisesCH2() {
	rightEncoderCount++;
}
void leftCountRisesCH1() {
	leftEncoderCount++;
	core_calcDistance();
}
void leftCountRisesCH2() {
	leftEncoderCount++;
}
void resetEncoders() {
	rightEncoderCount = 0;
	leftEncoderCount  = 0;
}

/* ======================================================================================================================== */

void core_calcDistance() {
    uint32_t rc, lc;
    __disable_irq();
    rc = rightEncoderCount;
    lc = leftEncoderCount;
    __enable_irq();

    walkedDistanceRight = DISPERCOUNT * (float)rc;
    walkedDistanceLeft  = DISPERCOUNT * (float)lc;
    walkedDistanceAvg = (walkedDistanceRight + walkedDistanceLeft) * 0.5f;
}

void core_calcRPM() {
    currTime = timers_stopwatch();

    // Right motor
    if ((currTime - lastSampleTime[0]) >= RPM_SAMPLE_INTERVAL) {
        uint32_t deltaCount = rightEncoderCount - lastCount[0];
        float dt = (currTime - lastSampleTime[0]) / 1000000.0f; // µs → s
        //uart_send("deltaCountr:%d",deltaCount); uart_send(",dtr:%f\n",dt);

        float revs = deltaCount / PRRwheel;
        float rpm  = (revs / dt) * 60.0f;

        // Smooth update
        MR.CURR_RPM = RPM_SMOOTH_ALPHA * rpm + (1.0f - RPM_SMOOTH_ALPHA) * MR.CURR_RPM;

        lastCount[0] = rightEncoderCount;
        lastSampleTime[0] = currTime;
    }

    // Left motor
    if ((currTime - lastSampleTime[1]) >= RPM_SAMPLE_INTERVAL) {
        uint32_t deltaCount = leftEncoderCount - lastCount[1];
        float dt = (currTime - lastSampleTime[1]) / 1000000.0f; // µs → s
        //uart_send("deltaCountl:%d",deltaCount); uart_send(",dtl:%f\n",dt);

        float revs = deltaCount / PRRwheel;
        float rpm  = (revs / dt) * 60.0f;

        // Smooth update
        ML.CURR_RPM = RPM_SMOOTH_ALPHA * rpm + (1.0f - RPM_SMOOTH_ALPHA) * ML.CURR_RPM;

        lastCount[1] = leftEncoderCount;
        lastSampleTime[1] = currTime;
    }
}

/* ========================================================================================================================
 * === GYROSCOPE LOGIC FUNCTIONS ===
 * - Integrates gyro Z-axis to achieve desired rotation
 * - @param targetAngle Angle in degrees (positive = CW, negative = CCW)
 * ======================================================================================================================== */
float core_calcAngles() {
    float gyroX_dps, gyroY_dps, gyroZ_dps;
    uint32_t currTime;

    mpu6050_readGyro(&MPU, &gyroX_dps, &gyroY_dps, &gyroZ_dps);
    currTime = timers_stopwatch();

    float dt = (currTime - lastGyroTime) / 1000000.0f; // Convert µs to seconds
    spunAngles += gyroZ_dps * dt;

    lastGyroTime = currTime;

	#ifdef DEBUG_MODE
	//uart_send("ANGLE : %f\n",spunAngles);
	#endif

    return spunAngles;
}

/* ========================================================================================================================
 * === INFRARED LOGIC FUNCTIONS ===
 * ======================================================================================================================== */

/* ========================================================================================================================
 * === BUZZER CONTROL FUNCTIONS ===
 * ======================================================================================================================== */
void core_buzz(uint16_t freq, uint16_t duration) {
	timers_pwm(buzzerPort, buzzerPin, freq);
	systick_delayMillis(duration);
	timers_pwm(buzzerPort, buzzerPin, 0);
}
void core_speak(TONE melody) {
	switch (melody) {
	case BEEP:
		core_buzz(1800, 150);
		core_buzz(2000, 50);
		break;
	case CELEBRATE1:
		core_buzz(2000, 200);
		core_buzz(1600, 200);
		core_buzz(0   , 200);
		core_buzz(1800, 600);
		core_buzz(1500, 200);
		break;
	case CELEBRATE2: break;
	case FAULT:      break;
	default:
	}
}

/* ========================================================================================================================
 * === DIP SWITCH FUNCTIONS ===
 * ======================================================================================================================== */
void core_modeSelect() {
	CORE_MODE = ((gpio_readPin(dip0Port,dip0Pin) << 0) | (gpio_readPin(dip1Port,dip1Pin) << 1));
}

/* ========================================================================================================================
 * === TEST FUNCTIONS ===
 * ======================================================================================================================== */
void core_spinOnce() {
	motors_control(&MR, FORWARD, 600);
	motors_control(&ML, FORWARD, 600);

	while ((walkedDistanceRight < CIRCUMFERENCE) || (walkedDistanceLeft < CIRCUMFERENCE)) {
	    if (walkedDistanceRight >= CIRCUMFERENCE)motors_control(&MR, FORWARD, 0);
	    if (walkedDistanceLeft  >= CIRCUMFERENCE)motors_control(&ML, FORWARD, 0);
	}
	motors_control(&MR, FORWARD, 0);
	motors_control(&ML, FORWARD, 0);
}
void core_rpmLimits(Motor* myMotor) {

	motors_control(myMotor, FORWARD, 1000);
	for(uint16_t i = 0 ; i < 5000 ; i++) {
		systick_delayMillis(1);
		core_calcRPM();
		uart_send("RPM : %f\n",myMotor->CURR_RPM);
	}
	motors_control(myMotor, STOP, 0);
	systick_delayMillis(1000);

	motors_control(myMotor, FORWARD, 500);
	for(uint16_t i = 0 ; i < 5000 ; i++) {
		systick_delayMillis(1);
		core_calcRPM();
		uart_send("RPM : %f\n",myMotor->CURR_RPM);
	}
	motors_control(myMotor, STOP, 0);
}
void core_printCount(Encoder* myEncoder) {
	uint32_t count = (myEncoder == &ENR) ? rightEncoderCount : leftEncoderCount;
	uart_send("%d\n",count);
}

/* ========================================================================================================================
 * === MAIN CONTROL FUNCTION ===
 * - Central control switch for micromouse actions
 * - @param remiActions Enum input defining the desired action
 * ======================================================================================================================== */
uint8_t remi(ACTIONS remiActions) {
	switch (remiActions) {
	case ONCE    :  walk(CIRCUMFERENCE, 200.0f, STOP); return 0;
	case WALLSTEP:  walk( 45.0f, SPEED, 0); return 0;
	case HALFSTEP:  walk( 90.0f, SPEED, 0); return 0;
	case FULLSTEP:  walk(distance, SPEED, 0); return 0;
	case SPINRIGHT: spin(-90.0f); return 0;
	case SPINLEFT:  spin( 90.0f); return 0;
	case DEADEND:   remi(SPINRIGHT); remi(SPINRIGHT); return 0;
	case READWALLS: return readWalls();
	case HALT:      stop(); return 0;
	default: return 1;
	}
}

/* ========================================================================================================================
 * === SUB CONTROL FUNCTIONS ===
 * - Predefined movement routines for the micromouse
 * ======================================================================================================================== */
void walk(float targetDistance, float baseRPM, MOTORDIR State) {
    // Encoder setup
    float targetCount = targetDistance / DISPERCOUNT;
    uint32_t avgEncoderCount = (rightEncoderCount + leftEncoderCount) * 0.5;
    uint32_t startCount = avgEncoderCount;

    // Gyro setup
    lastGyroTime = timers_stopwatch();
    spunAngles = 0.0f;
    float offsetAngles = core_calcAngles();
    float currAngle = 0, TEMP = 0;

    // Filters
    float filteredAngle = 0.0f;
    float filteredPidOut = 0.0f;
    float alphaAngle = 0.9f;   // gyro smoothing
    float alphaPID   = 0.9f;   // correction smoothing

    // Gyro Deadband
    float DEADBAND_G   = 0.003f;
    float correction_G = 0.0f;

    // IR Deadband
    //float DEADBAND_IR   = 110.0f;
    float correction_IR_RIGHT = 0.0f;
    float correction_IR_LEFT  = 0.0f;

    uart_send("MOVE STRAIGHT\n");

//    STABLE_IR_PID.last_error = 0;
//    STABLE_IR_PID.integral  = 0;
//    STABLE_IR_PID.last_time = timers_stopwatch();

    // ---------------- MAIN TRAVEL LOOP ----------------
    while ((avgEncoderCount - startCount) < targetCount) {
        remi(READWALLS);
        if ((walls & 0b0100) && (walls & 0b0010)) {
            frontCalibration();
            remi(HALT);
            return;   // stop immediately
        }

        avgEncoderCount = (rightEncoderCount + leftEncoderCount) * 0.5;

        // Gyro Feedback
        TEMP = core_calcAngles();
        currAngle = TEMP - (offsetAngles*0.5f);
        filteredAngle = (1.0f - alphaAngle) * filteredAngle + alphaAngle * currAngle;
        if (fabs(filteredAngle) > DEADBAND_G) {
            float rawGyroPidOut = pidController_compute(&STABLE_GYRO_PID, 0, filteredAngle);
            filteredPidOut = (1.0f - alphaPID) * filteredPidOut + alphaPID * rawGyroPidOut;
            correction_G = filteredPidOut;
        }
        if (fabs(filteredAngle) < 0.0015f) {
            offsetAngles = TEMP;
            filteredAngle = 0;
        }

        // IR Feedback With Both Walls
        if (IR_READINGS[0] < 3870) { correction_IR_RIGHT = 70.0f; }
        else { correction_IR_RIGHT = 0.0f; }

        if (IR_READINGS[3] < 3870) { correction_IR_LEFT = 70.0f; }
        else { correction_IR_LEFT = 0.0f; }

        float rightRPM = baseRPM + correction_G + correction_IR_RIGHT;
        float leftRPM  = baseRPM - correction_G + correction_IR_LEFT;

        core_MotorPidControl(&MR, FORWARD, rightRPM);
        core_MotorPidControl(&ML, FORWARD, leftRPM);
    }

    if (State == STOP) {
		// ---------------- POST-STABILIZATION LOOP ----------------
		alphaAngle = 1.0f;    // smoothing factor for angle
		alphaPID   = 0.001f;   // smoothing factor for PID output
		DEADBAND_G   = 0.9f;   // ignore small drift

		while (1) {
		TEMP = core_calcAngles();

		// Apply offset
		currAngle = TEMP - offsetAngles;

		// Low-pass filter (smooth drift + noise)
		filteredAngle = (1.0f - alphaAngle) * filteredAngle + alphaAngle * currAngle;

		if (filteredAngle > DEADBAND_G) {
			float rawPidOut = pidController_compute(&STABLE_GYRO_PID, 0, filteredAngle);

			// Low-pass filter PID output
			filteredPidOut = (1.0f - alphaPID) * filteredPidOut + alphaPID * rawPidOut;

			core_MotorPidControl(&MR, BACKWARD, filteredPidOut);
			core_MotorPidControl(&ML, FORWARD , filteredPidOut);
		} else if (filteredAngle < -DEADBAND_G) {
			float rawPidOut = pidController_compute(&STABLE_GYRO_PID, 0, filteredAngle);

			// Low-pass filter PID output
			filteredPidOut = (1.0f - alphaPID) * filteredPidOut + alphaPID * rawPidOut;

			core_MotorPidControl(&MR, FORWARD , filteredPidOut);
			core_MotorPidControl(&ML, BACKWARD, filteredPidOut);
		} else break;

		}
		// Stop motors once aligned
		remi(HALT);
    }
    uart_send("STRAIGHT DONE\n");
}

void spin(float targetAngle) {
    remi(READWALLS);
    if ((walls & 0b0100) || (walls & 0b0010)) {
    	walk(5.0f,200.0f,0);
    }

	resetPID();

	lastGyroTime = timers_stopwatch();
	spunAngles = 0.0f;

	float offsetAngles = fabs(core_calcAngles());
	float currAngle = 0;

	// Filters
	float filteredAngle  = 0.0f;
	float filteredPidOut = 0.0f;
	float alphaAngle = 0.5f;   // gyro smoothing
	float alphaPID   = 0.3f;   // correction smoothing

	MOTORDIR dir = (targetAngle > 0.0f) ? FORWARD : BACKWARD;
	targetAngle = fabs(targetAngle);

	do {
		currAngle = (fabs(core_calcAngles()) - offsetAngles);
		filteredAngle = (1.0f - alphaAngle) * filteredAngle + alphaAngle * currAngle;

		float rawPidOut = pidController_compute(&SPIN_PID, targetAngle, filteredAngle);
		filteredPidOut = (1.0f - alphaPID) * filteredPidOut + alphaPID * rawPidOut;

		core_MotorPidControl(&MR, dir , filteredPidOut); //uart_send("Right:%f\n",MR.CURR_RPM);
		core_MotorPidControl(&ML, !dir, filteredPidOut); //uart_send("Left:%f\n" ,ML.CURR_RPM);

		//uart_send("pidOut:%f\n", filteredPidOut);
		//uart_send("Angle:%f\n", filteredAngle);
	} while (currAngle < (targetAngle-3.5));
	stop();
	//backCalibration();
}
void stop() {
	motors_control(&MR, BACKWARD, 10000);
	motors_control(&ML, BACKWARD, 10000);
	systick_delayMillis(5);
	motors_control(&MR, STOP, 0);
	motors_control(&ML, STOP, 0);
	systick_delayMillis(500);
	resetCore();
}
uint8_t readWalls() {

	walls = 0;
	walls = (IR_READINGS[0] <= 4050) ? (walls | (1 << 0)) : (walls & ~(1 << 0));
	walls = (IR_READINGS[1] <= 4050) ? (walls | (1 << 1)) : (walls & ~(1 << 1));
	walls = (IR_READINGS[2] <= 4050) ? (walls | (1 << 2)) : (walls & ~(1 << 2));
	walls = (IR_READINGS[3] <= 4050) ? (walls | (1 << 3)) : (walls & ~(1 << 3));

	//uart_send("Sensor 1 = %d, Sensor 2 = %d, Sensor 3 = %d, Sensor 4 = %d\n",IR_READINGS[0],IR_READINGS[1],IR_READINGS[2],IR_READINGS[3]);

	gpio_writePin(rightLedPort, rightLedPin, (walls & 0b0001) ? 1 : 0);
	gpio_writePin(frontLedPort, frontLedPin, ((walls & 0b0100) && (walls & 0b0010)) ? 1 : 0);
	gpio_writePin(leftLedPort , leftLedPin , (walls & 0b1000) ? 1 : 0);

	return walls;
}

/* ========================================================================================================================
 * === HELPER CONTROL FUNCTIONS ===
 * ======================================================================================================================== */
void move(float targetDistance, float rpm) {
	//resetPID();
	float targetCount = targetDistance / DISPERCOUNT;// 18 -> 6,417
	uint32_t avgEncoderCount = (rightEncoderCount + leftEncoderCount) * 0.5;
	uint32_t startCount = avgEncoderCount;

	while ((avgEncoderCount - startCount) < targetCount) {
		avgEncoderCount = (rightEncoderCount + leftEncoderCount) * 0.5;
		core_MotorPidControl(&MR, FORWARD, rpm);
		core_MotorPidControl(&ML, FORWARD, rpm);
		/*  MOTORS RPM   */ uart_send("Right:%f\n",MR.CURR_RPM); uart_send("Left:%f\n" ,ML.CURR_RPM);
		/* ENCODER COUNT */ //uart_send("Right:%d\n",rightEncoderCount); uart_send("Left:%d\n" ,leftEncoderCount);
	}
}
void still() {
    lastGyroTime = timers_stopwatch();
    spunAngles = 0.0f;
    float offsetAngles = core_calcAngles();
    float currAngle = 0, TEMP = 0;

    // --- Low-pass filter state ---
    float filteredAngle = 0.0f;
    float filteredPidOut = 0.0f;

    // --- Filter & deadband constants ---
    float alphaAngle = 0.8f;   // smoothing factor for angle
    float alphaPID   = 0.3f;   // smoothing factor for PID output
    float DEADBAND   = 0.1f;   // ignore small drift

    uart_send("STILL\n");

    while (1) {
        TEMP = core_calcAngles();

        // Apply offset
        currAngle = TEMP - offsetAngles;

        // Low-pass filter (smooth drift + noise)
        filteredAngle = (1.0f - alphaAngle) * filteredAngle + alphaAngle * currAngle;

        if (filteredAngle > DEADBAND) {
            float rawPidOut = pidController_compute(&SPIN_PID, 0, filteredAngle);

            // Low-pass filter PID output
            filteredPidOut = (1.0f - alphaPID) * filteredPidOut + alphaPID * rawPidOut;

            core_MotorPidControl(&MR, BACKWARD, filteredPidOut);
            core_MotorPidControl(&ML, FORWARD , filteredPidOut);
        }
        else if (filteredAngle < -DEADBAND) {
            float rawPidOut = pidController_compute(&SPIN_PID, 0, filteredAngle);

            // Low-pass filter PID output
            filteredPidOut = (1.0f - alphaPID) * filteredPidOut + alphaPID * rawPidOut;

            core_MotorPidControl(&MR, FORWARD , filteredPidOut);
            core_MotorPidControl(&ML, BACKWARD, filteredPidOut);
        }
        else {
            break;
        }
    }
}
void frontCalibration() {
    // Target ADC values (tune closer/further here)
    const uint16_t DEADBAND     = 40;     // acceptable error

    // Low-pass filter states
    float pidOutL_filt = 0;
    float pidOutR_filt = 0;
    const float alpha = 0.6f; // smoothing factor (0..1)

    uart_send("FRONT CALIBRATION START\n");

    uint8_t rightDone = 0;
    uint8_t leftDone  = 0;

    while(1) {
        readWalls(); // DMA updated

        // --- Left front sensor (IR_READINGS[2]) ---
        if (walls & 0b0100) {
            int errorL = IR_READINGS[2] - LEFT_GOAL;  // far = positive
            if (fabs(errorL) <= DEADBAND) {
                core_MotorPidControl(&ML, STOP, 0);
                leftDone = 1;
            } else {
                float pidOutRaw = pidController_compute(&IR_PID, LEFT_GOAL, IR_READINGS[2]);
                pidOutL_filt = alpha * pidOutRaw + (1 - alpha) * pidOutL_filt;

                if (errorL > 0) core_MotorPidControl(&ML, FORWARD , pidOutL_filt);  // too far → forward
                else            core_MotorPidControl(&ML, BACKWARD, pidOutL_filt); // too close → back off
            }
        } else {
            core_MotorPidControl(&ML, STOP, 0);
        }

        // --- Right front sensor (IR_READINGS[1]) ---
        if (walls & 0b0010) {
            int errorR = IR_READINGS[1] - RIGHT_GOAL;
            if (fabs(errorR) <= DEADBAND) {
                core_MotorPidControl(&MR, STOP, 0);
                rightDone = 1;
            } else {
                float pidOutRaw = pidController_compute(&IR_PID, RIGHT_GOAL, IR_READINGS[1]);
                pidOutR_filt = alpha * pidOutRaw + (1 - alpha) * pidOutR_filt;

                if (errorR > 0) core_MotorPidControl(&MR, FORWARD, pidOutR_filt);  // too far → forward
                else            core_MotorPidControl(&MR, BACKWARD, pidOutR_filt); // too close → back off
            }
        } else {
            core_MotorPidControl(&MR, STOP, 0);
        }


        if (rightDone && leftDone) { break; }
    }
}

void backCalibration(ACTIONS x) {
	motors_control(&MR, BACKWARD, 4000);
	motors_control(&ML, BACKWARD, 4000);
	systick_delayMillis(1000);
	stop();
	systick_delayMillis(500);
	walk( 45.0f, 200.0f, x);
}
void resetPID() {
	// PID integrals + histories
	RIGHT_SPEED_PID.integral = 0;
	RIGHT_SPEED_PID.last_error = 0;
	RIGHT_SPEED_PID.last_time = timers_stopwatch();

	LEFT_SPEED_PID.integral = 0;
	LEFT_SPEED_PID.last_error = 0;
	LEFT_SPEED_PID.last_time = timers_stopwatch();

	SPIN_PID.integral  = 0;
	SPIN_PID.last_error = 0;
	SPIN_PID.last_time = timers_stopwatch();

	IR_PID.integral  = 0;
	IR_PID.last_error = 0;
	IR_PID.last_time = timers_stopwatch();

}
void resetCore() {
    // Encoders
    resetEncoders();
    lastSampleTime[0] = timers_stopwatch();
    lastSampleTime[1] = timers_stopwatch();
    lastCount[0] = 0;
    lastCount[1] = 0;

    // RPM
    MR.CURR_RPM  = 0;
    ML.CURR_RPM  = 0;

    // Gyro
    spunAngles   = 0.0f;

    //PID
    resetPID();
}
