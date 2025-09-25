#ifndef INC_HARDWARE_HARDWAREHEADERFILES_MPU6050_H_
#define INC_HARDWARE_HARDWAREHEADERFILES_MPU6050_H_

#include "peripherals/peripheralsHeaderFiles/i2c.h"
#include "peripherals/peripheralsHeaderFiles/uart.h"

typedef struct {
	I2C_TypeDef* I2Cx;
	uint8_t address;
	float sensitivity;
} MPU6050;

/* ========================================================================================================================
 * === MPU SETUP FUNCTION ===
 * ======================================================================================================================== */

void mpu6050_setup(MPU6050* myMPU, uint8_t mode, uint8_t dlpf, uint8_t config, uint8_t altPins);

/* ========================================================================================================================
 * === MPU BLOCK FUNCTIONS ===
 * ======================================================================================================================== */

void mpu6050_start(MPU6050* myMPU);
void mpu6050_address(MPU6050* myMPU, uint8_t RW);

/* ========================================================================================================================
 * === MPU COMMS FUNCTIONS ===
 * ======================================================================================================================== */

void mpu6050_tx(MPU6050* myMPU, uint8_t reg, uint8_t data);
void mpu6050_rx(MPU6050* myMPU, uint8_t reg, uint8_t* data, uint8_t dataLen);

/* ========================================================================================================================
 * === MPU READINGS FUNCTION ===
 * ======================================================================================================================== */

void mpu6050_readGyro(MPU6050* myMPU, float* gx, float* gy, float* gz);

#endif /* INC_HARDWARE_HARDWAREHEADERFILES_MPU6050_H_ */
