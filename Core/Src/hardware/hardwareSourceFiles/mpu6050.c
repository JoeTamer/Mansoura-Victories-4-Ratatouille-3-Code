#include "hardware/hardwareHeaderFiles/mpu6050.h"
//#define DEBUG_MODE

/* ========================================================================================================================
 *                                                    MPU SETUP FUNCTION
 * ======================================================================================================================== */

void mpu6050_setup(MPU6050* myMPU, uint8_t mode, uint8_t dlpf, uint8_t config, uint8_t altPins) {
	i2c_setup(myMPU->I2Cx, mode, altPins);
	// Apply MPU6050 configuration from struct
	mpu6050_tx(myMPU, 0x6B, 0x80);   // Reset
	systick_delayMillis(100);
	mpu6050_tx(myMPU, 0x6B, 0x00);   // Wake up
	mpu6050_tx(myMPU, 0x1A, dlpf);   // Set DLPF
	mpu6050_tx(myMPU, 0x1B, config); // Set gyro config

	switch(config) {
	case 0x00: myMPU->sensitivity = 131.0f; break; // ±250 °/s
	case 0x08: myMPU->sensitivity = 65.5f;  break; // ±500 °/s
	case 0x10: myMPU->sensitivity = 32.8f;  break; // ±1000 °/s
	case 0x18: myMPU->sensitivity = 16.4f;  break; // ±2000 °/s
	default:   myMPU->sensitivity = 131.0f; break;
	}
	#ifdef DEBUG_MODE
	uart_send("SUCCESS : MPU SETUP \n");
	#endif
}

/* ========================================================================================================================
 *                                                MPU BLOCK FUNCTIONS
 * ======================================================================================================================== */

void mpu6050_start(MPU6050* myMPU) {
	if (!(i2c_start(myMPU->I2Cx))) {
		#ifdef DEBUG_MODE
		uart_send("SUCCESS : MPU START \n");
	    #endif
	} else {
		#ifdef DEBUG_MODE
		uart_send("FAILURE : MPU START \n");
	    #endif
	}
}
void mpu6050_address(MPU6050* myMPU, uint8_t RW) {
	if (!(i2c_address(myMPU->I2Cx, myMPU->address, RW))) {
		#ifdef DEBUG_MODE
		uart_send("SUCCESS : MPU ADDRESS \n");
		#endif
	} else {
		#ifdef DEBUG_MODE
		uart_send("FAILURE : MPU SETUP \n");
		#endif
	}
}

/* ========================================================================================================================
 *                                                 MPU COMMS FUNCTIONS
 * ======================================================================================================================== */

void mpu6050_tx(MPU6050* myMPU, uint8_t reg, uint8_t data) {
	mpu6050_start(myMPU);
    mpu6050_address(myMPU, WRITE);
    i2c_sendData(myMPU->I2Cx, reg);
    i2c_sendData(myMPU->I2Cx, data);
    i2c_stop(myMPU->I2Cx);
}
void mpu6050_rx(MPU6050* myMPU, uint8_t reg, uint8_t* data, uint8_t dataLen) {
    uint16_t timeout;

    // 1. START + WRITE to select register
    mpu6050_start(myMPU);
    mpu6050_address(myMPU, WRITE);
    i2c_sendData(myMPU->I2Cx, reg);

    // 2. RESTART + READ
    mpu6050_start(myMPU);
    mpu6050_address(myMPU, READ);

    for (uint8_t i = 0; i < dataLen; i++) {
        // Set ACK/NACK before reading the byte
        if (i == dataLen - 1) { myMPU->I2Cx->CR1 &= ~(1 << 10); // NACK on last byte
        } else                { myMPU->I2Cx->CR1 |= (1 << 10);  // ACK for previous bytes
        }

        // Wait for RXNE with timeout
        timeout = i2c_timeout;
        while (!(myMPU->I2Cx->SR1 & (1 << 6)) && timeout--) systick_delayMillis(1);
        if (timeout == 0) { i2c_stop(myMPU->I2Cx); return; }

        // Read data
        data[i] = myMPU->I2Cx->DR;

        #ifdef DEBUG_MODE
        uart_send("READING...\n");
        #endif
    }

    // Generate STOP after last byte is read
    myMPU->I2Cx->CR1 |= (1 << 9); // STOP
}


/* ========================================================================================================================
 *                                                 MPU READINGS FUNCTION
 * ======================================================================================================================== */

void mpu6050_readGyro(MPU6050* myMPU, float* gx, float* gy, float* gz) {
	static uint8_t rawData[6];

	#ifdef DEBUG_MODE
	uart_send("GETTING GYROSCOPE DATA !!! \n");
	#endif
	mpu6050_rx(myMPU, 0x43, rawData, 6);

	#ifdef DEBUG_MODE
	uart_send("GYROSCOPE DATA AQUIRED !!! \n");
	for (uint8_t i = 0; i < 6; i++) uart_send("%d : ",*(rawData+i));
	#endif

	int16_t gyroX = (int16_t)((int16_t)rawData[0] << 8 | (int16_t)rawData[1]);
	int16_t gyroY = (int16_t)((int16_t)rawData[2] << 8 | (int16_t)rawData[3]);
	int16_t gyroZ = (int16_t)((int16_t)rawData[4] << 8 | (int16_t)rawData[5]);

	*gx = (float)gyroX / myMPU->sensitivity;
	*gy = (float)gyroY / myMPU->sensitivity;
	*gz = (float)gyroZ / myMPU->sensitivity;
}
