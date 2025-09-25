/******************************************************
 * File: i2c.c
 * Description: Implementation of I2C functions for STM32F103.
 *
 * Author: Jo
 * Date: ~
 ******************************************************/

#include "peripherals/peripheralsHeaderFiles/i2c.h"

/* ========================================================================================================================
 *                                                    I2C SETUP FUNCTION
 * ======================================================================================================================== */

/**
 * @brief Setup I2C peripheral with optional remap for I2C1
 *
 * @param I2Cx        Pointer to I2C peripheral (I2C1 or I2C2)
 * @param speedMode   SM or FM
 * @param alternatePins Only used for I2C1: 0 = default PB6/PB7, 1 = remapped PB8/PB9
 */
void i2c_setup(I2C_TypeDef* I2Cx, uint8_t speedMode, ...) {
    uint8_t i2c = (((uint32_t)I2Cx - (uint32_t)I2C1) / 0x400) + 1;
    uint8_t alternatePins = 0; // default

    if (i2c == 1) {
        va_list args;
        va_start(args, speedMode);
        alternatePins = (uint8_t)va_arg(args, int); // grab extra arg
        va_end(args);
    }

    // Enable clock
    RCC->APB1ENR |= (1 << (20 + i2c));

    if (i2c == 1) {
        RCC->APB2ENR |= (1 << 0); // AFIO clock

        if (alternatePins == 0) {
            AFIO->MAPR &= ~(1 << 8); // default pins
            gpio_setupPin(GPIOB, 6, OUT_50, AF_OD);
            gpio_setupPin(GPIOB, 7, OUT_50, AF_OD);
        } else if (alternatePins == 1) {
            AFIO->MAPR |= (1 << 8); // remapped
            gpio_setupPin(GPIOB, 8, OUT_50, AF_OD);
            gpio_setupPin(GPIOB, 9, OUT_50, AF_OD);
        }
    } else if (i2c == 2) {
        gpio_setupPin(GPIOB, 10, OUT_50, AF_OD);
        gpio_setupPin(GPIOB, 11, OUT_50, AF_OD);
    }

    // Software reset
    I2Cx->CR1 |= 0x8000;
    I2Cx->CR1 &= ~0x8000;

    // Configure speed
    I2Cx->CR2 = 36;
    I2Cx->CCR = speedMode;
    if (speedMode == SM) I2Cx->TRISE = 37;
    else if (speedMode == FM) { I2Cx->CCR |= (1 << 15); I2Cx->TRISE = 12; }

    I2Cx->CR1 |= (1 << 0); // Enable I2C
}

/* ========================================================================================================================
 *                                                   BLOCK FUNCTIONS
 * ======================================================================================================================== */

/**
 * @brief Sends a START condition.
 *
 * @param I2Cx Pointer to I2C peripheral
 * @return 0 if success, 1 if timeout
 */
uint8_t i2c_start(I2C_TypeDef* I2Cx) {
	uint16_t timeout = i2c_timeout;
	I2Cx->CR1 |= (1 << 8);
	while (!(I2Cx->SR1 & (1 << 0)) && timeout--) systick_delayMillis(1);
	return (I2Cx->SR1 & (1 << 0)) ? 0 : 1;
}

/**
 * @brief Sends the I2C address and R/W bit.
 *
 * @param I2Cx I2C peripheral
 * @param address 7-bit slave address
 * @param RW READ or WRITE
 * @return 0 if ACK received, 1 if NACK or timeout
 */
uint8_t i2c_address(I2C_TypeDef* I2Cx, uint8_t address, RWSTATUS RW) {
	I2Cx->DR = ((address << 1) | RW);

	uint16_t timeout = i2c_timeout;
	while (!(I2Cx->SR1 & (1 << 1)) && timeout--) systick_delayMillis(1);
	if (!(I2Cx->SR1 & (1 << 1))) {
		I2Cx->SR1 &= ~(1 << 10); // Clear AF
		return 1;
	}

	volatile uint32_t temp;
	temp = I2Cx->SR1;
	temp = I2Cx->SR2;
	(void)temp;

	return 0;
}

/**
 * @brief Sends one byte of data.
 *
 * @param I2Cx I2C peripheral
 * @param data Byte to send
 */
void i2c_sendData(I2C_TypeDef* I2Cx, uint8_t data) {
	while (!(I2Cx->SR1 & (1 << 7)));
	I2Cx->DR = data;
	while (!(I2Cx->SR1 & (1 << 7)));
}

/**
 * @brief Receives one byte from slave.
 *
 * @param I2Cx I2C peripheral
 * @param ACK_NACK Send ACK (1) or NACK (0)
 * @param data Pointer to store received byte
 */
void i2c_receiveData(I2C_TypeDef* I2Cx, ACKSTATUS ACK_NACK, uint8_t* data) {
	if (ACK_NACK == ACK) {
		I2Cx->CR1 |= (1 << 10);
	} else {
		I2Cx->CR1 &= ~(1 << 10);
	}

	while (!(I2Cx->SR1 & (1 << 6)));

	if (ACK_NACK == NACK) {
		I2Cx->CR1 |= (1 << 9); // STOP
	}

	*data = I2Cx->DR;
}

/**
 * @brief Sends STOP condition and ensures bus is released.
 *
 * @param I2Cx I2C peripheral
 */
void i2c_stop(I2C_TypeDef* I2Cx) {
	volatile uint32_t temp;
	while (I2Cx->SR1 & (1 << 1)) {
		temp = I2Cx->SR1;
		temp = I2Cx->SR2;
	}
	(void)temp;

	I2Cx->CR1 |= (1 << 9);
}

/* ========================================================================================================================
 *                                                   CONTROL FUNCTIONS
 * ======================================================================================================================== */

/**
 * @brief Scans for the first available I2C device.
 *
 * @param I2Cx I2C peripheral
 * @return Address of found device or 0 if none found
 */
uint8_t i2c_scan(I2C_TypeDef* I2Cx) {
	for (uint8_t addr = 1; addr < 127; addr++) {
		if (i2c_start(I2Cx)) continue;
		uint8_t res = i2c_address(I2Cx, addr, WRITE);
		i2c_stop(I2Cx);
		if (res == 0) return addr;
	}
	return 0;
}

/**
 * @brief Writes multiple bytes to a slave device.
 *
 * @param I2Cx I2C peripheral
 * @param address Slave address
 * @param data Pointer to data array
 * @param length Number of bytes to write
 */
void i2c_write(I2C_TypeDef* I2Cx, uint8_t address, uint8_t* data, uint16_t length) {
	if (!i2c_start(I2Cx)) {
		i2c_address(I2Cx, address, WRITE);
		for (uint16_t i = 0; i < length; i++) {
			i2c_sendData(I2Cx, data[i]);
		}
	}
	i2c_stop(I2Cx);
}

/**
 * @brief Reads multiple bytes from a slave device.
 *
 * @param I2Cx I2C peripheral
 * @param address Slave address
 * @param buffer Pointer to store received data
 * @param length Number of bytes to read
 */
void i2c_read(I2C_TypeDef* I2Cx, uint8_t address, uint8_t* buffer, uint16_t length) {
	if (!i2c_start(I2Cx)) {
		i2c_address(I2Cx, address, READ);
		for (uint16_t i = 0; i < length; i++) {
			uint8_t ack = (i < length - 1) ? ACK : NACK;
			i2c_receiveData(I2Cx, ack, &buffer[i]);
		}
	}
	i2c_stop(I2Cx);
}

