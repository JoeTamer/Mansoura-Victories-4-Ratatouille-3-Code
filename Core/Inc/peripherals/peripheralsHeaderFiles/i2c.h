/**
 * @file i2c.h
 * @brief Header for STM32 I2C control (Remi Micromouse Project)
 *
 * Declares I2C initialization and communication functions for master mode
 * on STM32F103 using I2C1/I2C2 with blocking transfers.
 */

#ifndef INC_PERIPHERALS_PERIPHERALSHEADERFILES_I2C_H_
#define INC_PERIPHERALS_PERIPHERALSHEADERFILES_I2C_H_

#include <stm32f103xb.h>
#include <stdint.h>
#include <stddef.h>
#include <stdarg.h>
#include "gpio.h"
#include "systick.h"

/**
 * @def FM SM
 * @brief Fast Mode (400kHz) I2C clock speed setting.
 * @brief Standard Mode (100kHz) I2C clock speed setting.
 */
#define FM 0x2D
#define SM 0xB4

/**
 * @def FM SM
 * @brief Timeout in ms to Peripheral doesnt get stuck
 */

#define i2c_timeout 50

/**
 * @enum RWStatus
 * @brief Specifies the direction of data transfer.
 */
typedef enum { WRITE, READ } RWSTATUS;

/**
 * @enum ACKSTATUS
 * @brief Represents the acknowledgment status of a communication operation.
 */
typedef enum { NACK, ACK } ACKSTATUS;

/*
 * I2C Pin Mapping:
 *   PB7  -> SDA1
 *   PB6  -> SCL1
 *   PB11 -> SDA2
 *   PB10 -> SCL2
 */

/* ========================================================================================================================
 * === I2C SETUP FUNCTION ===
 * ======================================================================================================================== */

void i2c_setup(I2C_TypeDef* I2Cx, uint8_t speedMode, ...);

/* ========================================================================================================================
 * === BLOCK FUNCTIONS ===
 * ======================================================================================================================== */

uint8_t i2c_start(I2C_TypeDef* I2Cx);
uint8_t i2c_address(I2C_TypeDef* I2Cx, uint8_t address, RWSTATUS RW);
void i2c_sendData(I2C_TypeDef* I2Cx, uint8_t data);
void i2c_receiveData(I2C_TypeDef* I2Cx, ACKSTATUS ACK_NACK, uint8_t* data);
void i2c_stop(I2C_TypeDef* I2Cx);

/* ========================================================================================================================
 * === CONTROL FUNCTIONS ===
 * ======================================================================================================================== */

uint8_t i2c_scan(I2C_TypeDef* I2Cx);
void i2c_write(I2C_TypeDef* I2Cx, uint8_t address, uint8_t* data, uint16_t length);
void i2c_read(I2C_TypeDef* I2Cx, uint8_t address, uint8_t* buffer, uint16_t length);

#endif /* INC_PERIPHERALS_PERIPHERALSHEADERFILES_I2C_H_ */
