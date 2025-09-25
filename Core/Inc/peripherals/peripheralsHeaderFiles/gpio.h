/**
 * @file gpio.h
 * @brief Header for STM32 GPIO control (Remi Micromouse Project)
 *
 * Declares functions for configuring, writing to, and reading from
 * GPIO pins on the STM32F103 microcontroller.
 */

#ifndef INC_PERIPHERALS_PERIPHERALSHEADERFILES_GPIO_H_
#define INC_PERIPHERALS_PERIPHERALSHEADERFILES_GPIO_H_

#include <stm32f103xb.h>
#include <stdint.h>
#include <stddef.h>

/**
 * @enum MODE
 * @brief GPIO output speed or input mode.
 */
typedef enum { IN, OUT_10, OUT_2, OUT_50 } MODE;

/**
 * @enum CONFIGURATION
 * @brief GPIO pin configuration (push-pull, open-drain, etc.).
 */
typedef enum {
	GP_PP, GP_OD, AF_PP, AF_OD,
	ANALOG = 0, FLOAT, PULLUP
} CONFIGURATION;

/**
 * @enum PINSTATE
 * @brief Logical state to write to a GPIO pin.
 */
typedef enum { LOW, HIGH, TOGGLE } PINSTATE;

/* ========================================================================================================================
 * === PIN SETUP AND CONTROL ===
 * ======================================================================================================================== */

void gpio_setupPin(GPIO_TypeDef* GPIOx, uint8_t pin, MODE mode, CONFIGURATION conf);
void gpio_writePin(GPIO_TypeDef* GPIOx, uint8_t pin, PINSTATE pinSTATE);
uint8_t gpio_readPin(GPIO_TypeDef* GPIOx, uint8_t pin);

#endif /* INC_PERIPHERALS_PERIPHERALSHEADERFILES_GPIO_H_ */
