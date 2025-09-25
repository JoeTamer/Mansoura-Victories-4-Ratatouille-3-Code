/******************************************************
 * File: gpio.c
 * Description: Implementation of GPIO functions for STM32F103.
 *
 * Author: Jo
 * Date: ~
 ******************************************************/

#include "peripherals/peripheralsHeaderFiles/gpio.h"

/* ========================================================================================================================
 *                                                     PIN SETUP AND CONTROL
 * ======================================================================================================================== */

/**
 * @brief Configures a GPIO pin with mode and configuration.
 *
 * Enables the required GPIO clock and sets the pin's mode and configuration.
 * If Alternate Function is used, AFIO clock is enabled.
 *
 * @param GPIOx Pointer to GPIO port (GPIOA, GPIOB, GPIOC, GPIOD)
 * @param pin Pin number (0–15)
 * @param mode Pin speed/mode (IN, OUT_10, OUT_2, OUT_50)
 * @param conf Pin configuration (GP_PP, GP_OD, AF_PP, AF_OD, ANALOG, FLOAT, PULLUP)
 */
void gpio_setupPin(GPIO_TypeDef* GPIOx, uint8_t pin, MODE mode, CONFIGURATION conf) {
	/* (RM0008 / 112) Enable clock for GPIO port */
	RCC->APB2ENR |= (1 << (((uint32_t)(GPIOx) - (uint32_t)GPIOA) / 0x400 + 2));
	if ((conf == AF_PP) || (conf == AF_OD))
		RCC->APB2ENR |= (1 << 0); // Bit 0 = AFIOEN

	/* (RM0008 / 171) Configure pin mode and function */
	if (pin <= 7) {
		GPIOx->CRL &= ~((0xF) << (pin * 4));
		GPIOx->CRL |= ((mode | (conf << 2)) << (pin * 4));
	} else {
		GPIOx->CRH &= ~((0xF) << ((pin - 8) * 4));
		GPIOx->CRH |= ((mode | (conf << 2)) << ((pin - 8) * 4));
	}
	gpio_writePin(GPIOx, pin, LOW);
}

/**
 * @brief Writes a state to a GPIO pin.
 *
 * Sets, clears, or toggles a pin atomically using BSRR/BRR/ODR registers.
 *
 * @param GPIOx Pointer to GPIO port
 * @param pin Pin number (0–15)
 * @param pinSTATE State to write (LOW, HIGH, TOGGLE)
 */
void gpio_writePin(GPIO_TypeDef* GPIOx, uint8_t pin, PINSTATE pinSTATE) {
	/* (RM0008 / 173–174) Atomic write to pin */
	switch (pinSTATE) {
		case LOW:    GPIOx->BRR  = (1 << pin); break;
		case HIGH:   GPIOx->BSRR = (1 << pin); break;
		case TOGGLE: GPIOx->ODR ^= (1 << pin); break;
	}
}

/**
 * @brief Reads the current state of a GPIO pin.
 *
 * @param GPIOx Pointer to GPIO port
 * @param pin Pin number (0–15)
 * @return uint8_t 1 if pin is HIGH, 0 if LOW
 */
uint8_t gpio_readPin(GPIO_TypeDef* GPIOx, uint8_t pin) {
	/* (RM0008 / 172) Read pin state */
	return (GPIOx->IDR & (1 << pin)) ? 1 : 0;
}
