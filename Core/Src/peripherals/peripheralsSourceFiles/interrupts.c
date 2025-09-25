/******************************************************
 * File: interrupts.c
 * Description: Implementation of INTERRUPT functions for STM32F103.
 *
 * Author: Jo
 * Date: ~
 ******************************************************/

#include "peripherals/peripheralsHeaderFiles/interrupts.h"

/* ========================================================================================================================
 *                                                     CALLBACK ARRAY
 * ======================================================================================================================== */

void (*interrupts_exti_callbacks[16])(void) = {0};  // Array of function pointers for each EXTI line

/* ========================================================================================================================
 *                                               INTERUPPTS CONTROL FUNCTIONS
 * ======================================================================================================================== */

/**
 * @brief Attaches an interrupt to a GPIO pin.
 *
 * Configures the pin for input, sets the EXTI line and NVIC interrupt.
 *
 * @param GPIOx Pointer to the GPIO port
 * @param pin Pin number (0–15)
 * @param edgeSTATE Type of edge to trigger the interrupt
 * @param function Pointer to callback function
 */
void interrupts_attach(GPIO_TypeDef* GPIOx, uint8_t pin, EDGESTATE edgeSTATE, void (*function)(void)) {
	__disable_irq();

	gpio_setupPin(GPIOx, pin, IN, PULLUP);

	AFIO->EXTICR[pin / 4] &= ~(0xF << ((pin % 4) * 4));
	AFIO->EXTICR[pin / 4] |= ((((uint32_t)(GPIOx) - (uint32_t)GPIOA) / 0x400) << ((pin % 4) * 4));

	EXTI->IMR &= ~(1 << pin);  // Mask before config

	switch (edgeSTATE) {
		case RISING:  EXTI->FTSR &= ~(1 << pin); EXTI->RTSR |=  (1 << pin); break;
		case FALLING: EXTI->RTSR &= ~(1 << pin); EXTI->FTSR |=  (1 << pin); break;
		case CHANGE:  EXTI->FTSR |=  (1 << pin); EXTI->RTSR |=  (1 << pin); break;
	}

	EXTI->IMR |= (1 << pin);  // Enable interrupt

	if      (pin <= 4) NVIC_EnableIRQ(EXTI0_IRQn + pin);
	else if (pin <= 9) NVIC_EnableIRQ(EXTI9_5_IRQn);
	else               NVIC_EnableIRQ(EXTI15_10_IRQn);

	interrupts_exti_callbacks[pin] = function;

	__enable_irq();
}

/**
 * @brief Detaches the interrupt from a GPIO pin.
 *
 * Clears EXTI settings and disables the callback.
 *
 * @param pin Pin number to detach (0–15)
 */
void interrupts_detach(uint8_t pin) {
	EXTI->IMR  &= ~(1 << pin);
	EXTI->RTSR &= ~(1 << pin);
	EXTI->FTSR &= ~(1 << pin);
	interrupts_exti_callbacks[pin] = 0;
}

/* ========================================================================================================================
 *                                                      IRQ-HANDLERS
 * ======================================================================================================================== */

/**
 * @brief EXTI0 interrupt handler
 */
void EXTI0_IRQHandler(void) {
	if (EXTI->PR & (1 << 0)) {
		EXTI->PR = (1 << 0);
		if (interrupts_exti_callbacks[0]) interrupts_exti_callbacks[0]();
	}
}

/**
 * @brief EXTI1 interrupt handler
 */
void EXTI1_IRQHandler(void) {
	if (EXTI->PR & (1 << 1)) {
		EXTI->PR = (1 << 1);
		if (interrupts_exti_callbacks[1]) interrupts_exti_callbacks[1]();
	}
}

/**
 * @brief EXTI2 interrupt handler
 */
void EXTI2_IRQHandler(void) {
	if (EXTI->PR & (1 << 2)) {
		EXTI->PR = (1 << 2);
		if (interrupts_exti_callbacks[2]) interrupts_exti_callbacks[2]();
	}
}

/**
 * @brief EXTI3 interrupt handler
 */
void EXTI3_IRQHandler(void) {
	if (EXTI->PR & (1 << 3)) {
		EXTI->PR = (1 << 3);
		if (interrupts_exti_callbacks[3]) interrupts_exti_callbacks[3]();
	}
}

/**
 * @brief EXTI4 interrupt handler
 */
void EXTI4_IRQHandler(void) {
	if (EXTI->PR & (1 << 4)) {
		EXTI->PR = (1 << 4);
		if (interrupts_exti_callbacks[4]) interrupts_exti_callbacks[4]();
	}
}

/**
 * @brief Shared EXTI5–EXTI9 interrupt handler
 */
void EXTI9_5_IRQHandler(void) {
	for (uint8_t i = 5; i <= 9; i++) {
		if (EXTI->PR & (1 << i)) {
			EXTI->PR = (1 << i);
			if (interrupts_exti_callbacks[i]) interrupts_exti_callbacks[i]();
		}
	}
}

/**
 * @brief Shared EXTI10–EXTI15 interrupt handler
 */
void EXTI15_10_IRQHandler(void) {
	for (uint8_t i = 10; i <= 15; i++) {
		if (EXTI->PR & (1 << i)) {
			EXTI->PR = (1 << i);
			if (interrupts_exti_callbacks[i]) interrupts_exti_callbacks[i]();
		}
	}
}
