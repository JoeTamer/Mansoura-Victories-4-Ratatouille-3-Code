/**
 * @file interrupts.h
 * @brief Header for STM32 EXTIs (Remi Micromouse Project)
 *
 * Provides functions to attach and detach interrupt handlers
 * to GPIO pins using STM32F103 EXTI lines.
 */

#ifndef INC_PERIPHERALS_PERIPHERALSHEADERFILES_INTERRUPTS_H_
#define INC_PERIPHERALS_PERIPHERALSHEADERFILES_INTERRUPTS_H_

#include <stm32f103xb.h>
#include <stdint.h>
#include <stddef.h>
#include "gpio.h"

/**
 * @enum EDGESTATE
 * @brief Enum for configuring the edge type that triggers the interrupt.
 */
typedef enum {
    RISING,   /**< Trigger on rising edge */
    FALLING,  /**< Trigger on falling edge */
    CHANGE    /**< Trigger on both rising and falling edges */
} EDGESTATE;

/* ========================================================================================================================
 * === INTERUPPTS CONTROL FUNCTIONS ===
 * ======================================================================================================================== */

void interrupts_attach(GPIO_TypeDef* GPIOx, uint8_t pin, EDGESTATE edgeSTATE, void (*function)(void));
void interrupts_detach(uint8_t pin);

#endif /* INC_PERIPHERALS_PERIPHERALSHEADERFILES_INTERRUPTS_H_ */
