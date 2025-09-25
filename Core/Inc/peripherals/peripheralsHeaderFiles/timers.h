/**
 * @file timers.h
 * @brief Header for STM32 advanced timer control (Remi Micromouse Project)
 *
 * Provides setup, delay, PWM, compare, and interrupt capabilities
 * for STM32 general-purpose timers.
 */

#ifndef INC_PERIPHERALS_PERIPHERALSHEADERFILES_TIMERS_H_
#define INC_PERIPHERALS_PERIPHERALSHEADERFILES_TIMERS_H_

#include <stm32f103xb.h>
#include <stdint.h>
#include <stddef.h>
#include "gpio.h"

#define F_CPU 72000000UL

/**
 * @brief Base time unit for timer operations.
 */
typedef enum {
    MICROS, /**< Microsecond base time */
    MILLIS  /**< Millisecond base time */
} TimeBase;

/**
 * @brief Timer channel pinout reference
 *
 * PA8  -> TIM1_CH1   | PA9  -> TIM1_CH2   | PA10 -> TIM1_CH3  | PA11 -> TIM1_CH4
 * PA0  -> TIM2_CH1   | PA1  -> TIM2_CH2   | PA2  -> TIM2_CH3  | PA3  -> TIM2_CH4
 * PA6  -> TIM3_CH1   | PA7  -> TIM3_CH2   | PB0  -> TIM3_CH3  | PB1  -> TIM3_CH4
 */

/* ========================================================================================================================
 * === TIMERS SETUP FUNCTION ===
 * ======================================================================================================================== */

void timers_setup(TIM_TypeDef* TIMx, TimeBase unit);

/* ========================================================================================================================
 * === BLOCKING DELAYS FUNCTION ===
 * ======================================================================================================================== */

void timers_delayMicros(TIM_TypeDef* TIMx, uint16_t del);
void timers_delayMillis(TIM_TypeDef* TIMx, uint16_t del);

/* ========================================================================================================================
 * === NON-BLOCKING FUNCTIONS ===
 * ======================================================================================================================== */

void timers_start(TIM_TypeDef* TIMx, uint16_t del, TimeBase unit);
void timers_stop(TIM_TypeDef* TIMx);
uint32_t timers_getElapsed(TIM_TypeDef* TIMx, TimeBase unit);

/* ========================================================================================================================
 * === INTERUPPT-BASED FUNCTIONS ===
 * ======================================================================================================================== */

void timers_irqStart(TIM_TypeDef* TIMx, void (*function)(void));
void timers_irqStop(TIM_TypeDef* TIMx);
void timers_irqRFlag(TIM_TypeDef* TIMx);
void timers_irqAttach(uint8_t timer, void (*function)(void));

/* ========================================================================================================================
 * === HELPER FUNCTIONS ===
 * ======================================================================================================================== */

uint8_t timers_getChannel(GPIO_TypeDef* GPIOx, uint8_t pin);
TIM_TypeDef* timers_getTimer(GPIO_TypeDef* GPIOx, uint8_t pin);
uint8_t timers_getNum(TIM_TypeDef* TIMx);

/* ========================================================================================================================
 * === COMPARE FUNCTION ===
 * ======================================================================================================================== */

void timers_compare(GPIO_TypeDef* GPIOx, uint8_t pin, TimeBase unit, uint16_t arr, uint16_t compare);

/* ========================================================================================================================
 * === PWM FUNCTION ===
 * ======================================================================================================================== */

void timers_pwmSetup(GPIO_TypeDef* GPIOx, uint8_t pin, uint32_t freq);
void timers_pwm(GPIO_TypeDef* GPIOx, uint8_t pin, uint32_t compare);

/* ========================================================================================================================
 * === STOPWATCH FUNCTIONS ===
 * ======================================================================================================================== */

void timers_ovfHandler();
void timers_stopwatchSetup(TIM_TypeDef* TIMx);
uint32_t timers_stopwatch();

#endif /* INC_PERIPHERALS_PERIPHERALSHEADERFILES_TIMERS_H_ */
