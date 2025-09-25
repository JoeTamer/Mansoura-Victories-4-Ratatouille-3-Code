/**
 * @file systick.h
 * @brief Header for STM32 SysTick-based delay functions (Remi Micromouse Project)
 *
 * Provides millisecond delay functionality using the SysTick timer.
 */

#ifndef INC_PERIPHERALS_PERIPHERALSHEADERFILES_SYSTICK_H_
#define INC_PERIPHERALS_PERIPHERALSHEADERFILES_SYSTICK_H_

#include <stm32f103xb.h>
#include <stdint.h>
#include <stddef.h>

/* ========================================================================================================================
 * === SETUP FUNCTIONS ===
 * ======================================================================================================================== */

void systick_setup(void);

/* ========================================================================================================================
 * === DELAY FUNCTIONS ===
 * ======================================================================================================================== */

void systick_milliSecond(void);
void systick_delayMillis(uint16_t del);

#endif /* INC_PERIPHERALS_PERIPHERALSHEADERFILES_SYSTICK_H_ */
