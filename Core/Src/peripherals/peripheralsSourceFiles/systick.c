/******************************************************
 * File: systick.c
 * Description: Implementation of SYSTICK functions for STM32F103.
 *
 * Author: Jo
 * Date: ~
 ******************************************************/

#include "peripherals/peripheralsHeaderFiles/systick.h"

/* ========================================================================================================================
 *                                                      SETUP FUNCTION
 * ======================================================================================================================== */

/**
 * @brief Configures the SysTick timer for 1ms ticks (72 MHz system clock).
 */
void systick_setup(void) {
	SysTick->CTRL = 0;         // Disable SysTick
	SysTick->LOAD = 72000 - 1; // 1ms tick @ 72 MHz
	SysTick->VAL  = 0;         // Reset current value
	SysTick->CTRL = 5;         // Enable SysTick with processor clock, no interrupt
}

/* ========================================================================================================================
 *                                                     DELAY FUNCTIONS
 * ======================================================================================================================== */

/**
 * @brief Blocks execution for exactly 1 millisecond.
 */
void systick_milliSecond(void) {
	SysTick->VAL = 0;                     // Reset current value
	while ((SysTick->CTRL & (1 << 16)) == 0); // Wait for COUNTFLAG
}

/**
 * @brief Blocks execution for a specified number of milliseconds.
 * @param del Number of milliseconds to delay.
 */
void systick_delayMillis(uint16_t del) {
	while (del--) systick_milliSecond();
}
