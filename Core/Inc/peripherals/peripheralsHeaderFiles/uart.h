/**
 * @file uart.h
 * @brief Header for STM32 basic uart (Remi Micromouse Project)
 *
 * Provides priting over serial for debugging purposes
 */

#ifndef INC_PERIPHERALS_PERIPHERALSHEADERFILES_UART_H_
#define INC_PERIPHERALS_PERIPHERALSHEADERFILES_UART_H_

#include <stm32f103xb.h>
#include <stdint.h>
#include <stddef.h>
#include "gpio.h"
#include "systick.h"

/* ========================================================================================================================
 * === UART SETUP FUNCTION ===
 * ======================================================================================================================== */

void uart_setup();

/* ========================================================================================================================
 * === SERIAL FUNCTIONS ===
 * ======================================================================================================================== */

void uart_sendChar(char c);
void uart_send(const char *fmt, ...);

#endif /* INC_PERIPHERALS_PERIPHERALSHEADERFILES_UART_H_ */
