/******************************************************
 * File: gpio.c
 * Description: Implementation of UART functions for STM32F103.
 *
 * Author: Jo
 * Date: ~
 ******************************************************/

#include "peripherals/peripheralsHeaderFiles/uart.h"
#include <stdio.h>
#include <stdarg.h>

/**
 * @def UART_BUFFER_SIZE
 * @brief Maximum number of characters the UART send buffer can hold.
 */
#define UART_BUFFER_SIZE 128

/* ========================================================================================================================
 *                                                   UART SETUP FUNCTION
 * ======================================================================================================================== */

/**
 * @brief Sets up USART1 for UART communication.
 *
 * Configures GPIOA9 (TX) as Alternate Function Push-Pull, sets baud rate to 115200,
 * enables USART1 transmitter, and sends a welcome message.
 *
 * @param strtDel Delay in milliseconds before sending the welcome message.
 */
void uart_setup() {
    RCC->APB2ENR |= RCC_APB2ENR_USART1EN; // Enable clock for USART1

    gpio_setupPin(GPIOA, 9, OUT_50, AF_PP); // TX Pin setup (PA9)

    // USART1 Baud Rate = 115200 @ 72 MHz
    USART1->BRR = 0x0271;
    USART1->CR1 = USART_CR1_TE | USART_CR1_UE; // Enable Transmitter and USART

    uart_send("\nHAPPY DEBUGGING <3 !! \n\n");
}

/* ========================================================================================================================
 *                                                   TRANSMISSION FUNCTIONS
 * ======================================================================================================================== */

/**
 * @brief Sends a single character over USART1.
 *
 * Waits until transmit data register is empty before sending.
 *
 * @param c Character to send.
 */
void uart_sendChar(char c) {
    while (!(USART1->SR & USART_SR_TXE)); // Wait until TX buffer is empty
    USART1->DR = c;
}

/**
 * @brief Sends a formatted string over USART1.
 *
 * Uses `vsnprintf` to safely format the string, then transmits it character by character.
 *
 * @param fmt Format string (e.g., "X = %d")
 * @param ... Variable arguments to format.
 */
void uart_send(const char *fmt, ...) {
    char buffer[UART_BUFFER_SIZE];

    va_list args;
    va_start(args, fmt);
    vsnprintf(buffer, UART_BUFFER_SIZE, fmt, args);
    va_end(args);

    const char *p = buffer;
    while (*p) {
        uart_sendChar(*p++);
    }
}
