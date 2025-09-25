/******************************************************
 * File: timers.c
 * Description: Implementation of TIMER functions for STM32F103.
 *
 * Author: Jo
 * Date: ~
 ******************************************************/

#include "peripherals/peripheralsHeaderFiles/timers.h"

/* ========================================================================================================================
 *                                           GLOBAL VARIABLES & CALLBACK ARRAY
 * ======================================================================================================================== */

void (*timers_exti_callbacks[4])(void) = {0};
TIM_TypeDef* stopwatchTimer = TIM2;
volatile uint32_t stopwatchOVF = 0;

/* ========================================================================================================================
 *                                                 TIMERS SETUP FUNCTION
 * ======================================================================================================================== */

/**
 * @brief Initializes the given timer with the desired time base (MICROS or MILLIS).
 */
void timers_setup(TIM_TypeDef* TIMx, TimeBase unit) {
	uint8_t timer = timers_getNum(TIMx);

	if      (timer == 1) { RCC->APB2ENR |= (1 << 11); }
	else if (timer <= 4) { RCC->APB1ENR |= (1 << (timer - 2)); }

	TIMx->CNT = 0;
	TIMx->PSC = (unit == MICROS) ? (F_CPU / 1000000UL) - 1 : (F_CPU / 1000UL) - 1;
	TIMx->CR1 &= ~(1 << 0); // Disable Counter
}

/* ========================================================================================================================
 *                                                 BLOCKING DELAYS FUNCTION
 * ======================================================================================================================== */

/**
 * @brief Delays the program execution for a specified number of microseconds using the given timer.
 */
void timers_delayMicros(TIM_TypeDef* TIMx, uint16_t del) {
	timers_start(TIMx, del, MICROS);
	TIMx->CR1 |= (1 << 3); // One Pulse Mode
	while (TIMx->CR1 & (1 << 0));
	TIMx->CR1 &= ~(1 << 3);
}

/**
 * @brief Delays the program execution for a specified number of milliseconds using the given timer.
 */
void timers_delayMillis(TIM_TypeDef* TIMx, uint16_t del) {
	while (del--) timers_delayMicros(TIMx, 1000);
}

/* ========================================================================================================================
 *                                                  NON-BLOCKING FUNCTIONS
 * ======================================================================================================================== */

/**
 * @brief Starts a timer in continuous mode with the specified delay and time base.
 */
void timers_start(TIM_TypeDef* TIMx, uint16_t del, TimeBase unit) {
	if (del == 0) return;

	TIMx->CNT = 0;
	TIMx->PSC = (unit == MICROS) ? (F_CPU / 1000000UL - 1) : (F_CPU / 1000UL - 1);
	TIMx->ARR = del - 1;
	TIMx->CR1 |= TIM_CR1_CEN;
}

/**
 * @brief Stops the timer and disables its clock and interrupts.
 */
void timers_stop(TIM_TypeDef* TIMx) {
	uint8_t timer = timers_getNum(TIMx);
	TIMx->CR1 = 0;

	if      (timer == 1) { RCC->APB2ENR &= ~(1 << 11); }
	else if (timer <= 4) { RCC->APB1ENR &= ~(1 << (timer - 2)); }

	timers_irqStop(TIMx);
}

/**
 * @brief Returns the elapsed time in either microseconds or milliseconds since timer start.
 */
uint32_t timers_getElapsed(TIM_TypeDef* TIMx, TimeBase unit) {
	uint16_t temp = (unit == MICROS) ? 1000000UL : 1000UL;
	return ((uint32_t)(TIMx->CNT) * (TIMx->PSC + 1) * temp) / F_CPU;
}

/* ========================================================================================================================
 *                                                INTERUPPT-BASED FUNCTIONS
 * ======================================================================================================================== */

/**
 * @brief Enables timer interrupt and attaches a callback to it.
 */
void timers_irqStart(TIM_TypeDef* TIMx, void (*function)(void)) {
	TIMx->DIER |= 1; // Enable update interrupt

	__disable_irq();
	uint8_t timer = timers_getNum(TIMx);

	switch (timer) {
		case 1: NVIC_EnableIRQ(TIM1_UP_IRQn); break;
		case 2: NVIC_EnableIRQ(TIM2_IRQn);    break;
		case 3: NVIC_EnableIRQ(TIM3_IRQn);    break;
		case 4: NVIC_EnableIRQ(TIM4_IRQn);    break;
	}

	timers_irqAttach(timer, function);
	__enable_irq();
}

/**
 * @brief Disables timer update interrupt and NVIC interrupt.
 */
void timers_irqStop(TIM_TypeDef* TIMx) {
	TIMx->DIER &= ~(1 << 0); // Disable update interrupt

	__disable_irq();
	uint8_t timer = timers_getNum(TIMx);

	switch (timer) {
		case 1: NVIC_DisableIRQ(TIM1_UP_IRQn); break;
		case 2: NVIC_DisableIRQ(TIM2_IRQn);    break;
		case 3: NVIC_DisableIRQ(TIM3_IRQn);    break;
		case 4: NVIC_DisableIRQ(TIM4_IRQn);    break;
	}
	__enable_irq();
}

/**
 * @brief Clears the update interrupt flag of the given timer.
 */
void timers_irqRFlag(TIM_TypeDef* TIMx) {
	TIMx->SR &= ~(1 << 0); // Clear update interrupt flag
}

/**
 * @brief Assigns a user callback function to the specified timer interrupt.
 */
void timers_irqAttach(uint8_t timer, void (*function)(void)) {
	if (timer >= 1 && timer <= 4)
		timers_exti_callbacks[timer - 1] = function;
}

/* ========================================================================================================================
 *                                                 IRQ-HANDLERS FUNCTIONS
 * ======================================================================================================================== */


/**
 * @brief TIM1 update interrupt handler.
 */
void TIM1_UP_IRQHandler(void) {
	timers_irqRFlag(TIM1);
	if (timers_exti_callbacks[0]) timers_exti_callbacks[0]();
}

/**
 * @brief TIM2 update interrupt handler.
 */
void TIM2_IRQHandler(void) {
	timers_irqRFlag(TIM2);
	if (timers_exti_callbacks[1]) timers_exti_callbacks[1]();
}

/**
 * @brief TIM3 update interrupt handler.
 */
void TIM3_IRQHandler(void) {
	timers_irqRFlag(TIM3);
	if (timers_exti_callbacks[2]) timers_exti_callbacks[2]();
}

/**
 * @brief TIM4 update interrupt handler.
 */
void TIM4_IRQHandler(void) {
	timers_irqRFlag(TIM4);
	if (timers_exti_callbacks[3]) timers_exti_callbacks[3]();
}

/* ========================================================================================================================
 *                                                    HELPER FUNCTIONS
 * ======================================================================================================================== */

/**
 * @brief Determines the timer channel (1–4) for a given GPIO pin.
 * @return Timer channel (1–4), or 0xFF if unsupported.
 */
uint8_t timers_getChannel(GPIO_TypeDef* GPIOx, uint8_t pin) {
    if (GPIOx == GPIOA) {
        switch (pin) {
            case 0: case 6: case 8:  return 1;
            case 1: case 7: case 9:  return 2;
            case 2: case 10:         return 3;
            case 3: case 11:         return 4;
        }
    } else if (GPIOx == GPIOB) {
        switch (pin) {
            case 0:  return 3; // TIM3_CH3
            case 1:  return 4; // TIM3_CH4
            case 6:  return 1; // TIM4_CH1
            case 7:  return 2; // TIM4_CH2
            case 8:  return 3; // TIM4_CH3
            case 9:  return 4; // TIM4_CH4
        }
    }
    return 0xFF;
}

/**
 * @brief Returns the timer instance for a given GPIO pin.
 */
TIM_TypeDef* timers_getTimer(GPIO_TypeDef* GPIOx, uint8_t pin) {
    if (GPIOx == GPIOA) {
        if      (pin < 4)  return TIM2;
        else if (pin < 8)  return TIM3;
        else if (pin < 12) return TIM1;
    } else if (GPIOx == GPIOB) {
        if (pin < 2) return TIM3;
        else if (pin >= 6 && pin <= 9) return TIM4;
    }
    return NULL;
}

/**
 * @brief Returns the timer number (1–4) for a given TIM_TypeDef pointer.
 */
uint8_t timers_getNum(TIM_TypeDef* TIMx) {
	if      (TIMx == TIM1) return 1;
	else if (TIMx == TIM2) return 2;
	else if (TIMx == TIM3) return 3;
	else if (TIMx == TIM4) return 4;
	return 1;
}

/* ========================================================================================================================
 *                                                     COMPARE FUNCTION
 * ======================================================================================================================== */

/**
 * @brief Sets up compare match output on the specified timer pin.
 */
void timers_compare(GPIO_TypeDef* GPIOx, uint8_t pin, TimeBase unit, uint16_t arr, uint16_t compare) {
	uint8_t CHANNEL = timers_getChannel(GPIOx, pin);
	TIM_TypeDef* TIMx = timers_getTimer(GPIOx, pin);

	if ((TIMx->CCER & (1 << ((CHANNEL - 1) * 4))) == 0) {
		timers_setup(TIMx, unit);
		gpio_setupPin(GPIOx, pin, OUT_50, AF_PP);
		switch (CHANNEL) {
			case 1: TIMx->CCMR1 |= 0x30;   break;
			case 2: TIMx->CCMR1 |= 0x3000; break;
			case 3: TIMx->CCMR2 |= 0x30;   break;
			case 4: TIMx->CCMR2 |= 0x3000; break;
		}
		TIMx->CCER |= (1 << ((CHANNEL - 1) * 4));
		TIMx->BDTR |= 0x8000;
		TIMx->CR1 |= (1 << 0); // Enable Counter
	}
	TIMx->ARR = arr - 1;
	switch (CHANNEL) {
		case 1: TIMx->CCR1 = compare; break;
		case 2: TIMx->CCR2 = compare; break;
		case 3: TIMx->CCR3 = compare; break;
		case 4: TIMx->CCR4 = compare; break;
	}
}

/* ========================================================================================================================
 *                                                      PWM FUNCTION
 * ======================================================================================================================== */

/**
 * @brief Sets up PWM output on the specified timer pin.
 */
/*void timers_pwm(GPIO_TypeDef* GPIOx, uint8_t pin, uint32_t freq, uint16_t compare) {
	uint8_t CHANNEL = timers_getChannel(GPIOx, pin);
	TIM_TypeDef* TIMx = timers_getTimer(GPIOx, pin);
	uint8_t timer = timers_getNum(TIMx);

	uint32_t T_CLK = (timer == 1) ? F_CPU : F_CPU / 2;
	uint32_t prescaler = (T_CLK / (freq * 10000UL));

	if ((TIMx->CCER & (1 << ((CHANNEL - 1) * 4))) == 0) {
		if      (timer == 1) { RCC->APB2ENR |= (1 << 11); }
		else if (timer <= 4) { RCC->APB1ENR |= (1 << (timer - 2)); }

		TIMx->CNT = 0;
		TIMx->PSC = prescaler - 1 ;
		TIMx->CR1 &= ~(1 << 0); // Disable Counter

		gpio_setupPin(GPIOx, pin, OUT_50, AF_PP);
		switch (CHANNEL) {
			case 1: TIMx->CCMR1 |= 0x60;   break;
			case 2: TIMx->CCMR1 |= 0x6000; break;
			case 3: TIMx->CCMR2 |= 0x60;   break;
			case 4: TIMx->CCMR2 |= 0x6000; break;
		}
		TIMx->CCER |= (1 << ((CHANNEL - 1) * 4));
		TIMx->BDTR |= 0x8000;
		TIMx->CR1 |= (1 << 0); // Enable Counter
	}
	uint32_t arr = (T_CLK / (prescaler * freq));
	TIMx->ARR = arr - 1;

	switch (CHANNEL) {
		case 1: TIMx->CCR1 = compare; break;
		case 2: TIMx->CCR2 = compare; break;
		case 3: TIMx->CCR3 = compare; break;
		case 4: TIMx->CCR4 = compare; break;
	}
}*/
// --- One-time setup ---
void timers_pwmSetup(GPIO_TypeDef* GPIOx, uint8_t pin, uint32_t freq) {
    TIM_TypeDef* TIMx = timers_getTimer(GPIOx, pin);
    uint8_t channel = timers_getChannel(GPIOx, pin);

    uint8_t timer = timers_getNum(TIMx);
    if      (timer == 1) { RCC->APB2ENR |= (1 << 11); }
    else if (timer <= 4) { RCC->APB1ENR |= (1 << (timer - 2)); }

    gpio_setupPin(GPIOx, pin, OUT_50, AF_PP);

    uint32_t T_CLK = 72000000UL;
    uint32_t prescaler = 1;
    uint32_t arr = (T_CLK / freq) - 1;

    while (arr > 0xFFFF) {
        prescaler++;
        arr = (T_CLK / (prescaler * freq)) - 1;
    }

    TIMx->PSC = prescaler - 1;
    TIMx->ARR = arr;

    // Configure PWM mode without nuking other bits
    switch(channel) {
        case 1:
            TIMx->CCMR1 &= ~(0x7 << 4);
            TIMx->CCMR1 |= (0x6 << 4);   // PWM mode 1
            TIMx->CCMR1 |= (1 << 3);     // preload enable
            TIMx->CCER  |= 0x01;
            break;

        case 2:
            TIMx->CCMR1 &= ~(0x7 << 12);
            TIMx->CCMR1 |= (0x6 << 12);
            TIMx->CCMR1 |= (1 << 11);
            TIMx->CCER  |= 0x10;
            break;

        case 3:
            TIMx->CCMR2 &= ~(0x7 << 4);
            TIMx->CCMR2 |= (0x6 << 4);
            TIMx->CCMR2 |= (1 << 3);
            TIMx->CCER  |= 0x100;
            break;

        case 4:
            TIMx->CCMR2 &= ~(0x7 << 12);
            TIMx->CCMR2 |= (0x6 << 12);
            TIMx->CCMR2 |= (1 << 11);
            TIMx->CCER  |= 0x1000;
            break;
    }

    TIMx->EGR |= 0x01;   // update registers
    TIMx->CR1 |= 0x01;   // enable counter

    // Only enable BDTR if it's TIM1 or TIM8 (advanced timers)
    if (TIMx == TIM1) {
        TIMx->BDTR |= 0x8000;
    }
}

// --- Fast CCR update (raw ticks) ---
void timers_pwm(GPIO_TypeDef* GPIOx, uint8_t pin, uint32_t compare) {
    TIM_TypeDef* TIMx = timers_getTimer(GPIOx, pin);
    uint8_t channel = timers_getChannel(GPIOx, pin);

    uint32_t arr = TIMx->ARR;
    if (compare > arr) compare = arr; // clamp

    switch(channel) {
        case 1: TIMx->CCR1 = compare; break;
        case 2: TIMx->CCR2 = compare; break;
        case 3: TIMx->CCR3 = compare; break;
        case 4: TIMx->CCR4 = compare; break;
    }
}


/* ========================================================================================================================
 *                                                   STOPWATCH FUNCTIONS
 * ======================================================================================================================== */

/**
 * @brief Overflow handler for the stopwatch timer.
 */
void timers_ovfHandler() {
	stopwatchOVF++;
	timers_irqRFlag(stopwatchTimer);
}

/**
 * @brief Initializes a timer to work as a high-resolution stopwatch.
 */
void timers_stopwatchSetup(TIM_TypeDef* TIMx) {
	timers_setup(TIMx, MICROS);
	timers_start(TIMx, 0xFFFF, MICROS);
	timers_irqStart(TIMx, timers_ovfHandler);
	stopwatchTimer = TIMx;
}

/**
 * @brief Returns the total time elapsed in microseconds since stopwatch start.
 */
uint32_t timers_stopwatch() {
	uint32_t ovf1, ovf2;
	uint16_t cnt;

	do {
		ovf1 = stopwatchOVF;
		cnt  = stopwatchTimer->CNT;
		ovf2 = stopwatchOVF;
	} while (ovf1 != ovf2);

	return ((ovf1 << 16) | cnt);
}
