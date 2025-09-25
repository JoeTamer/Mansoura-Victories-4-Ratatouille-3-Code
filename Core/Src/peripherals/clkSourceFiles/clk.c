/******************************************************
 * File: clk.c
 * Description: Adjusting system and bus clks for STM32F103.
 *
 * Author: Jo
 * Date: ~
 ******************************************************/

#include <peripherals/clkHeaderFiles/clk.h>

void clk_setup(void) {
    // 1. Enable HSE (external 8 MHz crystal)
    RCC->CR |= RCC_CR_HSEON;
    while (!(RCC->CR & RCC_CR_HSERDY));  // Wait for HSE ready

    // 2. Configure Flash prefetch and latency
    FLASH->ACR |= FLASH_ACR_PRFTBE;       // Enable prefetch buffer
    FLASH->ACR &= ~FLASH_ACR_LATENCY;     // Clear latency bits
    FLASH->ACR |= FLASH_ACR_LATENCY_2;    // 2 wait states (for 72 MHz)

    // 3. Set bus prescalers
    RCC->CFGR |= RCC_CFGR_HPRE_DIV1;      // AHB = SYSCLK / 1 = 72 MHz
    RCC->CFGR |= RCC_CFGR_PPRE2_DIV1;     // APB2 = AHB / 1 = 72 MHz (TIM1)
    RCC->CFGR |= RCC_CFGR_PPRE1_DIV2;     // APB1 = AHB / 2 = 36 MHz (TIM2–TIM4) → timers get x2 = 72 MHz

    // 4. Configure PLL: HSE as source, x9 multiplier → 8 MHz * 9 = 72 MHz
    RCC->CFGR &= ~RCC_CFGR_PLLXTPRE;      // HSE not divided
    RCC->CFGR |= RCC_CFGR_PLLSRC;         // PLL source = HSE
    RCC->CFGR &= ~RCC_CFGR_PLLMULL;
    RCC->CFGR |= RCC_CFGR_PLLMULL9;

    // 5. Enable PLL
    RCC->CR |= RCC_CR_PLLON;
    while (!(RCC->CR & RCC_CR_PLLRDY));   // Wait for PLL ready

    // 6. Switch system clock to PLL
    RCC->CFGR &= ~RCC_CFGR_SW;
    RCC->CFGR |= RCC_CFGR_SW_PLL;
    while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL);  // Wait for switch
}

