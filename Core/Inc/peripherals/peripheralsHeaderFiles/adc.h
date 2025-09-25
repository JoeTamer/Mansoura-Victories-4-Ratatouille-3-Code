/**
 * @file adc.h
 * @brief Header for STM32 adc (Remi Micromouse Project)
 *
 */

#ifndef INC_PERIPHERALS_PERIPHERALSHEADERFILES_ADC_H_
#define INC_PERIPHERALS_PERIPHERALSHEADERFILES_ADC_H_

#include <stm32f103xb.h>
#include <stdint.h>
#include <stddef.h>
#include <stdarg.h>
#include "gpio.h"
#include "systick.h"
#include "uart.h"

typedef enum { EOC = 0, AWD } TYPE;

/**
 * @brief ADC channel pinout reference
 *
 * PA0  -> ADC12_IN0  | PA1  -> ADC12_IN1  | PA2  -> ADC12_IN2  | PA3  -> ADC12_IN3  | PA4  -> ADC12_IN4
 * PA5  -> ADC12_IN5  | PA6  -> ADC12_IN6  | PA7  -> ADC12_IN7  | PB0  -> ADC12_IN8  | PB1  -> ADC12_IN9
 *
 * PC0  -> ADC12_IN10 | PC1  -> ADC12_IN11 | PC2  -> ADC12_IN12
 * PC3  -> ADC12_IN13 | PC4  -> ADC12_IN14 | PC5  -> ADC12_IN15
 *
 * ADC12_IN16 input channel which is used to convert output voltage to digital value
 */

/* ========================================================================================================================
 * === ADC SETUP FUNCTION ===
 * ======================================================================================================================== */

void adc_setupPin(ADC_TypeDef* ADCx, GPIO_TypeDef* GPIOx, uint8_t pin);
void adc_start(ADC_TypeDef* ADCx, uint16_t* dmaBuffer, uint16_t length);
uint8_t adc_getNum(ADC_TypeDef* ADCx);


/* ========================================================================================================================
 * === INTERUPPT-BASED FUNCTIONS ===
 * ======================================================================================================================== */

void adc_irqStart(ADC_TypeDef* ADCx, TYPE type, void (*function)(uint16_t), ...);
void adc_irqAttach(uint8_t adc, TYPE type, void (*function)(uint16_t));
void adc_irqRFlag(ADC_TypeDef* ADCx, TYPE type);


#endif /* INC_PERIPHERALS_PERIPHERALSHEADERFILES_ADC_H_ */
