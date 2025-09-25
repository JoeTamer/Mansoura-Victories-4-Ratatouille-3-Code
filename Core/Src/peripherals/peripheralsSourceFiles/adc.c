#include "peripherals/peripheralsHeaderFiles/adc.h"

void (*adc_exti_callbacks[2][2])(uint16_t) = {0};

/* per-ADC channel counters (index 0 => ADC1, index 1 => ADC2) */
static uint8_t adc_channel_count[2] = {0, 0};

void adc_setupPin(ADC_TypeDef *ADCx, GPIO_TypeDef *GPIOx, uint8_t pin)
{
    uint8_t adc_index;
    uint8_t channel;

    // Map ADCx pointer to index 0/1 for arrays (ADC1=0, ADC2=1)
    if      (ADCx == ADC1) adc_index = 0;
    else if (ADCx == ADC2) adc_index = 1;
    else return; // unsupported ADC

    // Convert GPIO pin to ADC channel number
    // Assuming PA0=0..PA7=7, PB0=8, PB1=9, etc.
    if (GPIOx == GPIOA)      channel = pin;         // PA0..PA7
    else if (GPIOx == GPIOB) channel = pin + 8;     // PB0..PB1
    else if (GPIOx == GPIOC) channel = pin + 10;    // PC0..PC5
    else return; // unsupported port

    // Enable GPIO clock
    if      (GPIOx == GPIOA) RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;
    else if (GPIOx == GPIOB) RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;
    else if (GPIOx == GPIOC) RCC->APB2ENR |= RCC_APB2ENR_IOPCEN;

    // Configure pin as analog input
    if (pin <= 7) {
        GPIOx->CRL &= ~(0xF << (pin * 4)); // MODE=00, CNF=00
    } else {
        GPIOx->CRH &= ~(0xF << ((pin - 8) * 4));
    }

    // Enable ADC clock
    if      (ADCx == ADC1) RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;
    else if (ADCx == ADC2) RCC->APB2ENR |= RCC_APB2ENR_ADC2EN;

    // Append channel to sequence
    uint8_t slot = adc_channel_count[adc_index]; // slot index 0..15

    if (slot < 6) {
        ADCx->SQR3 &= ~(0x1F << (slot * 5));
        ADCx->SQR3 |=  (channel & 0x1F) << (slot * 5);
    }
    else if (slot < 12) {
        uint8_t s = slot - 6;
        ADCx->SQR2 &= ~(0x1F << (s * 5));
        ADCx->SQR2 |=  (channel & 0x1F) << (s * 5);
    }
    else {
        uint8_t s = slot - 12;
        ADCx->SQR1 &= ~(0x1F << (s * 5));
        ADCx->SQR1 |=  (channel & 0x1F) << (s * 5);
    }

    adc_channel_count[adc_index]++;

    // Update sequence length (bits [23:20] in SQR1)
    ADCx->SQR1 &= ~(0xF << 20);
    ADCx->SQR1 |= ((adc_channel_count[adc_index] - 1) & 0xF) << 20;

    // Configure sample time for the channel (here: max = 239.5 cycles)
    if (channel <= 9) {
        ADCx->SMPR2 &= ~(0x7 << (channel * 3));
        ADCx->SMPR2 |=  (0x7 << (channel * 3));
    } else {
        uint8_t c = channel - 10;
        ADCx->SMPR1 &= ~(0x7 << (c * 3));
        ADCx->SMPR1 |=  (0x7 << (c * 3));
    }
}

void adc_start(ADC_TypeDef* ADCx, uint16_t* dmaBuffer, uint16_t length) {
    // 1. Enable ADC clock (safety net)
    if (ADCx == ADC1) RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;
    if (ADCx == ADC2) RCC->APB2ENR |= RCC_APB2ENR_ADC2EN;

    // 2. Enable ADC
    ADCx->CR2 |= ADC_CR2_ADON;
    systick_delayMillis(1);

    // 3. Calibrate
    ADCx->CR2 |= ADC_CR2_RSTCAL;
    while (ADCx->CR2 & ADC_CR2_RSTCAL);
    ADCx->CR2 |= ADC_CR2_CAL;
    while (ADCx->CR2 & ADC_CR2_CAL);

    // 4. Scan + continuous mode
    ADCx->CR1 |= ADC_CR1_SCAN;
    ADCx->CR2 |= ADC_CR2_CONT;

    // 5. Alignment
    ADCx->CR2 &= ~ADC_CR2_ALIGN; // Right alignment

    // 6. DMA mode if requested
    if (dmaBuffer && length > 0) {
        if (ADCx == ADC1) RCC->AHBENR |= RCC_AHBENR_DMA1EN; // DMA1 clock

        DMA1_Channel1->CCR = 0; // Disable & reset config
        DMA1_Channel1->CPAR = (uint32_t)&ADCx->DR;
        DMA1_Channel1->CMAR = (uint32_t)dmaBuffer;
        DMA1_Channel1->CNDTR = length;
        DMA1_Channel1->CCR = DMA_CCR_MINC | DMA_CCR_MSIZE_0 | DMA_CCR_PSIZE_0 |
                             DMA_CCR_CIRC | DMA_CCR_PL_1; // 16-bit mem & periph, circular, high prio
        DMA1_Channel1->CCR |= DMA_CCR_EN; // Enable channel

        ADCx->CR2 |= ADC_CR2_DMA; // Enable ADC DMA mode
    }

    // 7. Software start
    ADCx->CR2 &= ~(ADC_CR2_EXTSEL); // Clear EXTSEL
    ADCx->CR2 &= ~ADC_CR2_EXTSEL;
    ADCx->CR2 |= ADC_CR2_EXTSEL_0 | ADC_CR2_EXTSEL_1 | ADC_CR2_EXTSEL_2;
    ADCx->CR2 |= ADC_CR2_EXTTRIG; // Enable trigger
    ADCx->CR2 |= ADC_CR2_SWSTART;
}

uint8_t adc_getNum(ADC_TypeDef* ADCx) {
	return ((ADCx == ADC1) ? 1 : (ADCx == ADC2) ? 2 : 0);
}

/* ========================================================================================================================
 *                                                INTERUPPT-BASED FUNCTIONS
 * ======================================================================================================================== */

void adc_irqStart(ADC_TypeDef* ADCx, TYPE type, void (*function)(uint16_t), ...) {
	va_list args;
	va_start(args, function);

	if (type == EOC) {
		ADCx->CR1 |= (1<<5); // EOC Interrupt Enable
	} else  if (type == AWD) {
		ADCx->CR1 |= (1<<23); // WatchDog Interrupt Enable
		ADCx->HTR = va_arg(args, int32_t);
		ADCx->LTR = va_arg(args, int32_t);
		ADCx->CR1 |= (1<<6);
	} va_end(args);
	__disable_irq();

	NVIC_EnableIRQ(ADC1_2_IRQn);
	uint8_t adc = adc_getNum(ADCx);
	adc_irqAttach(adc, type, function);

	__enable_irq();
}

void adc_irqRFlag(ADC_TypeDef* ADCx, TYPE type) {
    if (type == EOC) {
        // Do nothing — EOC flag is cleared by reading ADCx->DR
    } else if (type == AWD) {
        ADCx->SR &= ~(1<<0); // Clear AWD flag
    }
}

void adc_irqAttach(uint8_t adc, TYPE type, void (*function)(uint16_t)) {
    if (adc == 1 || adc == 2) adc_exti_callbacks[adc - 1][type] = function;
}

/* ========================================================================================================================
 *                                                 IRQ-HANDLERS FUNCTIONS
 * ======================================================================================================================== */

void ADC1_2_IRQHandler() {
    // EOC for ADC1
    if ((ADC1->SR & (1 << 1)) && adc_exti_callbacks[0][0]) {
        uint16_t value = ADC1->DR;
        adc_irqRFlag(ADC1, EOC);
        adc_exti_callbacks[0][0](value);
    } // WATCHDOG for ADC1
    if ((ADC1->SR & (1 << 0)) && adc_exti_callbacks[0][1]) {
        uint16_t value = ADC1->DR;
        adc_irqRFlag(ADC1, AWD);
        adc_exti_callbacks[0][1](value);
    }
    // EOC for ADC2
    if ((ADC2->SR & (1 << 1)) && adc_exti_callbacks[1][0]) {
        uint16_t value = ADC2->DR;
        adc_irqRFlag(ADC2, EOC);
        adc_exti_callbacks[1][0](value);
    } // WATCHDOG for ADC2
    if ((ADC2->SR & (1 << 0)) && adc_exti_callbacks[1][1]) {
        uint16_t value = ADC2->DR;
        adc_irqRFlag(ADC2, AWD);
        adc_exti_callbacks[1][1](value);
    }
}


