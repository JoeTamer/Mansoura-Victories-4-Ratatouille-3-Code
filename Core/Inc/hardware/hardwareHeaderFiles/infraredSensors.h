#ifndef INC_HARDWARE_HARDWAREHEADERFILES_INFRAREDSENSORS_H_
#define INC_HARDWARE_HARDWAREHEADERFILES_INFRAREDSENSORS_H_

#include "peripherals/peripheralsHeaderFiles/adc.h"

typedef struct {
	ADC_TypeDef*  ADCx;
	GPIO_TypeDef* port;
	uint8_t       pin;
} INFRARED;

// ---------- Setup ----------
void infraredSensors_setup(ADC_TypeDef* ADCx, INFRARED* IRx, uint8_t count, uint16_t* dmaBuffer);

#endif /* INC_HARDWARE_HARDWAREHEADERFILES_INFRAREDSENSORS_H_ */
