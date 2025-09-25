#include "hardware/hardwareHeaderFiles/infraredSensors.h"

void infraredSensors_setup(ADC_TypeDef* ADCx, INFRARED* IRx, uint8_t count, uint16_t* dmaBuffer) {
	for(uint8_t i = 0;i<count;i++) { adc_setupPin(ADCx, IRx[i].port, IRx[i].pin); }
	adc_start(ADCx, dmaBuffer, count);
}
