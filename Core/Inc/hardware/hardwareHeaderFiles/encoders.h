#ifndef INC_HARDWARE_HARDWAREHEADERFILES_ENCODERS_H_
#define INC_HARDWARE_HARDWAREHEADERFILES_ENCODERS_H_

#include "peripherals/peripheralsHeaderFiles/interrupts.h"

typedef struct {
	GPIO_TypeDef* port;
	uint8_t       pin;
} encoderChannel;

typedef struct {
	encoderChannel channels[2];  // support for up to 2 channels
	uint8_t        numChannels;  // 1 or 2
	uint8_t        PPR;          // pulses per revolution
} Encoder;

// ---------- Setup ----------
void encoder_setupPin(Encoder* myEncoder,uint8_t channel, EDGESTATE ES, void (*callback)(void));

#endif /* INC_HARDWARE_HARDWAREHEADERFILES_ENCODERS_H_ */
