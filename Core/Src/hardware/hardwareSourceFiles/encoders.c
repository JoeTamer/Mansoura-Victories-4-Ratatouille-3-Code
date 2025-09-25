#include "hardware/hardwareHeaderFiles/encoders.h"

// ---------- Setup ----------
void encoder_setupPin(Encoder* myEncoder,uint8_t channel, EDGESTATE ES, void (*callback)(void)) {
	interrupts_attach(
		myEncoder->channels[channel-1].port,
		myEncoder->channels[channel-1].pin,
		ES,
		callback
	);
}
