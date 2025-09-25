#ifndef INC_HARDWARE_HARDWAREHEADERFILES_MOTORS_H_
#define INC_HARDWARE_HARDWAREHEADERFILES_MOTORS_H_

#include "peripherals/peripheralsHeaderFiles/gpio.h"
#include "peripherals/peripheralsHeaderFiles/timers.h"

typedef struct {
	GPIO_TypeDef* IN1port; uint8_t IN1pin;
	GPIO_TypeDef* IN2port; uint8_t IN2pin;
	GPIO_TypeDef* ENport ; uint8_t ENpin ;
	float CURR_RPM;
	uint32_t lastCount;
} Motor;

typedef enum { FORWARD, BACKWARD, STOP } MOTORDIR;

// ---------- Setup ----------
void motors_setup(Motor* myMotor, uint16_t FREQ);

// ---------- Control ----------
void motors_control(Motor* myMotor, MOTORDIR dir, uint16_t speed);
void motors_stop(Motor* myMotor);

#endif /* INC_HARDWARE_HARDWAREHEADERFILES_MOTORS_H_ */
