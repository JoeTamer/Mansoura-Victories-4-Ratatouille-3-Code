#ifndef INC_CONTROL_CONTROLHEADERFILES_PIDCONTROLLER_H_
#define INC_CONTROL_CONTROLHEADERFILES_PIDCONTROLLER_H_

#include "peripherals/peripheralsHeaderFiles/timers.h"
#include "peripherals/peripheralsHeaderFiles/uart.h"
#include <math.h>

typedef struct {
    float Kp, Ki, Kd;
    float last_error, last_time;
    float integral;
    float min, max;
} PID;

void pidController_setup(PID* pid, float Kp, float Ki, float Kd, float min, float max);

float pidController_compute(PID* pid, float goal, float measurement);


#endif /* INC_CONTROL_CONTROLHEADERFILES_PIDCONTROLLER_H_ */
