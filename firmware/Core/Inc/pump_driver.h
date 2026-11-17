#ifndef PUMP_DRIVER_H
#define PUMP_DRIVER_H

#include "stm32f3xx_hal.h"

// PUMP CONTROL FUNCTIONS

// Manual Mode: Turn the pump on at a specific speed and leave it on.
void Pump_SetSpeed(uint16_t pwm_speed);

#endif /* PUMP_DRIVER_H */
