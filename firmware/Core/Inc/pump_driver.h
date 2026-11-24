#ifndef PUMP_DRIVER_H
#define PUMP_DRIVER_H

#include "stm32f3xx_hal.h"

// adjust these two definitions calibrate the pump
#define FLOW 5833U // in mL/min
#define FREQ 2000U // in Hz

// PUMP CONTROL FUNCTIONS

// Manual Mode: Turn the pump on at a specific speed and leave it on.
void Pump_SetSpeed(uint32_t pwm_speed);
void pump(TIM_HandleTypeDef *htim, uint16_t amount);

#endif /* PUMP_DRIVER_H */
