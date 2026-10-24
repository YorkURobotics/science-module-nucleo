/*
 * weigh.h
 *
 *  Created on: Oct 1, 2026
 *      Author: Antonio lamacchia
 */

#ifndef INC_WEIGH_H_
#define INC_WEIGH_H_

#include <stdint.h>
#include "stm32f3xx_hal.h"

#define knownOriginal 142000.0f
#define knownHX711 126096.0f

extern volatile uint8_t weighRequest;
extern volatile uint8_t tareRequest;

extern uint8_t hx711Timeout;

int weigh(TIM_HandleTypeDef *tim);
void tare_request(TIM_HandleTypeDef *tim);

#endif /* INC_WEIGH_H_ */
