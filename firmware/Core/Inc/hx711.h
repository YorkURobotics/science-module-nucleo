/*
 * HX711.h
 *
 *  Created on: Oct 1, 2026
 *      Author: Antonio Lamacchia
 */

#ifndef INC_HX711_H_
#define INC_HX711_H_

#include <stdint.h>
#include "stm32f3xx_hal.h"

#define DT_PIN GPIO_PIN_8
#define DT_PORT GPIOB
#define SCK_PIN GPIO_PIN_9
#define SCK_PORT GPIOB

void microDelay(TIM_HandleTypeDef *timmy, uint16_t delay);
int32_t getHX711(TIM_HandleTypeDef *timmy);

#endif /* INC_HX711_H_ */
