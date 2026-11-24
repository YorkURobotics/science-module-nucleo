/*
 *
 */
#include "pump_driver.h"

// timer we set up in main.c
extern TIM_HandleTypeDef htim1;

void Pump_SetSpeed(uint32_t pwm_speed) {
	uint32_t max = __HAL_TIM_GET_AUTORELOAD(&htim1) + 1;

	if (pwm_speed > max) {
        pwm_speed = max; // Cap at max speed
    }
	// flow rate is calculated based on max speed
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, pwm_speed);
}

//amount is in mL
void pump(TIM_HandleTypeDef *htim, uint16_t amount) {

	uint32_t ticks = ((uint32_t)amount * 60U * FREQ) / FLOW;

	__HAL_TIM_SET_COUNTER(htim, 0);
	__HAL_TIM_SET_AUTORELOAD(htim, ticks - 1);
	__HAL_TIM_CLEAR_FLAG(htim, TIM_FLAG_UPDATE);

	Pump_SetSpeed(__HAL_TIM_GET_AUTORELOAD(&htim1) + 1);
	HAL_TIM_Base_Start_IT(htim);
}

