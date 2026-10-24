#include "pump_driver.h"

// timer we set up in main.c
extern TIM_HandleTypeDef htim1;

void Pump_SetSpeed(uint16_t pwm_speed) {
	if (pwm_speed > 1000) {
        pwm_speed = 1000; // Cap at max speed
    }
    // Cast it back to a uint32_t at the end to satisfy what STM expects
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, (uint32_t)pwm_speed);
}


