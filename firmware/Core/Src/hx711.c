/*
 * hx711.c
 *
 *  Created on: Oct 1, 2026
 *      Author: Antonio Lamacchia
 */
#include "hx711.h"
uint8_t hx711Timeout = 0;
/*
 * Delay to get microsecond precision required for bit banging
 */
void microDelay(TIM_HandleTypeDef *timmy, uint16_t delay) {
  __HAL_TIM_SET_COUNTER(timmy, 0);
  while (__HAL_TIM_GET_COUNTER(timmy) < delay);
}
/*
 * Bit banged method to get data from the amplifier
 * Refer to the README for more detail
 */
int32_t getHX711(TIM_HandleTypeDef *timmy) {
  uint32_t data = 0;
  uint32_t startTime = HAL_GetTick();
  while(HAL_GPIO_ReadPin(DT_PORT, DT_PIN) == GPIO_PIN_SET)
  {
    if(HAL_GetTick() - startTime > 200) {
      hx711Timeout = 1;
      return 0;
    }
  }
  for(int8_t len=0; len<24 ; len++)
  {
    HAL_GPIO_WritePin(SCK_PORT, SCK_PIN, GPIO_PIN_SET);
    microDelay(timmy, 1);
    data = data << 1;
    HAL_GPIO_WritePin(SCK_PORT, SCK_PIN, GPIO_PIN_RESET);
    microDelay(timmy, 1);
    if(HAL_GPIO_ReadPin(DT_PORT, DT_PIN) == GPIO_PIN_SET)
      data ++;
  }
  if (data & 0x800000) { // check if data is signed
	  data |= 0xFF000000;
  }
  HAL_GPIO_WritePin(SCK_PORT, SCK_PIN, GPIO_PIN_SET);
  microDelay(timmy, 1);
  HAL_GPIO_WritePin(SCK_PORT, SCK_PIN, GPIO_PIN_RESET);
  microDelay(timmy, 1);
  return data;
}
