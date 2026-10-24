/*
 * weigh.c
 *
 *  Created on: Oct 1, 2026
 *      Author: Antonio Lamacchia
 */
#include "weigh.h"
#include "hx711.h"

int32_t tare = -26;   /* was 8388582 (0x7FFFE6): the same zero in the offset-binary format hx711.c used before weight-cell 9f01201 */
/*
 * self explanatory
 */
int weigh(TIM_HandleTypeDef *tim) {
  int32_t total = 0;
  int32_t samples = 50;
  int32_t runningSum = 0;
  int32_t divisor = samples;
  int milligram = 0;
  float coefficient;
  for(uint16_t i=0 ; i<samples ; i++) {
	  runningSum = getHX711(tim);
	  if (runningSum == 0) {
		  divisor--;
	  }
	  else {
		  total += runningSum;
	  }
  }
  if (divisor < 1){
  		  // divisor is zero
  	  }
  	  else {
  	  int32_t average = (int32_t)(total / divisor);
  	  coefficient = knownOriginal / knownHX711;
  	  milligram = (int)(average-tare)*coefficient;
  	  }
  return milligram;
}
/*
 * This function updates the tare variable
 */
void tare_request(TIM_HandleTypeDef *tim) {
	  int32_t total = 0;
	  int32_t samples = 50;
	  int32_t runningSum = 0;
	  int32_t divisor = samples;
	  for(uint16_t i = 0 ; i < samples ; i++) {
		  runningSum = getHX711(tim);
		  if (runningSum == 0) {
			  divisor--;
		  }
		  else {
			  total += runningSum;
		  }
	  }
	  if (divisor < 1){
		  // divisor is zero
	  }
	  else {
	  tare = (int32_t)(total / divisor);
	  }
}


