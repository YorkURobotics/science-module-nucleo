/*
 * h2_sensor_driver.h
 *
 *  Created on: 19th May 2026
 *      Author: Mykola Bohomaz
 *
 */

#ifndef H2_SENSOR_DRIVER_H
#define H2_SENSOR_DRIVER_H

#include "main.h"
#include <stdint.h>

/*
 * ADC config
 */
#define ADC_MAX_VALUE 4095.0f
#define ADC_REF_VOLTAGE 3.3f

/*
 * Voltage divider config
 */
#define DIVIDER_R1 2000.0f
#define DIVIDER_R2 3000.0f

typedef struct {
  ADC_HandleTypeDef *hadc;
  uint16_t adc_raw;
  float sensor_voltage;
} H2_Sensor;

/*
 * Initialize H2 sensor driver
 * @param sensor H2 sensor struct pointer
 * @param hadc ADC handle pointer
 * @return HAL status
 */
HAL_StatusTypeDef h2_sensor_init(H2_Sensor *sensor, ADC_HandleTypeDef *hadc);

/*
 * Read ADC data and convert it to a human-readable sensor voltage
 * @param sensor H2 sensor struct pointer
 * @return HAL status
 */
HAL_StatusTypeDef read_ADC(H2_Sensor *sensor);

/*
 * CAN transmit wrapper
 * @param hcan CAN handle pointer
 * @param sensor H2 sensor struct pointer
 * @return HAL status
 */
HAL_StatusTypeDef transmit_h2_sensor_values(CAN_HandleTypeDef *hcan, H2_Sensor *sensor);

#endif
