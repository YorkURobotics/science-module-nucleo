/*
 * h2_sensor_driver.c
 *
 * Created on: 19th May 2026
 *     Author: Mykola Bohomaz
 */

#include "h2_sensor_driver.h"

//===================================================================================
// HELPER FUNCTIONS
//===================================================================================

static float convert_adc_to_sensor_voltage(uint16_t adc_value) {
  float adc_voltage = ((float)adc_value / ADC_MAX_VALUE) * ADC_REF_VOLTAGE;

  return adc_voltage * ( DIVIDER_R2 / (DIVIDER_R1 + DIVIDER_R2));
}

static uint16_t convert_voltage_to_millivolts(float voltage) {
  return (uint16_t)(voltage * 1000.0f);
}

// TODO: potentially implement DMA to improve the MCU scheduling

//===================================================================================
// USER FUNCTIONS
//===================================================================================

/*
 * Initialize H2 sensor driver
 * @param sensor H2 sensor struct pointer
 * @param hadc ADC handle pointer
 * @return HAL status
 */
HAL_StatusTypeDef h2_sensor_init(H2_Sensor *sensor, ADC_HandleTypeDef *hadc) {
  if (sensor == NULL || hadc == NULL) {
    return HAL_ERROR;
  }

  sensor->hadc = hadc;
  sensor->adc_raw = 0;
  sensor->sensor_voltage = 0.0f;

  return HAL_OK;
}

/*
 * Read ADC data and convert it to a human-readable sensor voltage
 * @param sensor H2 sensor struct pointer
 * @return HAL status
 */
HAL_StatusTypeDef read_ADC(H2_Sensor *sensor) {
  HAL_StatusTypeDef status;

  if (sensor == NULL || sensor->hadc == NULL) {
    return HAL_ERROR;
  }

  status = HAL_ADC_Start(sensor->hadc);
  if (status != HAL_OK) {
    return status;
  }

  status = HAL_ADC_PollForConversion(sensor->hadc, 10);
  if (status == HAL_OK) {
    sensor->adc_raw = HAL_ADC_GetValue(sensor->hadc);
    sensor->sensor_voltage = convert_adc_to_sensor_voltage(sensor->adc_raw);
  }

  if (HAL_ADC_Stop(sensor->hadc) != HAL_OK) {
    return HAL_ERROR;
  }

  return status;
}

/*
 * CAN transmit wrapper
 * @param hcan CAN handle pointer
 * @param sensor H2 sensor struct pointer
 * @return HAL status
 */
HAL_StatusTypeDef transmit_h2_sensor_values(CAN_HandleTypeDef *hcan, H2_Sensor *sensor) {
  uint8_t tx_data[2] = {0};
  uint16_t sensor_millivolts;

  if (hcan == NULL || sensor == NULL) {
    return HAL_ERROR;
  }

  sensor_millivolts = convert_voltage_to_millivolts(sensor->sensor_voltage);

  tx_data[0] = (sensor_millivolts >> 8) & 0xFF;
  tx_data[1] = sensor_millivolts & 0xFF;

  return CAN_TRANSMIT(hcan, CAN_H2_SENSOR_ID, tx_data, 2);
}
