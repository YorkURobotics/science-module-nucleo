/**
 ******************************************************************************
 * @file    SOIL_SENSOR.h
 * @brief   Driver for NHK RS485 Modbus Soil Sensor
 *
 * @note    UART config: 4800 baud, 8N1
 *          Default slave ID: 0x01
 ******************************************************************************
 */

#ifndef SOIL_SENSOR_H
#define SOIL_SENSOR_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>
#include <stdbool.h>

/* -------------------------------------------------------------------------
 * Types
 * ------------------------------------------------------------------------- */

typedef enum {
    SOIL_OK         = 0,
    SOIL_ERR_TX     = 1,
    SOIL_ERR_RX     = 2,
    SOIL_ERR_CRC    = 3,
} Soil_Status;

typedef struct {
    float    humidity;      /* %RH       */
    float    temperature;   /* degrees C */
    uint16_t conductivity;  /* us/cm     */
    float    ph;            /* pH        */
    uint16_t nitrogen;      /* mg/kg     */
    uint16_t phosphorus;    /* mg/kg     */
    uint16_t potassium;     /* mg/kg     */
} Soil_Data;

/* -------------------------------------------------------------------------
 * Public API — one function per datasheet command
 * ------------------------------------------------------------------------- */

/**
 * @brief Read humidity, temperature, conductivity, pH, N, P, K all at once
 *        Modbus: 0x01 0x03 0x00 0x00 0x00 0x07 CRC CRC
 */
Soil_Status Soil_ReadAll(UART_HandleTypeDef *huart, Soil_Data *data);

/**
 * @brief Read all soil parameters averaged over 10 samples
 *        Collects 10 readings with 500ms between each,
 *        averages only valid samples, ignores failed ones
 */
Soil_Status Soil_ReadAvg(UART_HandleTypeDef *huart, Soil_Data *data);

/**
 * @brief Query the slave ID of the connected sensor (uses broadcast 0xFF)
 *        Modbus: 0xFF 0x03 0x07 0xD0 0x00 0x01 0x91 0x59
 */
Soil_Status Soil_QuerySlaveID(UART_HandleTypeDef *huart, uint8_t *slave_id);

/**
 * @brief Set a new slave ID on the sensor
 *        Modbus: 0x01 0x06 0x07 0xD0 0x00 <id> CRC CRC
 * @param new_id  New slave ID (1-254)
 */
Soil_Status Soil_SetSlaveID(UART_HandleTypeDef *huart, uint8_t new_id);

/**
 * @brief Set the baud rate of the sensor
 *        Modbus: 0x01 0x06 0x07 0xD1 0x00 <rate> CRC CRC
 * @param rate  0=2400, 1=4800, 2=9600
 */
Soil_Status Soil_SetBaudRate(UART_HandleTypeDef *huart, uint8_t rate);

/**
 * @brief Write a nitrogen value into the sensor register
 *        Modbus: 0x01 0x06 0x00 0x04 <value high byte> <value low byte> CRC CRC
 */
Soil_Status Soil_WriteNitrogen(UART_HandleTypeDef *huart, uint16_t value);

/**
 * @brief Write a phosphorus value into the sensor register
 *        Modbus: 0x01 0x06 0x00 0x05 <value high byte> <value low byte> CRC CRC
 */
Soil_Status Soil_WritePhosphorus(UART_HandleTypeDef *huart, uint16_t value);

/**
 * @brief Write a potassium value into the sensor register
 *        Modbus: 0x01 0x06 0x00 0x06 <value high byte> <value low byte> CRC CRC
 */
Soil_Status Soil_WritePotassium(UART_HandleTypeDef *huart, uint16_t value);

/**
 * @brief Check if sensor is alive and responding
 * @retval true if sensor responds, false otherwise
 */
bool Soil_IsAlive(UART_HandleTypeDef *huart);

/**
 * @brief Format Soil_Data into a readable string for debug printing
 */
void Soil_FormatData(const Soil_Data *data, char *buf, uint16_t buflen);

/**
 * @brief Convert status code to string
 */
const char *Soil_StatusStr(Soil_Status status);


void Soil_CAN_Transmit(CAN_HandleTypeDef *hcan, Soil_Data *data);


#ifdef __cplusplus
}
#endif

#endif /* SOIL_SENSOR_H */
