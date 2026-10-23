/*
 * can_handler.h
 *
 *  Created on: Mar 6, 2026
 *      Author: Tirth Patel
 */

#ifndef CAN_HANDLER_H
#define CAN_HANDLER_H

#include <stdint.h>
#include "main.h"

// -----------------------------------------------------------------------
// Configuration (Define all CAN_IDs here)
// -----------------------------------------------------------------------

// IDs that ROS already decodes (Rover-Team-Codebase configs/can_id_catalog.json)
#define CAN_TEMPHUMID_ID  0x0D110000  // soil temperature/humidity
#define CAN_PHCOND_ID     0x0D110400  // soil pH/conductivity
#define CAN_NPK_ID        0x0D110800  // soil NPK
#define CAN_CO2_ID        0x0D110C00  // CO2

// Placeholders until IDs are assigned (no other node uses these values)
#define CAN_H2_SENSOR_ID  0x0F110040

// -----------------------------------------------------------------------
// Function Prototypes
// -----------------------------------------------------------------------

HAL_StatusTypeDef CAN_CONFIG(CAN_HandleTypeDef *hcan);
HAL_StatusTypeDef CAN_TRANSMIT(CAN_HandleTypeDef * hcan, uint32_t id, uint8_t *data, uint8_t size);

void CAN_Process_Incoming(uint32_t id, uint8_t *data, uint8_t len);

#endif /* CAN_HANDLER_H */
