/*
 * logger.c
 *
 *  Created on: Aug 3, 2026
 *      Author: artin59
 */

#include "logger.h"

UART_HandleTypeDef *luart;

void LOG_INIT(UART_HandleTypeDef *huart){
	luart = huart;
	uint8_t data[] = PURPLE "-------STARTING YURS LOGGING FRAMEWORK-------" FIN "\r\n";

	HAL_UART_Transmit(luart, data, sizeof(data), 100);
}

uint32_t _get_time(){
	return HAL_GetTick(); //TODO: change to actual time
}

HAL_StatusTypeDef logger(const char *type, const char *file, int line, const char *fmt, ...){

	char output [256];

	int j = snprintf(output, 256 ,"%lu %s" FIN " [%s:%d]: ", _get_time(), type, file, line);

	if (j < 0) j=0;
	if (j > 255) j=255;

	va_list args;
	va_start (args, fmt);

	int k = vsnprintf(output + j, 256-j, fmt, args);

	va_end(args);

	if (k < 0) k =0;

	int total = j+k;
	if (total > 255) total=255;

	if (total > 253) total = 253;
	output[total++] = '\r';
	output[total++] = '\n';

	return HAL_UART_Transmit(luart, (uint8_t *) output, total, 100);
}


