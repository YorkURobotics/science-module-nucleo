/*
 * logger.h
 *
 *  Created on: Aug 3, 2026
 *      Author: artin59
 */


#ifndef INC_LOGGER_H_
#define INC_LOGGER_H_

#include "stm32f3xx_hal.h"
#include <stdio.h>
#include <stdarg.h>
#include <string.h>

extern UART_HandleTypeDef *luart;

#define FIN "\x1b[0m"
#define RED "\x1b[31m"
#define BLUE "\x1b[34m"
#define GREEN "\x1b[32m"
#define YELLOW "\x1b[33m"
#define PURPLE "\x1b[35m"

#define INFO BLUE "INFO"
#define WARNING YELLOW "WARNING"
#define ERROR RED "ERROR"

#define __FILENAME__ (strrchr(__FILE__, '/') ? strrchr(__FILE__, '/') + 1 : __FILE__)

#define LOG_INFO(...) logger(INFO, __FILENAME__, __LINE__, __VA_ARGS__)
#define LOG_WARNING(...) logger(WARNING, __FILENAME__, __LINE__, __VA_ARGS__)
#define LOG_ERROR(...) logger(ERROR, __FILENAME__, __LINE__, __VA_ARGS__)


void LOG_INIT(UART_HandleTypeDef *huart);

uint32_t _get_time();

HAL_StatusTypeDef logger(const char *type, const char *file, int line, const char *fmt, ...);



#endif /* INC_LOGGER_H_ */
