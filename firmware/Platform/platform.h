#ifndef PLATFORM_H
#define PLATFORM_H

#include "main.h"

void Platform_Init(UART_HandleTypeDef *huart, I2C_HandleTypeDef *hi2c, ADC_HandleTypeDef *hadc);
void Platform_Step(void);

#endif