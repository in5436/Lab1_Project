#ifndef RTOS_APP_H
#define RTOS_APP_H

#include "main.h"

void RTOS_Start(UART_HandleTypeDef *huart);

void RTOS_PrintMemory(void);

#endif