#ifndef RTOS_UART_H
#define RTOS_UART_H

#include <stdint.h>
#include "main.h"

int RTOS_UART_Init(UART_HandleTypeDef *huart);
int RTOS_UART_Write(const char *data, int length);
int RTOS_UART_RequestLoad(uint16_t bytes);
uint32_t RTOS_UART_GetDropped(void);

void RTOS_UART_Task(void *argument);

void RTOS_UART_SetDiagnostic(uint8_t enabled);
void RTOS_UART_ResetDropped(void);

#endif