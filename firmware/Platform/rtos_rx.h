#ifndef RTOS_RX_H
#define RTOS_RX_H

#include <stdint.h>
#include "main.h"

#define RTOS_RX_CMD_SIZE 32U

typedef struct {
    uint32_t bytes;
    uint32_t lines;
    uint32_t overrun_events;
    uint32_t long_lines;
    uint32_t ring_dropped;
    uint32_t cmd_dropped;
} rtos_rx_stats_t;

int RTOS_RX_Init(UART_HandleTypeDef *huart);
int RTOS_RX_GetCommand(char command[RTOS_RX_CMD_SIZE]);

void RTOS_RX_GetStats(rtos_rx_stats_t *stats);
void RTOS_RX_ResetStats(void);

void RTOS_RX_Task(void *argument);
void RTOS_RX_IRQHandler(void);

#endif