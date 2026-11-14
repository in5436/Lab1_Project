#ifndef UART_STRESS_H
#define UART_STRESS_H

#include <stdint.h>

#define UART_STRESS_CHUNK     64U
#define UART_STRESS_MIN_BYTES 256U
#define UART_STRESS_MAX_BYTES 4096U

int uart_stress_parse(const char *command, uint16_t *bytes);
void uart_stress_make_chunk(uint8_t chunk[UART_STRESS_CHUNK]);

#endif