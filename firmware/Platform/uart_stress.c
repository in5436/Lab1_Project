#include "uart_stress.h"
#include <stddef.h>
#include <string.h>

int uart_stress_parse(const char *command, uint16_t *bytes)
{
    const char *p;
    uint32_t value = 0U;

    if (command == NULL || bytes == NULL ||
        strncmp(command, "load ", 5U) != 0) {
        return 0;
    }

    p = command + 5;

    if (*p == '\0') {
        return 0;
    }

    while (*p != '\0') {
        if (*p < '0' || *p > '9') {
            return 0;
        }

        value = value * 10U + (uint32_t)(*p - '0');

        if (value > UART_STRESS_MAX_BYTES) {
            return 0;
        }

        p++;
    }

    if (value < UART_STRESS_MIN_BYTES ||
        (value % UART_STRESS_CHUNK) != 0U) {
        return 0;
    }

    *bytes = (uint16_t)value;
    return 1;
}

void uart_stress_make_chunk(uint8_t chunk[UART_STRESS_CHUNK])
{
    uint32_t i;

    if (chunk == NULL) {
        return;
    }

    for (i = 0U; i < UART_STRESS_CHUNK - 2U; ++i) {
        chunk[i] = (uint8_t)('A' + (i % 26U));
    }

    chunk[UART_STRESS_CHUNK - 2U] = '\r';
    chunk[UART_STRESS_CHUNK - 1U] = '\n';
}