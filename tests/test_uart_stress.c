#include "uart_stress.h"
#include <assert.h>
#include <stdint.h>
#include <stdio.h>

int main(void)
{
    uint16_t count = 0U;
    uint8_t chunk[UART_STRESS_CHUNK];

    const char *invalid[] = {
        "load",
        "load ",
        "load 0",
        "load 255",
        "load 257",
        "load 4097",
        "load -256",
        "load 256x",
        "load  256",
        "status",
        "load 9999999999999"
    };

    unsigned i;

    assert(uart_stress_parse("load 256", &count) == 1);
    assert(count == 256U);

    assert(uart_stress_parse("load 1024", &count) == 1);
    assert(count == 1024U);

    assert(uart_stress_parse("load 4096", &count) == 1);
    assert(count == 4096U);

    assert(uart_stress_parse("load 256", NULL) == 0);
    assert(uart_stress_parse(NULL, &count) == 0);

    for (i = 0U;
         i < sizeof(invalid) / sizeof(invalid[0]);
         ++i) {
        assert(uart_stress_parse(invalid[i], &count) == 0);
    }

    uart_stress_make_chunk(chunk);

    assert(chunk[0] == 'A');
    assert(chunk[25] == 'Z');
    assert(chunk[26] == 'A');
    assert(chunk[UART_STRESS_CHUNK - 2U] == '\r');
    assert(chunk[UART_STRESS_CHUNK - 1U] == '\n');

    puts("UART stress tests passed");
    return 0;
}