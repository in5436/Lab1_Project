#ifndef PERF_H
#define PERF_H

#include <stdint.h>

void perf_init(void);

uint32_t perf_now_us(void);

#endif