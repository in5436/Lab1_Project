#ifndef PERF_H
#define PERF_H

#include <stdint.h>

typedef enum {
    PERF_UART_CMD = 0,
    PERF_ADC,
    PERF_IMU,
    PERF_LOGIC,
    PERF_LED,
    PERF_TELEMETRY,
    PERF_LOOP,
    PERF_TASK_COUNT
} perf_task_t;

void perf_init(void);
uint32_t perf_now_us(void);

void perf_record(perf_task_t task, uint32_t started_us);
void perf_note_schedule(perf_task_t task,
                        uint32_t skipped,
                        uint32_t lateness_ms);

void perf_note_deadline_miss(perf_task_t task);
void perf_stats_reset(void);
void perf_stats_print(void);

#endif