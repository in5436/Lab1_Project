#include "perf.h"
#include "stm32c0xx_hal.h"
#include <stdio.h>
#include <string.h>
#include "FreeRTOS.h"

typedef struct {
    uint32_t count;
    uint32_t exec_min_us;
    uint32_t exec_max_us;
    uint64_t exec_sum_us;

    uint32_t last_start_us;
    uint32_t interval_count;
    uint32_t interval_min_us;
    uint32_t interval_max_us;
    uint64_t interval_sum_us;

    uint32_t jitter_max_us;
    uint32_t missed;
    uint32_t overruns;

    uint32_t lateness_max_ms;
    uint32_t deadline_misses;
} perf_stat_t;

static uint32_t s_last_us = 0U;
static volatile uint32_t s_tick_us = 0U;
static volatile uint8_t s_tick_ready = 0U;
static uint32_t s_timer_backwards = 0U;
static uint32_t s_stats_start_ms = 0U;
static perf_stat_t s_stats[PERF_TASK_COUNT];
static uint32_t s_stats_start_us;

static perf_stat_t s_snapshot[PERF_TASK_COUNT];

static uint32_t perf_lock(void)
{
    uint32_t state = __get_PRIMASK();

    __disable_irq();

    return state;
}

static void perf_unlock(uint32_t state)
{
    __set_PRIMASK(state);
}

static const char *const s_names[PERF_TASK_COUNT] = {
    "UART_CMD",
    "ADC",
    "IMU",
    "LOGIC",
    "LED_SAMPLE",
    "TELEMETRY",
    "LOOP_SAMPLE"
};

static const uint32_t s_periods_us[PERF_TASK_COUNT] = {
    0U, 20000U, 20000U, 0U, 0U, 1000000U, 0U
};

void perf_init(void)
{
    s_last_us = 0U;
    s_tick_us = 0U;
    s_tick_ready = 0U;

    perf_stats_reset();
}

void vApplicationTickHook(void)
{
    if (s_tick_ready == 0U) {
        s_tick_us = HAL_GetTick() * 1000U;
        s_tick_ready = 1U;
    } else {
        s_tick_us += 1000000U / configTICK_RATE_HZ;
    }
}

uint32_t perf_now_us(void)
{
    uint32_t state = perf_lock();
    uint32_t result;

    if (s_tick_ready == 0U) {
        result = HAL_GetTick() * 1000U;
    } else {
        uint32_t pending_before;
        uint32_t pending_after;
        uint32_t value;

        do {
            pending_before =
                SCB->ICSR & SCB_ICSR_PENDSTSET_Msk;

            value = SysTick->VAL;

            pending_after =
                SCB->ICSR & SCB_ICSR_PENDSTSET_Msk;

        } while (pending_before != pending_after);

        result = s_tick_us;

        if (pending_after != 0U) {
            result += 1000000U / configTICK_RATE_HZ;
        }

        if (value != 0U) {
            result += (SysTick->LOAD + 1U - value) / 48U;
        }
    }

    if ((int32_t)(result - s_last_us) < 0) {
        s_timer_backwards++;
    }

    s_last_us = result;

    perf_unlock(state);

    return result;
}

void perf_stats_reset(void)
{
    uint32_t state = perf_lock();

    memset(s_stats, 0, sizeof(s_stats));

    s_stats_start_us = perf_now_us();
    s_stats_start_ms = HAL_GetTick();
    s_timer_backwards = 0U;

    perf_unlock(state);
}

void perf_record(perf_task_t task, uint32_t started_us)
{
    uint32_t duration_us;
    uint32_t ended_us;
    uint32_t state;
    perf_stat_t *s;
    uint32_t expected_us, gap_us, jitter_us;

    if ((unsigned)task >= (unsigned)PERF_TASK_COUNT) {
        return;
    }

    state = perf_lock();

    ended_us = perf_now_us();
    duration_us = ended_us - started_us;

    if (duration_us > (ended_us - s_stats_start_us)) {
        perf_unlock(state);
        return;
    }

    s = &s_stats[task];

    if (s->count == 0U || duration_us < s->exec_min_us) {
        s->exec_min_us = duration_us;
    }

    if (duration_us > s->exec_max_us) {
        s->exec_max_us = duration_us;
    }

    s->exec_sum_us += duration_us;
    s->count++;

    expected_us = s_periods_us[task];

    if (expected_us != 0U) {
        if (s->count > 1U) {
            gap_us = started_us - s->last_start_us;

            if (s->interval_count == 0U ||
                gap_us < s->interval_min_us) {
                s->interval_min_us = gap_us;
            }

            if (gap_us > s->interval_max_us) {
                s->interval_max_us = gap_us;
            }

            s->interval_sum_us += gap_us;
            s->interval_count++;

            jitter_us = gap_us > expected_us
                ? gap_us - expected_us
                : expected_us - gap_us;

            if (jitter_us > s->jitter_max_us) {
                s->jitter_max_us = jitter_us;
            }
        }

        s->last_start_us = started_us;

        if (duration_us > expected_us) {
            s->overruns++;
        }
    }

perf_unlock(state);
}

void perf_note_schedule(perf_task_t task,
                        uint32_t skipped,
                        uint32_t lateness_ms)
{
    perf_stat_t *s;
    uint32_t state;

    if ((unsigned)task >= (unsigned)PERF_TASK_COUNT) {
        return;
    }

    if (s_periods_us[task] == 0U) {
        return;
    }

    state = perf_lock();

    s = &s_stats[task];

    s->missed += skipped;

    if (lateness_ms > s->lateness_max_ms) {
        s->lateness_max_ms = lateness_ms;
    }

    perf_unlock(state);
}

void perf_note_deadline_miss(perf_task_t task)
{
    uint32_t state;

    if ((unsigned)task >= (unsigned)PERF_TASK_COUNT) {
        return;
    }

    state = perf_lock();

    s_stats[task].deadline_misses++;

    perf_unlock(state);
}

void perf_stats_print(void)
{
    unsigned i;
    uint32_t window_us;
    uint32_t hal_window_ms;
    uint32_t backwards;
    uint32_t state = perf_lock();

    window_us = perf_now_us() - s_stats_start_us;
    hal_window_ms = HAL_GetTick() - s_stats_start_ms;
    backwards = s_timer_backwards;

    memcpy(s_snapshot, s_stats, sizeof(s_snapshot));

    perf_unlock(state);

    printf("[STATS] window_ms=%lu\r\n",
        (unsigned long)(window_us / 1000U));

    printf("[TIME] hal_window_ms=%lu backward_reads=%lu\r\n",
        (unsigned long)hal_window_ms,
        (unsigned long)backwards);

    for (i = 0U; i < (unsigned)PERF_TASK_COUNT; ++i) {
        const perf_stat_t *s = &s_snapshot[i];

        uint32_t avg_us = s->count != 0U
            ? (uint32_t)(s->exec_sum_us / s->count)
            : 0U;

        printf("[TASK] name=%s n=%lu min_us=%lu avg_us=%lu max_us=%lu\r\n",
               s_names[i],
               (unsigned long)s->count,
               (unsigned long)s->exec_min_us,
               (unsigned long)avg_us,
               (unsigned long)s->exec_max_us);

        if (s_periods_us[i] != 0U) {
            uint32_t avg_period_us = s->interval_count != 0U
                ? (uint32_t)(s->interval_sum_us / s->interval_count)
                : 0U;

            printf("[PERIOD] name=%s n=%lu min_us=%lu avg_us=%lu max_us=%lu jitter_max_us=%lu missed=%lu late_max_ms=%lu deadline_miss=%lu overruns=%lu\r\n",
                s_names[i],
                (unsigned long)s->interval_count,
                (unsigned long)s->interval_min_us,
                (unsigned long)avg_period_us,
                (unsigned long)s->interval_max_us,
                (unsigned long)s->jitter_max_us,
                (unsigned long)s->missed,
                (unsigned long)s->lateness_max_ms,
                (unsigned long)s->deadline_misses,
                (unsigned long)s->overruns);
        }
    }
}