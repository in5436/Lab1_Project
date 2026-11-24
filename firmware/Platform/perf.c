#include "perf.h"
#include "stm32c0xx.h"
#include <stdio.h>
#include <string.h>

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

static volatile uint32_t s_tim3_overflows = 0U;
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
    RCC->APBENR1 |= RCC_APBENR1_TIM3EN;
    (void)RCC->APBENR1;

    TIM3->CR1 = 0U;
    TIM3->PSC = (SystemCoreClock / 1000000U) - 1U;
    TIM3->ARR = 0xFFFFU;
    TIM3->CNT = 0U;

    TIM3->EGR = TIM_EGR_UG;
    TIM3->SR = 0U;

    s_tim3_overflows = 0U;
    TIM3->DIER = TIM_DIER_UIE;

    NVIC_SetPriority(TIM3_IRQn, 2U);
    NVIC_ClearPendingIRQ(TIM3_IRQn);
    NVIC_EnableIRQ(TIM3_IRQn);

    TIM3->CR1 = TIM_CR1_CEN;

    perf_stats_reset();
}

void TIM3_IRQHandler(void)
{
    if ((TIM3->SR & TIM_SR_UIF) != 0U) {
        TIM3->SR = ~TIM_SR_UIF;
        s_tim3_overflows++;
    }
}

uint32_t perf_now_us(void)
{
    uint32_t state = perf_lock();

    uint32_t high = s_tim3_overflows;
    uint32_t count = TIM3->CNT & 0xFFFFU;

    if ((TIM3->SR & TIM_SR_UIF) != 0U) {
        high++;
        count = TIM3->CNT & 0xFFFFU;
    }

    perf_unlock(state);

    return (high << 16) | count;
}

void perf_stats_reset(void)
{
    uint32_t state = perf_lock();

    memset(s_stats, 0, sizeof(s_stats));
    s_stats_start_us = perf_now_us();

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
    uint32_t state = perf_lock();

    window_us = perf_now_us() - s_stats_start_us;

    memcpy(s_snapshot, s_stats, sizeof(s_snapshot));

    perf_unlock(state);

    printf("[STATS] window_ms=%lu\r\n",
            (unsigned long)(window_us / 1000U));

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