#include "perf.h"
#include "stm32c0xx.h"

static volatile uint32_t s_tim3_overflows = 0U;

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
    uint32_t primask = __get_PRIMASK();
    uint32_t overflows;
    uint32_t count;
    uint32_t sr_before;
    uint32_t sr_after;

    __disable_irq();

    overflows = s_tim3_overflows;

    sr_before = TIM3->SR;
    count = TIM3->CNT & 0xFFFFU;
    sr_after = TIM3->SR;

    if ((sr_before & TIM_SR_UIF) != 0U) {
        overflows++;
    } else if ((sr_after & TIM_SR_UIF) != 0U) {
        overflows++;
        count = TIM3->CNT & 0xFFFFU;
    }

    if (primask == 0U) {
        __enable_irq();
    }

    return (overflows << 16) | count;
}