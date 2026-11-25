#include "stm32c0xx_hal.h"

extern uint32_t uwTickPrio;

HAL_StatusTypeDef HAL_InitTick(uint32_t TickPriority)
{
    uint32_t clock_hz;

    (void)TickPriority;

    RCC->APBENR2 |= RCC_APBENR2_TIM14EN;
    (void)RCC->APBENR2;

    clock_hz = HAL_RCC_GetPCLK1Freq();

    if (clock_hz < 1000000U ||
        (clock_hz % 1000000U) != 0U) {
        return HAL_ERROR;
    }

    TIM14->CR1 = 0U;
    TIM14->PSC = (clock_hz / 1000000U) - 1U;
    TIM14->ARR = 999U;
    TIM14->CNT = 0U;

    TIM14->EGR = TIM_EGR_UG;
    TIM14->SR = 0U;
    TIM14->DIER = TIM_DIER_UIE;

    NVIC_SetPriority(TIM14_IRQn, 1U);
    NVIC_ClearPendingIRQ(TIM14_IRQn);
    NVIC_EnableIRQ(TIM14_IRQn);

    uwTickPrio = 1U;

    TIM14->CR1 = TIM_CR1_CEN;

    return HAL_OK;
}

void HAL_SuspendTick(void)
{
    TIM14->DIER &= ~TIM_DIER_UIE;
}

void HAL_ResumeTick(void)
{
    TIM14->DIER |= TIM_DIER_UIE;
}

void TIM14_IRQHandler(void)
{
    uint32_t state = __get_PRIMASK();

    __disable_irq();

    if ((TIM14->SR & TIM_SR_UIF) != 0U) {
        TIM14->SR = ~TIM_SR_UIF;
        HAL_IncTick();
    }

    __set_PRIMASK(state);
}