#include "../inc/drv_timer.h"

static volatile uint32 s_tickMs = 0U;

Status_t DRV_TIMER_Init(Timer_t timer)
{
    switch(timer)
    {
        case TIMER_4:

            RCC->APB1ENR |= RCC_APB1ENR_TIM4EN;

            /* Timer clock = 1 MHz */
            TIM4->PSC = 72U - 1U;
            TIM4->ARR = 0xFFFFU;
            TIM4->CNT = 0U;
            TIM4->EGR |= TIM_EGR_UG;
            TIM4->CR1 |= TIM_CR1_CEN;

            break;

        default:
            return STATUS_INVALID_PARAM;
    }

    return STATUS_OK;
}

/**
 * @brief Initialize SysTick (1 ms)
 */
Status_t DRV_SYSTICK_Init(void)
{
    SysTick->LOAD = 72000U - 1U;
    SysTick->VAL  = 0U;

    SysTick->CTRL =
            SysTick_CTRL_CLKSOURCE_Msk |
            SysTick_CTRL_TICKINT_Msk   |
            SysTick_CTRL_ENABLE_Msk;

    return STATUS_OK;
}

/**
 * @brief Blocking delay (microsecond)
 */
void DRV_DelayUs(uint32 us)
{
    TIM4->ARR = us - 1U;
    TIM4->CNT = 0U;

    TIM4->SR &= ~TIM_SR_UIF;

    while((TIM4->SR & TIM_SR_UIF) == 0U);

    TIM4->SR &= ~TIM_SR_UIF;
}

/**
 * @brief Blocking delay (millisecond)
 */
void DRV_DelayMs(uint32 ms)
{
    uint32 i;

    for(i = 0U; i < ms; i++)
    {
        DRV_DelayUs(1000U);
    }
}

/**
 * @brief Initialize system tick timer (TIM2)
 */
Status_t DRV_TIMER_TickInit(void)
{
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;

    TIM2->PSC = 72U - 1U;
    TIM2->ARR = 1000U - 1U;

    TIM2->CNT = 0U;

    TIM2->SR &= ~TIM_SR_UIF;

    TIM2->DIER |= TIM_DIER_UIE;

    TIM2->CR1 |= TIM_CR1_CEN;

    NVIC_EnableIRQ(TIM2_IRQn);

    return STATUS_OK;
}

/**
 * @brief Get system tick (ms)
 */
uint32 DRV_TIMER_GetTick(void)
{
    return s_tickMs;
}


void TIM2_IRQHandler(void)
{
    if(TIM2->SR & TIM_SR_UIF)
    {
        TIM2->SR &= ~TIM_SR_UIF;

        s_tickMs++;
    }
}